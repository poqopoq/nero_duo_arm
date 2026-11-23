#!/usr/bin/env python3
"""Make rollout ``/reset`` keep moving until the bridge reaches the home target.

LeRobot 0.6.2 interpolates to the reset pose for a fixed three seconds, then
stops issuing that target.  The NERO bridge rate-limits every stream target and
returns the *applied* target, so a slow arm can still be far from home when the
interpolation ends.  The following idle hold then latches that partial pose.

This patch keeps the interpolation ramp, then sends the true home target until
the bridge's applied action is within one bridge step of the target (or a
30 second safety timeout expires).

Usage:
    python deployment/scripts/patch_lerobot_reset_convergence.py \
        --lerobot-root /home/work/lerobot

    python ... --check   # exit 1 when the patch is missing
"""

from __future__ import annotations

import argparse
import py_compile
import shutil
import sys
from pathlib import Path

MARKER = "reset_settle_tolerance"
START = '''    @staticmethod
    def return_to_initial_position(hw: HardwareContext, duration_s: float = 3.0, fps: int = 50) -> bool:
'''
END = '''
    @staticmethod
    def _log_telemetry(
'''

REPLACEMENT = '''    @staticmethod
    def return_to_initial_position(hw: HardwareContext, duration_s: float = 3.0, fps: int = 50) -> bool:
        """Interpolate to home, then keep commanding home until it is reached.

        The bridge rate-limits every stream target and returns the applied
        target.  A fixed-duration interpolation can therefore leave the arm
        partway home, and the idle hold would latch that partial target.  The
        settle phase below deliberately resends the true home target instead of
        the last applied action.
        """
        # The bridge returns the applied (possibly rate-limited) target.  Only
        # when that value equals home is the measured pose inside one bridge step.
        reset_settle_tolerance = 0.0001
        reset_settle_timeout_s = 30.0
        robot = hw.robot_wrapper
        if robot.hardware_failure is not None:
            logger.warning(
                "Cannot return to initial position after robot I/O failure: %s", robot.hardware_failure
            )
            return False
        target = hw.initial_position
        if target is None:
            logger.warning("Could not return to initial position: none was captured at connect time")
            return False
        try:
            current_obs = robot.get_observation()
            current_pos = {k: float(current_obs[k]) for k in target}
            if not all(math.isfinite(current_pos[k]) and math.isfinite(target[k]) for k in target):
                raise ValueError("Return movement requires finite current and initial positions")
            if max(abs(current_pos[k] - target[k]) for k in target) <= reset_settle_tolerance:
                robot.send_action(target)
                return True
            steps = max(int(duration_s * fps), 1)
            for step in range(1, steps + 1):
                t = step / steps
                interp = {
                    k: current_pos[k] * (1 - t) + target[k] * t
                    for k in current_pos
                }
                robot.send_action(interp)
                precise_sleep(1 / fps)

            # The interpolation's final command is home, but the bridge may
            # have clamped the returned applied target.  Keep sending the true
            # target so the next idle hold retains home, not the partial pose.
            settle_steps = max(int(reset_settle_timeout_s * fps), 1)
            last_error = float("inf")
            for _ in range(settle_steps):
                applied = robot.send_action(target)
                last_error = max(abs(float(applied[k]) - target[k]) for k in target)
                if last_error <= reset_settle_tolerance:
                    return True
                precise_sleep(1 / fps)
            logger.warning(
                "Return to initial position did not converge within %.1f s; max error %.4f",
                reset_settle_timeout_s,
                last_error,
            )
            return False
        except Exception as e:
            logger.warning("Could not return to initial position: %s", e)
            return False
'''


def patch_text(text: str) -> tuple[str, bool]:
    if MARKER in text:
        return text, False
    start = text.find(START)
    if start < 0:
        raise RuntimeError("cannot find return_to_initial_position")
    end = text.find(END, start + len(START))
    if end < 0:
        raise RuntimeError("cannot find the end of return_to_initial_position")
    return text[:start] + REPLACEMENT + text[end:], True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--lerobot-root",
        type=Path,
        default=Path("/home/work/lerobot"),
        help="LeRobot checkout containing src/lerobot (default: %(default)s)",
    )
    parser.add_argument("--check", action="store_true", help="only verify the patch is present")
    args = parser.parse_args()

    path = args.lerobot_root / "src/lerobot/rollout/strategies/core.py"
    if not path.is_file():
        print(f"rollout strategy core not found: {path}", file=sys.stderr)
        return 2

    original = path.read_text()
    patched, changed = patch_text(original)
    if args.check:
        if changed:
            print("reset-convergence compatibility is missing", file=sys.stderr)
            return 1
        print(f"reset-convergence compatibility already present: {args.lerobot_root}")
        return 0
    if not changed:
        print(f"reset-convergence compatibility already present: {args.lerobot_root}")
        return 0

    backup = path.with_suffix(path.suffix + ".bak-joint16-reset")
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(patched)
    py_compile.compile(str(path), doraise=True)
    print(f"Patched {path}")
    print(f"Backup: {backup}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
