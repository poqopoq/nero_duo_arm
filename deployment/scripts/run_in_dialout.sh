#!/usr/bin/env bash
set -euo pipefail

if id -nG | tr ' ' '\n' | grep -qx dialout; then
  exec "$@"
fi

if [[ "${DIALOUT_REEXEC:-0}" == "1" ]]; then
  echo "error: dialout group is not active in this session" >&2
  echo "run 'newgrp dialout' or log out and back in" >&2
  exit 1
fi

user_name="$(id -un)"
if ! getent group dialout | cut -d: -f4 | tr ',' '\n' | grep -qx "${user_name}"; then
  echo "error: ${user_name} is not a member of the dialout group" >&2
  echo "run: sudo usermod -aG dialout ${user_name}" >&2
  echo "then log out and back in (or run 'newgrp dialout')" >&2
  exit 1
fi

quoted_command="$(printf '%q ' "$@")"
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  quoted_library_path="$(printf '%q' "${LD_LIBRARY_PATH}")"
  quoted_command="env LD_LIBRARY_PATH=${quoted_library_path} ${quoted_command}"
fi
exec env DIALOUT_REEXEC=1 sg dialout -c "${quoted_command}"
