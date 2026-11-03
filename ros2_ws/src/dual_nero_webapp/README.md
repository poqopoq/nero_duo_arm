# dual_nero_webapp

Browser dashboard for the table-mounted dual NERO workspace. It provides:

- a live 3D view of the same dual-arm xacro used by ROS;
- seven joint sliders per arm with feedback from the drivers;
- one-click **Both Send** and **Both Park** using a synchronized `both_arms`
  MoveIt trajectory;
- per-arm and combined **Park** targets loaded from the installed support-pose YAML;
- webapp-side gate, motor-enable, and emergency-stop controls;
- per-arm arm/motor health plus **Clear faults** and guarded **Recover arm**;
- scene, left-wrist, and right-wrist camera tiles with color/depth switching;
- a compact synchronized dual-arm toolbar, independently scrollable desktop
  controls, and a responsive camera gallery that preserves the complete image
  and lets the page scroll when the browser viewport is too small.

Every **Send** and **Park** request goes to MoveIt's `/move_action` action.
MoveIt plans for `left_arm`, `right_arm`, or `both_arms`, checks the full
dual-arm robot and table collision model, and executes through the single
`dual_arm_controller`. A both-arm request is one 14-joint goal, so both arms
follow one synchronized trajectory rather than racing two independent requests.
The app does not bypass MoveIt with direct joint targets.

The collision model is currently static: URDF links, table, and configured
planning-scene geometry. Camera point clouds and OctoMap dynamic obstacles are
not connected, so the workspace must still be kept clear.

The complete Chinese operating manual, button reference, and HTTP/ROS command
examples are in `deployment/docs/12_usage_and_commands.md`.

## Run

Start the existing driver and camera stacks separately:

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_webapp webapp.launch.py
```

Or start everything together with safe defaults:

```bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_cameras:=true start_moveit:=true
```

Open `http://<robot-host>:8080`. The default bind address is `0.0.0.0`; use
`http_host:=127.0.0.1` when only local access is wanted.

`start_moveit:=true` starts `move_group` and the trajectory bridge.
`moveit_dry_run` defaults to `true`, which simulates bridge execution without
commanding the drivers. Use the following only after the real-hardware checks
have passed:

```bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_cameras:=true \
  start_moveit:=true moveit_dry_run:=false
```

If MoveIt is not running, the dashboard reports `MoveIt: unavailable` and
rejects motion requests. It never falls back to unchecked direct commands.
The status badge distinguishes `MoveIt: ready (DRY-RUN)` from
`MoveIt: ready (LIVE)`, and every dry-run Send/Park result explicitly reports
that no driver command was sent.

## Safety interlocks

Both the webapp and the driver enforce control state:

1. The driver starts with `control_enabled=false`.
2. The webapp starts with its own gate closed.
3. Open the webapp gate with **Open gate**. This calls
   `/<side>/control_enable` and opens the driver gate.
4. Send commands only after the intended arm is enabled and the workspace is
   clear.
5. **E-stop** dispatches `/<side>/emergency_stop` first, then closes that
   side's gate and cancels the active trajectory/MoveIt goal. **Stop both**
   sends both hardware stops in parallel.

**Park** uses the support pose installed at
`share/dual_nero_webapp/config/support_pose.yaml`. It is available per arm
only when that arm's webapp gate is open, its motors are marked enabled, its
feedback is fresh (less than 1.5 seconds old), and the configured pose passes
the same joint-limit validation as manual commands. The browser asks for
confirmation before sending it. **Both Park** requires both per-arm poses and
the same readiness checks for both arms.

Park sends the configured support pose through MoveIt. The app waits for the
MoveIt result, so a successful response means MoveIt reported successful
planning and execution. It is collision-checked against the static robot/table
model, not against live camera data.

Each arm shows a health pill derived from the driver's `arm_status` and
`motor_status` diagnostics. When motors are disabled, `arm_status=6`
(`brake not released`) is displayed as `brakes set`; it is not by itself a
left/right-specific fault. If it appears while motors are enabled, the UI
shows `brake stuck`.

**Clear faults** sends the NERO joint-driver fault-clear command. It does not
reset the motion controller, release brakes, enable motors, or move the arm.
**Recover arm** is intentionally guarded: the control gate must be closed and
feedback fresh, and the browser asks for confirmation that the arm is
physically supported. It then resets the motion controller, clears joint
faults, and re-enables motors. It does not send the arm to Park.

Joint positions are clamped server-side to the conservative intersection of
the NERO driver and URDF limits before the MoveIt goal is built. The webapp
gate state is local to this process; if another tool changes
`/<side>/control_enable`, restart the webapp or use the gate button to
synchronize it. After a non-dry-run trajectory, the bridge closes the driver
gate but the webapp may still display its local gate as open until it is
toggled or restarted.

For cancellation, the webapp calls
`/dual_nero_trajectory_bridge/cancel_motion`, then requests cancellation of the
MoveIt goal. This is required because a MoveGroup cancel request alone does not
reliably interrupt the controller trajectory that is already executing. The
bridge latches the cancellation state, and the webapp calls
`/dual_nero_trajectory_bridge/clear_cancel` before submitting every new motion.
The top-bar **Cancel motion** button uses the same path without closing the
control gate or disabling the motors.

## Camera topics

The app subscribes to ROS topics, so it does not open the RealSense devices a
second time. Defaults match `dual_nero_cameras`:

```text
/camera_scene/color/image_raw
/camera_scene/aligned_depth_to_color/image_raw
/camera_left_wrist/color/image_raw
/camera_left_wrist/aligned_depth_to_color/image_raw
/camera_right_wrist/color/image_raw
/camera_right_wrist/aligned_depth_to_color/image_raw
```

Override any topic with a ROS parameter (for example
`scene_color_topic:=/my_camera/color/image_raw`) or set
`enable_cameras:=false` for an arm-only dashboard.

## HTTP endpoints

| Endpoint | Purpose |
| --- | --- |
| `/` | Dashboard |
| `/ws` | Joint commands and feedback |
| `/api/config` | Joint names, limits, and camera metadata |
| `/api/state` | Current feedback and control state |
| `/api/motion` | MoveIt availability and busy/active side |
| `/api/control_gate` | Open or close one driver control gate |
| `/api/enable` | Enable or disable one arm's motors |
| `/api/clear_faults` | Send the NERO joint fault-clear command for one arm |
| `/api/recover` | Reset motion state, clear faults, and re-enable one arm |
| `/api/move` | Plan and execute one arm or both arms through MoveIt |
| `/api/cancel_motion` | Request cancellation of the active MoveIt motion |
| `/api/cameras` | Per-stream frame statistics |
| `/api/park` | Plan and execute one or both configured support poses |
| `/api/emergency_stop` | Stop one arm and close its gate |
| `/api/emergency_stop_all` | Stop both arms in parallel |
| `/stream/<camera>/<mode>` | MJPEG stream (`color` or `depth`) |
| `/robot_urdf` | Xacro-expanded URDF for the browser 3D view |

The viewer loads Three.js and `urdf-loader` from CDN URLs in `static/index.html`.
Joint and safety controls still work if the CDN is unavailable, but the 3D view
will not load. The dashboard has no authentication; keep it on a trusted LAN.

## Not implemented yet

- camera point-cloud/OctoMap dynamic collision objects and scene editing;
- camera TF integration into MoveIt's `world` planning frame;
- trajectory preview, collision pre-check UI, and browser trajectory display;
- Cartesian, IK, and named-pose targets;
- gripper planning groups and gripper controls;
- velocity/acceleration controls in the UI (currently fixed at `0.05`);
- full hardware diagnostics, execution progress, authentication, and TLS.
