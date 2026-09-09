# Stopping the arms

## Read this first

**There is no safety-rated emergency stop in this software stack.** Everything
below is best-effort software on a non-deterministic path. None of it helps if
`move_group`, `controller_manager`, the Kortex driver, or the network link to
an arm is the thing that has wedged.

**For a real emergency, use the physical e-stop / power cutoff.** The Gen3 has
no e-stop button on the arm itself, so this is whatever external mushroom
button or contactor is wired on the bench.

## What the driver does and does not give you

`kortex_driver` does call `base_.ApplyEmergencyStop()`
(`kortex_driver/src/hardware_interface.cpp:898`), but **only inside the fault
*reset* routine** — it stops, clears faults, then restores the servoing mode.
That is a recovery path, not a stop, and it is gated on
`fault_controller_running_`.

On this system the fault controllers are **not loaded** at all. `robot.launch.py`
spawns `left_fault_controller` / `right_fault_controller`, and the
`picknik_reset_fault_controller` plugin is registered, but
`ros2 control list_controllers` shows only five active controllers. So that code
path is unreachable as things currently stand.

There is no `/emergency_stop` service, and no stop service of any kind.

## Ranked options

### 1. Physical e-stop
The only real one. Everything else is software.

### 2. RViz "Stop" button
The MotionPlanning panel has one. It cancels via `move_group`, so it only
reaches goals MoveIt dispatched, and it needs you to find and click it.

### 3. Ctrl-C in `dual_arm_demo.py`
Cancels the active goal before exiting. **This only works for motion that
script started** — and only while it is still running. It is also the way to
stop the script's default infinite home <-> demo loop.

### 4. Deactivate the arm controllers
Blunt, and there is no script for it — stops the controllers writing commands
at all. The arms need reactivating afterwards:

```bash
ros2 control switch_controllers --deactivate left_arm_controller right_arm_controller
ros2 control switch_controllers --activate   left_arm_controller right_arm_controller
```

## What does NOT stop the arms

- **Killing `move_group`.** The trajectory controller already holds the entire
  trajectory and will keep executing it to the end. Killing the planner does
  nothing.
- **Ctrl-C in any script that does not explicitly cancel.** The goal lives on
  the action server, not in the client.
- **Closing RViz.**

## Untested

Cancelling *during motion*, and deactivating the controllers, have not been
tested on moving hardware — do that deliberately, at low speed, with a hand on
the physical e-stop, before relying on either.

## Removed

There used to be a `scripts/dual_arm_stop.py` that cancelled every in-flight
goal closest-to-hardware first (both `follow_joint_trajectory` servers, then the
grippers, then `execute_trajectory` and `move_action`). It was deleted on
2026-09-09. Options 2-4 above are what is left.
