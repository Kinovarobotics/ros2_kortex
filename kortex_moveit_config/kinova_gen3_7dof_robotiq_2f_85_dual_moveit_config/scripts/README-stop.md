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

### 2. `scripts/dual_arm_stop.py`
Cancels every in-flight goal, closest-to-hardware first: the two
`follow_joint_trajectory` servers (which are what actually drive the arms), the
grippers, then `execute_trajectory` and `move_action`. The
`JointTrajectoryController` halts and holds position on cancel.

```bash
./scripts/dual_arm_stop.py                # cancel everything, arms hold
./scripts/dual_arm_stop.py --deactivate   # also deactivate the arm controllers
```

Keep a terminal open with this command typed and ready before any run.

`--deactivate` is the heavier hammer: it stops the controllers writing commands
at all. The arms will need reactivating before they can move again:

```bash
ros2 control switch_controllers --activate left_arm_controller right_arm_controller
```

### 3. RViz "Stop" button
The MotionPlanning panel has one. Same mechanism as option 2 (cancels via
`move_group`), but it only reaches goals MoveIt dispatched, and it needs you to
find and click it.

### 4. Ctrl-C in `dual_arm_demo.py`
Now cancels the active goal before exiting. **This only works for motion that
script started** — and only while it is still running.

## What does NOT stop the arms

- **Killing `move_group`.** The trajectory controller already holds the entire
  trajectory and will keep executing it to the end. Killing the planner does
  nothing.
- **Ctrl-C in any script that does not explicitly cancel.** The goal lives on
  the action server, not in the client.
- **Closing RViz.**

## Untested

Cancel-all has been exercised against an idle system (clean no-op on all six
servers). Cancelling *during motion*, and `--deactivate`, have not been tested
on moving hardware — do that deliberately, at low speed, with a hand on the
physical e-stop, before relying on either.
