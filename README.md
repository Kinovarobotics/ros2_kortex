# gen3_optimization_ws: 1 kHz ROS 2 control of a Kinova Gen3

This workspace runs the Kinova Gen3 7dof (with a Robotiq 2F-85) through `ros2_kortex` + MoveIt and
reaches about **1 kHz** on the ros2_control loop, and on the PC↔arm link. This page lists every setting
that was in effect when it reached that rate, and the PC it ran on.

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.1.10
```

Result on 2026-10-06:

| Condition | Measured on `enp129s0` |
|---|---|
| Camera view open | about 1104 tx packets/s, which includes some non-cyclic traffic |
| Camera view closed | about 990 Hz |

The earlier notes in `ros2_gen3_ws` recorded only 857 Hz for the same YAML change; the newer
ros2_control stack below is the main difference.

---

## 1. PC specifications

| Item | Value |
|---|---|
| Machine | Lenovo Legion Pro 7 16IAX10H (product 83F5) |
| CPU | Intel Core Ultra 9 275HX: 24 cores (8 P-cores at 5.3–5.4 GHz on CPU 0–7, 16 E-cores at 4.7 GHz on CPU 8–23), 1 thread per core, single NUMA node |
| RAM | 62 GiB |
| OS | Ubuntu 24.04.3 LTS |
| Kernel | `7.0.0-31-realtime` (PREEMPT_RT) |
| NIC to the arm | `enp129s0`: Intel I226-V (`igc` driver, firmware 2032:88b6), 1000 Mb/s full duplex, direct cable |
| PC address / arm address | `192.168.1.11/24` / `192.168.1.10` |
| User groups | `aarp` is in the `realtime` group |

## 2. Software versions

| Component | Version / source |
|---|---|
| ROS 2 | Jazzy (`/opt/ros/jazzy`) |
| ros2_control (`controller_manager`, `hardware_interface`) | **4.49.0**, `src/ros2_control` (jazzy branch, cc6bbd66) |
| ros2_controllers (incl. `joint_trajectory_controller`) | **4.43.0**, `src/ros2_controllers` (jazzy, 46358426) |
| realtime_tools | **3.13.0**, `src/realtime_tools` (jazzy, 3c9ce67). Needed for `trySetFeedback()` |
| control_msgs, control_toolbox, ros2_control_cmake, gz_ros2_control | from source, jazzy branches |
| generate_parameter_library | from source, main |
| ros2_kortex / kortex_driver | jazzy (462dab9), kortex_driver 0.2.6 |
| Kortex API (C++) | 2.8.0 (`linux_x86-64_gcc_5.4.zip`, fetched by `kortex_api`) |
| Arm firmware (7dof) | *(record from Web App)* |

## 3. Build

```bash
cd ~/workspace/gen3_optimization_ws
source /opt/ros/jazzy/setup.bash
# One time only, after realtime_tools was added to src/:
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-clean-cache --allow-overriding realtime_tools
# After that:
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`realtime_tools` from source overrides the `/opt/ros/jazzy` copy. Without it the from-source
ros2_controllers fail to build (`trySetFeedback` missing).

## 4. Controller manager settings (the change that reached 1 kHz)

This block was added under `controller_manager: ros__parameters:` in all six `ros2_controllers.yaml`
files (copied from `ros2_gen3_ws`):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz
    cpu_affinity: 2        # pin RT update loop to isolated core (isolcpus=2,3,6,7)
    lock_memory: true      # mlockall to avoid page-fault jitter
    thread_priority: 80    # SCHED_FIFO priority (under ulimit -r 98 ceiling)
    hardware_synchronization:
      expect_blocking_read_write: true
      minimum_cycle_time: 0.0001
```

Files:

```
src/ros2_kortex/kortex_description/arms/gen3/7dof/config/ros2_controllers.yaml
src/ros2_kortex/kortex_description/arms/gen3/6dof/config/ros2_controllers.yaml
src/ros2_kortex/kortex_description/arms/gen3_lite/6dof/config/ros2_controllers.yaml
src/ros2_kortex/kortex_moveit_config/kinova_gen3_7dof_robotiq_2f_85_moveit_config/config/ros2_controllers.yaml
src/ros2_kortex/kortex_moveit_config/kinova_gen3_6dof_robotiq_2f_85_moveit_config/config/ros2_controllers.yaml
src/ros2_kortex/kortex_moveit_config/kinova_gen3_lite_moveit_config/config/ros2_controllers.yaml
```

Why it works: the Gen3's blocking `Refresh()` round trip takes about 1–1.3 ms, so with the default
fixed-period policy the loop overran and alternated 1 ms / 2 ms periods (about 500–550 Hz). With
`expect_blocking_read_write` the loop is paced by the arm's reply instead. It free-runs and locks onto
the arm's own 1 kHz tick.

`cpu_affinity`, `lock_memory` and `thread_priority` only take effect if the host settings in section 5
are in place. Without them the node starts, but logs that it could not set priority or lock memory.

No other changes were needed: no async hardware interface, no xacro changes, no driver code changes.

## 5. Host real-time settings

These were installed on 2026-09-08 from `~/workspace/ros2_gen3_ws/rt-tune/` (see its README) and apply
automatically at boot. **No rt-tune script was run by hand for the 1 kHz result.**

### 5.1 Kernel command line (GRUB)

```
isolcpus=managed_irq,domain,2,3,6,7 nohz_full=2,3,6,7 rcu_nocbs=2,3,6,7 rcu_nocb_poll
irqaffinity=0,1,4,5,8-23 skew_tick=1 nosoftlockup nmi_watchdog=0 audit=0
split_lock_detect=off tsc=nowatchdog
```

The `isolcpus` and `nohz_full` flags reserve P-cores 2, 3, 6 and 7 for real-time work, and the
ros2_control loop runs on core 2. All other work and interrupts go to the housekeeping cores
0, 1, 4, 5 and 8–23.

### 5.2 `/etc/security/limits.d/99-realtime.conf`

```
@realtime   -   rtprio      98
@realtime   -   memlock     unlimited
@realtime   -   nice        -20
```

This lets members of `realtime` use SCHED_FIFO 80 and `mlockall`. It needs a fresh login to take effect.

### 5.3 `/etc/sysctl.d/99-realtime.conf`

```
kernel.sched_rt_runtime_us = -1     # no RT throttling (avoids 50 ms dropouts)
kernel.timer_migration = 0
vm.stat_interval = 120
kernel.numa_balancing = 0
vm.dirty_ratio = 10
vm.dirty_background_ratio = 3
net.core.rmem_max = 4194304
net.core.wmem_max = 4194304
```

### 5.4 `rt-runtime-tune.service` (enabled, oneshot at boot)

This service runs `/usr/local/sbin/rt-runtime-tune apply`. Its settings come from
`/etc/default/rt-tuning`:

```
RT_CORES="2,3,6,7"   HK_CORES="0,1,4,5,8-23"   RT_LOOP_CORE="2"
RT_IRQ_CORE="3"      RT_NIC="enp129s0"          RT_IRQ_PRIO="85"   RT_BUSY_POLL="0"
```

What it does:

- **Idle states:** disables C-states with more than 10 µs exit latency on cores 2, 3, 6, 7. Only POLL
  and C1 remain.
- **CPU governor:** sets `performance` on cores 2, 3, 6, 7.
- **Interrupts:** moves all movable IRQs to the housekeeping cores.
- **NIC interrupts:** points the `enp129s0` IRQs at core 3 and runs their threads at SCHED_FIFO 85.
- **NIC latency settings:** interrupt coalescing off (`rx-usecs 0`), GRO/LRO/TSO/GSO off, pause frames
  off, busy-polling off.
- **Kernel work queues:** restricts unbound workqueues to the housekeeping cores.

Check what's in effect with `sudo rt-runtime-tune status`.

**Live state observed on 2026-10-08** (worth knowing if you try to reproduce this):

| Setting | Intended | Observed |
|---|---|---|
| NIC IRQ threads | on core 3 | SCHED_FIFO 85 as intended, but IRQ affinity is the housekeeping set (`0-1,4-5,8-23`) and the threads run on cores 16–20 |
| Interrupt coalescing | `rx-usecs 0` | `rx-usecs 3` / `tx-usecs 3`. Probably reset when the NIC re-initialised |
| GRO / TSO / GSO | off | off, as intended |

So 1 kHz does not depend on the NIC tweaks being exactly as intended.

### 5.5 Optional per-launch script (not used for the 1 kHz result)

`sudo ~/workspace/ros2_gen3_ws/rt-tune/rt-fix-userspace.sh`, run after the driver has started, does
two things:

- raises the minimum clock of cores 2, 3, 6, 7 to their maximum (5.3 GHz)
- moves the Kortex transport threads to cores 6–7 at SCHED_FIFO 85

It was run later on 2026-10-06 while investigating faults, which made no difference to them. The clock
floor stays until the next reboot.

Check the controller thread:

```bash
ps -L -o tid,cls,rtprio,psr,comm -p $(pgrep -f ros2_control_node | head -1) | awk 'NR==1 || $2=="FF"'
```

## 6. Measuring the rate

```bash
# Packets/s the PC sends to the arm (about 1 per control cycle, plus a little other traffic)
a=$(cat /sys/class/net/enp129s0/statistics/tx_packets); sleep 10
b=$(cat /sys/class/net/enp129s0/statistics/tx_packets); echo $(( (b-a)/10 )) pkt/s
```

## 7. Does this carry over to another PC?

| Part | Carries over? |
|---|---|
| YAML (section 4) and the source-built stack (sections 2–3) | Yes, on any PC |
| `cpu_affinity: 2` | Only if core 2 is a fast, isolated core on that machine. Adjust per machine |
| Host settings (section 5) | No. Needs a PREEMPT_RT kernel, the `realtime` group, and isolated cores |
| NIC name (`enp129s0`) and core lists | Specific to this PC. Need to be redone per machine |

Without the host settings, expect a lower and less steady rate.

## If the rate drops to about 500 Hz

Check that the `hardware_synchronization` block (section 4) is active and correctly indented in the
config you launch. For the 2026-10-06 fault tests it was commented out in the 7dof MoveIt config; it
was restored on 2026-10-08. After any edit, rebuild and check the YAML:

```bash
colcon build --packages-select kinova_gen3_7dof_robotiq_2f_85_moveit_config
python3 -c "import yaml,sys; yaml.safe_load(open(sys.argv[1]))" \
  src/ros2_kortex/kortex_moveit_config/kinova_gen3_7dof_robotiq_2f_85_moveit_config/config/ros2_controllers.yaml
```

## Known issue: joint faults in low-level servoing

The arm sometimes faults during low-level control. This is **not caused by the 1 kHz settings**:

- It happens at about 500 Hz too.
- It happens without ROS, with `scripts/gen3_lowlevel_loop` using the Kortex API only.
- It happened on a second arm, a Gen3 6dof.

The cause: now and then the arm returns feedback where some joints' positions are 2–6 s old. That
reading triggers following-error or current-mismatch faults. High-level control
(`scripts/gen3_highlevel_loop`) runs clean. The case is with Kinova.

Test and diagnostic tools are in `scripts/`:

| Tool | What it does |
|---|---|
| `gen3_home_zero_loop.py` | MoveIt home↔zero loop |
| `gen3_lowlevel_loop` and `gen3_lowlevel_loop_6dof` | Kortex low-level home↔zero loop, about 1 kHz |
| `gen3_highlevel_loop` | Kortex high-level home↔zero loop |
| `gen3_faults` | Read-only: arm state and fault/warning bits per joint |
| `gen3_release` | Lists connected clients; with `--release`, switches the arm back to single-level servoing |
| `gen3_fwinfo` | Read-only: firmware versions |
| `gen3_limits` | Read-only: joint safety limits |
