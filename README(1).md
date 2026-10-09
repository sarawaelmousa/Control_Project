# Bicycle Gym: Autonomous Racetrack Control (ROS 2)

## 1. Student Information

| | |
|---|---|
| Name | SARA WAEL MOUSA |
| Course |Control |



## 2. Overview

A simulated self-driving car (extended kinematic bicycle model) drives around a closed racetrack in ROS 2 (Jazzy). Velocity is a **state**, not an input: the car accelerates through throttle and is slowed by aerodynamic drag and rolling resistance. The project builds the full stack: vehicle dynamics, teleoperation, speed control, a curvature-based velocity profiler, three swappable steering controllers (Lateral PID, Pure Pursuit, MPC) and a lap analyzer used to benchmark them.

## 3. System Architecture

### Packages

| Package | Responsibility |
|---|---|
| `bicycle_sim` | Vehicle model (`bicycle_model.py`), simulator node, URDF/Xacro, RViz config, launch file |
| `bicycle_control` | `teleop_bridge.py`, `longitudinal_pid.py`, `velocity_profiler.py`, `lateral_pid.py`, `pure_pursuit.py`, `mpc.py`, `controller_node.py` |
| `track_environment` | CSV track loading, `/path` publisher, boundary cones, `lap_analyzer.py` |

### ROS graph

```
/path_gen ──> /path ──────────────┬──> /controller ──> /throttle, /steer ──> /kinematic_bicycle
         └──> /track_bounds       │                                              │
                                  └──> /lap_analyzer <──────── /state <──────────┘
                                          │
                                          └──> /telemetry/*, /lap/metrics, /lap/visualization
/kinematic_bicycle ──> /joint_states ──> /robot_state_publisher ──> /tf, /robot_description
```

Every controller reads `/path` and `/state` and writes `/throttle` and `/steer`, which is why the three lateral controllers can be swapped without changing anything else.

### Topics

| Topic | Type | Units | Publisher | Subscriber |
|---|---|---|---|---|
| `/state` | `nav_msgs/Odometry` | position m, yaw as quaternion, `twist.linear.x` = v (m/s) | `/kinematic_bicycle` | controller, lap analyzer |
| `/throttle` | `std_msgs/Float32` | normalised, [-1, 1] | teleop bridge / controller | `/kinematic_bicycle` |
| `/steer` | `std_msgs/Float32` | rad, positive = left | teleop bridge / controller | `/kinematic_bicycle` |
| `/path` | `nav_msgs/Path` | m | `/path_gen` | controller, lap analyzer |
| `/track_bounds` | `visualization_msgs/MarkerArray` | m | `/path_gen` | RViz |
| `/telemetry/cte` | `std_msgs/Float32` | m (positive = left of path) | lap analyzer | plotting tools |
| `/telemetry/heading_err_deg` | `std_msgs/Float32` | degrees | lap analyzer | plotting tools |
| `/telemetry/speed` | `std_msgs/Float32` | m/s | lap analyzer | plotting tools |
| `/telemetry/lap_time` | `std_msgs/Float32` | s | lap analyzer | plotting tools |
| `/lap/metrics` | `std_msgs/String` | JSON summary | lap analyzer | user |
| `/lap/visualization` | `visualization_msgs/MarkerArray` | m | lap analyzer | RViz |

### Vehicle parameters

| Parameter | Value |
|---|---|
| Wheelbase L | 1.25 m |
| Track width | 1.18 m |
| Wheel radius / width | 0.5 m / 0.3 m |
| Max steering | 35° (0.6109 rad) |
| Powertrain gain `k_a` | 4.0 m/s² |
| Drag `c_drag` | 0.005 (v²) |
| Rolling resistance `c_roll` | 0.05 (v) |
| Max speed | 25 m/s |
| Simulation step | dt = 0.1 s (10 Hz) |

Track: `centerline_0.csv`, 1,001 waypoints (1,000 plus the first point appended to close the loop), perimeter 528.2 m.

## 4. Mathematical Formulations

### 4.1 Extended kinematic bicycle model (Milestone 2)

State `[x, y, θ, v]` (rear-axle position, yaw, speed), inputs throttle `u ∈ [-1, 1]` and steering `δ`:

```
ẋ = v·cos θ
ẏ = v·sin θ
θ̇ = (v / L)·tan δ
v̇ = k_a·u − c_drag·v² − c_roll·v
```

Forward Euler, `x_{k+1} = x_k + ẋ_k·dt`, with all derivatives computed from the old state. After each step the heading is wrapped to [-π, π] with `atan2(sin θ, cos θ)` and the speed is clamped to `[0, v_max]` (braking does not reverse the car).

Terminal speed (`v̇ = 0`): throttle 0.5 gives about 15.6 m/s, throttle 1.0 gives about 23.7 m/s. A constant steering angle δ gives a circle of radius `R = L / tan δ`.

### 4.2 Teleoperation bridge (Milestone 3)

Open-loop mapping from `geometry_msgs/Twist`:

```
throttle = clip(linear.x / v_max_cmd, -1, 1)              v_max_cmd = 5.0 m/s
δ        = clip((angular.z / ω_max)·δ_max, -δ_max, δ_max)  ω_max = 1.0 rad/s
```

A watchdog zeroes both commands if no `/cmd_vel` arrives for 0.5 s. Open-loop speed is inaccurate: a request of 0.5 m/s gives throttle 0.1, whose terminal speed is about 5.2 m/s. This motivates the speed PID.

### 4.3 Longitudinal PID (Milestone 4)

```
e = v_target − v
u = Kp·e + Ki·∫e dt − Kd·dv/dt          (clipped to [-1, 1])
```

- Derivative on the **measurement** (no derivative kick when the target jumps).
- Anti-windup in two layers: the integral is clamped to ±`integral_limit`, and integration is paused while the output is saturated and the error pushes further into saturation.
- Gains: Kp = 1.0, Ki = 0.2, Kd = 0.05, dt = 0.1.

### 4.4 Velocity profiler (Milestone 5.1)

Lateral acceleration `a_lat = v²·κ`, so the cornering speed limit is

```
v_target = clip( sqrt(a_lat_max / |κ|), v_min, v_max )
```

with `a_lat_max = 5 m/s²`, `v_min = 1 m/s`, `v_max = 7.5 m/s`. Curvature is estimated from the change of path yaw between the neighbouring waypoints divided by their distance. A straight line (κ ≈ 0) returns `v_max`.

### 4.5 Lateral PID (Milestone 5.2)

Sign conventions: `cte > 0` means the car is **left** of the path, `heading_err = ψ_vehicle − ψ_path`. Both errors require a right (negative) correction, so

```
δ = −( Kp·cte + Ki·∫cte dt + Kd·d(cte)/dt ) − K_yaw·heading_err
```

clipped to ±35°, with the same two-layer anti-windup and the heading error wrapped to [-π, π].

Linearised on a straight road, the closed loop is second order with `ω_n = v·sqrt(Kp/L)` and `ζ = (Kd·v + K_yaw) / (2·sqrt(Kp·L))`. Both the bandwidth and the damping depend on speed, so gains that are well damped at low speed can be too fast for the 10 Hz loop at high speed.

TODO: final gains used for the benchmark (defaults in the code: Kp = 0.8, Ki = 0.02, Kd = 0.15, K_yaw = 0.5).

### 4.6 Pure Pursuit (Milestone 5.3)

Adaptive look-ahead `Ld = clip(k_v·v + l_min, l_min, l_max)` with `k_v = 0.25`, `l_min = 0.8 m`, `l_max = 2.5 m`. The target is the first waypoint at least `Ld` away, searched forward from the nearest waypoint (wrapping around the loop). In the car frame:

```
x_l =  cos θ·Δx + sin θ·Δy
y_l = −sin θ·Δx + cos θ·Δy
α   = atan2(y_l, x_l)
δ   = atan2( 2·L·sin α, Ld )          (clipped to ±35°)
```

`Ld` is the actual distance to the chosen waypoint. A target on the left gives `α > 0` and `δ > 0`, so no extra sign flip is needed.

### 4.7 Extended kinematic MPC (Milestone 5.4)

Decision variables over a horizon of N = 10 steps (1 s): `[δ_0, a_0, …, δ_{N-1}, a_{N-1}]`, with `|δ| ≤ 35°` and `|a| ≤ k_a`. The prediction model is the simulator model, including drag and rolling resistance. For each step the error is projected into the Frenet frame of reference point k:

```
e_lat  = −sin ψ_ref·(x − x_ref) + cos ψ_ref·(y − y_ref)
e_long =  cos ψ_ref·(x − x_ref) + sin ψ_ref·(y − y_ref)
e_yaw  = wrap(θ − ψ_ref),   e_v = v − v_ref
```

```
J = Σ_k  w_lat·e_lat² + w_long·e_long² + w_yaw·e_yaw² + w_v·e_v²
        + w_steer·δ_k² + w_dsteer·(δ_k − δ_{k−1})² + w_accel·a_k²
```

Weights: `w_lat = 30`, `w_long = 1`, `w_yaw = 10`, `w_v = 1`, `w_steer = 0.2`, `w_dsteer = 6`, `w_accel = 0.1` (TODO: update if you retuned). The problem is solved with SciPy SLSQP (`maxiter = 25`, `ftol = 1e-3`). **Receding horizon:** only `δ_0` and `a_0/k_a` (the throttle) are applied, then the problem is re-solved at the next tick. **Warm start:** the previous solution is shifted forward by one step (last step repeated) and used as the initial guess.

### 4.8 Lap analyzer (Milestone 5.5)

CTE is the signed distance from the rear axle to the nearest path segment (positive = left, same convention as the controllers). A lap is counted when the arc-length progress `s` wraps from above 75% to below 25% of the perimeter while the car is moving, with the crossing time interpolated between two state messages. Per lap it reports mean |CTE|, max |CTE|, RMS CTE, mean and max speed. Live signals go to `/telemetry/*`, a JSON summary to `/lap/metrics`, and RViz shows a CTE whisker (green to red) and a HUD with lap, time, speed, CTE and best lap.

## 5. Benchmark

All controllers use the same track, the same speed profiler and the same longitudinal PID, started with a fresh launch for every run.


| Controller | Best lap (s) | Top speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps |
|---|---|---|---|---|---|---|
| Lateral PID (initial gains) | 77.28| 7.68 | 0.39| 2.878 | 0.55| 1 |
| Pure Pursuit | 69.5| 7.74 | 0.065 | 0.350 | 0.090 | 1 |
| MPC (constant 4 m/s reference) | 88.4 | 7.05 | 0.064 | 0.360 | 0.089 | 1 |

Notes on these first runs:

- The Lateral PID drove about 493 m for a lap, against about 447 m for the other two, which is consistent with weaving around the path.
- The MPC's lap time is longer only because its reference speed was a constant 4 m/s (mean speed 4.90 m/s against 6.32 m/s for Pure Pursuit). Its tracking error matches Pure Pursuit. TODO: rerun with profiler speeds in the MPC reference and update the row.
- TODO: rerun the Lateral PID after retuning and update the row.
- TODO: check that every counted lap is a full lap (reported distance should be close to the 528.2 m perimeter).

## 6. Critical Comparison

| | Lateral PID | Pure Pursuit | MPC |
|---|---|---|---|
| Preview of the road | none | yes (look-ahead point) | yes (1 s horizon) |
| Uses a vehicle model | no | geometry only | yes, including speed dynamics |
| Handles actuator limits | clipping only | clipping only | built into the optimisation |
| Tuning | 4 coupled gains, speed dependent | mainly `Ld` | 7 cost weights, horizon, solver limits |
| Computation | trivial | trivial | heaviest (SLSQP every 100 ms) |
| Behaviour observed | weaves, large errors in corners | smooth, small steering | accurate, but slower with the constant reference |

- **Lateral PID** reacts only to the error that already exists. In a steady corner it needs a standing offset to produce the required steering, and its stability depends on speed. This was the worst tracker in the benchmark.
- **Pure Pursuit** is cheap and smooth. Its look-ahead point acts as a low-pass filter, so it steers gently. The cost is corner cutting when `Ld` is long relative to the corner radius.
- **MPC** reached the lowest or equal tracking error but is the most complex to tune and the most expensive. Its result depends on the quality of its reference (path yaw, speed profile) and on the solver converging within the iteration limit.

On this track the measured tracking error of MPC and Pure Pursuit is almost identical (about 0.09 m RMS). TODO: update this paragraph with the final 3-lap numbers and say which controller actually won.

## 7. Why MPC Should Track Better Than Pure Pursuit and Lateral PID

1. **Preview with a model.** The Lateral PID sees only the current error. Pure Pursuit sees one point ahead but assumes a geometric arc and ignores the car's dynamics. MPC predicts the trajectory it will really drive for the next second and chooses the inputs that minimise the error along the whole horizon, so it starts steering before the corner.
2. **Error is penalised along the path, not at one point.** Pure Pursuit aims at a single target, which produces corner cutting. MPC's cost penalises cross-track error at every predicted step, so it takes the curved path into account directly.
3. **Constraints are part of the problem.** Steering limits and acceleration limits are bounds in the optimisation, so the controller plans with them. The PID and Pure Pursuit compute an unconstrained command and clip it afterwards, so they can ask for something the car cannot do and discover the problem too late.
4. **Speed and steering are coupled.** The model includes `θ̇ = (v/L)·tan δ` and the powertrain, so MPC can trade speed against steering in a corner. The other two controllers handle speed separately and ignore this coupling.
5. **Smoothness is explicit.** The steering-rate term `w_dsteer·(δ_k − δ_{k−1})²` makes smooth steering part of the objective instead of a side effect of low gains.

These advantages depend on a good model, a good reference and a solver that converges in time. With a short 1 s horizon and an iteration-limited SLSQP, the practical gain over a well-tuned Pure Pursuit is modest on a smooth track like this one, which matches the benchmark above. TODO: adjust this paragraph if your final results differ.

## 8. Reproduction Guide

### Requirements

Ubuntu 24.04, ROS 2 Jazzy, `python3-scipy`, `python3-numpy`, `rviz2`, `ros-jazzy-teleop-twist-keyboard`, optionally `ros-jazzy-plotjuggler-ros` and `ros-jazzy-rqt-plot`.


### Run an autonomous controller

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid analyzer:=true
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit analyzer:=true
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=mpc analyzer:=true
```

Restart the launch between controllers, because the lap analyzer accumulates statistics for the whole run.

### Read the results

```bash
ros2 topic echo /lap/metrics --field data
ros2 topic echo /telemetry/cte
ros2 run plotjuggler plotjuggler          # ROS 2 Topic Subscriber, select /telemetry/* and /steer
ros2 run rqt_plot rqt_plot /telemetry/speed/data /telemetry/cte/data
```

### Manual driving (Milestones 3 and 4)

```bash
ros2 run bicycle_control teleop_bridge                                   # open loop
ros2 run bicycle_control teleop_bridge --ros-args -p use_cruise_control:=true   # speed PID
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Test the watchdog by sending a single command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 3.0}, angular: {z: 0.5}}"
ros2 topic echo /throttle      # drops to 0.0 after about 0.5 s
```

## 9. Source Files

`bicycle_model.py`, `teleop_bridge.py`, `longitudinal_pid.py`, `velocity_profiler.py`, `lateral_pid.py`, `pure_pursuit.py`, `mpc.py`, `controller_node.py`, `lap_analyzer.py`.
