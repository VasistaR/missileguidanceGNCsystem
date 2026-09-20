# Missile GNC System (MATLAB / Simulink)

A guidance, navigation, and control (GNC) simulation built in MATLAB and Simulink. The project takes a linearized longitudinal airframe model, designs an LQG controller for it (LQR state feedback plus a Kalman filter state estimator), and closes a guidance loop around it that flies a point-mass vehicle from a start location toward a target location while watching for an obstacle zone along the way.

This is an educational controls project. Everything here is simulation: a two-state linear model, constant speed, and planar kinematics. It is intended for learning and demonstrating state-space design, optimal control, and state estimation.

## What it demonstrates

- State-space modelling of short-period pitch dynamics (angle of attack and pitch rate, with normal acceleration `Az` and pitch rate `q` as outputs)
- LQR design with `lqr`, including open-loop vs. closed-loop pole comparison
- Kalman filter design with `kalman`, giving an observer that estimates the states from noisy measurements
- LQG integration in Simulink: plant and observer built from `A`, `B`, `C`, `D`, `K`, and `L` matrices, with band-limited white noise injected as process and measurement noise
- A guidance layer that computes range and a flight path angle command from latitude, longitude, and altitude using the haversine formula
- An outer PID loop that turns flight path angle error into a rate-limited pitch rate command for the inner LQG loop

## Repository contents

| File | Purpose |
| --- | --- |
| `model.m` | Setup script. Defines the state-space model, computes the LQR gain `K` and Kalman gain `L`, sets noise parameters, and defines the scenario (start, target, and obstacle coordinates, speed, initial range, azimuth, and flight path angle). Must be run before the Simulink model. |
| `simulinkmodelMissileGNC.slx` | The Simulink model containing the guidance logic, LQG controller, airframe, and kinematics. |
| `MissileGNCSystem.prj` | MATLAB project file. |

## Model architecture

```
 TARGET / OBSTACLE / current LLA
              |
              v
   +--------------------+   FPA_cmd    +-----+  q_cmd   +------------+   +-----------------------------+
   |  GUIDANCE COMMAND  |---(+)------->| PID |--------->| q max/min  |-->| LQG Controller and Airframe |
   |  (MATLAB Function) |    ^ -       +-----+          | saturation |   |  plant + Kalman observer +  |
   +--------------------+    |                          +------------+   |  LQR feedback on x_hat      |
              ^              |                                           +-----------------------------+
              |              |  FPA (true) = theta - AoA                              |
              |              +--------------------------------------------------------+
              |                                                                       |
   +---------------------+      x, z      +-----------------------------+            |
   | Flat Earth to LLA   |<---------------| Kinematics: V*cos, V*sin    |<-----------+
   +---------------------+                | integrated to position      |
                                          +-----------------------------+
```

**Guidance Command.** A MATLAB Function block takes the obstacle location, current position, and target location, then outputs the commanded flight path angle, range to target, distance to obstacle, altitude, and a warning flag. Outside a 2 km obstacle threshold it commands a line-of-sight flight path angle to the target. Inside the threshold it raises the warning flag and commands level flight.

**Missile LQG Controller and Airframe Model.** This subsystem holds the true plant (`x' = Ax + Bu + w`, `y = Cx + Du + v`) and the Kalman observer running in parallel. The LQR gain acts on the estimated state rather than the true state. Pitch rate and its estimate are integrated to give `theta` and `theta_hat`, and a scope plots the estimation error `q - q_hat`.

**Kinematics.** True flight path angle is formed from pitch angle and angle of attack. A constant speed is resolved into horizontal and vertical components, integrated to position, and converted to latitude, longitude, and altitude with the Flat Earth to LLA block. The simulation stops when altitude falls below the target elevation.

## Requirements

- MATLAB R2025b or newer (the model was saved in R2025b, Update 2)
- Simulink
- Control System Toolbox (`ss`, `tf`, `lqr`, `kalman`)
- Mapping Toolbox (`azimuth`)
- Aerospace Blockset (Flat Earth to LLA block)

## Getting started

1. Clone the repository.

   ```
   git clone https://github.com/VasistaR/missileguidanceGNCsystem.git
   ```

2. Open MATLAB in the repository folder, or open `MissileGNCSystem.prj`.

3. Run the setup script so the workspace variables exist.

   ```matlab
   model
   ```

   The script prints the open-loop poles, the closed-loop eigenvalues of `A - BK`, the feedback gain `K`, and the observer eigenvalues of `A - LC`.

4. Open and run the Simulink model.

   ```matlab
   open_system('simulinkmodelMissileGNC')
   sim('simulinkmodelMissileGNC')
   ```

5. Check the `height`, `FPA (view)`, and `error (q-qhat)` scopes, along with the range, obstacle distance, and warning flag displays.

## Tuning and experiments

All design parameters live in `model.m`, so they can be changed without touching the Simulink diagram.

- `Q` and `R` set the LQR state and control weighting. Compare the eigenvalues printed before and after a change.
- `QBar` and `RBar` set the Kalman filter's process and measurement noise covariances. Raising `RBar` makes the filter trust the model more and the sensors less.
- `dT1` and `dT2` are the sample times of the two noise sources.
- The `LAT_*`, `LON_*`, and `ELEV_*` values define the scenario geometry.

A good first exercise is to watch how the `q - q_hat` error scope responds as the ratio of `QBar` to `RBar` changes.

## Assumptions and limitations

- Linear, time-invariant, two-state pitch-plane model at a single flight condition
- Constant speed, no thrust, drag, or mass variation
- Planar motion only (cross-range is fixed at zero)
- Flat-earth position propagation
- Obstacle handling is a simple threshold rule rather than a path planner

## Roadmap

- Result plots and screenshots in this README
- A scripted run that logs signals and generates plots automatically

## License

No license file is included yet. Until one is added, all rights are reserved by the author.

## Author

[VasistaR](https://github.com/VasistaR)
