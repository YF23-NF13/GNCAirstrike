# GNCAirstrike: missile guidance with an LQR/LQG pitch autopilot

> **This repository is a fork of [Vinayak-D/GNCAirstrike](https://github.com/Vinayak-D/GNCAirstrike).**
> The original idea and code belong to its author, Vinayak D. The `main` branch holds **my own version** of the project. The author's original files are kept, unchanged, on the [`master`](https://github.com/YF23-NF13/GNCAirstrike/tree/master) branch.

This Simulink model guides a missile towards a ground target while it avoids an obstacle. It includes:

- a short-period pitch model of the missile (Reichert's missile benchmark: angle of attack α and pitch rate q),
- an **LQR** state-feedback inner loop that controls the pitch rate,
- a **Kalman filter** (LQG) that estimates the states from noisy measurements,
- a guidance outer loop that computes the flight-path-angle command from the range to the target and monitors the distance to the obstacle,
- an optional real-time 3D view in **FlightGear**.

## Credits

| | |
|---|---|
| Original project | [GNCAirstrike](https://github.com/Vinayak-D/GNCAirstrike) by **Vinayak D** ([@Vinayak-D](https://github.com/Vinayak-D)), from his YouTube tutorial |
| Original files | `model.m`, `runfg.bat`, `simulink_model.slx`, `simulink_model_FG.slx` (see the [`master`](https://github.com/YF23-NF13/GNCAirstrike/tree/master) branch) |
| Missile model | Reichert's missile benchmark (linearised short-period dynamics) |

The original repository has no license, so all rights to the original code remain with its author. This fork is a personal, educational project made to learn GNC. It is not intended for commercial use.

## My contribution

Starting from the original project, I made my own version:

- **`Missile_guidance_system.m`**: I rewrote the parameter script step by step and documented every part. It covers the state-space model, the transfer function and pole analysis, the LQR design, the Kalman filter design and observer stability, the haversine distance, and the initial azimuth and flight-path angle. I also fixed the latitude used in the haversine formula.
- **`Simulink_guidance_true2.slx`**: I rebuilt the Simulink model. It contains the state-space plant, LQR feedback on the Kalman estimate, process and measurement noise, the guidance loop (range and flight-path-angle command), the obstacle distance and warning flag, the flat-Earth to LLA conversion, the FlightGear interface, and an automatic stop when the missile reaches the target altitude.

## Repository structure

| File | Description |
|---|---|
| `Missile_guidance_system.m` | Parameters: missile model, LQR and Kalman gains, initial geometry. Run it first. |
| `Simulink_guidance_true2.slx` | Simulink model of the full GNC loop |

## How to run

Requirements: MATLAB and Simulink with the Control System Toolbox, the Mapping Toolbox (`azimuth`) and the Aerospace Blockset (flat-Earth/LLA conversion and FlightGear blocks).

1. Run `Missile_guidance_system.m` to load the parameters into the workspace.
2. Open `Simulink_guidance_true2.slx` and start the simulation.
3. *(Optional)* For the 3D view, start FlightGear first with the `runfg.bat` script from the [`master`](https://github.com/YF23-NF13/GNCAirstrike/tree/master) branch. Adapt the FlightGear install path to your machine.

## Author

[@YF23-NF13](https://github.com/YF23-NF13): my version of the project (the two files above and this README).
