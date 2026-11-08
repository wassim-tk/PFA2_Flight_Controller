# Quadcopter control – Simulink model

Quadcopter (X frame) 6-DOF model + cascaded PID flight controller, built
programmatically so you don't have to wire anything by hand.

## Quick start (MATLAB + Simulink, R2018a or newer)
1. Put this folder as MATLAB's current folder.
2. Run `run_sim` → builds `quad_sim.slx`, simulates 32 s, plots, animates.

Mission (edit `blocks/reference.m`): take-off to 1.5 m, move 2 m in x, 2 m in y,
yaw 90°, come back, land, motors stop.

## Files
| File | Role |
|---|---|
| `quad_params.m` | All parameters: mass, inertia, motor/prop constants, mixer, noise, PID gains |
| `build_quad_model.m` | Creates `quad_sim.slx` (blocks, wiring, solver settings) |
| `blocks/quad_dynamics.m` | Plant: rigid-body Newton-Euler + 1st-order motors + ground contact |
| `blocks/controller.m` | Position → angle → rate cascade + altitude loop (runs at 250 Hz) |
| `blocks/mixer.m` | [T, τx, τy, τz] → 4 motor speeds |
| `blocks/sensors.m`, `blocks/state_out.m` | Measurement noise, signal split |
| `blocks/reference.m` | Setpoint profile |
| `plot_results.m`, `animate_quad.m` | Post-processing |
| `linearize_hover.m` | A, B matrices around hover (for tuning / LQR) |

The MATLAB Function blocks get their code from `blocks/*.m`: edit there, then
re-run `build_quad_model` (or edit directly inside Simulink).

## Conventions
Body frame x forward, y left, z up; Euler ZYX. Motors: 1 FR (CCW), 2 RL (CCW),
3 FL (CW), 4 RR (CW). `P.ctl.mode = 0` switches to angle mode (roll/pitch
setpoints from the reference + altitude hold), like a pilot flying.

## Adapting to your drone
Measure/estimate m, arm length, inertia, kf, km, motor time constant and max
speed (thrust-stand test), put them in `quad_params.m`, then retune the gains
inner loop first (rate → angle → altitude → position).
