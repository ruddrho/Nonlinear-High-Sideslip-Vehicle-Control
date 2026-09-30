# Comparative Nonlinear High-Sideslip Vehicle Stabilization Using PID, LQR, Gain-Scheduled LQR, Constrained MPC, and EKF State Estimation

**Final Research Report - V3 Validated MATLAB Simulation Study**  
**Author:** Ruddrho Mollik  
**Date:** 30 September 2026

> **Scope:** MATLAB simulation only. This report does not claim
> physical-vehicle validation and is not a real-vehicle operating guide.

## Abstract

This study presents a reproducible MATLAB framework for nonlinear
high-sideslip vehicle stabilization using PID, fixed LQR, gain-scheduled
LQR (GS-LQR), constrained MPC, and EKF state estimation. The validation
plant is a planar four-wheel rear-wheel-drive model with individual
Fiala-type tyre forces, combined longitudinal/lateral friction sharing,
quasi-static load transfer, aerodynamic and rolling resistance, and
steering/throttle actuator dynamics. The research benchmark is near 32
km/h and -35° sideslip. Six deterministic scenarios and a matched
100-case Monte Carlo study are used. The final V3 numerical gate passed.
Fixed LQR and GS-LQR pass all six deterministic scenarios and achieve
100% Monte Carlo success; constrained MPC passes three of six
deterministic scenarios and achieves 87% Monte Carlo success; PID does
not satisfy the predefined high-sideslip criterion. Under high sensor
noise, EKF feedback reduces MPC sideslip RMSE from 43.587° to 0.221°
(99.49% relative reduction). Conclusions are limited to the declared
simulation model, tuning, and uncertainty set.

## 1. Research objective and contributions

The study is designed as a fair controller comparison under common
nonlinear dynamics, references, disturbance timing, matched uncertainty,
and explicit success criteria. It separates controller-law performance
from estimator performance and separates the steady research benchmark
from the visual Figure-8 demonstration.

Key contributions include a nonlinear four-wheel validation plant; PID,
fixed LQR, GS-LQR, and constrained MPC in one framework; six
standardized scenarios; matched Monte Carlo uncertainty; common EKF
output feedback; EKF and MPC ablations; local control analysis; and a
separate Figure-8 visualization.

## 2. Nonlinear vehicle and tyre model

The state is `x = [X, Y, psi, vx, vy, r, deltaAct, throttleAct]`. The
front wheels steer and the rear wheels receive drive force. Each wheel
has an individual normal load, slip angle, Fiala lateral force,
longitudinal contribution, and combined-force utilization. The plant
includes quasi-static load transfer, drag, rolling resistance,
steering/throttle actuator dynamics, steering magnitude limits, and a
physical steering-rate limit.

Nominal parameters: 1280 kg mass, 2100 kg m² yaw inertia, 2.60 m
wheelbase, 1.55 m track, friction coefficient 1.12, 76/82 kN/rad
front/rear axle cornering stiffness, 12 kN maximum drive force, and 38°
maximum steering.

## 3. Controllers and estimator

**PID:** equilibrium feedforward plus sideslip/yaw and speed
corrections.

**Fixed LQR:** custom CARE-based state feedback around a representative
high-sideslip equilibrium.

**GS-LQR:** local gains interpolated over 30/35/40 km/h and absolute
sideslip angles of 30°/35°/40°.

**Constrained MPC:** scheduled local linear prediction model, horizon
10, 16 projected-gradient iterations, input limits, and explicit
command-rate constraints.

**EKF:** nonlinear state estimation from simulated speed, yaw rate,
lateral acceleration, steering position, and throttle measurements.

## 4. Experimental design

The research step is 0.02 s over 24 s; Monte Carlo uses 0.04 s. Random
seed is 20260930. The steady research reference is approximately 32 km/h
and -35° sideslip. Deterministic scenarios are dry baseline, wet-road
friction drop, crosswind impulse, +15% mass/+10% inertia mismatch, high
sensor noise, and a combined challenge.

The first four scenarios use truth-state feedback to isolate
controller-law behavior. High-noise and combined scenarios use common
EKF output feedback. Monte Carlo perturbs mass, yaw inertia, front/rear
cornering stiffness, and friction using the same 100 cases for every
controller.

## 5. Results

### 5.1 Validation gate

The final numerical validation status is **PASS**. Baseline trajectories
and Monte Carlo metrics are finite. Fixed LQR, GS-LQR, and constrained
MPC stabilize the nominal benchmark. PID remains a comparison baseline.

### 5.2 Baseline

<figure>
<img src="../outputs/paper_results_v3/Fig_01_baseline_tracking.png"
alt="Baseline tracking" />
<figcaption aria-hidden="true">Baseline tracking</figcaption>
</figure>

| Controller         | Beta RMSE (deg) | Yaw RMSE (deg/s) | Speed RMSE (km/h) | Mean compute (ms) | Result |
|--------------------|----------------:|-----------------:|------------------:|------------------:|-------:|
| PID                |       32.291118 |        99.678194 |          1.424456 |          0.003305 |   FAIL |
| Fixed LQR          |        0.001446 |         0.005130 |          0.001484 |          0.005494 |   PASS |
| Gain-Scheduled LQR |        0.000938 |         0.003766 |          0.001021 |          0.024218 |   PASS |
| Constrained MPC    |        0.012860 |         0.013080 |          0.012910 |          0.472759 |   PASS |

### 5.3 Six-scenario benchmark

<figure>
<img src="../outputs/paper_results_v3/Fig_03_scenario_matrix.png"
alt="Scenario matrix" />
<figcaption aria-hidden="true">Scenario matrix</figcaption>
</figure>

| Scenario                          |       PID | Fixed LQR |   GS-LQR | Constrained MPC |
|-----------------------------------|----------:|----------:|---------:|----------------:|
| Dry-road baseline                 | 32.291° ✗ |  0.001° ✓ | 0.001° ✓ |        0.013° ✓ |
| Wet-road friction drop            | 89.716° ✗ |  3.926° ✓ | 3.834° ✓ |       31.966° ✗ |
| Crosswind impulse                 | 32.347° ✗ |  0.211° ✓ | 0.210° ✓ |        6.706° ✗ |
| +15% mass / +10% inertia mismatch | 32.522° ✗ |  0.314° ✓ | 0.323° ✓ |        1.097° ✓ |
| High sensor noise                 | 40.355° ✗ |  0.176° ✓ | 0.173° ✓ |        0.259° ✓ |
| Combined worst-case challenge     | 88.850° ✗ |  4.791° ✓ | 4.676° ✓ |       28.966° ✗ |

Fixed LQR and GS-LQR pass 6/6 scenarios. Constrained MPC passes 3/6. PID
passes 0/6.

<figure>
<img src="../outputs/paper_results_v3/Fig_02_combined_robustness.png"
alt="Combined robustness" />
<figcaption aria-hidden="true">Combined robustness</figcaption>
</figure>

### 5.4 EKF ablation

<figure>
<img src="../outputs/paper_results_v3/Fig_04_ekf_ablation.png"
alt="EKF ablation" />
<figcaption aria-hidden="true">EKF ablation</figcaption>
</figure>

| Variant          | Beta RMSE (deg) | Yaw RMSE (deg/s) | Speed RMSE (km/h) | Result |
|------------------|----------------:|-----------------:|------------------:|--------|
| MPC truth states |        0.012860 |         0.013080 |          0.012910 | PASS   |
| MPC raw sensors  |       43.586948 |        90.114104 |         27.789735 | FAIL   |
| MPC + EKF        |        0.220668 |         0.567298 |          0.185018 | PASS   |

EKF feedback reduces sideslip RMSE by 99.49% relative to raw sensors in
this high-noise experiment.

### 5.5 MPC command-rate ablation

| Variant                | Beta RMSE (deg) | Position RMSE (m) | Rate exceedances | Result |
|------------------------|----------------:|------------------:|-----------------:|--------|
| MPC no slew constraint |       19.695724 |         16.535018 |              540 | FAIL   |
| Constrained MPC        |       30.081924 |         13.560719 |                0 | FAIL   |

Explicit rate constraints eliminate raw command-rate exceedances but do
not recover successful tracking in the severe combined challenge.

### 5.6 Monte Carlo robustness

<figure>
<img src="../outputs/paper_results_v3/Fig_05_monte_carlo_statistics.png"
alt="Monte Carlo statistics" />
<figcaption aria-hidden="true">Monte Carlo statistics</figcaption>
</figure>

| Controller         | Mean beta RMSE |   Std. |     P95 |   Worst | Success |
|--------------------|---------------:|-------:|--------:|--------:|--------:|
| PID                |        35.180° | 3.590° | 40.012° | 40.375° |      0% |
| Fixed LQR          |         1.690° | 1.279° |  4.080° |  6.623° |    100% |
| Gain-Scheduled LQR |         1.640° | 1.240° |  3.938° |  6.391° |    100% |
| Constrained MPC    |         4.439° | 5.245° |  9.853° | 36.695° |     87% |

### 5.7 Local control analysis and computation

<figure>
<img src="../outputs/paper_results_v3/Fig_06_stability_computation.png"
alt="Stability and computation" />
<figcaption aria-hidden="true">Stability and computation</figcaption>
</figure>

The five-state augmented local model has controllability rank 5/5 and
observability rank 5/5. The reported fixed-LQR local poles all have
negative real parts.

| Controller         | Mean compute (ms) | Maximum (ms) |
|--------------------|------------------:|-------------:|
| PID                |          0.003305 |     0.032100 |
| Fixed LQR          |          0.005494 |     0.062100 |
| Gain-Scheduled LQR |          0.024218 |     0.151300 |
| Constrained MPC    |          0.472759 |     1.119300 |

## 6. Supporting Figure-8 outputs

The visual demo is separate from the research benchmark. Its
equilibrium-search target is 29.6 km/h and -30° sideslip. The choreographed
Figure-8 uses a 28.8 km/h nominal display speed while sideslip varies
approximately from -40° to +35°. The saved equilibrium is approximately
9.82 m radius, -17.2° steering, and 35.8% throttle.

<figure>
<img src="../outputs/trajectory.png" alt="Trajectory" />
<figcaption aria-hidden="true">Trajectory</figcaption>
</figure>

<figure>
<img src="../outputs/states.png" alt="State history" />
<figcaption aria-hidden="true">State history</figcaption>
</figure>

For readability, the visual-demo yaw-rate panel is display-clipped to
±120 deg/s; raw values remain in `simulation_data.mat`. This plotting
choice does not affect the research benchmark or controller results.

<figure>
<img src="../outputs/controls.png" alt="Control commands" />
<figcaption aria-hidden="true">Control commands</figcaption>
</figure>

<figure>
<img src="../outputs/tire_forces.png" alt="Tyre forces" />
<figcaption aria-hidden="true">Tyre forces</figcaption>
</figure>

<figure>
<img src="../outputs/reference_final_frame.png"
alt="Final reference frame" />
<figcaption aria-hidden="true">Final reference frame</figcaption>
</figure>

## 7. Discussion

The results do not establish a universal controller ranking. They show
that fixed LQR and GS-LQR are the most consistent for this plant,
tuning, and uncertainty set. The tested MPC implementation is very
accurate nominally and enforces command-rate constraints, but is more
sensitive to severe friction/disturbance mismatch. The EKF ablation
demonstrates that estimator quality materially affects output-feedback
performance.

## 8. Limitations

- Simulation only; no physical vehicle, HIL, or experimental tyre-data
  validation.
- Planar dynamics and quasi-static load transfer simplify real vehicle
  behavior.
- Local high-sideslip stabilization rather than autonomous global path
  tracking.
- MPC uses a local linear prediction model, finite horizon, and finite
  iterations.
- Controller tuning is specific to the tested envelope.
- Monte Carlo covers selected parametric uncertainty only.
- Computation times depend on MATLAB and hardware.
- Non-MPC command-rate exceedance counts are diagnostic; physical
  actuator dynamics/rate limiting remain in the plant.

## 9. Reproducibility

Run the frozen source in this order:

``` matlab
self_test
generate_paper_results
main
record_github_video
```

The two large V7.3 generated research workspaces are omitted from the
GitHub package to keep it compact; they are regenerated by
`generate_paper_results`.

## 10. Conclusion

The V3 project provides a reproducible nonlinear high-sideslip benchmark
with four controllers, EKF state estimation, deterministic disturbances,
matched Monte Carlo uncertainty, ablations, local control analysis, and
a separate Figure-8 visualization. Fixed LQR and GS-LQR are the most
robust in the tested experiment set; constrained MPC combines excellent
nominal regulation and explicit rate compliance with greater
severe-mismatch sensitivity; and EKF state estimation is essential for
the tested high-noise MPC case.

## References

1.  Rajamani, R. *Vehicle Dynamics and Control*, 2nd ed. Springer, 2012.
2.  Pacejka, H. B. *Tire and Vehicle Dynamics*, 3rd
    ed. Butterworth-Heinemann, 2012.
3.  Rawlings, J. B., Mayne, D. Q., and Diehl, M. M. *Model Predictive
    Control: Theory, Computation, and Design*, 2nd ed. Nob Hill
    Publishing, 2017.
4.  Simon, D. *Optimal State Estimation: Kalman, H-infinity, and
    Nonlinear Approaches*. Wiley, 2006.
5.  Chen, C.-T. *Linear System Theory and Design*, 4th ed. Oxford
    University Press, 2013.

## Author

**Ruddrho Mollik**
