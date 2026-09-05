# LiMO Pro 2D-LiDAR TTC Unprotected-Left-Turn Decision

ROS2-based mobile-robot research project that estimates collision risk from a 2D LiDAR and applies TTC-based `STOP / KEEP` control during an unprotected-left-turn scenario.

> **Research scope.** This was a personally led university research project. A coauthor assisted with experimental operation such as running the opposing robot. I led the ROS2 environment setup, algorithm structure, code modification/integration, experiment design and execution, CSV/graph analysis, and paper/poster preparation.

## Final research scope

- LiMO Pro + 2D LiDAR
- ROS2 Humble / Cartographer / Nav2-based navigation environment
- ROI-based target selection
- Ego-motion compensation for target-motion estimation
- Spatial and temporal filtering
- Relative closing-speed / TTC calculation
- Position-based TTC activation zone
- `STOP / KEEP` command filtering
- Two-robot validation in four representative speed scenarios

Cartographer, Nav2, AMCL, DWB, and MPPI are existing ROS packages/plugins; this repository does **not** claim they were developed from scratch.

## System architecture

```mermaid
flowchart TD
    L[/scan\n2D LiDAR] --> T[ROI / Target Selection]
    P[Robot Pose / Odometry] --> E[Ego-motion Compensation]
    T --> E
    E --> V[Target Motion / Closing Speed]
    V --> TTC[TTC = distance / closing speed]
    P --> Z[Spatial TTC Activation Zone]
    TTC --> Z

    N[Nav2] --> S[Velocity Smoother]
    S -->|/cmd_vel_smooth| F[TTC Speed Filter]
    Z --> F
    F -->|KEEP: pass-through\nSTOP: zero Twist| C[/cmd_vel]
```

## Core implementation

- `src/speed_only_from_lidar.py` — LiDAR target selection, ego-motion compensation, speed/TTC outputs, risk-zone logic and CSV logging
- `src/ttc_speed_filter.py` — filters Nav2 velocity output into `STOP` or transparent `KEEP`
- `config/development_risk_areas.yaml` — development-stage map risk-zone snapshot
- `config/development_ttc_areas.yaml` — development-stage TTC sub-area snapshot
- `config/final_experiment_reference.yaml` — final-paper reference values, separated from development defaults

### Important version note
The Python source files preserve a **late-stage development snapshot**. Some parameter defaults in code (for example a TTC stop threshold of 5 s) were overridden during experiments. The final paper/poster uses a **3.0 s TTC threshold** and the final reference values are documented separately in `config/final_experiment_reference.yaml`.

## Key engineering problems solved

### 1. Static wall appeared to have velocity
Consecutive LiDAR points were originally compared in different moving `base_link` frames. I changed the logic so the previous target point is transformed from the previous local frame to the world frame and then reprojected into the current robot frame before differencing.

```text
previous target (local_t-1)
        -> world/map
        -> current local frame
        -> compare with current target
```

This separated ego motion from target motion and became a core part of the final paper logic.

### 2. Nav2 planned a path but LiMO did not move
Nav2 was producing `geometry_msgs/Twist`, while the Ackermann driver expected `ackermann_msgs/AckermannDrive` on a different command topic. Direct publication to the Ackermann command topic verified that the hardware/driver was alive, then a Twist-to-Ackermann bridge connected the software stack to the vehicle.

### 3. False stop with no opposing robot
A 4 m ROI allowed a wall around ~2.1 m to become the nearest target. Comparing target-present and target-absent range values led to reducing the forward range to about 2 m and re-running the route without the unnecessary stop.

### 4. STOP worked, but the robot did not resume
The initial suspicion was that the TTC filter kept publishing zero velocity. Topic/CSV tracing showed the upstream Nav2 command had stopped instead. The cause was interaction with Nav2 `ProgressChecker`; increasing `movement_time_allowance` from 2 s to 4 s allowed the planned route to resume after the intentional TTC stop.

### 5. LiDAR range/speed jitter
The target estimator evolved from ROI mean/centroid toward a nearest-beam-centered local window. Up to 5 neighboring beams were used and late-stage implementations applied a local median. Temporal speed smoothing compared multiple window/filter variants, with a 5-sample median used in the final paper for speed filtering.

## Final experiment reference

| Scenario | Ego | Opposing robot | Verified behavior |
|---|---:|---:|---|
| 1 | 0.5 m/s | 0.5 m/s | STOP under collision-risk condition |
| 2 | 0.8 m/s | 0.8 m/s | STOP under collision-risk condition |
| 3 | 0.5 m/s | 0.8 m/s | KEEP; opposing robot passes first |
| 4 | 0.8 m/s | 0.5 m/s | KEEP; ego robot passes first |

The result should be stated only as: **the expected STOP/KEEP behavior was confirmed in four representative scenarios.** No 100% success rate, accuracy, or false-stop percentage was measured.

## Research limitations

- Four representative scenarios; no statistical success-rate analysis
- Simplified indoor/intersection geometry
- Nearest-target selection can switch under multi-object scenes
- Closing-speed model is simplified for the opposing-approach experiment
- 2D LiDAR alone does not provide object class/identity
- Map-specific spatial activation zone
- Some development branches (one-shot stop, TTC sub-area, controller choice) changed during testing and are not presented as universal final settings

See [`docs/version-history.md`](docs/version-history.md), [`docs/troubleshooting.md`](docs/troubleshooting.md), and [`CONTRIBUTION.md`](CONTRIBUTION.md).
