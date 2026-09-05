# Troubleshooting Record

## Static wall false velocity
**Situation**: a stationary wall produced non-zero estimated motion.  
**Initial hypothesis**: LiDAR noise / insufficient filtering.  
**Root cause**: points from different timestamps were compared in different moving local frames.  
**Action**: previous local -> world/map -> current local reprojection before differencing.  
**Result**: ego motion and target motion were structurally separated; the correction became part of the final algorithm description.  
**Lesson**: coordinate-frame assumptions should be checked before parameter tuning.

## Nav2 output but vehicle stationary
**Situation**: RViz path and velocity command were present, but the LiMO did not move.  
**Root cause**: Nav2 produced Twist while the Ackermann driver expected AckermannDrive on `/limo/ack_cmd`.  
**Verification**: direct Ackermann command moved the vehicle, separating hardware/driver from navigation logic.  
**Action**: added a Twist-to-Ackermann bridge.  
**Lesson**: trace topic, message type, publisher/subscriber and hardware layers separately.

## Wall false stop
**Situation**: the ego robot stopped during a run with no opposing robot.  
**Evidence**: target-present range was around 1.8xx m; target-absent wall return around 2.1xx m.  
**Action**: reduced forward max range from 4 m to about 2 m and re-tested.  
**Result**: the no-target route completed without the unnecessary stop in that test.  
**Lesson**: use measured values to close the loop from symptom -> hypothesis -> parameter change -> re-test.

## STOP but no resume
**Situation**: TTC stop occurred correctly, but the robot did not resume after release.  
**Initial hypothesis**: the TTC filter was still overwriting velocity with zero.  
**Evidence**: after filter release, the upstream Nav2 command itself disappeared.  
**Root cause**: Nav2 ProgressChecker interpreted the intentional stop as lack of progress.  
**Action**: `movement_time_allowance` 2 s -> 4 s.  
**Result**: route resumed after the intentional stop.  
**Lesson**: troubleshoot upstream state machines, not only the node that appears closest to the symptom.

## LiDAR jitter / target estimator
**Situation**: ROI mean was contaminated by walls/background and differentiation amplified small range noise.  
**Action**: nearest-beam-centered local window, spatial median in late code, temporal speed filtering and sampling/gate tuning.  
**Lesson**: estimator design should reflect actual sensor distributions rather than rely on a single generic average.
