# Parameter / Design Version History

| Item | Early / exploration | Intermediate | Final-paper reference |
|---|---|---|---|
| LiDAR FOV | broad 180° / ~70° concepts | 40° / 60° / 70° runs | ~90° |
| Lateral ROI | wider/mixed bands | converged to -0.3~+0.3 m | ±0.3 m |
| Forward range | up to 12 m, then 3~4 m | 4 m caused wall false stop | ~2 m |
| Range estimator | ROI mean / centroid | nearest point -> ±2-beam local window | 5 nearby data points; late code uses local median |
| Speed interval | ~0.5 s records | 0.1 -> 0.2 s tuning | ~0.2 s late reference |
| Speed filter | raw / mean | 3 vs 5 sample, mean vs median | 5-sample median |
| Speed spike gate | none / varied | ~8 m/s gate | implementation aid, not paper headline |
| TTC threshold | 5 s examples/defaults | 3.5 s and other trials | **3.0 s** |
| Decision states | STOP/KEEP/ACCEL concepts | TTC-t_clear branches | **STOP / KEEP** |
| Spatial gating | multiple rectangles / YAML versions | Risk Area + TTC sub-area | stop-line-adjacent activation zone (~1 m concept) |
| ProgressChecker | 2.0 s | intentional stop conflicted | 4.0 s after resume troubleshooting |

Exact development risk-zone map coordinates and one-shot hold/dwell values remain version-dependent and should not be presented as final-paper constants.
