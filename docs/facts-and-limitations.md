# Facts and Caution Notes

- Final paper reference: TTC threshold 3.0 s, final behavior STOP/KEEP, ~90° FOV, lateral ±0.3 m, forward ~2 m, four representative scenarios.
- Development code contains different default parameters. Runtime overrides and later experiment settings must not be confused with source defaults.
- `vrel_speed` naming changed in meaning across development versions. For interview explanation, use the final-paper physical definition of closing relative speed rather than assuming every historical variable had the same meaning.
- Final runtime controller being MPPI has supporting weekly-record evidence but conflicts with at least one exported YAML that still contains DWB. Do not claim an unqualified final MPPI runtime without the final workspace snapshot.
- Exact risk-zone map coordinates changed across YAML versions; the final paper's spatial concept is more reliable than a single development coordinate set.
- One-shot stop was an experimental branch, not the core solution to the resume problem.
