# Jev experiment working rules

Read README.md, PROGRESS.md and the relevant backlog entry. This folder currently contains research, not a motor-control application.

- Public repository: never add real keys, tokens, credentials, account-specific console exports, raw private telemetry or environment dumps. Runtime credentials belong outside version control.
- Use stable task IDs and claim ownership before concurrent work. Separate notes from shared summaries.
- Cite primary sources; distinguish vendor claims, code inspection, actual measurements and hypotheses. Do not claim official Doom provenance for an independent implementation.
- Before hardware work, read ../../deployments/2026-09-08-m3-pro/AGENTS.md, arms.json and docs/OPERATING_NOTES.md. Preserve existing guards and backups.
- Model output must not become unbounded motor commands. Keep execution ownership, calibration, freshness and stop behavior local.
- No test should assume cloud availability or interpret model confidence as physical safety proof.
- Use synthetic data for early offline work. Published traces require review for secrets and private scene data.
