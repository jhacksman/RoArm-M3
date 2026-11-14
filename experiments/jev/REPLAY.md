# Offline decision replay

Implemented subset of RJ-04. Python 3.10+ standard library only. This builds a Choice request and validates synthetic responses; it has no HTTP client, camera driver, robot driver or actuator callback. `accepted_for_shadow` means the contract accepted a hypothetical decision, not permission to move hardware.

From `experiments/jev`:

```sh
python3 -m unittest discover -s tests -v
python3 -m jev_replay examples/synthetic-replay.json
```

The supplied fixture contains six independent synthetic cases and seven response attempts: a valid choice, its duplicate, an expired response, a changed observation revision, lost tracking, changed calibration, and a malformed probability distribution. Timings, probabilities and sensor labels are hand-authored examples, not measured Jev/Thor behavior or recommended operating thresholds.

Exit status: 0 means all expected verdicts matched; 1 means at least one mismatch; 2 means an unreadable/malformed fixture. Output contains case IDs, verdicts and selected candidate IDs, not raw sensor state or complete API responses. IDs must also be public-safe. This is not a redaction tool; only deliberately reviewed synthetic data belongs in version control.

## Contract

`Observation.parse` validates and freezes the session, arm, semantic epoch, calibration version, capture time, task, descriptive labels, boolean facts and candidate definitions. Candidate IDs must be unique. Every candidate identifies its skill, target, description, positive proposed lease and named preconditions. Missing/unknown facts are rejected; false facts remove candidates before request construction. Geometry and physical motion limits are not modeled in this offline contract.

`Pending.create` binds the immutable observation and currently feasible candidates to a unique request ID, issue time, deadline and maximum observation age. All times use one session's host-monotonic millisecond domain. Production sensor timestamps would require validated conversion into that domain. The replay clock is injected, not wall-clock time.

`build_request` creates the [documented Jev Choice shape](https://docs.typesafe.ai/api) with candidate meanings in criteria and the actual question in instructions. It does not transmit a request. `parse_choice` checks the chosen ID, answer type, complete candidate distribution, finite probabilities in range, sum within floating-point tolerance, highest-probability choice, model identifier and confidence field. Raw response fields are not rewritten into fabricated one-hot probabilities. Confidence is validated as data; it does not establish physical safety. Live API compatibility remains untested.

`ShadowGate.evaluate` checks request/deadline ordering, both original and current capture age, session/arm/calibration identity, semantic revision and state consistency, and exact candidate identity plus current preconditions. Rejecting an attempted response consumes its request ID. Accepting a response consumes that arm/session epoch so changing request IDs cannot repeat a hypothetical action on the same revision. Time regressions fail closed.

Epoch means a decision-relevant state revision, not necessarily every camera frame. A refreshed capture may retain an epoch only while its task, labels, candidate meanings and required facts remain valid. After an accepted action or significant scene change, the state producer must issue a new revision. The gate cannot prove that sensor producers are truthful or calibrated.

The gate is sequential, in-memory and intended for short replays. It has no restart persistence, bounded history, concurrency lock, shared command ownership, action execution, watchdog or physical stop behavior. Those belong to subsequent tasks; this must not be connected directly to motors. A lease is recorded and protected against mutation but is not physically enforced here.

## File format

The top-level fixture has `schema_version: 1`, `data_kind: "synthetic"`, optional descriptive metadata and a nonempty `cases` array. Each case has a submitted observation, request ID, timing window and one or more attempts containing the current observation, a synthetic response, response time and expected reason. Each case gets a fresh gate; attempts within it share duplicate/clock state. The supplied JSON is the complete runnable example.

Malformed observation objects are rejected rather than coerced. Duplicate JSON keys and nonfinite JSON constants are rejected. The package reads only the explicitly selected fixture; it never loads environment variables or credentials.

## Next integration work

- RJ-10: verify the actual Thor platform, then replay reviewed camera data through a perception adapter.
- RJ-04: add a separately tested live transport/shadow evaluator once model access, latency budget and private credential handling are established. Bind every transport response to its pending request locally; the model need not echo request IDs.
- RJ-02/RJ-03: validate command ownership, stopping, calibrated motion and physical outcomes independently before any executor integration.

These checks measure software behavior only. They provide no task-success, latency or robotics safety result.
