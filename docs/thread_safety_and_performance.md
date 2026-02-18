# Thread Safety And Performance Guarantees

Author: Watson

## Thread safety

`Dtm2020Operational` and `Dtm2020Research` are designed for concurrent read-only use:
- loading (`LoadFromFile`) constructs immutable model state
- evaluation (`Evaluate`) is `const` and uses only stack-local temporaries
- no global mutable model state is used in evaluation paths

Practical guidance:
- build/load a model once
- share the model instance across worker threads
- call `Evaluate` concurrently from multiple threads

Validation:
- `dtm2020_thread_safety_smoke` validates concurrent `Evaluate` calls on a shared
  `Dtm2020Operational` instance against a sequential baseline.

## Performance invariants

The repository maintains explicit performance and numerical guardrails:
- throughput benchmarks:
  - `dtm2020_operational_perf_benchmark`
  - `dtm2020_research_perf_benchmark`
- deterministic numerical regression:
  - `dtm2020_operational_numerical_regression`

When changing hot paths, keep this sequence:
1. Measure baseline using profile preset.
2. Apply optimization without changing model equations/order unless intentional.
3. Re-run debug + ASan + numerical regression + benchmarks.
4. Record deltas in `docs/perf_baseline.md`.
