# Test Inputs

- Any `*.fir` file in this directory is auto-discovered by `make fir-tests` and by the GitHub CI `fir-regression` job.
- A same-name `*.cpp` file is compiled with the generated model and run under
  AddressSanitizer and UndefinedBehaviorSanitizer.
- repro-usefulreset.fir: Minimized FIR reproducer for GSIM issue #106, used to guard against ConstantAnalysis hangs and OOM regressions.
- signed-node-shr.fir: Checks constant propagation of a signed right shift
  whose operand is a node reference.
- mt-token-reduce.fir: Four-lane three-stage register chain where every
  stage-3 lane reads its stage-1 producers directly, so the middle-stage token
  path covers the direct producer token. `make fir-tests` checks the golden
  recurrence in mt-token-reduce.cpp against the default single-thread model;
  the same pair exercises `--mt-reduce-tokens=1` under the multithreaded
  emitter with the command below.

## Per-config output isolation

`make fir-tests` / `make run-fir-test` write every artifact of a case into
`FIR_TEST_OUTPUT_DIR/<case>` (default `build/fir-tests`). Whenever a case is
run with a non-default flag set, pass its own `FIR_TEST_OUTPUT_DIR` so the
configuration's generated model, logs and binary do not overwrite the default
run's artifacts. For the mt-token-reduce token reduction:

```bash
GSIM_THREADS=2 make run-fir-test FIR_TEST=mt-token-reduce \
  FIR_TEST_OUTPUT_DIR=build/fir-tests-mt-reduce \
  GSIM_FLAGS_EXTRA="--mt-mode=on --mt-reduce-tokens=1"
```

`GSIM_THREADS` must be exported at generation (it bakes the worker count into
the model) and the driver sets the runtime `GSIM_THREADS` from
`MT_TOKEN_REDUCE_THREADS` (default 2); compile the driver with
`-DMT_TOKEN_REDUCE_THREADS=N` when generating with a different width. The
generation log then shows `[cppEmitter-mt] token-reduce: edges=... -> ...`
with `dropped>0`, confirming redundant cross-worker waits were removed.
