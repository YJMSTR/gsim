"""Verify state-task ownership, register timing, reset and pending commits.

Run after `make build-gsim`: python3 test/test_mt_state_tasks.py

Correctness is checked by differential traces across executor configurations
(lookahead-0 strict vs lookahead windows, workers 1/2/4) plus targeted semantic
assertions on the emitted trace — never by emitted source-string comparison.
The old hand-written register/reset oracle predates the intentional
ASY->UINT change (0821ede) and failed on every variant including serial
--mt-mode=off; it was removed rather than carried red.
"""
import csv
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
GSIM = ROOT / 'build/gsim/gsim'


# Trace-only differential harness: drives the generated mt-state-tasks model
# with a reset/async/late-reset stimulus pattern; every observable is printed
# each cycle so complete traces can be compared across executions.
TRACE_HARNESS = r'''
#include "MtStateTasks.h"
#include <cstdint>
#include <cstdio>

int main() {
  SMtStateTasks dut;
  for (unsigned cycle = 0; cycle < 256; ++cycle) {
    bool reset = cycle < 3 || cycle % 79 == 0;
    bool asyncReset = cycle < 2 || cycle % 47 == 0;
    bool enable = cycle % 5 != 0;
    bool request = cycle % 13 < 2;
    uint8_t a = cycle * 17 + 3, b = cycle * 29 + 5;
    dut.set_clock(cycle & 1);
    dut.set_reset(reset);
    dut.set_async_reset(asyncReset);
    dut.set_reset_request(request);
    dut.set_enable(enable);
    dut.set_a(a);
    dut.set_b(b);
    dut.step();
    std::printf("%u %u %u %u %u %u %u %u\n", cycle,
                dut.get_value0(), dut.get_value1(), dut.get_value_async(),
                dut.get_value_reg_reset(), dut.get_reset_value(),
                dut.get_value_late_reset(), dut.get_mix());
  }
}
'''


def async_reset_at(cycle):
    return cycle < 2 or cycle % 47 == 0



def state_tasks_reference_trace(cycles=256):
    """Per-beat reference model of test/mt-state-tasks.fir.

    Commit semantics follow the cycle-start state task exactly: input reset
    conditions use the current cycle's input value, register-driven
    conditions use the previous committed value, computed node conditions
    (late_reset) hold the previous cycle's computed value at the commit, and
    next-state values come from the previous cycle's pending writes.
    """
    q0 = q1 = qa = qb = ql = 0
    n0 = n1 = na = nb = nl = 0
    late = qr_prev = req_prev = 0
    lines = []
    for cycle in range(cycles):
        reset = cycle < 3 or cycle % 79 == 0
        async_reset = cycle < 2 or cycle % 47 == 0
        enable = cycle % 5 != 0
        request = cycle % 13 < 2
        a = (cycle * 17 + 3) & 0xff
        b = (cycle * 29 + 5) & 0xff
        q0 = 0x12 if reset else n0
        q1 = 0x34 if reset else n1
        qa = 0x56 if async_reset else na
        qb = 0x78 if qr_prev else nb
        qr = 0 if reset else req_prev
        ql = 0x9a if late else nl
        lines.append(f'{cycle} {q0} {q1} {qa} {qb} {qr} {ql} '
                     f'{q0 ^ q1 ^ qa ^ qb}')
        n0 = ((q1 + a) & 0xff) if enable else q0
        n1 = q0 ^ b
        na = (qa + q0) & 0xff
        nb = (qb + b) & 0xff
        nl = (ql + a) & 0xff
        late = 1 if request != enable else 0
        qr_prev = qr
        req_prev = 1 if request else 0
    return '\n'.join(lines) + '\n'

def run(command, env):
    result = subprocess.run(list(map(str, command)), env=env, text=True,
                            capture_output=True, timeout=90)
    if result.returncode:
        raise AssertionError(f'{command} failed:\n{result.stdout}\n{result.stderr}')
    return result


class StateTasksTest(unittest.TestCase):
    def generate(self, out, design, workers, scheduler='heft', lookahead=0, extra=(),
                 state_tokens=False):
        env = {k: v for k, v in os.environ.items()
               if not k.startswith('GSIM_MT_')}
        env.update(GSIM_THREADS=str(workers), GSIM_MT_EXECUTOR='dense')
        if state_tokens:
            env['GSIM_MT_STATE_TOKENS'] = '1'
        run([GSIM, '--mt-mode=on', '--mt-target-tasks=64',
             '--mt-scheduler=' + scheduler, f'--mt-lookahead-window={lookahead}',
             *extra, '--dir', out, ROOT / 'test' / f'{design}.fir'], env)
        publish = 'state_token_publish' if state_tokens else 'state_publish'
        acquire = 'state_token_acquire' if state_tokens else 'state_acquire'
        with (out / 'mt-task-phases.csv').open() as stream:
            phases = list(csv.DictReader(stream))
        with (out / 'mt-state-task-members.csv').open() as stream:
            members = list(csv.DictReader(stream))
        states = [r for r in phases if r['phase'] == 'state']
        compute = [r for r in phases if r['phase'] == 'compute']
        owned = [r for r in members if r['role'] == 'member']
        self.assertEqual(len(states), workers)
        self.assertEqual({int(r['worker']) for r in states}, set(range(workers)))
        self.assertTrue(all(r['task'] == r['worker'] for r in states))
        self.assertTrue(all(r['barrier'] == publish for r in states))
        self.assertTrue(all(r['reg_src_members'] == '0' and
                            r['barrier'] == acquire for r in compute))
        self.assertEqual(len({r['node_id'] for r in owned}), len(owned))
        self.assertEqual(sum(int(r['members']) for r in states), len(owned))
        self.assertEqual(sum(int(r['reg_src_members']) for r in states),
                         sum(r['node_type'] == 'reg_src' for r in owned))
        return env, members

    def trace(self, out, design, env):
        (out / 'trace-harness.cpp').write_text(TRACE_HARNESS)
        exe = out / 'emu-trace'
        run(['clang++', '-std=c++17', '-O1', '-Wno-parentheses-equality',
             '-I' + str(out), *sorted(out.glob('*.cpp')),
             '-pthread', '-o', exe], env)
        return run([exe], env).stdout

    def simulate(self, out, design, env):
        exe = out / 'emu-test'
        run(['clang++', '-std=c++17', '-O1', '-Wno-parentheses-equality',
             '-I' + str(out), *sorted(out.glob('*.cpp')),
             ROOT / 'test' / f'{design}.cpp', '-pthread', '-o', exe], env)
        return run([exe], env).stdout.strip()

    def test_lookahead_trace_equivalence(self):
        # lookahead-0 strict is the baseline executor; every lookahead window
        # (including the promoted 64) must produce byte-identical traces, and
        # the async-reset register must show the synchronous semantics the
        # MT path intentionally adopts (reset value visible in the asserted
        # cycle's committed state).
        baseline = None
        for workers in (1, 2, 4):
            for lookahead in (0, 16, 64):
                with self.subTest(workers=workers, lookahead=lookahead), \
                        tempfile.TemporaryDirectory(
                            prefix='mt-state-trace-') as directory:
                    out = Path(directory)
                    env, members = self.generate(out, 'mt-state-tasks', workers,
                                                 lookahead=lookahead)
                    registers = {r['node'] for r in members
                                 if r['node_type'] == 'reg_src'}
                    self.assertEqual(registers,
                                     {'q0', 'q1', 'qa', 'qb', 'qr', 'ql'})
                    trace = self.trace(out, 'mt-state-tasks', env)
                    if baseline is None:
                        baseline = trace
                        self.assertTrue(baseline)
                    self.assertEqual(trace, baseline)
        for line in baseline.strip().splitlines():
            fields = line.split()
            cycle, value_async = int(fields[0]), int(fields[3])
            if async_reset_at(cycle):
                self.assertEqual(value_async, 0x56,
                                 f'cycle {cycle}: async-reset register did not '
                                 f'commit the reset value synchronously')

    def test_state_tokens_per_beat_reference_matrix(self):
        # GSIM_MT_STATE_TOKENS replaces the all-worker state barrier with
        # per-owner commit tokens. The independent per-beat reference model
        # must hold for every knob x worker x lookahead combination, proving
        # the token protocol preserves the two-phase commit semantics
        # (including computed reset conditions read live by the commit).
        expected = state_tasks_reference_trace()
        for state_tokens in (False, True):
            for workers in (1, 4):
                for lookahead in (0, 16):
                    with self.subTest(state_tokens=state_tokens,
                                      workers=workers, lookahead=lookahead), \
                            tempfile.TemporaryDirectory(
                                prefix='mt-state-tokens-') as directory:
                        out = Path(directory)
                        env, _ = self.generate(out, 'mt-state-tasks', workers,
                                               lookahead=lookahead,
                                               state_tokens=state_tokens)
                        self.assertEqual(self.trace(out, 'mt-state-tasks', env),
                                         expected)

    def test_state_tokens_computed_reset_reference_matrix(self):
        # mt-state-tokens.fir pins the reverse reset-condition hazard: both
        # reset conditions are computed nodes read by the cycle-start commit,
        # so their producers must be ordered after the commits. The per-beat
        # cpp reference checks every cycle; signatures must match across the
        # whole matrix and against tokens disabled.
        signatures = []
        for state_tokens in (False, True):
            for workers in (1, 4):
                for lookahead in (0, 16):
                    with self.subTest(state_tokens=state_tokens,
                                      workers=workers, lookahead=lookahead), \
                            tempfile.TemporaryDirectory(
                                prefix='mt-computed-reset-') as directory:
                        out = Path(directory)
                        env, _ = self.generate(out, 'mt-state-tokens', workers,
                                               lookahead=lookahead,
                                               state_tokens=state_tokens)
                        signatures.append(
                            self.simulate(out, 'mt-state-tokens', env))
        self.assertEqual(len(set(signatures)), 1, signatures)

    def test_state_tokens_off_is_byte_identical(self):
        # The knob must not perturb generation when off: explicit 0 and unset
        # produce identical output trees (layout stays the baseline's).
        with tempfile.TemporaryDirectory(prefix='mt-token-off-') as directory:
            root = Path(directory)
            trees = []
            for index, extra in ((0, {}), (1, {'GSIM_MT_STATE_TOKENS': '0'})):
                out = root / f'r{index}'
                out.mkdir()
                env, _ = self.generate(out, 'mt-state-tasks', 4, lookahead=16)
                env.update(extra)
                run([GSIM, '--mt-mode=on', '--mt-target-tasks=64',
                     '--mt-lookahead-window=16', '--dir', out,
                     ROOT / 'test' / 'mt-state-tasks.fir'], env)
                trees.append({path.name: path.read_bytes()
                              for path in sorted(out.iterdir())})
            self.assertEqual(trees[0], trees[1])

    def test_lookahead_generation_determinism(self):
        # Identical invocations must emit byte-identical models (the worker
        # local field order is pinned by node id, not pointer/hash order).
        with tempfile.TemporaryDirectory(prefix='mt-determinism-') as directory:
            root = Path(directory)
            trees = []
            for run_index in (0, 1):
                out = root / f'la64-r{run_index}'
                out.mkdir()
                self.generate(out, 'mt-state-tasks', 4, lookahead=64)
                trees.append({path.name: path.read_bytes()
                              for path in sorted(out.iterdir())})
            self.assertEqual(trees[0], trees[1])

    def test_memory_commits_against_reference(self):
        signatures = []
        for state_tokens in (False, True):
            for lookahead in (0, 16, 64):
                with self.subTest(state_tokens=state_tokens, lookahead=lookahead), \
                        tempfile.TemporaryDirectory(
                            prefix='mt-state-memory-') as directory:
                    out = Path(directory)
                    env, members = self.generate(out, 'mt-memory-commit', 4,
                                                 lookahead=lookahead,
                                                 state_tokens=state_tokens)
                    self.assertTrue(any(r['node_type'] == 'memory' and r['role'] == 'member'
                                        for r in members))
                    self.assertTrue(any(r['node_type'] == 'memory_writer' and
                                        r['role'] == 'pending_write_producer' for r in members))
                    signatures.append(self.simulate(out, 'mt-memory-commit', env))
        self.assertEqual(len(set(signatures)), 1, signatures)

    def test_sparse_array_commits_and_resets_match_dense(self):
        for design in ('mt-sparse-register-reset', 'mt-sparse-register-async-reset'):
            signatures = []
            for state_tokens in (False, True):
                for max_writes in (0, 8):
                    with self.subTest(design=design, state_tokens=state_tokens,
                                      max_writes=max_writes), tempfile.TemporaryDirectory(
                            prefix='mt-state-sparse-') as directory:
                        out = Path(directory)
                        env, members = self.generate(out, design, 2, extra=(
                            '--mt-sparse-register-min-bytes=1',
                            f'--mt-sparse-register-max-writes={max_writes}'),
                            state_tokens=state_tokens)
                        self.assertEqual(sum(r['node_type'] == 'reg_src' and
                                             r['node'] == 'entries' for r in members), 1)
                        self.assertEqual(sum(r['node_type'] == 'sparse_writer' for r in members),
                                         0 if max_writes == 0 else 2)
                        signatures.append(self.simulate(out, design, env))
            self.assertEqual(len(set(signatures)), 1, (design, signatures))


if __name__ == '__main__':
    unittest.main()
