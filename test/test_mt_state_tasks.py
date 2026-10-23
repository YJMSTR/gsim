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


def run(command, env):
    result = subprocess.run(list(map(str, command)), env=env, text=True,
                            capture_output=True, timeout=90)
    if result.returncode:
        raise AssertionError(f'{command} failed:\n{result.stdout}\n{result.stderr}')
    return result


class StateTasksTest(unittest.TestCase):
    def generate(self, out, design, workers, scheduler='heft', lookahead=0, extra=(),
                 wakeup=None):
        env = {k: v for k, v in os.environ.items()
               if not k.startswith('GSIM_MT_')}
        env.update(GSIM_THREADS=str(workers), GSIM_MT_EXECUTOR='dense')
        if wakeup is not None:
            env['GSIM_MT_WAKEUP_PERWORKER'] = wakeup
        run([GSIM, '--mt-mode=on', '--mt-target-tasks=64',
             '--mt-scheduler=' + scheduler, f'--mt-lookahead-window={lookahead}',
             *extra, '--dir', out, ROOT / 'test' / f'{design}.fir'], env)
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
        self.assertTrue(all(r['barrier'] == 'state_publish' for r in states))
        self.assertTrue(all(r['reg_src_members'] == '0' and
                            r['barrier'] == 'state_acquire' for r in compute))
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

    def test_wakeup_perworker_trace_equivalence(self):
        # GSIM_MT_WAKEUP_PERWORKER=1 replaces the single shared generation
        # line with one 64-byte wake slot per worker (main fans the new
        # generation out with plain release stores) and turns the in-order
        # join into a bitmap batch scan. Neither change may alter any
        # observable trace on either executor path; a lost wakeup or a
        # missed done-flag would hang the model and fail the run timeout
        # (the liveness gate for this protocol change).
        baseline = None
        for workers in (1, 2, 4):
            for lookahead in (0, 16, 64):
                for wakeup in (None, '1'):
                    with self.subTest(workers=workers, lookahead=lookahead,
                                      wakeup=wakeup), \
                            tempfile.TemporaryDirectory(
                                prefix='mt-wakeup-pp-') as directory:
                        out = Path(directory)
                        env, _ = self.generate(out, 'mt-state-tasks', workers,
                                               lookahead=lookahead,
                                               wakeup=wakeup)
                        trace = self.trace(out, 'mt-state-tasks', env)
                        if baseline is None:
                            baseline = trace
                            self.assertTrue(baseline)
                        self.assertEqual(trace, baseline)
        # mt-memory-commit exercises real cross-worker token groups plus the
        # full wake/join cycle with 4 workers.
        signatures = []
        for wakeup in (None, '1'):
            with tempfile.TemporaryDirectory(
                    prefix='mt-wakeup-pp-mem-') as directory:
                out = Path(directory)
                env, _ = self.generate(out, 'mt-memory-commit', 4,
                                       lookahead=16, wakeup=wakeup)
                signatures.append(self.simulate(out, 'mt-memory-commit', env))
        self.assertEqual(len(set(signatures)), 1, signatures)

    def test_wakeup_perworker_generation_determinism(self):
        # Repeated knob-on generation must stay byte-deterministic.
        with tempfile.TemporaryDirectory(
                prefix='mt-wakeup-pp-det-') as directory:
            root = Path(directory)
            trees = []
            for run_index in (0, 1):
                out = root / f'wakeup-r{run_index}'
                out.mkdir()
                self.generate(out, 'mt-state-tasks', 4, lookahead=64,
                              wakeup='1')
                trees.append({path.name: path.read_bytes()
                              for path in sorted(out.iterdir())})
            self.assertEqual(trees[0], trees[1])

    def test_memory_commits_against_reference(self):
        signatures = []
        for lookahead in (0, 16, 64):
            with self.subTest(lookahead=lookahead), tempfile.TemporaryDirectory(
                    prefix='mt-state-memory-') as directory:
                out = Path(directory)
                env, members = self.generate(out, 'mt-memory-commit', 4,
                                             lookahead=lookahead)
                self.assertTrue(any(r['node_type'] == 'memory' and r['role'] == 'member'
                                    for r in members))
                self.assertTrue(any(r['node_type'] == 'memory_writer' and
                                    r['role'] == 'pending_write_producer' for r in members))
                signatures.append(self.simulate(out, 'mt-memory-commit', env))
        self.assertEqual(len(set(signatures)), 1, signatures)

    def test_sparse_array_commits_and_resets_match_dense(self):
        for design in ('mt-sparse-register-reset', 'mt-sparse-register-async-reset'):
            signatures = []
            for max_writes in (0, 8):
                with self.subTest(design=design, max_writes=max_writes), tempfile.TemporaryDirectory(
                        prefix='mt-state-sparse-') as directory:
                    out = Path(directory)
                    env, members = self.generate(out, design, 2, extra=(
                        '--mt-sparse-register-min-bytes=1',
                        f'--mt-sparse-register-max-writes={max_writes}'))
                    self.assertEqual(sum(r['node_type'] == 'reg_src' and
                                         r['node'] == 'entries' for r in members), 1)
                    self.assertEqual(sum(r['node_type'] == 'sparse_writer' for r in members),
                                     0 if max_writes == 0 else 2)
                    signatures.append(self.simulate(out, design, env))
            self.assertEqual(len(set(signatures)), 1, (design, signatures))


if __name__ == '__main__':
    unittest.main()
