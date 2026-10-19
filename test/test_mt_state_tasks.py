"""Verify state-task ownership, register timing, reset and pending commits.

Run after `make build-gsim`: python3 test/test_mt_state_tasks.py

GSIM_MT_DIRECT_DISPATCH=1 only changes lookahead-0 strict emission; it is
covered by whole-model byte identity and a trace-only differential harness,
never by emitted source-string assertions. The register/reset oracle below
predates the intentional ASY->UINT late-reset change (0821ede) and fails on
every variant including serial --mt-mode=off; it is retained unmodified and
is not this feature's contract.
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
# with the same stimulus pattern as mt-state-tasks.cpp but carries no expected
# values; every observable is printed each cycle so complete traces can be
# compared across strict/direct/lookahead executions.
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


def run(command, env):
    result = subprocess.run(list(map(str, command)), env=env, text=True,
                            capture_output=True, timeout=90)
    if result.returncode:
        raise AssertionError(f'{command} failed:\n{result.stdout}\n{result.stderr}')
    return result


class StateTasksTest(unittest.TestCase):
    def generate(self, out, design, workers, scheduler='heft', lookahead=0, extra=(),
                 direct=None):
        env = dict(os.environ, GSIM_THREADS=str(workers), GSIM_MT_EXECUTOR='dense')
        env.pop('GSIM_MT_DIRECT_DISPATCH', None)
        if direct is not None:
            env['GSIM_MT_DIRECT_DISPATCH'] = direct
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

    def simulate(self, out, design, env):
        exe = out / 'emu-test'
        run(['clang++', '-std=c++17', '-O1', '-Wno-parentheses-equality',
             '-I' + str(out), *sorted(out.glob('*.cpp')),
             ROOT / 'test' / f'{design}.cpp', '-pthread', '-o', exe], env)
        return run([exe], env).stdout.strip()

    def test_registers_and_reset_against_reference(self):
        signatures = []
        for workers in (1, 4):
            for scheduler in ('heft', 'list'):
                for lookahead in (0, 16):
                    with self.subTest(workers=workers, scheduler=scheduler,
                                      lookahead=lookahead), tempfile.TemporaryDirectory(
                                          prefix='mt-state-tasks-') as directory:
                        out = Path(directory)
                        env, members = self.generate(out, 'mt-state-tasks', workers,
                                                     scheduler, lookahead)
                        registers = [r for r in members if r['node_type'] == 'reg_src']
                        self.assertEqual({r['node'] for r in registers},
                                         {'q0', 'q1', 'qa', 'qb', 'qr', 'ql'})
                        signatures.append(self.simulate(out, 'mt-state-tasks', env))
        self.assertEqual(len(set(signatures)), 1, signatures)

    def test_direct_dispatch_trace_equivalence(self):
        traces = {}
        for lookahead, direct in ((0, None), (0, '1'), (16, None), (16, '1')):
            with self.subTest(lookahead=lookahead, direct=direct), \
                    tempfile.TemporaryDirectory(
                        prefix='mt-state-trace-') as directory:
                out = Path(directory)
                env, _ = self.generate(out, 'mt-state-tasks', 4,
                                       lookahead=lookahead, direct=direct)
                (out / 'trace-harness.cpp').write_text(TRACE_HARNESS)
                exe = out / 'emu-trace'
                run(['clang++', '-std=c++17', '-O1', '-Wno-parentheses-equality',
                     '-I' + str(out), *sorted(out.glob('*.cpp')),
                     '-pthread', '-o', exe], env)
                traces[(lookahead, direct)] = run([exe], env).stdout
        baseline = traces[(0, None)]
        self.assertTrue(baseline)
        self.assertEqual(traces[(0, '1')], baseline)
        self.assertEqual(traces[(16, None)], baseline)
        self.assertEqual(traces[(16, '1')], baseline)

    def test_direct_dispatch_default_off_generation_identity(self):
        with tempfile.TemporaryDirectory(prefix='mt-direct-identity-') as directory:
            root = Path(directory)
            trees = {}
            for name, lookahead, direct in (('off-la0', 0, None),
                                            ('explicit-off-la0', 0, '0'),
                                            ('on-la0', 0, '1'),
                                            ('off-la16', 16, None),
                                            ('on-la16', 16, '1')):
                out = root / name
                out.mkdir()
                self.generate(out, 'mt-state-tasks', 4, lookahead=lookahead,
                              direct=direct)
                trees[name] = {path.name: path.read_bytes()
                               for path in sorted(out.iterdir())}
            self.assertEqual(trees['off-la0'], trees['explicit-off-la0'])
            self.assertNotEqual(trees['off-la0'], trees['on-la0'])
            self.assertEqual(trees['off-la16'], trees['on-la16'])

    def test_memory_commits_against_reference(self):
        signatures = []
        for lookahead in (0, 16):
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
