"""Verify GSIM_MT_COST_OVERRIDE: measured HEFT costs, missing-id fallback, neutrality.

Run after `make build-gsim`: python3 test/test_mt_cost_override.py

The knob feeds task.cost with total_ns/calls means from a
GSIM_MT_BUBBLE_TASKTIMES-style CSV before any scheduling decision reads it.
Correctness here is threefold: (1) with the knob unset the emitted model is
byte-identical to the baseline, (2) rows that exist replace the analytic cost
while tasks without a row keep it exactly, and (3) placement-only changes keep
the cycle trace identical — costs steer HEFT scoring, never semantics.
"""
import csv
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
GSIM = ROOT / 'build/gsim/gsim'
DESIGN = ROOT / 'test' / 'mt-state-tasks.fir'

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
                            capture_output=True, timeout=120)
    if result.returncode:
        raise AssertionError(f'{command} failed:\n{result.stdout}\n{result.stderr}')
    return result


def generate(out, env_extra=()):
    env = {k: v for k, v in os.environ.items()
           if not k.startswith('GSIM_MT_')}
    env.update(GSIM_THREADS='2', GSIM_MT_EXECUTOR='dense')
    env.update(env_extra)
    run([GSIM, '--mt-mode=on', '--mt-target-tasks=64', '--dir', out, DESIGN], env)
    return env


def compute_costs(directory):
    with (Path(directory) / 'mt-task-phases.csv').open() as stream:
        return {int(row['task']): int(row['cost'])
                for row in csv.DictReader(stream) if row['phase'] == 'compute'}


class CostOverrideTest(unittest.TestCase):
    def test_knob_off_is_byte_identical(self):
        with tempfile.TemporaryDirectory(prefix='mt-cost-off-') as directory:
            trees = []
            for run_index in (0, 1):
                out = Path(directory) / f'off-r{run_index}'
                out.mkdir()
                # Once with the variable absent, once set-but-empty: both must
                # reproduce the default analytic path byte for byte.
                extra = () if run_index == 0 else {'GSIM_MT_COST_OVERRIDE': ''}
                generate(out, extra)
                trees.append({path.name: path.read_bytes()
                              for path in sorted(out.iterdir())})
            self.assertEqual(trees[0], trees[1])

    def test_rows_replace_and_missing_ids_fall_back(self):
        with tempfile.TemporaryDirectory(prefix='mt-cost-on-') as directory:
            root = Path(directory)
            baseline = root / 'baseline'
            baseline.mkdir()
            generate(baseline)
            baseline_costs = compute_costs(baseline)

            # Rounded means: task0 -> 1000/10=100, task2 -> 999999/100=10000.
            # Every other task keeps its analytic cost.
            csv_path = root / 'tasktimes.csv'
            csv_path.write_text('0,10,1000\n2,100,999999\n')
            overridden = root / 'overridden'
            overridden.mkdir()
            env = generate(overridden, {'GSIM_MT_COST_OVERRIDE': str(csv_path)})
            self.assertIn('GSIM_MT_COST_OVERRIDE', env)
            overridden_costs = compute_costs(overridden)

            self.assertEqual(overridden_costs[0], 100)
            self.assertEqual(overridden_costs[2], 10000)
            kept = {task: cost for task, cost in overridden_costs.items()
                    if task not in (0, 2)}
            expected = {task: cost for task, cost in baseline_costs.items()
                        if task not in (0, 2)}
            self.assertEqual(kept, expected)

    def test_override_keeps_trace_identical(self):
        # Different costs can reshuffle placement, but the model semantics are
        # untouched: the driven trace must match the baseline byte for byte
        # across both knob states.
        with tempfile.TemporaryDirectory(prefix='mt-cost-trace-') as directory:
            root = Path(directory)
            traces = {}
            for name, extra in (('off', {}), ('on', None)):
                out = root / name
                out.mkdir()
                if extra is None:
                    csv_path = root / 'tasktimes.csv'
                    csv_path.write_text(''.join(
                        f'{task},50,{1000 + task * 37}\n' for task in range(13)))
                    generate(out, {'GSIM_MT_COST_OVERRIDE': str(csv_path)})
                else:
                    generate(out)
                (out / 'trace-harness.cpp').write_text(TRACE_HARNESS)
                exe = out / 'emu-trace'
                env = {k: v for k, v in os.environ.items()
                       if not k.startswith('GSIM_MT_')}
                env.update(GSIM_THREADS='2', GSIM_MT_EXECUTOR='dense')
                run(['clang++', '-std=c++17', '-O1',
                     '-Wno-parentheses-equality', '-I' + str(out),
                     *sorted(out.glob('*.cpp')), '-pthread', '-o', exe], env)
                traces[name] = run([exe], env).stdout
            self.assertTrue(traces['off'])
            self.assertEqual(traces['off'], traces['on'])

    def test_rheft_emits_task_id_map(self):
        # rheft reorders tasks after scheduling, so profiling ids need the
        # persisted permutation to map back onto pre-schedule indices.
        with tempfile.TemporaryDirectory(prefix='mt-cost-rheft-') as directory:
            out = Path(directory)
            env = {k: v for k, v in os.environ.items()
                   if not k.startswith('GSIM_MT_')}
            env.update(GSIM_THREADS='4', GSIM_MT_EXECUTOR='dense')
            run([GSIM, '--mt-mode=on', '--mt-target-tasks=64',
                 '--mt-scheduler=rheft', '--dir', out,
                 ROOT / 'test' / 'mt-memory-commit.fir'], env)
            with (out / 'mt-emitted-task-ids.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            tasks = compute_costs(out)
            self.assertEqual(len(rows), len(tasks))
            self.assertEqual(sorted(int(r['emitted_id']) for r in rows),
                             sorted(tasks))
            for row in rows:
                self.assertIn(int(row['pre_schedule_id']), tasks)


if __name__ == '__main__':
    unittest.main()
