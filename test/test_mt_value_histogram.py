"""Verify GSIM_MT_VALUE_HISTOGRAM: deterministic fingerprints, safe skips, CSV.

Run after `make build-gsim`: python3 test/test_mt_value_histogram.py

The probe is emitted only when the variable is set at generation time (unset
runs must be byte-identical to the baseline) and enabled at runtime by the
same variable holding the CSV path. A skip must never change the observable
trace: the driven outputs are compared across every knob state, including a
constant-stimulus run whose whole point is to make tasks skip.
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
CYCLES = 300

# Constant stimulus after a short ramp: value gating should skip most bodies.
TRACE_HARNESS = r'''
#include "MtStateTasks.h"
#include <cstdint>
#include <cstdio>

int main() {
  SMtStateTasks dut;
  for (unsigned cycle = 0; cycle < ''' + str(CYCLES) + r'''; ++cycle) {
    bool reset = cycle < 3;
    uint8_t a = cycle < 8 ? cycle * 3 : 42, b = cycle < 8 ? cycle * 5 : 77;
    dut.set_clock(0);
    dut.set_reset(reset);
    dut.set_async_reset(cycle < 2);
    dut.set_reset_request(false);
    dut.set_enable(true);
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


def generate(out, value_histogram=None):
    env = {k: v for k, v in os.environ.items()
           if not k.startswith('GSIM_MT_')}
    env.update(GSIM_THREADS='2', GSIM_MT_EXECUTOR='dense')
    if value_histogram is not None:
        env['GSIM_MT_VALUE_HISTOGRAM'] = value_histogram
    run([GSIM, '--mt-mode=on', '--mt-target-tasks=64', '--dir', out, DESIGN], env)
    return env


def build_and_run(out, env, histogram_path=None):
    (out / 'trace-harness.cpp').write_text(TRACE_HARNESS)
    exe = out / 'emu-trace'
    run(['clang++', '-std=c++17', '-O1', '-Wno-parentheses-equality',
         '-I' + str(out), *sorted(out.glob('*.cpp')), '-pthread', '-o', exe], env)
    runtime = dict(env)
    if histogram_path is not None:
        runtime['GSIM_MT_VALUE_HISTOGRAM'] = str(histogram_path)
    result = run([exe], runtime)
    return result.stdout, result.stderr


class ValueHistogramTest(unittest.TestCase):
    def test_generation_determinism_knob_on(self):
        # The fingerprint field order follows node ids, so two instrumented
        # generations must agree byte for byte.
        with tempfile.TemporaryDirectory(prefix='mt-value-det-') as directory:
            root = Path(directory)
            trees = []
            for run_index in (0, 1):
                out = root / f'vh-r{run_index}'
                out.mkdir()
                generate(out, value_histogram=str(out / f'vh{run_index}.csv'))
                trees.append({path.name: path.read_bytes()
                              for path in sorted(out.iterdir())})
            self.assertEqual(trees[0], trees[1])

    def test_skips_preserve_trace_and_produce_histogram(self):
        with tempfile.TemporaryDirectory(prefix='mt-value-run-') as directory:
            root = Path(directory)
            baseline = root / 'baseline'
            baseline.mkdir()
            generate(baseline)
            env = {k: v for k, v in os.environ.items()
                   if not k.startswith('GSIM_MT_')}
            env.update(GSIM_THREADS='2', GSIM_MT_EXECUTOR='dense')
            baseline_trace, _ = build_and_run(baseline, env)

            probed = root / 'probed'
            probed.mkdir()
            csv_path = probed / 'value-histogram.csv'
            generate(probed, value_histogram=str(csv_path))

            # Runtime knob off: instrumentation present but inert.
            idle_trace, _ = build_and_run(probed, env)
            self.assertEqual(idle_trace, baseline_trace)

            # Runtime knob on: bodies skip, outputs must not move.
            gated_trace, stderr = build_and_run(probed, env, histogram_path=csv_path)
            self.assertEqual(gated_trace, baseline_trace)

            self.assertTrue(csv_path.exists())
            with csv_path.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 13)
            total_skipped = 0
            for row in rows:
                calls, skipped = int(row['calls']), int(row['skipped'])
                self.assertEqual(calls, CYCLES)
                self.assertGreaterEqual(skipped, 0)
                self.assertLessEqual(skipped, calls)
                self.assertIn(row['event_free'], ('0', '1'))
                self.assertIn(row['wide_input'], ('0', '1'))
                self.assertIn(row['side_effect'], ('0', '1'))
                total_skipped += skipped
            # Constant stimulus after the ramp must produce real skips.
            self.assertGreater(total_skipped, CYCLES)
            self.assertIn('[mt-value] event-free subset', stderr)
            self.assertIn('[mt-value] histogram written to', stderr)

    def test_uninstrumented_generation_ignores_runtime_knob(self):
        # The runtime variable alone must not change an uninstrumented model.
        with tempfile.TemporaryDirectory(prefix='mt-value-off-') as directory:
            root = Path(directory)
            plain = root / 'plain'
            plain.mkdir()
            generate(plain)
            env = {k: v for k, v in os.environ.items()
                   if not k.startswith('GSIM_MT_')}
            env.update(GSIM_THREADS='2', GSIM_MT_EXECUTOR='dense')
            quiet_trace, _ = build_and_run(plain, env)
            noisy_trace, _ = build_and_run(plain, env,
                                           histogram_path=root / 'unused.csv')
            self.assertEqual(quiet_trace, noisy_trace)
            self.assertFalse((root / 'unused.csv').exists())


if __name__ == '__main__':
    unittest.main()
