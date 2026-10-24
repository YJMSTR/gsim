"""Verify GSIM_MT_UNIFORM_FILL: byte-identity knob-off, trace equivalence knob-on, merge rejections.

Run after `make build-gsim`: python3 test/test_mt_uniform_fill.py

The run merge collapses consecutive single-line same-base same-rhs
consecutive-index element stores (>=8) into one loop AFTER every scheduling
decision, so the knob may change emitted body shape but never placement,
task count, or traces. Rejections (when-block marker between fills,
self-referencing base, index gaps below the minimum run) must keep the
unrolled form.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
GSIM = ROOT / 'build/gsim/gsim'
DESIGN = ROOT / 'test' / 'mt-uniform-fill.fir'

TRACE_HARNESS = r'''
#include "UFTest.h"
#include <cstdint>
#include <cstdio>

int main() {
  SUFTest dut;
  for (unsigned cycle = 0; cycle < 128; ++cycle) {
    dut.set_clock(cycle & 1);
    dut.set_reset(cycle < 2);
    dut.set_en(cycle % 3 == 0);
    dut.set_a(static_cast<uint8_t>(cycle * 7 + 1));
    dut.set_idx(static_cast<uint8_t>(cycle % 16));
    dut.step();
    std::printf("%u %u %u\n", cycle, dut.get_out(), dut.get_out2());
  }
}
'''


def run(command, env):
    result = subprocess.run(list(map(str, command)), env=env, text=True,
                            capture_output=True, timeout=120)
    if result.returncode:
        raise AssertionError(f'{command} failed:\n{result.stdout}\n{result.stderr}')
    return result


def generate(out, fill=False):
    env = {k: v for k, v in os.environ.items()
           if not k.startswith('GSIM_MT_')}
    env.update(GSIM_THREADS='2', GSIM_MT_EXECUTOR='dense')
    if fill:
        env['GSIM_MT_UNIFORM_FILL'] = '1'
    run([GSIM, '--mt-mode=on', '--mt-target-tasks=64', '--dir', out, DESIGN], env)
    return env


def model_text(out):
    return ''.join(p.read_text(errors='ignore')
                   for p in sorted(Path(out).glob('*.cpp')))


class UniformFillTest(unittest.TestCase):
    def test_knob_off_deterministic_and_unmerged(self):
        with tempfile.TemporaryDirectory(prefix='mt-uf-off-') as directory:
            trees = []
            for run_index in (0, 1):
                out = Path(directory) / f'off-r{run_index}'
                out.mkdir()
                generate(out, fill=False)
                trees.append({p.name: p.read_bytes() for p in sorted(out.iterdir())})
            self.assertEqual(trees[0], trees[1])
            self.assertNotIn('__gsim_uf_i', ''.join(
                p.read_text(errors='ignore') for p in sorted(trees and Path(directory, 'off-r0').glob('*.cpp'))))

    def test_knob_on_merges_run_and_keeps_trace(self):
        with tempfile.TemporaryDirectory(prefix='mt-uf-on-') as directory:
            root = Path(directory)
            off, on = root / 'off', root / 'on'
            off.mkdir(); on.mkdir()
            generate(off, fill=False)
            generate(on, fill=True)
            # the 12-element zero run must fold into one loop; the en-gated
            # 4-element tail run stays unrolled (below the minimum run of 8)
            text_on = model_text(on)
            self.assertIn('__gsim_uf_i0 = 0; __gsim_uf_i0 < 12', text_on)
            self.assertIn('tmp[__gsim_uf_i0] = 0x0;', text_on)
            self.assertNotIn('__gsim_uf_i0 < 4', text_on)
            # task count and placement identical: same phase rows, same owners
            def owners(out):
                import csv
                with (out / 'mt-task-phases.csv').open() as stream:
                    return [(r['phase'], r['task'], r['worker'])
                            for r in csv.DictReader(stream)]
            self.assertEqual(owners(off), owners(on))

            traces = {}
            for name, out in (('off', off), ('on', on)):
                (out / 'trace-harness.cpp').write_text(TRACE_HARNESS)
                exe = out / 'emu-trace'
                env = {k: v for k, v in os.environ.items()
                       if not k.startswith('GSIM_MT_')}
                env.update(GSIM_THREADS='2', GSIM_MT_EXECUTOR='dense')
                run(['clang++', '-std=c++17', '-O1', '-Wno-parentheses-equality',
                     '-I' + str(out), *sorted(out.glob('*.cpp')), '-pthread',
                     '-o', exe], env)
                traces[name] = run([exe], env).stdout
            self.assertTrue(traces['off'])
            self.assertEqual(traces['off'], traces['on'])

    def test_rejections_keep_unrolled_form(self):
        # marker-intervened: the when block splits the zero run at arr[11],
        # leaving runs of 12 (0..11) and 4 (12..15 gated); the marker itself
        # is a different statement shape, and any candidate whose rhs names
        # the destination base bails. We assert via a self-referencing
        # variant design constructed inline.
        self_reffing = DESIGN.with_name('mt-uniform-fill.fir').read_text()
        # Build a self-reference variant: arr[i] fed from arr[0]-derived rhs.
        variant = self_reffing.replace('connect tmp[0], UInt<8>(0)',
                                       'connect tmp[0], tmp[1]')
        with tempfile.TemporaryDirectory(prefix='mt-uf-rej-') as directory:
            out = Path(directory) / 'v'
            out.mkdir()
            fir = out / 'v.fir'
            fir.write_text(variant)
            env = {k: v for k, v in os.environ.items()
                   if not k.startswith('GSIM_MT_')}
            env.update(GSIM_THREADS='2', GSIM_MT_EXECUTOR='dense',
                       GSIM_MT_UNIFORM_FILL='1')
            run([GSIM, '--mt-mode=on', '--mt-target-tasks=64',
                 '--dir', out, fir], env)
            text = model_text(out)
            # tmp[1..11] still share rhs 0 and can merge, but tmp[0]'s rhs
            # names the destination base, so its run never opens; no loop may
            # start at index 0.
            self.assertNotIn('__gsim_uf_i0 = 0; __gsim_uf_i0 < 12', text)


if __name__ == '__main__':
    unittest.main()
