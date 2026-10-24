"""Verify the attribution-driven replication transform (GSIM_MT_REPCUT_TOP).

Run after `make build-gsim`: python3 test/test_mt_repcut_top.py

Correctness oracle: the transformed model must be functionally identical to
the untouched model — the emitted simulator's whole observable behavior
(output signature and printf trace) may not change, at any worker count,
lookahead window, or depth-tier setting. Structural coverage is asserted on
the transform's own reports (mt-repcut-top.csv, the generate.log summary
line, mt-task-phases.csv) so the equivalence claims are never vacuous.
"""
import csv
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
GSIM = ROOT / 'build/gsim/gsim'
DESIGN = 'mt-repcut-top'


def run(command, env):
    result = subprocess.run(list(map(str, command)), env=env, text=True,
                            capture_output=True, timeout=300)
    if result.returncode:
        raise AssertionError(f'{command} failed:\n{result.stdout}\n{result.stderr}')
    return result


def model_tree(out):
    return {path.name: path.read_bytes()
            for path in sorted(out.iterdir()) if path.suffix in ('.cpp', '.h')}


class RepcutTopTest(unittest.TestCase):
    def generate(self, out, workers, lookahead, repcut=None, csv_path=None,
                 depth=None, max_cost_pct=None, cone=None, max_ops=None,
                 split=None):
        env = {k: v for k, v in os.environ.items() if not k.startswith('GSIM_MT_')}
        env.update(GSIM_THREADS=str(workers), GSIM_MT_EXECUTOR='dense')
        if repcut is not None:
            env['GSIM_MT_REPCUT_TOP'] = str(repcut)
        if depth is not None:
            env['GSIM_MT_REPCUT_DEPTH'] = str(depth)
        if csv_path is not None:
            env['GSIM_MT_REPCUT_CSV'] = str(csv_path)
        if max_cost_pct is not None:
            env['GSIM_MT_REPCUT_MAX_COST_PCT'] = str(max_cost_pct)
        if cone is not None:
            env['GSIM_MT_REPCUT_MODE'] = str(cone)
        if split is not None:
            env['GSIM_MT_REPCUT_SPLIT_PUBLISH'] = str(split)
        if max_ops is not None:
            env['GSIM_MT_REPCUT_MAX_OPS'] = str(max_ops)
        result = run([GSIM, '--mt-mode=on', '--mt-target-tasks=64',
                      '--mt-scheduler=heft', f'--mt-lookahead-window={lookahead}',
                      '--dir', out, ROOT / 'test' / f'{DESIGN}.fir'], env)
        with (Path(out) / 'mt-task-phases.csv').open() as stream:
            phases = list(csv.DictReader(stream))
        return env, result.stderr, phases

    def simulate(self, out, env):
        exe = Path(out) / 'emu-test'
        run(['clang++', '-std=c++17', '-O1', '-Wno-parentheses-equality',
             '-I' + str(out), *sorted(Path(out).glob('*.cpp')),
             ROOT / 'test' / f'{DESIGN}.cpp', '-pthread', '-o', exe], env)
        return run([exe], env).stdout

    def craft_csv(self, directory, phases, workers):
        """One attribution row per (compute task, spinner worker) pair.

        The transform structurally filters the rows, so claiming every pair
        exercises eligibility (cross-worker, ordering, cost cap) without the
        test needing to know the schedule. A neighbouring
        bubble-worker-summary.csv supplies a generous compute calibration so
        the cost cap never rejects for lack of profile data — the calibrated
        path itself is what runs.
        """
        directory.mkdir(parents=True, exist_ok=True)
        tasks = [int(row['task']) for row in phases if row['phase'] == 'compute']
        self.assertTrue(tasks)
        csv_path = directory / 'bubble-top-wait-slots.csv'
        with csv_path.open('w') as stream:
            stream.write('rank,slot_id,producer_task,producer_worker,'
                         'spinner_workers,total_ns,share_of_spin_pct,cross_ccd\n')
            rank = 0
            for task in tasks:
                for spinner in range(workers):
                    rank += 1
                    stream.write(f'{rank},{rank},{task},0,{spinner},'
                                 f'{(rank + 1) * 1000000},0.1,1\n')
        with (directory / 'bubble-worker-summary.csv').open('w') as stream:
            stream.write('worker,task_compute_ns,spin_ready_ns,total_ns\n')
            for worker in range(workers):
                # Generous compute: the cost cap stays above every replica.
                stream.write(f'{worker},1000,0,1000\n')
        return csv_path

    def test_knob_off_is_identity(self):
        # The knob-off path must be byte-identical whether the variable is
        # absent, empty or explicitly zero.
        with tempfile.TemporaryDirectory(prefix='mt-repcut-off-') as directory:
            root = Path(directory)
            trees = []
            for index, value in enumerate((None, '', '0', 'junk')):
                out = root / f'run{index}'
                out.mkdir()
                env, stderr, _ = self.generate(out, 4, 64, repcut=value)
                self.assertNotIn('repcut-top', stderr)
                trees.append(model_tree(out))
            for tree in trees[1:]:
                self.assertEqual(trees[0], tree)

    def test_replication_trace_equivalence(self):
        replicated_somewhere = False
        for workers, lookahead, depth in ((2, 0, None), (4, 0, None),
                                          (4, 64, None), (4, 64, 1),
                                          (4, 16, 1), (8, 64, None),
                                          (4, 64, 'cone'), (8, 64, 'cone'),
                                          (4, 64, 'split'), (2, 0, 'split'),
                                          (8, 64, 'split')):
            with self.subTest(workers=workers, lookahead=lookahead,
                              depth=depth), tempfile.TemporaryDirectory(
                    prefix='mt-repcut-on-') as directory:
                root = Path(directory)
                base = root / 'base'
                base.mkdir()
                env, _, phases = self.generate(base, workers, lookahead)
                baseline = self.simulate(base, env)
                self.assertTrue(baseline.strip())

                csv_path = self.craft_csv(root / 'profile', phases, workers)
                top = 4 * workers * len(
                    [row for row in phases if row['phase'] == 'compute'])
                out = root / 'repcut'
                out.mkdir()
                cone_mode = 1 if depth == 'cone' else None
                split_mode = 1 if depth == 'split' else None
                depth_arg = None if depth == 'cone' else depth
                env, stderr, on_phases = self.generate(
                    out, workers, lookahead, repcut=top, csv_path=csv_path,
                    depth=depth_arg, max_cost_pct=100, cone=cone_mode,
                    split=split_mode)
                summary = [line for line in stderr.splitlines()
                           if 'repcut-top' in line]
                self.assertEqual(len(summary), 1, summary)
                fields = dict(token.split('=', 1) for token in
                              summary[0].split() if '=' in token)
                replicas = int(fields['replicas'])
                if replicas > 0:
                    replicated_somewhere = True
                    base_count = len([row for row in phases
                                      if row['phase'] == 'compute'])
                    on_count = len([row for row in on_phases
                                    if row['phase'] == 'compute'])
                    self.assertEqual(on_count, base_count + replicas)
                    self.assertEqual(
                        int(fields['groups-killed']) +
                        int(fields['groups-residual']),
                        int(fields['consumers-rewired']))
                    decisions = (out / 'mt-repcut-top.csv').read_text()
                    self.assertTrue(',ok,' in decisions or ',cone-ok,' in decisions, decisions)
                    self.assertIn('repcut-tokens-net=',
                                  stderr.splitlines()[-2] + stderr.splitlines()[-1])
                    self.assertNotEqual(model_tree(base), model_tree(out))
                self.assertEqual(self.simulate(out, env), baseline)
        self.assertTrue(replicated_somewhere,
                        'no configuration produced a replica; the fixture '
                        'or the eligibility rules made the test vacuous')

    def test_knob_on_determinism(self):
        with tempfile.TemporaryDirectory(prefix='mt-repcut-det-') as directory:
            root = Path(directory)
            base = root / 'base'
            base.mkdir()
            _, _, phases = self.generate(base, 4, 64)
            csv_path = self.craft_csv(root / 'profile', phases, 4)
            trees = []
            for index in range(2):
                out = root / f'on{index}'
                out.mkdir()
                self.generate(out, 4, 64, repcut=999, csv_path=csv_path,
                              max_cost_pct=100)
                trees.append(model_tree(out))
            self.assertEqual(trees[0], trees[1])



    def test_value_trace_localizer(self):
        # The value-fingerprint localizer must report identical cycle hashes
        # for functionally equivalent models: a traced knob-off and a traced
        # knob-on generation of the same design must emit byte-equal VTRACE
        # streams, and repeated runs must be self-consistent.
        with tempfile.TemporaryDirectory(prefix='mt-repcut-vt-') as directory:
            root = Path(directory)
            traces = []
            for name, repcut in (('off', None), ('on', 999)):
                out = root / name
                out.mkdir()
                env = {k: v for k, v in os.environ.items()
                       if not k.startswith('GSIM_MT_')}
                env.update(GSIM_THREADS='4', GSIM_MT_EXECUTOR='dense',
                           GSIM_MT_VALUE_TRACE='1')
                if repcut is not None:
                    env['GSIM_MT_REPCUT_TOP'] = str(repcut)
                    env['GSIM_MT_REPCUT_MAX_COST_PCT'] = '100'
                run([GSIM, '--mt-mode=on', '--mt-target-tasks=64',
                     '--mt-scheduler=heft', '--mt-lookahead-window=64',
                     '--dir', out, ROOT / 'test' / f'{DESIGN}.fir'], env)
                _, _, phases = None, None, None
                exe = out / 'emu-test'
                run(['clang++', '-std=c++17', '-O1', '-Wno-parentheses-equality',
                     '-I' + str(out), *sorted(out.glob('*.cpp')),
                     ROOT / 'test' / f'{DESIGN}.cpp', '-pthread', '-o', exe], env)
                result = run([exe], env)
                lines = [line for line in result.stderr.splitlines()
                         if line.startswith('VTRACE ')]
                self.assertTrue(lines)
                traces.append(lines)
            self.assertEqual(traces[0], traces[1])


if __name__ == '__main__':
    unittest.main()
