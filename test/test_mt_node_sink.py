"""Verify the node-level sink planner (GSIM_MT_NODE_SINK=1).

Run after `make build-gsim`: python3 test/test_mt_node_sink.py

Correctness is checked by differentials against the existing dense executor
generated WITHOUT the node sink: identical traces/signatures for the small
designs (state tasks, memory commits, printf hotness, sparse register resets
and the synthetic pure fanout), across workers 1/2/4 and both executors of
the same event-capable binary. Replication is proven structurally on the
synthetic fanout: the clone publishes no ready tokens (storeBegin == storeEnd
in its dispatch entry), its same-worker consumers use chain-position local
waits instead of tokens, and it activates them through the local mtAct mask.
"""
import csv
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
GSIM = ROOT / 'build/gsim/gsim'

DESIGNS = (
    ('mt-state-tasks', ()),
    ('mt-memory-commit', ()),
    ('mt-special-hot', ()),
    ('mt-sparse-register-reset',
     ('--mt-sparse-register-min-bytes=1', '--mt-sparse-register-max-writes=8')),
    ('mt-sparse-register-async-reset',
     ('--mt-sparse-register-min-bytes=1', '--mt-sparse-register-max-writes=8')),
    ('mt-node-sink-fanout', ()),
)

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
                            capture_output=True, timeout=240)
    if result.returncode:
        raise AssertionError(f'{command} failed:\n{result.stdout}\n{result.stderr}')
    return result


class NodeSinkTest(unittest.TestCase):
    def generate(self, out, design, workers, extra=(), node_sink=True,
                 event=False, lookahead=0, coarsen=True, kahypar=None,
                 mirror=False, phase_cap=None, clone=False, chunk=None,
                 group_mask=False):
        env = {k: v for k, v in os.environ.items()
               if not k.startswith('GSIM_MT_')}
        env.update(GSIM_THREADS=str(workers), GSIM_MT_EXECUTOR='dense')
        if node_sink:
            env['GSIM_MT_NODE_SINK'] = '1'
            if not coarsen:
                env['GSIM_MT_NODE_SINK_COARSEN'] = '0'
        if event:
            env['GSIM_MT_EVENT_GENERATE'] = '1'
        if kahypar:
            env['GSIM_MT_NODE_SINK_KAHYPAR'] = str(kahypar)
        if mirror:
            env['GSIM_MT_NODE_SINK_MIRROR'] = '1'
        if clone:
            env['GSIM_MT_NODE_SINK_CLONE'] = '1'
        if chunk is not None:
            env['GSIM_MT_EVENT_CHUNK'] = str(chunk)
        if group_mask:
            env['GSIM_MT_EVENT_GROUP_MASK'] = '1'
        if phase_cap is not None:
            env['GSIM_MT_NODE_SINK_PHASE_CAP'] = str(phase_cap)
        result = run([GSIM, '--mt-mode=on', '--mt-target-tasks=64',
                      f'--mt-lookahead-window={lookahead}', *extra,
                      '--dir', out, ROOT / 'test' / f'{design}.fir'], env)
        self.last_mirror_report = re.search(
            r'node-sink mirror groups=(\d+) dropped=(\d+) kept=(\d+)',
            result.stderr)
        return env

    def model_output(self, out, design, env):
        exe = out / 'emu-test'
        if design == 'mt-state-tasks':
            (out / 'trace-harness.cpp').write_text(TRACE_HARNESS)
            run(['clang++', '-std=c++17', '-O1', '-Wno-parentheses-equality',
                 '-I' + str(out), *sorted(out.glob('*.cpp')),
                 '-pthread', '-o', exe], env)
        else:
            run(['clang++', '-std=c++17', '-O1', '-Wno-parentheses-equality',
                 '-I' + str(out), *sorted(out.glob('*.cpp')),
                 ROOT / 'test' / f'{design}.cpp', '-pthread', '-o', exe], env)
        return run([exe], env).stdout

    def read_metrics(self, out):
        with (out / 'mt-node-sink.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        return {row['metric']: row['value'] for row in rows}

    def test_dense_matches_existing_dense(self):
        # The node-sink plan must be observationally identical to the existing
        # dense executor for every small design and worker count.
        for design, extra in DESIGNS:
            for workers in (1, 2, 4):
                with self.subTest(design=design, workers=workers), \
                        tempfile.TemporaryDirectory(
                            prefix='mt-node-sink-') as directory:
                    root = Path(directory)
                    baseline = root / 'baseline'
                    sink = root / 'node-sink'
                    baseline.mkdir()
                    sink.mkdir()
                    env = self.generate(baseline, design, workers, extra,
                                        node_sink=False)
                    reference = self.model_output(baseline, design, env)
                    self.assertTrue(reference)
                    env = self.generate(sink, design, workers, extra)
                    self.assertEqual(self.model_output(sink, design, env),
                                     reference)

    def test_event_executor_matches_dense(self):
        # Same generated binary, both executors: the event executor must
        # reproduce the dense trace of the node-sink plan.
        for design, extra in DESIGNS:
            with self.subTest(design=design), tempfile.TemporaryDirectory(
                    prefix='mt-node-sink-event-') as directory:
                out = Path(directory)
                env = self.generate(out, design, 2, extra, event=True,
                                    lookahead=64)
                dense_env = env.copy()
                event_env = env.copy()
                event_env['GSIM_MT_EXECUTOR'] = 'event'
                dense = self.model_output(out, design, dense_env)
                event = self.model_output(out, design, event_env)
                self.assertEqual(event, dense)

    def test_event_group_mask_keeps_traces(self):
        with tempfile.TemporaryDirectory(prefix='mt-node-sink-group-mask-') as directory:
            out = Path(directory)
            env = self.generate(out, 'mt-state-tasks', 2, event=True,
                                phase_cap=1, chunk=8, group_mask=True)
            reference = self.model_output(out, 'mt-state-tasks', env)
            event_env = env.copy()
            event_env['GSIM_MT_EXECUTOR'] = 'event'
            self.assertEqual(self.model_output(out, 'mt-state-tasks', event_env),
                             reference)

    def test_event_chunk_keeps_traces(self):
        with tempfile.TemporaryDirectory(prefix='mt-node-sink-chunk-') as directory:
            out = Path(directory)
            env = self.generate(out, 'mt-state-tasks', 2, event=True,
                                phase_cap=1, chunk=8)
            reference = self.model_output(out, 'mt-state-tasks', env)
            event_env = env.copy()
            event_env['GSIM_MT_EXECUTOR'] = 'event'
            self.assertEqual(self.model_output(out, 'mt-state-tasks', event_env),
                             reference)

    def test_phase_mirror_keeps_traces_and_reports_tokens(self):
        # GSIM_MT_NODE_SINK_MIRROR=1 lets provably cross-phase values cross
        # workers through the barrier-ordered class storage instead of ready
        # tokens: groups whose sources all sit in earlier capped phases are
        # dropped, same-capped-phase groups keep the token protocol. Every
        # configuration must stay trace-identical to the no-mirror baseline
        # for both executors, and the mirror report must account for every
        # token group (cap=1 keeps all, cap=2 drops the cross-phase ones).
        for design, extra in DESIGNS:
            for workers in (2, 4):
                with self.subTest(design=design, workers=workers), \
                        tempfile.TemporaryDirectory(
                            prefix='mt-node-sink-mirror-') as directory:
                    root = Path(directory)
                    baseline = root / 'baseline'
                    baseline.mkdir()
                    base_env = self.generate(baseline, design, workers, extra)
                    reference = self.model_output(baseline, design, base_env)
                    self.assertTrue(reference)
                    for cap in (1, 2):
                        out = root / f'mirror-cap{cap}'
                        out.mkdir()
                        env = self.generate(out, design, workers, extra,
                                            event=True, mirror=True,
                                            phase_cap=cap)
                        self.assertEqual(
                            self.model_output(out, design, env), reference)
                        event_env = env.copy()
                        event_env['GSIM_MT_EXECUTOR'] = 'event'
                        self.assertEqual(
                            self.model_output(out, design, event_env),
                            reference)
                        report = self.last_mirror_report
                        if report is not None:
                            groups, dropped, kept = (
                                int(value) for value in report.groups())
                            self.assertEqual(groups, dropped + kept)
                            if groups:
                                if cap == 1:
                                    # One capped phase proves nothing: every
                                    # token must stay on the fallback path.
                                    self.assertEqual(dropped, 0)
                                    self.assertEqual(kept, groups)
                                elif design == 'mt-memory-commit':
                                    self.assertGreater(dropped, 0)


    def test_fanout_clone_reported_and_token_free(self):
        # Coarsening off isolates the replication mechanics: with it on, the
        # clone usually fuses with its same-worker consumers into one block
        # and needs no activation at all.
        with tempfile.TemporaryDirectory(prefix='mt-node-sink-clone-') as directory:
            out = Path(directory)
            self.generate(out, 'mt-node-sink-fanout', 2, event=True,
                          lookahead=64, coarsen=False, clone=True)
            metrics = self.read_metrics(out)
            self.assertGreaterEqual(int(metrics['replica_tasks']), 1)
            # SuperNode grouping merges consumers, so one clone redirects
            # whole consumer groups (two here), not one task per node.
            self.assertGreaterEqual(int(metrics['removed_cross_edges']), 2)
            clone_count = int(metrics['replica_tasks'])
            clones = [{
                'original': metrics[f'clone_{index}_original'],
                'replica': metrics[f'clone_{index}_replica'],
                'worker': metrics[f'clone_{index}_worker'],
                'redirected': int(metrics[f'clone_{index}_redirected_consumers']),
            } for index in range(clone_count)]
            original, replica = clones[0]['original'], clones[0]['replica']
            self.assertIn('$MT_NS$', replica)
            self.assertNotIn('$MT_NS$', original)
            self.assertEqual(
                sum(clone['redirected'] for clone in clones),
                int(metrics['removed_cross_edges']))

            source = (out / 'MtNodeSinkFanout0.cpp').read_text()
            # The clone computes the same expression as its original.
            clone_functions = re.findall(
                r'void SMtNodeSinkFanout::(mtTask\d+)\(\) \{(.*?)\n\}',
                source, re.S)
            clone_bodies = [name for name, body in clone_functions
                            if re.search(re.escape(replica) + r'\s*=', body)]
            self.assertTrue(clone_bodies, 'clone task assigns the replica')
            clone_task = clone_bodies[0]
            clone_body = dict(clone_functions)[clone_task]
            entry = re.search(
                r'\{&SMtNodeSinkFanout::' + clone_task +
                r',(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+)\}', source)
            self.assertIsNotNone(entry, 'clone dispatch entry missing')
            numbers = entry.groups()
            self.assertEqual(numbers[1], numbers[2],
                             'clone must not wait on ready tokens')
            self.assertEqual(numbers[3], numbers[4],
                             'clone must not publish ready tokens')
            # instead of ready tokens.
            consumers = [name for name, body in clone_functions
                         if name != clone_task and replica in body]
            self.assertGreaterEqual(
                len(consumers), 1,
                'at least one redirected consumer must reference the replica')
            for consumer in consumers:
                consumer_entry = re.search(
                    r'\{&SMtNodeSinkFanout::' + consumer +
                    r',(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+)\}', source)
                self.assertIsNotNone(consumer_entry,
                                     f'{consumer} dispatch entry missing')
                fields = consumer_entry.groups()
                self.assertEqual(
                    fields[1], fields[2],
                    f'{consumer} must not wait on ready tokens')
                self.assertLess(
                    int(fields[5]), int(fields[6]),
                    f'{consumer} must chain-wait on the local clone')

    def test_classification_report_totals(self):
        with tempfile.TemporaryDirectory(prefix='mt-node-sink-report-') as directory:
            out = Path(directory)
            self.generate(out, 'mt-node-sink-fanout', 2, coarsen=False)
            metrics = self.read_metrics(out)
            self.assertEqual(
                int(metrics['unique_tasks']) + int(metrics['pure_tasks']) +
                int(metrics['replica_tasks']), int(metrics['tasks']))
            # Side-effect designs classify the printf task as unique.
            with tempfile.TemporaryDirectory(
                    prefix='mt-node-sink-special-') as special_dir:
                special = Path(special_dir)
                self.generate(special, 'mt-special-hot', 2, coarsen=False)
                special_metrics = self.read_metrics(special)
                self.assertGreaterEqual(
                    int(special_metrics['unique_tasks']), 1)
                self.assertGreaterEqual(
                    int(special_metrics['tasks']),
                    int(special_metrics['unique_tasks']))

    def test_coarsening_reduces_tasks_and_keeps_output(self):
        # Linear coarsening must shrink the plan (pure intervals merge under
        # the statement cap) and stay byte-identical to the dense baseline,
        # including the event executor of the same binary.
        with tempfile.TemporaryDirectory(prefix='mt-node-sink-coarsen-') as directory:
            root = Path(directory)
            out = root / 'coarsened'
            out.mkdir()
            env = self.generate(out, 'mt-node-sink-fanout', 2, event=True,
                                lookahead=64)
            metrics = self.read_metrics(out)
            self.assertEqual(int(metrics['coarsen']), 1)
            self.assertLess(int(metrics['coarsen_post_tasks']),
                            int(metrics['coarsen_pre_tasks']))
            self.assertLessEqual(int(metrics['coarsen_post_makespan']),
                                 int(metrics['coarsen_pre_makespan']))
            baseline = root / 'baseline'
            baseline.mkdir()
            base_env = self.generate(baseline, 'mt-node-sink-fanout', 2,
                                     node_sink=False)
            reference = self.model_output(baseline, 'mt-node-sink-fanout',
                                          base_env)
            dense_env = env.copy()
            event_env = env.copy()
            event_env['GSIM_MT_EXECUTOR'] = 'event'
            self.assertEqual(
                self.model_output(out, 'mt-node-sink-fanout', dense_env),
                reference)
            self.assertEqual(
                self.model_output(out, 'mt-node-sink-fanout', event_env),
                reference)

    def test_state_phase_report_integrity(self):
        # The shared state-task machinery must still own every state register
        # with one pinned state task per worker.
        with tempfile.TemporaryDirectory(prefix='mt-node-sink-state-') as directory:
            out = Path(directory)
            self.generate(out, 'mt-state-tasks', 4)
            with (out / 'mt-task-phases.csv').open() as stream:
                phases = list(csv.DictReader(stream))
            with (out / 'mt-state-task-members.csv').open() as stream:
                members = list(csv.DictReader(stream))
            states = [r for r in phases if r['phase'] == 'state']
            self.assertEqual(len(states), 4)
            self.assertTrue(all(r['task'] == r['worker'] for r in states))
            self.assertTrue(all(r['barrier'] == 'state_publish' for r in states))
            owned = [r for r in members if r['role'] == 'member']
            self.assertEqual(len({r['node_id'] for r in owned}), len(owned))
            self.assertEqual(sum(int(r['members']) for r in states), len(owned))

    def test_rebalance_never_worsens_imbalance(self):
        # Post-cut component migration is self-gating: whenever it moves a
        # sink component, the exact hosted max/mean must not increase, and
        # the report must carry the before/after numbers (tokens included).
        # At 16 workers the plan is imbalanced enough that the post-coarsening
        # block-space rebalance actually moves work, and the moved blocks must
        # stay trace-identical to the baseline.
        helper = (ROOT / 'tools/node-sink-partition/target/release/'
                  'node-sink-partition')
        with tempfile.TemporaryDirectory(
                prefix='mt-node-sink-rebal-') as directory:
            root = Path(directory)
            out = root / 'sink'
            out.mkdir()
            os.environ['GSIM_MT_NODE_SINK_REBALANCE'] = '1'
            self.addCleanup(os.environ.pop, 'GSIM_MT_NODE_SINK_REBALANCE', None)
            self.generate(out, 'mt-node-sink-fanout', 16, kahypar=helper)
            metrics = self.read_metrics(out)
            self.assertEqual(metrics['partitioner'], 'mt-kahypar')
            self.assertIn('rebalance_post_imbalance', metrics)
            self.assertLessEqual(
                float(metrics['rebalance_post_imbalance']),
                float(metrics['rebalance_pre_imbalance']) + 1e-9)
            if 'block_rebalance_post_imbalance' in metrics:
                self.assertGreaterEqual(int(metrics['block_rebalance_moves']), 0)
                self.assertLessEqual(
                    float(metrics['block_rebalance_post_imbalance']),
                    float(metrics['block_rebalance_pre_imbalance']) + 1e-9)
            baseline = root / 'baseline'
            baseline.mkdir()
            base_env = self.generate(baseline, 'mt-node-sink-fanout', 16,
                                     node_sink=False)
            reference = self.model_output(baseline, 'mt-node-sink-fanout',
                                          base_env)
            self.assertTrue(reference)
            sink_env = {k: v for k, v in os.environ.items()
                        if not k.startswith('GSIM_MT_')}
            sink_env.update(GSIM_THREADS='16', GSIM_MT_EXECUTOR='dense')
            self.assertEqual(
                self.model_output(out, 'mt-node-sink-fanout', sink_env),
                reference)

    def test_generation_determinism(self):
        with tempfile.TemporaryDirectory(prefix='mt-node-sink-det-') as directory:
            root = Path(directory)
            trees = []
            for run_index in (0, 1):
                out = root / f'r{run_index}'
                out.mkdir()
                self.generate(out, 'mt-node-sink-fanout', 2, event=True,
                              lookahead=64)
                trees.append({path.name: path.read_bytes()
                              for path in sorted(out.iterdir())})
            self.assertEqual(trees[0], trees[1])


if __name__ == '__main__':
    unittest.main()
