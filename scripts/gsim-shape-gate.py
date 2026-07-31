#!/usr/bin/env python3
"""gsim shape/perf gate for the deliverable pipeline.

Guarantee without determinism: every generated model must pass
  1. structural shape check vs the champion registry (fingerprint exact OR metrics within tolerance), and
  2. NEMU correctness gate (runbook step), and
  3. measured-perf check vs the champion artifact in the SAME session (runbook step).

Usage:
  gsim-shape-gate.py <candidate_model_dir> [--registry <registry.json>] [--register]
    --registry: champion registry to compare against (default: champions/xiangshan-t32-v371-lookahead/registry.json relative to this script's workspace root)
    --register: write the candidate's shape as a NEW registry entry (for a new RTL/recipe baseline)

Exit 0 = structural PASS, 1 = FAIL (prints metrics), 2 = usage/data error.
"""
import json
import os
import sys

FINGERPRINT_FIELDS = ('edges', 'layers')


def load_shape(model_dir):
    path = os.path.join(model_dir, 'SimTop_mt_dense_schedule.json')
    if not os.path.exists(path):
        alt = os.path.join(model_dir, 'model', 'SimTop_mt_dense_schedule.json')
        path = alt if os.path.exists(alt) else path
    if not os.path.exists(path):
        return None, path
    d = json.load(open(path))
    a = d.get('dense_assignment_current', {})
    return {
        'dense_mtask_count': d.get('dense_mtask_count'),
        'dense_worker0_only_mtask_count': d.get('dense_worker0_only_mtask_count'),
        'dense_worker0_contaminated_mtask_count': d.get('dense_worker0_contaminated_mtask_count'),
        'dense_runtime_dependency_edge_count': d.get('dense_runtime_dependency_edge_count'),
        'predicted_makespan': a.get('predicted_makespan'),
        'fingerprint_payload': {
            'edges': d.get('edges'), 'layers': d.get('layers'),
            'assignment': d.get('dense_assignment_current'),
            'worker_mtask_counts': d.get('dense_worker_mtask_counts'),
            'worker_static_costs': d.get('dense_worker_static_costs'),
            'mtask_count': d.get('dense_mtask_count'),
            'dag_edge_count': d.get('dense_mtask_edge_count'),
        },
    }, path


def fingerprint(payload):
    import hashlib
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()[:24]


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 2
    model_dir = args[0]
    registry_path = None
    register = False
    i = 1
    while i < len(args):
        if args[i] == '--registry':
            registry_path = args[i + 1]
            i += 2
        elif args[i] == '--register':
            register = True
            i += 1
        else:
            print(f'unknown argument: {args[i]}')
            return 2
    if registry_path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        registry_path = os.path.join(here, '..', 'champions', 'xiangshan-t32-v371-lookahead', 'registry.json')

    shape, sched_path = load_shape(model_dir)
    if shape is None:
        print(f'FAIL: no schedule JSON found at {sched_path}')
        return 2

    if register:
        reg_dir = os.path.dirname(registry_path)
        os.makedirs(reg_dir, exist_ok=True)
        reg = {
            'shape': {k: shape[k] for k in shape if k != 'fingerprint_payload'},
            'fingerprint': fingerprint(shape['fingerprint_payload']),
        }
        with open(registry_path, 'w') as f:
            json.dump(reg, f, indent=1, ensure_ascii=False)
        print(f'REGISTERED new champion baseline at {registry_path} (fingerprint {reg["fingerprint"]})')
        return 0

    if not os.path.exists(registry_path):
        print(f'FAIL: registry not found at {registry_path}; run with --register to create a baseline')
        return 2
    reg = json.load(open(registry_path))
    ref_fp = reg.get('fingerprint')
    ref_shape = reg.get('shape', {})
    tol = reg.get('gate', {}).get('structural_tolerance', {})
    cand_fp = fingerprint(shape['fingerprint_payload'])

    if cand_fp == ref_fp:
        print(f'PASS (fingerprint exact): {cand_fp}')
        return 0

    failures = []
    mtol = tol.get('mtask_count_pct', 3.0)
    wtol = tol.get('worker0_only_pct', 15.0)
    ref_m = ref_shape.get('dense_mtask_count')
    ref_w = ref_shape.get('dense_worker0_only_mtask_count')
    if ref_m is not None and abs(shape['dense_mtask_count'] - ref_m) > ref_m * mtol / 100.0:
        failures.append(f'mtask_count {shape["dense_mtask_count"]} vs {ref_m} (> {mtol}%)')
    if ref_w is not None and abs(shape['dense_worker0_only_mtask_count'] - ref_w) > max(8, ref_w * wtol / 100.0):
        failures.append(f'worker0_only {shape["dense_worker0_only_mtask_count"]} vs {ref_w} (> {wtol}%)')
    if shape['dense_worker0_contaminated_mtask_count'] != tol.get('contamination', 0):
        failures.append(f'worker0 contamination {shape["dense_worker0_contaminated_mtask_count"]} != 0')

    if failures:
        print(f'FAIL (fingerprint mismatch {cand_fp} vs {ref_fp}):')
        for f in failures:
            print(f'  - {f}')
        print('action: regenerate (new epoch) or fall back to the champion artifact per registry.gate.on_failure')
        return 1
    print(f'PASS (fingerprint mismatch but metrics within tolerance): {cand_fp} vs {ref_fp}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
