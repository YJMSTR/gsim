#!/usr/bin/env python3
import json
import sys
from pathlib import Path


def fail(msg: str) -> None:
    raise SystemExit(f"mt-repcut-lite-same-source-dep failed: {msg}")


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: check-mt-repcut-lite-same-source-dep.py <model_dir>")
    model_dir = Path(sys.argv[1])
    reports = sorted(model_dir.glob("*_mt_repcut_lite.json"))
    if len(reports) != 1:
        fail(f"expected one report, found {len(reports)} under {model_dir}")
    report = json.loads(reports[0].read_text())
    if report.get("cut_edge_count", 0) <= 0:
        fail(f"expected cut_edge_count > 0, got {report.get('cut_edge_count', 0)}")
    reasons = {}
    for batch in report.get("cut_batches", []):
        for reason, count in batch.get("clone_fallback_reasons", {}).items():
            reasons[reason] = reasons.get(reason, 0) + count
    failed_batches = [batch for batch in report.get("cut_batches", [])
                      if batch.get("clone_fallback_reasons", {}).get("same_source_dependency_without_clone", 0) > 0]
    if len(failed_batches) != 1:
        fail(f"expected one same-source failed batch, found {len(failed_batches)}")
    failed_batch = failed_batches[0]
    if failed_batch.get("clone_count") != failed_batch.get("cut_edge_count", 0) - 1:
        fail(f"expected exactly one failed clone in batch, got clone_count={failed_batch.get('clone_count')} cut_edge_count={failed_batch.get('cut_edge_count')}")
    clones = report.get("duplicated_nodes", [])
    if not any(clone.get("source_node") == "b_mix" for clone in clones):
        fail(f"expected b_mix clone, got {clones}")
    if any(clone.get("source_node") == "source_value" for clone in clones):
        fail(f"source_value clone should be rejected by same-source guard: {clones}")
    if reasons.get("same_source_dependency_without_clone", 0) <= 0:
        fail(f"missing same_source_dependency_without_clone fallback; reasons={reasons}")
    if any(batch.get("parallel_safe") for batch in report.get("cut_batches", [])):
        fail("same-source non-local dependency unexpectedly parallel-safe")
    if any(task.get("runtime_applied") for task in report.get("tasks", [])):
        fail("runtime_applied should be false for rejected same-source dependency")
    print(f"mt-repcut-lite-same-source-dep ok: same_source_dependency_without_clone={reasons.get('same_source_dependency_without_clone', 0)} from {reports[0]}")


if __name__ == "__main__":
    main()
