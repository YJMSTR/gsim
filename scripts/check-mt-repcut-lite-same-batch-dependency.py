#!/usr/bin/env python3
import json
import sys
from pathlib import Path


def fail(msg: str) -> None:
    raise SystemExit(f"mt-repcut-lite-same-batch-dependency failed: {msg}")


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: check-mt-repcut-lite-same-batch-dependency.py <model_dir>")
    model_dir = Path(sys.argv[1])
    reports = sorted(model_dir.glob("*_mt_repcut_lite.json"))
    if len(reports) != 1:
        fail(f"expected one report, found {len(reports)} under {model_dir}")
    report = json.loads(reports[0].read_text())

    reasons = {}
    failed_batches = []
    for batch in report.get("cut_batches", []):
        batch_reasons = batch.get("clone_fallback_reasons", {})
        for reason, count in batch_reasons.items():
            reasons[reason] = reasons.get(reason, 0) + count
        if batch_reasons.get("same_batch_dependency_without_clone", 0) > 0:
            failed_batches.append(batch)

    if len(failed_batches) != 1:
        fail(f"expected one same-batch dependency failed batch, found {len(failed_batches)}")
    failed_batch = failed_batches[0]
    if failed_batch.get("clone_count") != failed_batch.get("cut_edge_count", 0) - 1:
        fail(f"expected exactly one failed clone in batch, got clone_count={failed_batch.get('clone_count')} cut_edge_count={failed_batch.get('cut_edge_count')}")
    if reasons.get("multi_consumer_not_supported", 0) != 0:
        fail(f"this fixture should cover expression dependency, not node multi-consumer fallback: reasons={reasons}")

    clones = report.get("duplicated_nodes", [])
    clone_sources = {clone.get("source_node") for clone in clones}
    if "shared" not in clone_sources:
        fail(f"expected shared clone into the intermediate same-batch sink, got {clones}")
    if "b_mix" not in clone_sources:
        fail(f"expected b_mix clone into the final sink, got {clones}")
    if any(batch.get("parallel_safe") for batch in report.get("cut_batches", [])):
        fail("same-batch unresolved dependency unexpectedly parallel-safe")
    if any(task.get("runtime_applied") for task in report.get("tasks", [])):
        fail("runtime_applied should be false when same-batch dependency is rejected")

    print(f"mt-repcut-lite-same-batch-dependency ok: same_batch_dependency_without_clone={reasons.get('same_batch_dependency_without_clone', 0)} from {reports[0]}")


if __name__ == "__main__":
    main()
