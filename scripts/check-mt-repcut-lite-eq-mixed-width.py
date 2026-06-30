#!/usr/bin/env python3
import json
import sys
from pathlib import Path


def fail(msg: str) -> None:
    raise SystemExit(f"mt-repcut-lite-eq-mixed-width failed: {msg}")


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: check-mt-repcut-lite-eq-mixed-width.py <model_dir>")
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
        if batch_reasons.get("compare_width_unsupported", 0) > 0:
            failed_batches.append(batch)

    if len(failed_batches) != 1:
        fail(f"expected one compare-width failed batch, found {len(failed_batches)}; reasons={reasons}")
    if reasons.get("compare_width_unsupported", 0) <= 0:
        fail(f"missing compare_width_unsupported fallback; reasons={reasons}")
    if any(batch.get("parallel_safe") for batch in report.get("cut_batches", [])):
        fail("mixed-width EQ unexpectedly became parallel-safe")
    if any(task.get("runtime_applied") for task in report.get("tasks", [])):
        fail("runtime_applied should be false for rejected mixed-width EQ")

    clones = report.get("duplicated_nodes", [])
    if any(clone.get("source_node") == "eq_hit" for clone in clones):
        fail(f"eq_hit clone should be rejected by compare-width guard: {clones}")
    if not any(clone.get("source_node") == "neq_hit" for clone in clones):
        fail(f"expected independent neq_hit clone before the batch is rejected: {clones}")

    failed = failed_batches[0]
    if failed.get("clone_count") != failed.get("cut_edge_count", 0) - 1:
        fail(f"expected exactly one failed clone in batch, got clone_count={failed.get('clone_count')} cut_edge_count={failed.get('cut_edge_count')}")

    print(f"mt-repcut-lite-eq-mixed-width ok: compare_width_unsupported={reasons.get('compare_width_unsupported', 0)} from {reports[0]}")


if __name__ == "__main__":
    main()
