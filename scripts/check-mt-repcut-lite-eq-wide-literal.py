#!/usr/bin/env python3
import json
import sys
from pathlib import Path


def fail(msg: str) -> None:
    raise SystemExit(f"mt-repcut-lite-eq-wide-literal failed: {msg}")


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: check-mt-repcut-lite-eq-wide-literal.py <model_dir>")
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
        if batch_reasons.get("unsupported_expr", 0) > 0:
            failed_batches.append(batch)

    if reasons.get("unsupported_expr", 0) <= 0:
        fail(f"missing unsupported_expr fallback for wide literal; reasons={reasons}")
    if len(failed_batches) != 1:
        fail(f"expected one unsupported wide-literal batch, found {len(failed_batches)}; reasons={reasons}")
    if any(batch.get("parallel_safe") for batch in report.get("cut_batches", [])):
        fail("wide-literal EQ unexpectedly became parallel-safe")
    if any(task.get("runtime_applied") for task in report.get("tasks", [])):
        fail("runtime_applied should be false for rejected wide-literal EQ")

    clones = report.get("duplicated_nodes", [])
    if any(clone.get("source_node") == "eq_hit" for clone in clones):
        fail(f"eq_hit clone should be rejected by wide literal guard: {clones}")
    if not any(clone.get("source_node") == "neq_hit" for clone in clones):
        fail(f"expected independent neq_hit clone before the batch is rejected: {clones}")

    print(f"mt-repcut-lite-eq-wide-literal ok: unsupported_expr={reasons.get('unsupported_expr', 0)} from {reports[0]}")


if __name__ == "__main__":
    main()
