#!/usr/bin/env python3
import json
import sys
from pathlib import Path


def fail(msg: str) -> None:
    raise SystemExit(f"mt-repcut-lite-multiconsumer-outside failed: {msg}")


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: check-mt-repcut-lite-multiconsumer-outside.py <model_dir>")
    model_dir = Path(sys.argv[1])
    reports = sorted(model_dir.glob("*_mt_repcut_lite.json"))
    if len(reports) != 1:
        fail(f"expected one report, found {len(reports)} under {model_dir}")
    report = json.loads(reports[0].read_text())

    reasons = {}
    parallel_batches = []
    for batch in report.get("cut_batches", []):
        for reason, count in batch.get("clone_fallback_reasons", {}).items():
            reasons[reason] = reasons.get(reason, 0) + count
        if batch.get("parallel_safe"):
            parallel_batches.append(batch)

    if reasons.get("multi_consumer_not_supported", 0) != 0:
        fail(f"outside-batch consumer was rejected as multi-consumer: reasons={reasons}")
    if not parallel_batches:
        fail(f"expected a parallel-safe cut batch, got batches={report.get('cut_batches', [])}")
    if not any(task.get("runtime_applied") for task in report.get("tasks", [])):
        fail("expected a runtime_applied sink for the accepted outside-consumer cut")

    clones = report.get("duplicated_nodes", [])
    clone_sources = {clone.get("source_node") for clone in clones}
    if "b_mix" not in clone_sources:
        fail(f"expected b_mix clone despite an outside-batch consumer, got {clones}")
    if "a_mix" not in clone_sources:
        fail(f"expected a_mix clone for the same accepted cut, got {clones}")

    tasks_by_id = {task.get("cpp_id"): task for task in report.get("tasks", [])}
    b_mix_clone = next(clone for clone in clones if clone.get("source_node") == "b_mix")
    b_mix_task = tasks_by_id.get(b_mix_clone.get("source_cpp_id"))
    if not b_mix_task:
        fail(f"missing b_mix source task for clone {b_mix_clone}")
    if b_mix_task.get("repcut_fanout", 0) <= b_mix_task.get("cut_out_edges", 0):
        fail(f"b_mix source has no extra non-cut consumer: task={b_mix_task}")

    forced_sink_ids = set()
    for batch in parallel_batches:
        forced_sink_ids.update(batch.get("forced_sink_cpp_ids", []))
    if len(forced_sink_ids) != 1:
        fail(f"expected one forced sink for this fixture, got {sorted(forced_sink_ids)}")
    if len(parallel_batches) != 1:
        fail(f"expected one parallel batch, got {parallel_batches}")
    max_parallel_end = max(batch.get("end_cpp_id", -1) for batch in parallel_batches)
    if not any(task.get("task_kind") == "serial" and task.get("cpp_id", -1) >= max_parallel_end
               for task in report.get("tasks", [])):
        fail("expected the extra tap consumer to be a serial task after every parallel cut batch")

    print(f"mt-repcut-lite-multiconsumer-outside ok: parallel_safe={len(parallel_batches)} duplicated_nodes={len(clones)} b_mix_fanout={b_mix_task.get('repcut_fanout')} from {reports[0]}")


if __name__ == "__main__":
    main()
