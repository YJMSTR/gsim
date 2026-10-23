#!/usr/bin/env python3
"""Cone-activity probe report: aggregates the probe CSV, per-task bubble
timings and wall-clock samples into analysis/cone-activity-<workload>.json.

Inputs (produced by /tmp/cone-window style runs):
  --cone      task_id,calls,would_skip,event_free,hot_cone,cone_words
  --bits      bit,sets                       (optional, copied into summary)
  --times     task_id,calls,task_ns          (GSIM_MT_BUBBLE_TASKTIMES csv)
  --wall-off  / --wall-on                    seconds, one per line
"""
import argparse
import json
import sys


def read_wall(paths):
    values = []
    for path in paths or []:
        with open(path) as handle:
            for line in handle:
                line = line.strip()
                if line:
                    values.append(float(line))
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workload", required=True)
    parser.add_argument("--cycles", type=int, default=0)
    parser.add_argument("--cone", required=True)
    parser.add_argument("--times")
    parser.add_argument("--bits")
    parser.add_argument("--wall-off", nargs="*")
    parser.add_argument("--wall-on", nargs="*")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    ns_per_call = {}
    total_task_ns = 0.0
    times_calls = 0
    if args.times:
        with open(args.times) as handle:
            for line in handle:
                parts = line.strip().split(",")
                if len(parts) != 3 or parts[0] == "task_id":
                    continue
                task_id = int(parts[0])
                calls = int(parts[1])
                task_ns = float(parts[2])
                ns_per_call[task_id] = task_ns / calls if calls else 0.0
                total_task_ns += task_ns
                times_calls += calls

    rows = []
    with open(args.cone) as handle:
        for line in handle:
            parts = line.strip().split(",")
            if len(parts) != 6 or parts[0] == "task_id":
                continue
            rows.append({
                "task_id": int(parts[0]),
                "calls": int(parts[1]),
                "would_skip": int(parts[2]),
                "event_free": parts[3] == "1",
                "hot_cone": parts[4] == "1",
                "cone_words": int(parts[5]),
            })

    total_calls = sum(r["calls"] for r in rows)
    total_skip = sum(r["would_skip"] for r in rows)
    ef_calls = sum(r["calls"] for r in rows if r["event_free"])
    ef_skip = sum(r["would_skip"] for r in rows if r["event_free"])
    ck_calls = sum(r["calls"] for r in rows if not r["hot_cone"])
    ck_skip = sum(r["would_skip"] for r in rows if not r["hot_cone"])

    hot_always = sum(1 for r in rows if r["hot_cone"])
    checkable = len(rows) - hot_always
    words_hist = {}
    for r in rows:
        words_hist[str(r["cone_words"])] = words_hist.get(str(r["cone_words"]), 0) + 1

    savable_ns = 0.0
    matched = 0
    for r in rows:
        if r["task_id"] not in ns_per_call:
            continue
        matched += 1
        savable_ns += r["would_skip"] * ns_per_call[r["task_id"]]

    report = {
        "workload": args.workload,
        "cycles": args.cycles,
        "tasks": len(rows),
        "calls": total_calls,
        "would_skip": total_skip,
        "would_skip_rate_pct": round(100.0 * total_skip / total_calls, 3)
        if total_calls else 0.0,
        "checkable_subset": {
            "tasks": checkable,
            "calls": ck_calls,
            "would_skip": ck_skip,
            "rate_pct": round(100.0 * ck_skip / ck_calls, 3) if ck_calls else 0.0,
        },
        "event_free_subset": {
            "tasks": sum(1 for r in rows if r["event_free"]),
            "calls": ef_calls,
            "would_skip": ef_skip,
            "rate_pct": round(100.0 * ef_skip / ef_calls, 3) if ef_calls else 0.0,
        },
        "hot_cone_distribution": {
            "hot_tasks": hot_always,
            "hot_calls_pct": round(100.0 * (total_calls - ck_calls) / total_calls, 3)
            if total_calls else 0.0,
            "cone_words_histogram": words_hist,
        },
    }
    if args.times:
        report["task_ns_weighted"] = {
            "total_task_ns": round(total_task_ns, 1),
            "times_calls": times_calls,
            "matched_tasks": matched,
            "skippable_ns_upper_bound": round(savable_ns, 1),
            "upper_bound_pct_of_task_bodies":
                round(100.0 * savable_ns / total_task_ns, 3) if total_task_ns else None,
        }

    wall_off = read_wall(args.wall_off)
    wall_on = read_wall(args.wall_on)
    if wall_off and wall_on:
        off = min(wall_off)
        on = min(wall_on)
        report["overhead"] = {
            "wall_off_s_min": off,
            "wall_on_s_min": on,
            "delta_s": round(on - off, 3),
            "delta_pct": round(100.0 * (on - off) / off, 3) if off else None,
        }

    if args.bits:
        sets = []
        with open(args.bits) as handle:
            for line in handle:
                parts = line.strip().split(",")
                if len(parts) != 2 or parts[0] == "bit":
                    continue
                if int(parts[1]) > 0:
                    sets.append((int(parts[0]), int(parts[1])))
        sets.sort(key=lambda item: -item[1])
        report["dirty_bits"] = {
            "bits_ever_set": len(sets),
            "total_sets": sum(s for _, s in sets),
            "avg_bits_per_cycle": round(
                sum(s for _, s in sets) / args.cycles, 3) if args.cycles else None,
            "top16": [{"bit": b, "sets": s} for b, s in sets[:16]],
        }

    with open(args.out, "w") as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")
    json.dump(report, sys.stdout, indent=2)


if __name__ == "__main__":
    main()
