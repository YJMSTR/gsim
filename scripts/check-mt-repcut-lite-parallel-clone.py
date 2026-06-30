#!/usr/bin/env python3
import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path


def fail(message: str) -> None:
    raise SystemExit(f"mt-repcut-lite-parallel-clone failed: {message}")


def find_one(model_dir: Path, suffix: str) -> Path:
    matches = sorted(model_dir.glob(f"*{suffix}"))
    if not matches:
        fail(f"missing *{suffix} under {model_dir}")
    if len(matches) != 1:
        fail(f"expected one *{suffix}, found {len(matches)}")
    return matches[0]


def read_texts(model_dir: Path) -> str:
    chunks = []
    for path in sorted(model_dir.glob("*.cpp")):
        chunks.append(path.read_text(errors="ignore"))
    return "\n".join(chunks)


def model_class_name(model_dir: Path) -> str:
    headers = [p for p in model_dir.glob("*.h") if not p.name.endswith(".tmp.h")]
    if len(headers) != 1:
        fail(f"expected one generated header, found {len(headers)}")
    text = headers[0].read_text(errors="ignore")
    match = re.search(r"class\s+(S\w+)\s*\{", text)
    if not match:
        fail(f"could not find model class in {headers[0]}")
    return match.group(1)


def ensure_emu(model_dir: Path) -> Path:
    emu = model_dir / "emu"
    class_name = model_class_name(model_dir)
    header = next(p.name for p in model_dir.glob("*.h") if not p.name.endswith(".tmp.h"))
    harness = model_dir / "repcut_trace_main.cpp"
    harness.write_text(f'''#include "{header}"
#include <cstdio>
int main() {{
  {class_name} dut;
  for (int i = 0; i < 16; ++i) {{
    dut.set_io_a((uint8_t)(i * 17 + 3));
    dut.set_io_b((uint8_t)(i * 29 + 5));
    dut.step();
    std::printf("%02x %02x\\n", (unsigned)dut.get_io_x(), (unsigned)dut.get_io_y());
  }}
  return 0;
}}
''')
    sources = sorted(p.name for p in model_dir.glob("*.cpp"))
    cmd = ["clang++", "-std=c++20", "-O2", "-pthread", "-I", ".", *sources, "-o", emu.name]
    result = subprocess.run(cmd, cwd=str(model_dir), text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
    if result.returncode != 0:
        fail(f"failed to compile generated harness: rc={result.returncode}\n{result.stdout}")
    return emu

def run_emu(emu: Path, threads: int, profile: bool = False) -> str:
    env = os.environ.copy()
    env["GSIM_THREADS"] = str(threads)
    if profile:
        env["GSIM_MT_PROFILE"] = "1"
    result = subprocess.run([f"./{emu.name}"], cwd=str(emu.parent), env=env, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
    if result.returncode != 0:
        fail(f"emu failed for GSIM_THREADS={threads}: rc={result.returncode}\n{result.stdout}")
    return result.stdout


def check_profile_counters(profile_text: str, expected_sink_ids: set[int]) -> str:
    total_match = re.search(r"\[mt-profile\] repcut_runtime cloned_task_calls=(\d+)", profile_text)
    if not total_match:
        fail("profile output missing repcut_runtime cloned_task_calls")
    total = int(total_match.group(1))
    if total <= 0:
        fail(f"expected positive cloned_task_calls, got {total}")

    by_cppid = {}
    by_cppid_match = re.search(r"\[mt-profile\] repcut_runtime_by_cppid([^\n]*)", profile_text)
    if by_cppid_match:
        for cpp_id, count in re.findall(r"(\d+):(\d+)", by_cppid_match.group(1)):
            by_cppid[int(cpp_id)] = int(count)
    missing = sorted(cpp_id for cpp_id in expected_sink_ids if by_cppid.get(cpp_id, 0) <= 0)
    if missing:
        fail(f"profile output missing positive repcut_runtime_by_cppid counts for sinks {missing}; got {by_cppid}")
    return f", profile cloned_task_calls={total} by_cppid={dict(sorted(by_cppid.items()))}"


def normalize_trace(text: str) -> str:
    lines = []
    for line in text.splitlines():
        if line.startswith("emu compiled at "):
            continue
        if line.startswith("Host time spent:"):
            continue
        if line.startswith("[mt-profile]"):
            continue
        lines.append(line)
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("model_dir", type=Path)
    parser.add_argument("--check-profile-counters", action="store_true")
    parser.add_argument("--threads", type=int, default=4)
    args = parser.parse_args()

    model_dir = args.model_dir
    if not model_dir.exists():
        fail(f"missing model dir {model_dir}")
    report_path = find_one(model_dir, "_mt_repcut_lite.json")
    report = json.loads(report_path.read_text())

    cut_batches = report.get("cut_batches", [])
    parallel_batches = [b for b in cut_batches if b.get("parallel_safe") is True and b.get("forced_serial") is False]
    if not parallel_batches:
        fail(f"no parallel-safe cut batch in {report_path}")
    clones = report.get("duplicated_nodes", [])
    if not clones:
        fail("duplicated_nodes is empty")
    if not any(b.get("clone_count", 0) > 0 and b.get("forced_sink_activation") is True for b in parallel_batches):
        fail("parallel-safe batches lack clone_count/forced_sink_activation proof")
    expected_profile_sinks = {
        task.get("cpp_id") for task in report.get("tasks", [])
        if task.get("runtime_applied") is True
    }
    expected_profile_sinks = {cpp_id for cpp_id in expected_profile_sinks if isinstance(cpp_id, int)}

    text = read_texts(model_dir)
    if "workerCount = 1" in "\n".join(
        line for line in text.splitlines()
        if any(f"case {b.get('begin_cpp_id')}:" in line for b in parallel_batches)
    ):
        fail("parallel-safe cut batch still has local workerCount = 1 guard")
    if "mtRepCutLiteTask" not in text:
        fail("generated C++ has no mtRepCutLiteTask helper")
    for clone in clones:
        clone_name = clone.get("clone_name")
        if clone_name and clone_name not in text:
            fail(f"clone {clone_name} not found in generated C++")

    emu = ensure_emu(model_dir)
    ref = normalize_trace(run_emu(emu, 1))
    got = normalize_trace(run_emu(emu, args.threads))
    if ref != got:
        fail(f"trace mismatch between GSIM_THREADS=1 and {args.threads}")
    trace_note = f", trace GSIM_THREADS=1/{args.threads} matched"
    if args.check_profile_counters:
        if not expected_profile_sinks:
            fail("--check-profile-counters requires runtime_applied sink tasks in the report")
        trace_note += check_profile_counters(run_emu(emu, args.threads, profile=True), expected_profile_sinks)

    print(f"mt-repcut-lite-parallel-clone ok: {len(parallel_batches)} parallel-safe cut batches, {len(clones)} duplicated nodes{trace_note} from {report_path}")


if __name__ == "__main__":
    main()
