"""Run a fresh simulation without animation, then browse its saved robot maps."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(
        description="Run three robots quickly and open the finished result. "
                    "Additional simulation options, e.g. --range 30, are forwarded to run.sh.")
    parser.add_argument("--solver-ms", type=int, default=80,
                        help="VRP search budget per call (default: 80, matching normal mode; use 10 for a shorter search).")
    parser.add_argument("--no-open", action="store_true", help="Save results without opening a window.")
    args, extra = parser.parse_known_args()
    if args.solver_ms <= 0:
        parser.error("--solver-ms must be positive")
    if "--paused" in extra:
        parser.error("The fast launcher runs immediately; use run.sh for paused animation.")
    root = Path(__file__).resolve().parent
    command = [str(root / "run.sh"), "--robots", "3", "--headless", "--no-frames",
               "--no-open", "--solver-ms", str(args.solver_ms), *extra]
    print(f"Starting a NEW run without animation ({args.solver_ms} ms VRP search per call).", flush=True)
    print("Exploration planning rules match run.sh; runtime depends on the scenario and computer.", flush=True)
    started = time.monotonic()
    run_directory = None
    with tempfile.TemporaryDirectory(prefix="mtare-launch-") as temporary:
        directory_file = Path(temporary) / "run-directory.txt"
        command.extend(["--run-directory-file", str(directory_file)])
        process = subprocess.Popen(command, cwd=root)
        try:
            returncode = process.wait()
        except KeyboardInterrupt:
            process.terminate()
            process.wait()
            return 130
        if directory_file.exists():
            run_directory = Path(directory_file.read_text().strip())
    if returncode:
        return returncode
    if run_directory is None or not (run_directory / "summary.json").exists():
        print("No completed result was saved; inspect the simulation output above.", file=sys.stderr)
        return 1
    summary = json.loads((run_directory / "summary.json").read_text())
    final = summary["final"]
    elapsed = time.monotonic() - started
    print(f"\n{summary['status'].upper()}: {100*final['explored_ratio']:.2f}% observed "
          f"in {elapsed:.1f} seconds ({final['cycle']} cycles).", flush=True)
    (run_directory / "fast_launcher.json").write_text(json.dumps({
        "launcher": "run_fast.sh", "elapsed_seconds": elapsed,
        "solver_ms": args.solver_ms, "animation": False,
        "note": "A fresh simulation; shorter search can produce different plans from the 80 ms default."
    }, indent=2) + "\n")
    if args.no_open:
        return 0
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("No desktop display available. Open the saved final.png to see the result.")
        return 0
    os.environ.setdefault("MPLCONFIGDIR", str(root / ".mplconfig"))
    from mtare_2d.results_viewer import show_results
    show_results(run_directory)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
