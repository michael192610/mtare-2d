import argparse
import logging
import os
from pathlib import Path

from .config import Config


def range_value(value):
    return None if value.lower() == "full" else float(value)


def main():
    parser = argparse.ArgumentParser(description="M-TARE hierarchical exploration adapted to 2D")
    parser.add_argument("--robots", type=int, default=3)
    parser.add_argument("--seed", type=int, default=755)
    parser.add_argument("--range", dest="comms_range", type=range_value, default=None,
                        help="Communication range in map units, or full (default).")
    parser.add_argument("--width", type=float, default=100)
    parser.add_argument("--height", type=float, default=100)
    parser.add_argument("--cell-size", type=float, default=2)
    parser.add_argument("--obstacles", type=int, default=30)
    parser.add_argument("--reveal-radius", type=float, default=7.5)
    parser.add_argument('--coverage-model', choices=('simple','mtare','frontier'), default='mtare',
                        help='M-TARE ordinary/frontier coverage (default), simplified frontier baseline, or legacy prototype.')
    parser.add_argument('--relaxed-frontier-thresholds', dest='strict_frontier', action='store_false',
                        help='Use the historical relaxed sample gates instead of original strict frontier comparisons.')
    parser.add_argument('--coverage-radius', type=float,
                        help='Abstract ordinary coverage radius; default is 2/3 of discovery radius.')
    parser.add_argument("--plan-every", type=int, default=4)
    parser.add_argument("--message-delay", type=int, default=1)
    parser.add_argument("--solver-ms", type=int, default=80)
    parser.add_argument("--completion-min-coverage", type=float, default=0.91,
                        help="Minimum team observed fraction for idle completion (default: 0.91).")
    parser.add_argument("--original-region-protocol", dest="repair_region_reports", action="store_false",
                        help="Disable the region-report conflict repair for historical comparisons.")
    parser.add_argument("--no-lookahead", dest="use_lookahead", action="store_false",
                        help="Use the historical fixed-start local route executor for comparisons.")
    parser.add_argument("--lookahead-distance", type=float, default=8.0,
                        help="Original-style local lookahead distance in metres (default: 8).")
    parser.add_argument("--architecture", choices=("hierarchical", "flat"), default="hierarchical",
                        help="Region-level VRP plus local viewpoint coverage (default), or the earlier flat prototype.")
    parser.add_argument("--viewpoint-spacing",type=float,default=.6,help="Local viewpoint spacing in metres (original indoor configuration: 0.6).")
    parser.add_argument("--region-size", type=float, default=6)
    parser.add_argument("--region-state-model", choices=("mtare", "frontier"), default="mtare",
                        help="M-TARE-style region states (default), or the prior frontier-derived hierarchy.")
    parser.add_argument("--local-region-radius", type=int, default=2, help="Region rings around the current region; 2 gives a 30 m window with 6 m regions.")
    parser.add_argument("--planning-mode", choices=("parallel", "sequential"), default="parallel",
                        help="Parallel robot VRP searches with synchronized cycles (default), or sequential reference.")
    parser.add_argument("--max-targets", type=int, default=0,
                        help="Optional global target cap for experiments; default 0 includes all eligible targets.")
    parser.add_argument("--target-spacing", type=float, default=6, help="Frontier spacing in flat prototype mode only.")
    parser.add_argument("--task-arc-offset", type=int, default=500)
    parser.add_argument("--max-cycles", type=int, default=1200)
    parser.add_argument("--save-every", type=int, default=10)
    parser.add_argument("--frame-delay", type=float, default=0.15)
    parser.add_argument("--paused", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-open", action="store_true",
                        help="Do not open the results window after a headless run.")
    parser.add_argument("--run-directory-file", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--no-frames", action="store_true")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    if args.headless and args.paused:
        parser.error("--paused is only available with a window")
    values = vars(args).copy()
    for key in ("headless", "paused", "no_frames", "output", "no_open", "run_directory_file"):
        values.pop(key)
    try:
        config = Config(**values)
    except ValueError as exc:
        parser.error(str(exc))
    os.environ.setdefault("MPLCONFIGDIR", str(args.output.resolve() / ".mplconfig"))
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    import matplotlib
    matplotlib.use("Agg" if args.headless else "TkAgg")
    from .recording import Journal
    from .simulation import Simulation
    from .viewer import Viewer
    journal = Journal(args.output, config)
    if args.run_directory_file is not None:
        args.run_directory_file.write_text(str(journal.directory.resolve()) + "\n")
    logging.basicConfig(level=logging.INFO, format="%(message)s", handlers=[
        logging.StreamHandler(), logging.FileHandler(journal.directory / "console.log")], force=True)
    logging.info("M-TARE 2D: %d robots, %s communication; seed %d", config.robots,
                 "full" if config.comms_range is None else f"{config.comms_range:g} m", config.seed)
    logging.info("Planning architecture: %s", config.architecture)
    logging.info("VRP searches: %s; movement waits for every robot's result", config.planning_mode)
    sim, viewer = None, None
    try:
        sim = Simulation(config, journal)
        journal.initialize(sim)
        viewer = Viewer(sim, journal, args.headless, args.paused, not args.no_frames)
        if args.headless:
            while not sim.done:
                if args.no_frames:
                    stats = sim.step()
                    if sim.tick % 20 == 0:
                        logging.info("Cycle %d: %.2f%% observed; %.1f m travelled", sim.tick,
                                     stats["explored_ratio"]*100, stats["team_distance"])
                else:
                    viewer.advance()
            from .completion import COMPLETION_REASONS
            viewer.finish("finished" if sim.reason in COMPLETION_REASONS else "stopped")
        else:
            logging.info("Space: pause/resume | N: one cycle | 0/1/2: robot map | A: team view | Q: save/stop")
            viewer.show()
    except KeyboardInterrupt:
        if sim:
            sim.reason = "keyboard_interrupt"
        if viewer:
            viewer.finish("stopped")
    except Exception as exc:
        logging.exception("Run failed")
        if sim:
            sim.reason = str(exc)
            if viewer:
                viewer.finish("error")
            else:
                journal.finish(sim, "error")
        return 1
    finally:
        if sim:
            sim.close()
        if sim and not journal.finished:
            journal.finish(sim, "stopped", "viewer_exited")
    failed = bool((viewer and viewer.failed) or
                  (sim and sim.event_counts["objective_verification_failure"]))
    if args.headless and not args.no_open and not failed and journal.finished:
        if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
            viewer.plt.close(viewer.figure)
            logging.info("Opening results window.")
            from .results_viewer import show_results
            show_results(journal.directory)
        else:
            logging.info("No desktop display available; skipping the results window.")
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
