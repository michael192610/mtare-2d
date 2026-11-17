"""Independent search jobs; workers receive costs and hints, never robot maps."""
import os
import time

from .vrp import solve


def run_search(matrices, config, initial_routes):
    started = time.perf_counter()
    result = solve(matrices, config, initial_routes)
    return result, dict(worker_pid=os.getpid(), started_seconds=started,
                        finished_seconds=time.perf_counter())
