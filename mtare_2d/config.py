from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Config:
    seed: int = 755
    robots: int = 3
    width: float = 100.0
    height: float = 100.0
    cell_size: float = 2.0
    obstacles: int = 30
    reveal_radius: float = 7.5
    comms_range: float | None = None
    message_delay: int = 1
    message_ttl: int = 8
    plan_every: int = 4
    publish_every: int = 1
    solver_ms: int = 80
    planning_mode: str = "parallel"
    architecture: str = "hierarchical"
    viewpoint_spacing: float = 0.6
    visited_viewpoint_radius: float = 3.0
    region_size: float = 6.0
    local_region_radius: int = 2
    local_optimization_iterations: int = 10
    local_greedy_sample_range: int = 3
    local_min_gain: int = 1
    # Preserve the ordinary/frontier procedure; samples replace surface points.
    coverage_model: str = "mtare"
    strict_frontier: bool = True
    settle_idle_almost_covered: bool = True
    respect_peer_coverage: bool = True
    repair_region_reports: bool = True
    use_lookahead: bool = True
    lookahead_distance: float = 8.0
    coverage_radius: float | None = None
    local_frontier_min_gain: int = 1
    region_state_model: str = "mtare"
    # Integer sample thresholds are explicit planar adaptations, not raw 3D
    # point-cloud counts. The original strict/non-strict comparisons are kept.
    region_small_gain: int = 1
    region_big_gain: int = 2
    region_frontier_gain: int = 1
    region_exploring_to_covered_count: int = 1
    region_covered_to_exploring_count: int = 10
    region_max_exploring_count: int = 2
    span_coefficient: int = 100
    drop_penalty: int = 9_999_999
    task_arc_offset: int = 500
    max_targets: int = 0  # No artificial target cap in the default baseline.
    target_spacing: float = 6.0
    completion_startup_cycles: int = 300  # Logical-cycle translation of original >300 s guard.
    # Simulation observer's minimum observed reachable-free fraction. None
    # preserves the historical source-style global-list completion predicate.
    completion_min_coverage: float | None = 0.91
    max_cycles: int = 1200
    save_every: int = 10
    frame_delay: float = 0.15

    def __post_init__(self):
        if self.viewpoint_spacing <= 0 or self.visited_viewpoint_radius < 0:
            raise ValueError("Viewpoint spacing must be positive and visit radius nonnegative")
        if self.lookahead_distance <= 0:
            raise ValueError("Lookahead distance must be positive")
        if self.coverage_model not in ('simple','mtare','frontier'):
            raise ValueError('coverage_model must be simple, mtare or frontier')
        if self.coverage_radius is not None and not 0 < self.coverage_radius <= self.reveal_radius:
            raise ValueError('coverage_radius must be positive and no greater than reveal_radius')
        if self.local_frontier_min_gain < 1:
            raise ValueError('local_frontier_min_gain must be positive')
        if self.region_state_model not in ("mtare", "frontier"):
            raise ValueError("region_state_model must be mtare or frontier")
        if min(self.region_small_gain, self.region_big_gain, self.region_exploring_to_covered_count) < 1 or min(self.region_frontier_gain, self.region_covered_to_exploring_count, self.region_max_exploring_count) < 0:
            raise ValueError("Region coverage thresholds are out of range")
        if self.architecture not in ("hierarchical", "flat"):
            raise ValueError("architecture must be hierarchical or flat")
        if self.region_size < self.cell_size or abs(self.region_size/self.cell_size-round(self.region_size/self.cell_size)) > 1e-9:
            raise ValueError("region_size must be a positive multiple of cell_size")
        if self.local_region_radius < 1 or min(self.local_optimization_iterations, self.local_greedy_sample_range, self.local_min_gain) < 1:
            raise ValueError("Local coverage parameters must be positive")
        if self.planning_mode not in ("parallel", "sequential"):
            raise ValueError("planning_mode must be parallel or sequential")
        if not 1 <= self.robots <= 8:
            raise ValueError("Choose between 1 and 8 robots.")
        for name in ("width", "height", "cell_size", "reveal_radius", "solver_ms", "plan_every",
                     "publish_every", "max_cycles", "save_every"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.completion_startup_cycles < 0:
            raise ValueError("completion_startup_cycles cannot be negative")
        if self.completion_min_coverage is not None and not 0 < self.completion_min_coverage <= 1:
            raise ValueError("completion_min_coverage must be in (0, 1] or None")
        if self.message_delay < 1:
            raise ValueError("Message delay must be at least one simulation cycle.")
        if self.comms_range is not None and self.comms_range <= 0:
            raise ValueError("Communication range must be positive or full.")
        if self.target_spacing < 0 or self.max_targets < 0 or self.frame_delay < 0:
            raise ValueError("Target/display limits cannot be negative.")
        for length in (self.width, self.height):
            if abs(length / self.cell_size - round(length / self.cell_size)) > 1e-9:
                raise ValueError("Environment dimensions must be multiples of cell size.")

    def to_dict(self):
        return asdict(self)

    @property
    def effective_coverage_radius(self):
        return self.coverage_radius if self.coverage_radius is not None else 2*self.reveal_radius/3
