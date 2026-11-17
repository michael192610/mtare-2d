"""Explicit historical profiles for regression tests; new defaults tested separately."""
from mtare_2d.config import Config


def simple_config(**kwargs):
    defaults = dict(coverage_model="simple", viewpoint_spacing=2.0,
                    region_covered_to_exploring_count=1, strict_frontier=False,
                    settle_idle_almost_covered=False, respect_peer_coverage=False, use_lookahead=False,
                    repair_region_reports=False, completion_min_coverage=None)
    defaults.update(kwargs)
    return Config(**defaults)


def sample_config(**kwargs):
    defaults = dict(coverage_model="mtare", viewpoint_spacing=2.0,
                    region_covered_to_exploring_count=1, strict_frontier=False,
                    settle_idle_almost_covered=False, respect_peer_coverage=False, use_lookahead=False,
                    repair_region_reports=False, completion_min_coverage=None)
    defaults.update(kwargs)
    return Config(**defaults)
