"""Successful stopping reasons, shared by live and saved-result viewers."""

COMPLETION_REASONS = frozenset({
    "all_reachable_free_cells_observed",
    "no_remaining_exploration_work",
})
