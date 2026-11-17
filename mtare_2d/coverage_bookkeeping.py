"""2D sample/covered-point adapter for PlanningEnv coverage bookkeeping.

Observed free-cell centres are persistent ordinary samples. They become covered
only through executed positions, not planned viewpoints. Unknown-side boundary
patches form a separate, dynamically rebuilt frontier pool; no hidden truth is
read to classify them. Global cell IDs provide stable sample identities.
"""
import numpy as np

from .world import FREE


class CoverageBookkeeping:
    def __init__(self, world, robot_id, emit=None):
        self.world, self.robot_id, self.emit = world, robot_id, emit
        self.observed = np.zeros(world.size, dtype=bool)
        self.covered = np.zeros(world.size, dtype=bool)
        self.visited_visibility = np.zeros(world.size, dtype=bool)
        self.processed_trajectory = 0
        self.last_position = None

    def observe(self, known, trajectory, position):
        """Commit observations and coverage from actual poses (including history)."""
        new_poses = list(trajectory[self.processed_trajectory:])
        if not new_poses or tuple(new_poses[-1]) != tuple(position):
            if self.last_position != tuple(position):
                new_poses.append(position)
        for pose in new_poses:
            delta = self.world.centres-np.asarray(pose)
            self.visited_visibility |= np.sum(delta*delta, axis=1) <= self.world.config.effective_coverage_radius**2
        self.processed_trajectory = len(trajectory)
        self.last_position = tuple(position)
        observed = (known.ravel() == FREE)
        new_observed = np.flatnonzero(observed & ~self.observed).tolist()
        self.observed |= observed
        new_covered = np.flatnonzero(self.observed & self.visited_visibility & ~self.covered).tolist()
        self.covered[self.observed & self.visited_visibility] = True
        if self.emit and (new_observed or new_covered):
            self.emit('coverage_samples_updated', robot_id=self.robot_id,
                      observed_sample_cells=new_observed, covered_sample_cells=new_covered)

    def credit_visited_viewpoints(self, cells, viewpoint_ids):
        """Original covered-area update also credits visited candidate visibility."""
        indices=np.asarray(sorted(cells),dtype=int)
        additional=indices[self.observed[indices] & ~self.covered[indices]]
        self.covered[additional]=True
        # Preserve coverage flags; do not mark unknown cells observed or covered.
        if self.emit and len(additional):
            self.emit('coverage_samples_updated',robot_id=self.robot_id,
                      observed_sample_cells=[],covered_sample_cells=additional.tolist(),
                      coverage_source='visited_viewpoints',visited_viewpoint_ids=viewpoint_ids)

    def snapshot(self):
        return dict(model='mtare_2d_samples', coverage_radius=self.world.config.effective_coverage_radius,
                    discovery_radius=self.world.config.reveal_radius,
                    sample_spacing=self.world.config.cell_size,
                    sample_area=self.world.config.cell_size**2,
                    observed_sample_cells=np.flatnonzero(self.observed).tolist(),
                    covered_sample_cells=np.flatnonzero(self.covered).tolist(),
                    uncovered_sample_cells=np.flatnonzero(self.observed & ~self.covered).tolist())
