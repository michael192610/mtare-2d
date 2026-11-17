"""M-TARE exploration states, distinct from occupancy; 2D evidence adapters.

Reference: grid_world.cpp SetCellStatus, UpdateLocalCellStatus,
UpdateGlobalCellStatus, UpdateNeighborCells and coordination manager UpdateCells.
Private 2D traversability replaces keypose connectivity; mobility is homogeneous.
"""
from dataclasses import asdict, dataclass, field
from enum import IntEnum


class CellStatus(IntEnum):
    UNSEEN = 0
    EXPLORING = 1
    COVERED = 2
    COVERED_BY_OTHERS = 3
    NOGO = 4
    EXPLORING_BY_OTHERS = 5


ACTIVE = (CellStatus.EXPLORING, CellStatus.EXPLORING_BY_OTHERS)
LOCAL_ELIGIBLE = (CellStatus.UNSEEN, *ACTIVE)
COVERED = (CellStatus.COVERED, CellStatus.COVERED_BY_OTHERS)


@dataclass(frozen=True)
class RegionUpdate:
    # Same fields as tare_msgs/Cell.msg. Region IDs are from our shared coarse grid.
    id: int
    update_id: int
    known_by_robot_ids: int
    mobility_type: int
    status: int

    def to_dict(self):
        return asdict(self)


@dataclass
class RegionState:
    status: CellStatus = CellStatus.UNSEEN
    covered_to_others: bool = False
    exploring_robot_id: int = -1
    explored_robot_id: int = -1
    update_id: int = 0
    known_by_robot_ids: int = 0
    visit_count: int = 0
    exploring_count: int = 0
    almost_covered: bool = False
    has_exploring_priority: bool = True
    sync_count: int = 0
    exploring_synced_count: dict = field(default_factory=dict)
    explored_synced_count: dict = field(default_factory=dict)
    mobility_type: int = 0


@dataclass(frozen=True)
class Evidence:
    candidates: int = 0
    selected: int = 0
    above_small: int = 0
    above_big: int = 0
    above_frontier: int = 0
    has_graph_node: bool = False
    near_robot: bool = False


class RegionKnowledge:
    def __init__(self, robot_id, region_count, config, emit=None):
        self.robot_id, self.config, self.emit = robot_id, config, emit
        self.regions = [RegionState() for _ in range(region_count)]
        for cell in self.regions:
            cell.known_by_robot_ids = 1 << robot_id
        self.window = set()
        self.evidence = {}

    def snapshot(self):
        return [{"id":i, **asdict(cell), "status":int(cell.status), "status_name":cell.status.name}
                for i, cell in enumerate(self.regions)]

    def log(self, kind, **data):
        if self.emit:
            self.emit(kind, robot_id=self.robot_id, **data)

    def repairs_reports(self):
        return self.config.coverage_model == 'mtare' and self.config.repair_region_reports

    @staticmethod
    def report_status(cell):
        """Compare wire meanings, not receiver-relative internal labels."""
        if cell.status in COVERED or cell.covered_to_others:
            return CellStatus.COVERED
        if cell.status in ACTIVE:
            return CellStatus.EXPLORING
        return None

    def reset_report_sync(self, cell):
        cell.known_by_robot_ids = 1 << self.robot_id
        cell.sync_count = 0
        cell.exploring_synced_count.clear()
        cell.explored_synced_count.clear()

    def set_status(self, region, status, actor, reason, bump=False):
        cell = self.regions[region]
        before = asdict(cell)
        previous = cell.status
        cell.status = CellStatus(status)
        if cell.status in ACTIVE:
            cell.exploring_robot_id = actor
            cell.covered_to_others = False
            if previous not in ACTIVE:
                cell.exploring_count += 1
        elif cell.status in COVERED:
            cell.explored_robot_id = actor
            cell.covered_to_others = True
        if bump:
            cell.update_id += 1
            cell.known_by_robot_ids = 1 << self.robot_id
            cell.sync_count = 0
            if self.repairs_reports():
                self.reset_report_sync(cell)
        if before != asdict(cell):
            self.log("region_state_changed", region_id=region, previous_status=int(previous),
                     status=int(cell.status), status_name=cell.status.name, reason=reason,
                     update_id=cell.update_id, visit_count=cell.visit_count,
                     exploring_count=cell.exploring_count, actor=actor,
                     covered_to_others=cell.covered_to_others)

    def update_local(self, window, evidence):
        window = set(window)
        for region in sorted(window-self.window):
            cell = self.regions[region]
            cell.visit_count += 1
            self.log("region_window_entered", region_id=region, visit_count=cell.visit_count)
        self.window, self.evidence = window, evidence
        for region in sorted(window):
            cell, ev = self.regions[region], evidence.get(region, Evidence())
            below = ev.above_frontier < self.config.region_exploring_to_covered_count and \
                    ev.above_small < self.config.region_exploring_to_covered_count
            high = ev.above_frontier > self.config.region_covered_to_exploring_count or \
                   ev.above_big > self.config.region_covered_to_exploring_count
            if cell.status == CellStatus.UNSEEN:
                if ev.selected:
                    self.set_status(region, CellStatus.EXPLORING, self.robot_id, "selected_local_viewpoint", True)
                    cell.almost_covered = False
                elif ev.candidates and not cell.covered_to_others:
                    cell.covered_to_others = True
                    cell.explored_robot_id = self.robot_id
                    cell.update_id += 1
                    cell.known_by_robot_ids = 1 << self.robot_id
                    cell.sync_count = 0
                    if self.repairs_reports():
                        self.reset_report_sync(cell)
                    self.log("region_covered_to_others", region_id=region, update_id=cell.update_id)
            elif cell.status == CellStatus.EXPLORING:
                if ev.selected:
                    cell.almost_covered = False
                elif ev.candidates:
                    if below:
                        self.set_status(region, CellStatus.COVERED, self.robot_id, "coverage_below_threshold", True)
                    else:
                        cell.almost_covered = True
                elif (cell.visit_count == 1 and not ev.has_graph_node) or ev.near_robot:
                    self.set_status(region, CellStatus.COVERED, self.robot_id, "no_local_candidates", True)
            elif cell.status == CellStatus.EXPLORING_BY_OTHERS:
                if ev.selected:
                    # Original branch changes status without incrementing update_id.
                    self.set_status(region, CellStatus.EXPLORING, self.robot_id, "selected_peer_region_locally")
                elif ev.candidates:
                    cell.almost_covered = True
            elif cell.status == CellStatus.COVERED:
                if high and cell.exploring_count <= self.config.region_max_exploring_count:
                    self.set_status(region, CellStatus.EXPLORING, self.robot_id, "new_coverage_reopens_region", True)
            # COVERED_BY_OTHERS and NOGO retain the original no-local-transition rule.
        for region, cell in enumerate(self.regions):
            if cell.almost_covered and region not in window:
                self.set_status(region, CellStatus.COVERED, self.robot_id, "almost_covered_left_window",
                                not cell.covered_to_others)
                cell.almost_covered = False

    def apply_peer(self, update, sender):
        if not 0 <= update.id < len(self.regions):
            raise ValueError("Received out-of-grid region ID")
        incoming = CellStatus(update.status)
        cell = self.regions[update.id]
        if (self.repairs_reports() and incoming == CellStatus.EXPLORING
                and update.update_id == cell.update_id
                and (cell.status == CellStatus.COVERED or
                     (cell.status == CellStatus.UNSEEN and cell.covered_to_others))
                and cell.explored_robot_id == self.robot_id):
            # Reissue an existing LOCAL completion decision with a fresh revision.
            # Do not change coverage decisions, accept equal-version peer states,
            # or manufacture completion from elapsed idle time/global truth.
            self.set_status(update.id, cell.status, self.robot_id,
                            'equal_revision_completion_republished', True)
            self.log('region_completion_republished', region_id=update.id, sender=sender,
                     incoming_update_id=update.update_id, update_id=cell.update_id)
            return False
        if update.id in self.window and cell.status == CellStatus.EXPLORING:
            self.log("region_update_ignored", region_id=update.id, sender=sender,
                     reason="protected_local_exploration", incoming_update_id=update.update_id)
            return False
        if update.update_id <= cell.update_id:
            self.log("region_update_ignored", region_id=update.id, sender=sender,
                     reason="not_newer", incoming_update_id=update.update_id)
            return False
        previous = cell.status
        cell.update_id = update.update_id
        cell.sync_count = 0
        if self.repairs_reports():
            self.reset_report_sync(cell)
        cell.mobility_type = update.mobility_type
        if previous in (CellStatus.UNSEEN, CellStatus.COVERED_BY_OTHERS) and incoming == CellStatus.EXPLORING:
            self.set_status(update.id, CellStatus.EXPLORING_BY_OTHERS, sender, "received_exploring_state")
            cell.known_by_robot_ids = 1 << self.robot_id
        elif previous in (*ACTIVE, CellStatus.UNSEEN) and incoming == CellStatus.COVERED:
            self.set_status(update.id, CellStatus.COVERED_BY_OTHERS, sender, "received_covered_state")
            cell.known_by_robot_ids = 1 << self.robot_id
        self.log("region_update_applied", region_id=update.id, sender=sender, update_id=cell.update_id,
                 incoming_status=int(incoming), previous_status=int(previous), status=int(cell.status))
        return True

    def settle_idle(self):
        """Finalize pending low-evidence regions without excluded home motion.

        Original UpdateLocalCellStatus finalizes almost-covered regions on
        leaving the horizon. A robot that stays idle here cannot trigger that
        transition. The simulator calls this only after no global tasks and
        local coverage completion; selected/high-evidence regions remain active.
        """
        settled = []
        for region in sorted(self.window):
            cell = self.regions[region]
            ev = self.evidence.get(region, Evidence())
            if (cell.status not in ACTIVE or not cell.almost_covered
                    or not ev.candidates or ev.selected
                    or ev.above_small >= self.config.region_exploring_to_covered_count
                    or ev.above_frontier >= self.config.region_exploring_to_covered_count):
                continue
            self.set_status(region, CellStatus.COVERED, self.robot_id,
                            "idle_almost_covered_without_return_home", True)
            cell.almost_covered = False
            settled.append(region)
        return settled

    def update_global(self, connected_regions, neighbors):
        for region, cell in enumerate(self.regions):
            if region in self.window:
                continue
            cell.has_exploring_priority = True
            if cell.status not in ACTIVE:
                continue
            adjacent = neighbors(region)
            surrounded = len(adjacent) == 8 and all(
                self.regions[i].status in COVERED or self.regions[i].covered_to_others for i in adjacent)
            if surrounded or region not in connected_regions:
                self.set_status(region, CellStatus.COVERED, self.robot_id,
                                "surrounded_by_covered" if surrounded else "disconnected_region", True)

    def update_priority(self, peer_local_regions):
        for region in self.window:
            self.regions[region].has_exploring_priority = not any(
                sender < self.robot_id and region in ids for sender, ids in peer_local_regions.items())

    def known_by(self, region, robot):
        return bool(self.regions[region].known_by_robot_ids & (1 << robot))

    def sync_peer(self, update, sender):
        """SyncCells runs separately from UpdateCells, including equal revisions.

        In particular, local-state protection does not suppress this sync pass.
        """
        cell = self.regions[update.id]
        if update.update_id < cell.update_id:
            return
        if self.repairs_reports() and (update.update_id != cell.update_id
                or self.report_status(cell) != CellStatus(update.status)):
            # Receipt of a counter is not acknowledgement of the same STATE.
            # Especially do not ACK a newer completion rejected by local-work
            # protection. Keep advertising until state AND revision agree.
            self.reset_report_sync(cell)
            self.log('region_sync_rejected', region_id=update.id, sender=sender,
                     incoming_update_id=update.update_id, update_id=cell.update_id,
                     incoming_status=int(update.status),
                     report_status=(int(self.report_status(cell))
                                    if self.report_status(cell) is not None else None),
                     reason='state_or_revision_mismatch')
            return
        counts = (cell.exploring_synced_count if update.status == CellStatus.EXPLORING
                  else cell.explored_synced_count)
        for robot in range(self.config.robots):
            if update.known_by_robot_ids & (1 << robot):
                counts[robot] = counts.get(robot, 0) + 1
        cell.known_by_robot_ids |= update.known_by_robot_ids
        if cell.known_by_robot_ids == (1 << self.config.robots)-1:
            for robot in range(self.config.robots):
                if robot == self.robot_id:
                    continue
                if cell.status in ACTIVE:
                    n = cell.exploring_synced_count.get(robot, 0)
                elif cell.status in COVERED or cell.covered_to_others:
                    n = cell.explored_synced_count.get(robot, 0)
                else:
                    n = max(cell.exploring_synced_count.get(robot, 0),
                            cell.explored_synced_count.get(robot, 0))
                if n > 5:
                    cell.sync_count = max(0, cell.sync_count-1)
                    cell.exploring_synced_count[robot] = 0
                    cell.explored_synced_count[robot] = 0
        self.log('region_sync', region_id=update.id, sender=sender,
                 incoming_update_id=update.update_id, update_id=cell.update_id,
                 known_by_robot_ids=cell.known_by_robot_ids, sync_count=cell.sync_count)

    def updates(self, connected_regions, global_regions=()):
        result = []
        for region, cell in enumerate(self.regions):
            synced = cell.known_by_robot_ids == (1 << self.config.robots)-1
            if synced and cell.sync_count >= 5 and region not in global_regions:
                if cell.status in COVERED or cell.covered_to_others or cell.status in ACTIVE:
                    self.log('region_transmission_suppressed', region_id=region,
                             update_id=cell.update_id, sync_count=cell.sync_count)
                continue
            if cell.status in COVERED or cell.covered_to_others:
                status = CellStatus.COVERED
            elif cell.status in ACTIVE and region in connected_regions:
                status = CellStatus.EXPLORING
            else:
                continue
            result.append(RegionUpdate(region, cell.update_id, cell.known_by_robot_ids,
                                       cell.mobility_type, int(status)))
            if synced:
                cell.sync_count = min(1000, cell.sync_count+1)
        return tuple(result)
