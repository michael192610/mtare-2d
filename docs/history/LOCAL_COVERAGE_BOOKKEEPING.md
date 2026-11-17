CURRENT DEFAULT HAS CHANGED: the user requested simple sample counts and a
return to 2 m viewpoints, retaining 3 m visited-neighborhood suppression.
See SIMPLE_COVERAGE_COUNTS.md for current behavior. The document below records
an earlier implementation stage and its historical validation.

# Current dense-viewpoint update

As of 12 November 2026, default local coverage uses a separate .6 m viewpoint
lattice, source-style 3 m visited neighborhoods and visited-candidate coverage
credit. 40/60/20 thresholds use documented area-equivalent sample weights, and
region count thresholds are 1/10. The occupancy map remains 2 m.
See DENSE_VIEWPOINTS_AND_COVERAGE.md and the current comparison for the latest
implementation. The remainder records the earlier fine-cell-sample stage and its
historical 1/2/1 thresholds; those are not the current default dense score rules.

# Local coverage bookkeeping: source-guided 2D adaptation

Implemented 12 November 2026. Scope: replace the frontier-only local score with
persistent ordinary coverage samples, separate frontier samples, and the original
style of coverage selection and region-state inputs. No sensing/SLAM, dynamics,
ROS, pursuit, convoy or rendezvous was added. The original M-TARE source is unchanged.

## What changed

The earlier planner used one set of unknown frontier-neighbor grid cells for both
ordinary and frontier scores. It could not represent an observed but uncovered
ordinary sample, and used one greedy selection queue.

The default stateful hierarchy now maintains, per robot:

- A persistent ordinary sample ID at each discovered free-cell centre.
- Separate observed and covered flags. A known sample is not automatically covered.
- Actual coverage accumulated only from the robot's executed positions (including
  previously visited positions). Planning a viewpoint never commits coverage.
- A separate dynamic frontier pool, represented by unknown-side patches adjacent
  to locally observed free frontiers. Their actual occupancy is never consulted.
- Per-viewpoint ordinary and frontier incidence lists rebuilt for the local
  window. Only samples visible from unvisited candidates enter the scored pools.
- Separate predicted masks inside a planning call; these do not modify the
  persistent coverage ledger or other robots' knowledge.

The sequence follows the original structure: check reusable viewpoints against
empty masks; credit reused and mandatory navigation viewpoints in predicted
masks; enqueue ordinary candidates first, otherwise frontier candidates; select
ordinary viewpoints with marginal-score updates; update frontier predictions;
select from the frontier queue; combine reused/navigation selections; solve the
local TSP. Ordinary work triggers ten route-selection trials; otherwise the
frontier/reuse/navigation branch runs once. Minimum route length chooses a trial.

As in the source, the frontier wrapper uses a strict `>` test on its initial top
score; candidate enqueue uses `>=`. Frontier greedy continuation also checks the
ordinary minimum. Live candidate sampling is bounded after every removal rather
than reproducing a possible stale-queue indexing problem.

Local completion means no additional coverage viewpoints were selected; a
navigation-only path can still remain. The existing team stopping check also
requires no global/local work or transit and stable knowledge across planning
updates. The existing 100% observation and cycle-limit exits remain unchanged.
Thus 100% observed in a result is not proof that every private ordinary sample
was covered or all robots independently declared local completion.

## Original source mapping

Original root: <reference-workspace>/mtare_planner/src/tare_planner/src.

| Original method | 2D implementation |
| --- | --- |
| planning_env/PlanningEnv::UpdateCoveredArea | coverage_bookkeeping.py: accumulate coverage from actual robot positions and historical visited positions. |
| PlanningEnv::GetUncoveredArea | coverage_planner.py context: separate ordinary/frontier pools and per-unvisited-viewpoint coverage lists. |
| ViewPointManager::GetViewPointCoveredPointNum and frontier equivalent | Explicit incidence sets, residual counts against separate predicted masks. |
| LocalCoveragePlanner::SolveLocalCoverageProblem | Reuse, mandatory navigation masks, ordinary/frontier branches, trial selection and local completion. |
| EnqueueViewpointCandidates | Ordinary threshold first; frontier queue only in the else branch. |
| SelectViewPoint/SelectViewPointFromFrontierQueue | Randomized top-k marginal-gain selection in two phases, preserving threshold comparison structure. |
| GridWorld::UpdateLocalCellStatus | Existing state machine now receives distinct ordinary-small, ordinary-big and frontier viewpoint counts. |

## Explicit 2D model choices and thresholds

Ordinary sample positions remain on the 2 m fine grid: one sample represents a
4 m^2 free-area patch. Frontiers represent potential unobserved boundary patches,
not already classified free terrain. Neither pool is a measured 3D point cloud.

Abstract discovery radius remains 7.5 m. Actual/predicted ordinary coverage uses
5 m by default (2/3 of discovery radius); frontier prediction uses discovery range.
This separates discovery from adequate ordinary coverage without simulating
sensors. With equal full-circle discovery and coverage radii, every discovered
sample would immediately be covered, making the ordinary queue degenerate.
This ratio is a documented modeling assumption, not a value claimed from M-TARE.
`--coverage-radius` can set another positive radius <= discovery range.

Default small/big ordinary gains are 1/2 samples. The frontier queue minimum and
region frontier-score threshold are both 1; region checks retain strict `>`.
The previous region frontier threshold was 0. Region-level cover/reopen counts
remain adapted to 1 because each region has at most nine 2 m viewpoints, unlike
about 100 nominal positions at original 0.6 m spacing. Reopening limits and state
transition structure are unchanged. These are not claimed numerically equivalent
to the indoor 40/60/20 surface/frontier point thresholds or >10 viewpoint guard.

The greedy top-k range is now 3, matching indoor.yaml (previously 5). Ten local
optimization trials are retained. Grid paths/TSP, global roadmap costs, compact
sharing, synchronized parallel global searches, movement and termination policy
were not otherwise changed during the coverage update. The then-existing
32-target cap was subsequently removed from the default baseline on 12 November
(max_targets = 0); an explicit positive cap remains an experiment option. Coverage dilation is not separately
modeled: the source indoor 0.5 m dilation is smaller than the 2 m sample spacing.

`--coverage-model frontier` keeps the former frontier-only local scoring algorithm
available for reference. To reproduce its older numerical defaults as well, use
an archived source snapshot; current defaults include top-k 3 and frontier score 1.
Flat and region-state-model frontier reference architectures remain available.

## Evidence and validation

coverage_bookkeeping.json saves persistent observed/covered/uncovered ordinary
sample IDs per robot. Local call files include both pools, both incidence lists,
previous/reused/navigation viewpoints, queue memberships, per-selection marginal
gains, selected trial and local TSP route. Metrics include per-robot sample counts.
Light blue continues to mean observed free occupancy, not ordinary coverage.

The independent auditor replays committed coverage from executed poses and local
observations; checks every coverage snapshot and frontier pool; recalculates
viewpoint scores; verifies queue classification, marginal gains/top-k choices,
mask updates, trial selection and final sample counts. It retains all prior
roadmap, protocol, state, proposal, local TSP and movement checks.

The exact run results and test counts are recorded in LOCAL_COVERAGE_RESULTS.txt.
These selected checks establish functional behavior, not universal equivalence or
parameter robustness across environments. Sampling/visibility, numerical
threshold translation, pose graph, asynchronous timing and the extra evaluator
completion exit remain explicit fidelity boundaries.
