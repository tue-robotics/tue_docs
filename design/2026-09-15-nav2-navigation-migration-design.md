# Migrating tue-robotics base navigation to ROS 2 / Nav2

**Date:** 2026-09-15
**Status:** Draft, awaiting review
**Scope:** `cb_base_navigation`, `cb_base_navigation_msgs`, `ed_navigation`, and the
`NavigateTo` entry point in `robot_smach_states`.

## 1. Goal

Replace the custom TU/e base-navigation stack with Nav2, without losing
constraint-based goal specification, and with as little custom code as the result
allows. Custom code that remains must live in supported Nav2 extension points
(pluginlib plugins, behavior-tree nodes, action clients) rather than in forks.

Secondary requirement, stated during design: **the real robot and the simulator
must behave the same.** This is a constraint on how configuration is structured,
not an afterthought — see §8.

## 2. What exists today

### 2.1 cb_base_navigation

Three separable features, not one:

1. **The goal is a region, not a pose.** `PositionConstraint{frame, constraint}`
   carries an exprtk expression over `x, y`. `AStarPlannerGPP::makePlan`
   rasterises it over the global costmap, buckets the matching cells into
   free / low-cost / high-cost, and runs a **multi-goal A\***
   (`planner_->plan(mx_goal, my_goal, mx_start, my_start, ...)` takes a *vector*
   of goal cells), retrying with worse buckets when a bucket yields no path.
2. **Yaw decoupled from the path.** `OrientationConstraint{frame, look_at,
   angle_offset}` rides on the `LocalPlanner` action; the base yaw is
   recomputed each cycle to face a point in a possibly-moving frame.
3. **Constraint frames may be ED entities**, resolved through `ed/simple_query`
   rather than TF.

`global_planner_interface` exposes `get_plan_srv` and `check_plan_srv` as plain
services. It has **no internal timer**: all replanning cadence is the caller's
responsibility.

### 2.2 How arrival is decided today

`LocalPlannerInterface::doSomeMotionPlanning()`:

```cpp
// 2) Check if we are already there
if (local_planner_->isGoalReached()) { ... action_server_->setSucceeded(); }
```

That is the stock `base_local_planner` check against **the last pose of the
plan** — the single cell the multi-goal A\* selected. The constraint plays no
part in the arrival test. `base.py`'s `__doneCallback` maps this to `"arrived"`.

### 2.3 How replanning is driven today

`robot_smach_states/navigation/navigation.py` (337 lines). The replanning logic
in it is largely **dead code**:

- `checkBetterPlan()` — the ≥20 % *and* ≥1 m hysteresis swap — is commented out
  at its only call site (`#self.checkBetterPlan()`).
- `breakOut()` returns `'passed'` unconditionally; the goal-handle logic beneath
  it is commented out. Exactly one subclass overrides it
  (`navigate_to_explore.py`).
- `checkPlan` / `check_plan_srv` is called only from `follow_operator.py` and
  `follow_operator2_0.py`, never from `NavigateTo`.

The live behaviour is therefore: **plan once in `GET_PLAN`, drive until arrived
or blocked.** The sole replan trigger is `PLAN_BLOCKED` — stop, poll for up to
3 s at 2 Hz, then re-`getPlan` with the same constraints. A moving constraint
frame is not tracked during execution.

Because `GET_PLAN` and `EXECUTE_PLAN` are sequential smach states, replanning
structurally cannot overlap with driving.

### 2.4 The constraint language actually in use

Six generators in `robot_smach_states/navigation/constraint_functions/`, plus
`ed_navigation`:

| Source | Produces | Frame |
|---|---|---|
| `symbolic_constraints.py` | `ed.navigation.get_position_constraint()` | `map` |
| `ed_navigation` C++ (`constructConstraint`) | `-(x-xs)*dy+(y-ys)*dx > 0` conjunctions = dilated convex hulls, `or`-ed across sub-shapes | `map` |
| `radius_constraints.py` | dilated convex hull (same half-plane form), or annulus | `map` |
| `arms_reach_constraints.py` | annulus (`ri and ro`) | `map` |
| `pose_constraints.py` | disc | **caller-supplied `frame_id`** |
| `waypoint_constraints.py` | disc | `map` |
| `look_at_constraints.py` | orientation only (`pc = None`) | entity pose frame |
| `compound_constraints.py` | string concatenation with `" and "`; asserts equal frames | — |

A further eight sites build `PositionConstraint` directly rather than through
a generator. All stay inside the same language:

| Site | Shape | Frame |
|---|---|---|
| `manipulation/open_door.py` | annulus | `map` |
| `manipulation/place_designator.py` | annulus | `frame_stamped` — always `map` in practice (lines 75, 277) |
| `navigation/navigate_to_explore.py` | dilated hull `and` N exclusion discs | `map` |
| `navigation/door_opening.py` | disc | `map` |
| `challenge_navigation` | disc | `""` (unset) and `"/map"` |
| `challenge_dishwasher/simple_grab.py` | annulus | **`amigo/torso_laser` frame** (see §9, risk 2) |
| `robot_skills/base.py` `move()` | raw caller string | caller — **no callers in the tree** |
| `robot_skills/base.py` `turn_towards()` | disc of 0.1 m at current pose | `map` |

`navigate_to_explore` is the one shape that combines a polygon with exclusion
discs, and it is the reason `ConvexArea` carries both (§4.2). `base.move()` is
the only entry point for an arbitrary expression, and nothing calls it.

**There is no arbitrary mathematics anywhere.** The entire live language is:

- **primitives:** convex polygon (as a half-plane conjunction), disc, annulus
- **combinators:** `and` (intersection), `or` (union, only from ED composite shapes)

This is the single most important finding in this document. It means exprtk is
not needed in the ROS 2 stack.

### 2.5 Migration context

- Workspace `/home/amigo/ros/jazzy/system/src` already contains migrated `ed`,
  `ed_interfaces`, `geolib2`, `rgbd`, `tue_config`, `code_profiler`.
- Established conventions: one `ros2` PR per repo; message packages renamed
  `*_msgs` → `*_interfaces`.
- **Nav2 is not yet in `.env/targets`.**
- No `ros2` branch exists on `cb_base_navigation`, `ed_navigation`,
  `tue_robocup`, or the `executive_smach` fork. Upstream `ros/executive_smach`
  does have a `ros2` branch.

## 3. What Nav2 provides, and what it does not

Verified against `ros-navigation/navigation2` at `jazzy`, `kilted` and `main`.

| Capability | Nav2 |
|---|---|
| Goal region / constraint | **Absent.** `nav2_core::GlobalPlanner::createPlan(start, goal, cancel_checker)` takes one `PoseStamped`. |
| Goal *set* (cheapest of N) | **Absent.** `ComputePathThroughPoses` takes `nav_msgs/Goals`, but sequentially; `viapoints` on rolling is also sequential. |
| Nearest approximation | planner `tolerance` — closest reachable cell within a radius, i.e. the degenerate circular case. |
| Look-at-a-moving-frame while driving | **Absent.** MPPI ships `goal_angle_critic`; there is no look-at-point critic. |
| Arrival test | `nav2_core::GoalChecker`, a pluginlib plugin — **extension point we will use.** |
| Replanning + recovery | `bt_navigator` + behavior trees — **replaces §2.3 wholesale.** |
| Goal changed mid-flight | `GoalUpdater` decorator, `GlobalUpdatedGoal` / `GoalUpdated` conditions. |
| Useful neighbours | `RemoveInCollisionGoals`, `nav2_following`/`FollowObject`, `nav2_route`, `nav2_docking`. |

So: the **planner** has no equivalent of our constraints and the **arrival test**
has no equivalent either, but both are clean plugin points. The **replanning and
recovery layer** is strictly better than ours and should simply be adopted.

## 4. Design

### 4.1 Overview

```
 constraint_functions (Python, robot_smach_states)
            │  ConstraintRegion (structured, no strings)
            ▼
 ed_navigation ──► GetGoalConstraint  ──► ConstraintRegion
            │
            ▼
 ┌───────────────────────────────────────────────────┐
 │ tue_nav_constraints  (predicate + sampler, C++/py)│  ← shared library
 └───────────────────────────────────────────────────┘
        │                                   │
        ▼                                   ▼
 goal_resolver node                 RegionGoalChecker
  (region → candidate poses)        (nav2_core::GoalChecker plugin)
        │                                   │
        │ {goal} via GoalUpdater            │ selected via GoalCheckerSelector
        ▼                                   ▼
 ┌───────────────────────────────────────────────────┐
 │           Nav2: bt_navigator / planner /          │
 │           controller / costmaps (stock)           │
 └───────────────────────────────────────────────────┘
        ▲
        │ NavigateToPose action
 NavigateTo shim (thin, executive-agnostic)
```

### 4.2 Decision 1 — replace the expression language with a structured type

Given §2.4, define messages in a new `tue_nav_constraints_interfaces` package:

```
# ConstraintRegion.msg
std_msgs/Header header     # frame_id: "map" or an ED entity id
ConvexArea[] areas         # union (OR); empty ⇒ unsatisfiable
```

```
# ConvexArea.msg
# Membership = inside ALL polygons AND inside ALL inclusions AND outside ALL exclusions
ConvexPolygon[] polygons
Disc[] inclusions
Disc[] exclusions
```

```
# ConvexPolygon.msg
geometry_msgs/Point[] vertices    # counter-clockwise; interior is left of every edge
```

```
# Disc.msg
geometry_msgs/Point center
float64 radius
```

`geometry_msgs/Polygon` is deliberately **not** reused: it is built on
`Point32`, and mixing `float32` vertices with `float64` radii and costmap
coordinates invites precision bugs at map scale. `ed_navigation` already writes
its half-planes at `setprecision(6)` fixed; keeping everything `float64` means
the structured form is never the lossy one.

Mapping from §2.4:

- ED dilated convex hull → one `ConvexArea` per sub-shape, unioned.
- `radius_constraints` with hull → one `ConvexArea` with the dilated polygon.
- annulus (`radius_constraints` fallback, `arms_reach`) →
  `inclusions=[outer]`, `exclusions=[inner]`.
- disc (`pose`, `waypoint`) → `inclusions=[disc]`.
- `combine_position_constraints` (`and`) → intersection. Because `ConvexArea`
  holds a *list* of polygons AND-ed together, intersecting two areas is list
  concatenation; no convex-polygon clipping is needed. Intersecting two unions
  is the union of pairwise intersections.

Why a polygon *list* rather than a single clipped polygon: it makes intersection
total and allocation-free, and the predicate cost stays linear in the number of
half-planes — the same work exprtk was doing, without a parser.

Rationale for replacing exprtk rather than porting it: the expression string is
an unvalidated, untyped interface that we parse on the robot at runtime; every
producer in the tree already thinks in polygons and discs, and every consumer
(sampler, predicate, visualiser) wants the structure back. Keeping strings means
re-deriving structure that we threw away one function earlier.

### 4.3 Decision 2 — goal candidates, not a goal-region planner

`tue_nav_constraints` provides:

- `bool contains(const ConstraintRegion&, double x, double y)` — the predicate.
- `std::vector<geometry_msgs::Pose> sample(region, costmap, options)` — candidate
  goal poses.

Sampling strategy, in order of preference per `ConvexArea`:

1. Analytic where the shape allows it — for a disc or annulus, sample rings at
   the preferred radius rather than rasterising an area.
2. Otherwise, an axis-aligned bounding box from polygon vertices and inclusion
   discs, sampled on a lattice at costmap resolution, rejecting non-members.

Candidates are then filtered and ordered by costmap cost using the same
three-bucket rule as `AStarPlannerGPP` (free / `< INSCRIBED_INFLATED_OBSTACLE/4`
/ rest), and capped at `max_candidates` (default 50).

The `goal_resolver` node publishes the best candidate for `GoalUpdater`, and the
behavior tree plans to it with a stock planner. Where multiple candidates must be
compared by *path length* rather than Euclidean distance, the tree calls
`ComputePathToPose` per candidate and keeps the shortest.

Rejected alternative: porting the multi-goal A\* as a `nav2_core::GlobalPlanner`.
It is truer to the original and performs one search instead of N, but
`createPlan` has no parameter for a region, so the constraint must arrive
out-of-band via a topic or parameter — permanently at odds with the interface.
Revisit only if candidate-wise planning proves too slow in practice; Smac and
NavFn plan in milliseconds, so N ≈ 50 should be affordable. **This is a measured
decision, not an assumed one — see §9.**

### 4.4 Decision 3 — arrival is tested against the region

A `RegionGoalChecker` implementing `nav2_core::GoalChecker`:

```cpp
bool isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,   // robot pose
  const geometry_msgs::msg::Pose & goal_pose,    // ignored for position
  const geometry_msgs::msg::Twist & velocity,
  const nav_msgs::msg::Path & transformed_global_plan) override;
```

It ignores `goal_pose` for the positional test and calls
`tue_nav_constraints::contains()` on `query_pose`, falling back to
`SimpleGoalChecker` semantics when no region is active. Yaw and velocity
tolerances behave as in `SimpleGoalChecker`.

Also required by the interface, and easy to get wrong:

- `getTolerances()` must report the region's bounding-box half-extents (not
  `lowest()`), because `IsGoalNearby` and the rotation-shim controller consume it.
- `isGoalXYReached()` must be implemented alongside `isGoalReached()`.

This makes arrival **strictly more faithful than the current stack** (§2.2),
because the same predicate decides both where we plan to and where we accept
arrival — today those can disagree.

Limitation to record: a region goal checker can only stop the robot *early*, when
it enters the region. It cannot redirect. Tracking a region that moves away
remains the replanning layer's job.

### 4.5 Decision 4 — dynamic constraint frames

`ConstraintRegion.header.frame_id` may name an ED entity. The `goal_resolver`
resolves it exactly as `AStarPlannerGPP::queryEntityPose` does today — via ED,
with `map` short-circuited to identity — and falls back to TF when the id is a
real frame. We do **not** push all ED entities into TF; only the active
constraint's frame is ever needed.

The resolver re-resolves at a configurable rate (default 1 Hz, matching the
tree's `RateController`) and republishes the goal. `GlobalUpdatedGoal` then
forces a replan. This gives the stack the moving-region tracking that
`cb_base_navigation` was designed for but, per §2.3, does not currently perform.

**Only ED entity frames are re-resolved.** A TF frame is transformed to `map`
**once**, when the goal is accepted, and the region is frozen there. Otherwise
a region expressed in a frame attached to the robot, such as
`simple_grab.py`'s laser frame (§2.4), would move with the robot and could never
be reached. `cb_base_navigation` got this right by accident: it resolves the
frame once, inside `getPlan`, and never again.

### 4.6 Decision 5 — the orientation constraint

**HERO drives holonomically** — confirmed 2026-09-15. This matches
`robot_skills/base.py`, which exposes `force_drive(vx, vy, vth, ...)` and
populates `v.linear.y`. The decoupled look-at yaw is therefore physically
meaningful and is preserved.

Two consequences for Nav2 configuration, both belonging in the shared parameter
file per §8:

- The controller must be holonomic-capable. **MPPI** (`nav2_mppi_controller`)
  supports omnidirectional motion models; Regulated Pure Pursuit does not.
  `vy_max` must be set alongside `vx_max` and `wz_max`.
- The global planner need not be kinematically constrained. `SmacPlanner2D` or
  NavFn suffices and is cheaper than Hybrid-A*, whose Dubins/Reeds-Shepp models
  exist to respect a turning radius HERO does not have.

Three tiers, cheapest first:

1. **Static look-at** (`look_at` resolves in `map` and does not move during the
   motion): bake the yaw into the candidate pose in `goal_resolver`. No custom
   code. This covers `pose_constraints`, `waypoint_constraints` and
   `arms_reach_constraints`, all of which already pass `frame="map"`.
2. **Moving look-at**: a custom MPPI critic (~150 lines) penalising heading error
   toward a TF/ED frame. Needed only if tier 1 proves insufficient in practice.
3. **Person following**: try `nav2_following`'s `FollowObject` with
   `tracked_frame` before writing anything. Out of scope here (§10), noted so
   `follow_operator` work does not duplicate tier 2.

### 4.7 Decision 6 — replanning and recovery come from Nav2

Delete `getPlan`, `executePlan`, `planBlocked`, and the dead `checkBetterPlan` /
`breakOut` bodies. `NavigateTo` becomes a thin shim around `NavigateToPose`.

Per §2.3, behaviour parity with today is
`navigate_w_replanning_only_if_path_becomes_invalid.xml` — replan at 1 Hz only
when `ValidatePath` fails or `GlobalUpdatedGoal` fires, with `PipelineSequence`
so planning overlaps driving:

```xml
<PipelineSequence name="NavigateWithReplanning">
  <RateController hz="1.0">
    <Fallback>
      <ReactiveSequence>
        <Inverter><GlobalUpdatedGoal/></Inverter>
        <ValidatePath path="{path}"/>
      </ReactiveSequence>
      <ComputePathToPose goal="{goal}" path="{path}" .../>
    </Fallback>
  </RateController>
  <FollowPath path="{path}" .../>
</PipelineSequence>
```

The target end state is `navigate_to_pose_w_replanning_and_recovery.xml`, which
adds `RecoveryNode` per stage and a `RoundRobin` of clear-costmaps → `Spin` →
`Wait` → `BackUp` with six retries — none of which we have today. Moving between
the two is a parameter change (`default_nav_to_pose_bt_xml`), which is what makes
the phasing in §7 low-risk.

`breakOut()` is preserved by cancelling the `NavigateToPose` goal from the shim,
not by a custom BT condition node. One subclass uses it; a BT node is not worth
the maintenance.

### 4.8 Decision 7 — the shim is executive-agnostic

The executive's long-term future (smach on upstream's `ros2` branch,
BehaviorTree.CPP, or Yasmin) is explicitly undecided. Therefore:

- The navigation-facing API is a **plain `rclpy` action client class** —
  `tue_nav_client.NavigateClient` — with no smach import.
- `NavigateTo` is a smach `State` wrapping that client, preserving the existing
  `'arrived'` / `'unreachable'` / `'goal_not_defined'` outcome contract so the
  ~10 `navigate_to_*.py` subclasses compile unchanged.
- Nav2 error codes map to that contract in one place:
  `NO_VALID_PATH`, `GOAL_OCCUPIED`, `GOAL_OUTSIDE_MAP`, `START_OCCUPIED` →
  `'unreachable'`; an empty or unsatisfiable region → `'goal_not_defined'`;
  success → `'arrived'`.

Swapping the executive later touches one file.

### 4.9 Decision 8 — plan-only queries and out-of-band status

`NavigateTo` is not the only consumer of the planner. The ROS 1 `global_planner`
and `local_planner` proxies in `robot_skills/base.py` are also used directly:

| Use | Callers | ROS 2 replacement |
|---|---|---|
| plan without driving, for reachability or path length | `place_designator` (ranks placement spots by path length), `give_directions` (describes the route aloud), `door_opening`, `challenge_navigation` | `NavigateClient.compute_path(region)` |
| cancel whatever the base is doing | `guidance`, `challenge_following_and_guiding`, `hmc_states` | `NavigateClient.cancel_all()` |
| poll the status of a navigation started elsewhere | `guidance` (runs concurrently with the navigating state) | `NavigateClient.status` |
| path length after arrival | `navigation.py` (`reset_pose` if > 0.5 m) | `NavigateToPose` feedback |

`compute_path` goes through the same `goal_resolver` sampling as navigation and
calls Nav2's `ComputePathToPose` action on `planner_server` directly, with no
behavior tree. `place_designator` is the most frequent caller, once per
placement candidate, so it goes through the same cost measurement as §9 risk 1.

`cancel_all()` uses the action protocol's cancel-all-goals request, so a state
can stop the base without holding the goal handle — which is how these callers
use `cancelCurrentPlan()` today. `NavigateClient` is therefore **one instance
per robot**, held on the robot object as `base.global_planner` and
`base.local_planner` are today, not one per state.

## 5. Packages

| Package | Contents | Language |
|---|---|---|
| `tue_nav_constraints_interfaces` | `ConstraintRegion`, `ConvexArea`, `Disc` | IDL |
| `tue_nav_constraints` | `contains()`, `sample()`, costmap bucketing, RViz markers | C++ + Python bindings |
| `tue_nav_goal_resolver` | resolver node: region → candidate poses, ED/TF frame resolution, `GoalUpdater` publisher | C++ |
| `tue_nav_goal_checkers` | `RegionGoalChecker` (`nav2_core::GoalChecker` plugin) | C++ |
| `tue_nav_bringup` | Nav2 params, BT XMLs, launch, sim/real overlays | YAML/XML |
| `tue_nav_client` | `NavigateClient` action wrapper, `compute_path`, `cancel_all`, error-code mapping | Python |
| `ed_navigation` (ported) | `GetGoalConstraint` returning `ConstraintRegion` | C++ |
| `robot_smach_states` (edited) | `NavigateTo` shim; `constraint_functions` emit structured regions | Python |

Retired: `cb_base_navigation`, `cb_base_navigation_msgs`.
Naming follows the `ed` / `ed_interfaces` convention; the `tue_nav_` prefix is a
proposal, not a requirement.

## 6. Testing

**Differential testing against exprtk is the core of this plan.** Keep exprtk as
a **test-only** dependency:

- For every constraint the existing generators can emit, produce both the legacy
  expression string and the new `ConstraintRegion`.
- Sample points over the bounding box (lattice plus randomised) and assert
  `exprtk_eval(string, x, y) == contains(region, x, y)` for every point.
- Seed the corpus from the real generators and from `ed_navigation` against
  recorded ED worlds, so the tested shapes are the shapes we actually fly.

This converts "did we reimplement the constraint semantics correctly?" from a
judgement call into a test, and it is the reason replacing exprtk is safe rather
than merely appealing.

Additionally:

- Unit tests for `sample()`: every returned pose satisfies `contains()`; no pose
  exceeds the cost bucket it claims; `max_candidates` is honoured; an
  unsatisfiable region returns empty rather than throwing.
- `RegionGoalChecker`: entering the region reports reached; leaving it reports
  not-reached; `getTolerances()` returns a box that encloses the region.
- Port `robot_smach_states/test/navigation/constraint_functions/*` — these
  already exist and should keep passing against the structured output.
- System tests in simulation per §8, asserting identical outcomes to the recorded
  legacy behaviour on a fixed set of goals.

## 7. Phasing

Each phase is independently mergeable and leaves the robot in a working state.

| Phase | Work | Done when |
|---|---|---|
| 0 | Nav2 into `.env/targets`; `tue_nav_bringup` with HERO params; sim + real launch | Nav2 drives HERO to an RViz goal in both sim and real |
| 1 | `tue_nav_constraints_interfaces` + `tue_nav_constraints` + differential test harness (§6) | exprtk-equivalence suite green over the full corpus |
| 2 | `ed_navigation` ROS 2 port; `GetGoalConstraint` returns `ConstraintRegion` | ED emits structured regions; equivalence suite green against recorded worlds |
| 3 | `constraint_functions` emit `ConstraintRegion`; `tue_nav_goal_resolver`; `tue_nav_goal_checkers` | Resolver publishes candidates; region goal checker passes unit tests |
| 4 | `tue_nav_client` + `NavigateTo` shim on the **parity** BT (§4.7); plan-only and cancel callers (§4.9) ported | `navigate_to_*.py` subclasses pass their existing tests; no caller outside `follow_operator*` touches `global_planner` / `local_planner` |
| 5 | Switch to `navigate_to_pose_w_replanning_and_recovery.xml`; tune costmaps; look-at MPPI critic **only if** §4.6 tier 1 proves insufficient | Recovery behaviours verified in sim and on the robot |
| 6 | Delete `cb_base_navigation`, `cb_base_navigation_msgs`; drop their `.env/targets` | No references remain — **blocked on `follow_operator`, see below** |

Phases 1 and 2 carry the risk and are also the most testable without hardware.
Phase 0 should not wait on them.

**Phase 6 cannot complete within this spec's scope.** `follow_operator.py` and
`follow_operator2_0.py` are the only remaining callers of `check_plan_srv`, and
they are out of scope (§10). Every other direct planner caller is ported in
Phase 4 (§4.9). Phases 0–5 deliver the full `NavigateTo` migration
and leave `cb_base_navigation` running solely for the follow-operator states;
deletion happens once that separate spec lands. Plan for the two stacks to
coexist for that interval — it is a known, bounded cost, not an oversight.

## 8. Simulator / robot parity

Explicit requirement. Structure `tue_nav_bringup` so parity is enforced by
construction rather than by discipline:

- **One** `nav2_params.yaml` holding every planner, controller, goal-checker and
  behavior-tree setting. Sim and real both load it.
- A **single** overlay file per environment, restricted to sensor topics, frame
  names, and robot footprint source. Anything appearing in an overlay that is not
  in that list is a parity bug.
- The robot's kinematic limits (`vx_max`, `vy_max`, `wz_max`, acceleration) live
  in the shared file, never the overlay — they are properties of HERO, not of
  where it runs.
- A CI job runs the §6 system tests against the simulator on every PR; the same
  test script is runnable against the robot and is expected to produce the same
  pass/fail set.

Not sufficient on its own: `nav2_loopback_sim` bypasses the controller and
costmaps entirely, so it can validate the behavior tree and the goal checker but
cannot validate parity. Use it for fast BT tests, not as the parity harness.

## 9. Risks and open questions

1. **Candidate-wise planning cost.** N `ComputePathToPose` calls replace one
   multi-goal search. Mitigation: cap `max_candidates`, prefer analytic sampling,
   and measure in Phase 3. If the p95 planning time exceeds the 1 Hz replan
   budget, reconsider §4.3's rejected alternative. **Decide on measurement.**
2. ~~**`pose_constraints.py` passes a caller-supplied `frame_id`.**~~
   **Resolved 2026-09-22.** No caller of `pose_constraints()` or
   `NavigateToPose` passes `frame_id`; all use the `map` default. Across every
   `PositionConstraint` built in `tue_robocup` (§2.4), the only non-`map` frame
   in production code is `challenge_dishwasher/simple_grab.py`, which uses the
   frame of the AMIGO-only `/amigo/torso_laser/scan` topic. That is a
   robot-attached TF frame, not an ED entity, and it led to the freeze-on-accept
   rule in §4.5. ED entity frames appear only in *orientation* constraints
   (`look_at_constraints.py`). Consequence: the moving-region half of §4.5 has
   **no current production caller**. It stays in the design because
   `navigate_to_*` subclasses can reach it through `ed_navigation`, but Phase 3
   may implement ED re-resolution last.
3. **`compound_constraints` asserts equal frames.** The structured type could
   support mixed-frame intersection by transforming to `map` first. Deliberately
   not designed now — no current caller needs it.
4. **Executive undecided** (§4.8). Contained to `tue_nav_client` plus one shim
   file, but the longer it stays open the more `navigate_to_*.py` subclasses
   accumulate against the smach contract.
5. **Costmap tuning is not a port.** `cb_base_navigation`'s costmap parameters
   were tuned against its own planner. Expect Phase 5 to be real work, not
   configuration transcription.

## 10. Out of scope

Each needs its own spec:

- `follow_operator.py` / `follow_operator2_0.py` → likely `nav2_following` /
  `FollowObject`. These are the only current users of `check_plan_srv`, so
  `cb_base_navigation` cannot be deleted until they move (§7, Phase 6). **This is
  a sequencing dependency, not merely a related project**, and it is the one
  place where this spec's completion depends on work outside it.
- `topological_action_planner` → likely `nav2_route`.
- Docking and `nav2_docking`.
- Migration of the executive itself (§4.8).
