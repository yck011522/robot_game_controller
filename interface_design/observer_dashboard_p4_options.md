# P4 Observer Dashboard

Observer-facing full-screen dashboard for P4. This is not the player UI and not the legacy Tk control panel.

## Fixed constraints

- Team A is always on the right.
- Team B is always on the left.
- Team colors are profile-configurable. Current default: Team A = blue, Team B = red.
- Joint swimlanes are vertical. Positive direction is up, negative is down.
- Each swimlane uses the robot joint hard limits as its full range.
- The per-joint proximity strip is the dynamic collision-aware motion allowance around the current pose.
- Dashboard reads target pose from `state.full`, not from `cmd.robot.target.<team>`.
- `cmd.robot.target.<team>` remains on the bus for RobotIO and debugging; it is not the UI source of truth.
- Disabled teams or disconnected axes should render as greyed-out swimlanes.
- Pause/fault state uses a centered semi-transparent overlay that asks for game-master intervention.

## P4 data dictionary

Canonical source means the intended `state.full` schema from `docs/architecture/BUS.md`. Temporary source means the smaller shape currently emitted by `src/apps/game_controller/__main__.py`.

| Widget | Canonical `state.full` source | Temporary source today | Units | Display rule |
|---|---|---|---|---|
| Global stage | `stage` | `stage` | enum | Large center label. Show `paused` state distinctly. |
| Global pause reason | `pause_reason` | `pause_reason` | string/null | Used inside overlay message. |
| Global timer | `stage_t_s` now, later stage-specific remaining time from GC | not available yet | seconds | Large centered timer. P4 can start with elapsed play time until GC publishes remaining game time. |
| Active teams | `active_teams` | inferred from `teams` keys | team list | Grey out lanes for inactive team. |
| Team score total | `score.a`, `score.b` | not available yet | integer points | Large per-team score block. Needs GC to publish canonical score block. |
| Bucket scores / weights | `buckets.11..23` and later bucket points if distinct | not available yet | grams or points | Can render raw bucket values first, then point mapping once hardware path lands. |
| Team robot actual q | `robots.<team>.q_actual` | `teams.<team>.robot.q_rad` | radians | Convert to lane marker position. |
| Team robot target q | `robots.<team>.q_target` or `q_planned` | `teams.<team>.robot.q_target_rad` | radians | Render as target marker or velocity arrow anchor. |
| Team haptic dial pose | `haptic.<team>.dial_pos` | `teams.<team>.haptic.dial_pos_rad` | radians | Optional secondary marker if distinct from robot target semantics. |
| Team haptic board loop rate | `haptic.<team>.board_loop_hz[i]` | `teams.<team>.haptic.board_loop_hz[i]` | Hz | Small per-lane diagnostic for dial board health. |
| Axis connection state | `haptic.<team>.connected[i]` and robot-side connectivity if available | `teams.<team>.haptic.connected[i]` | bool | Grey out a single swimlane when one dial/axis is unavailable. |
| Team collision state | `teams.<team>.collision` equivalent should become canonical under team summary | `teams.<team>.collision.in_collision` | bool | Team badge and lane highlight. |
| Team first forward hit | canonical team collision summary | `teams.<team>.collision.first_hit.distance_deg` | degrees | Annotate forward-path bar. |
| Team path scalar | canonical team collision summary | `teams.<team>.collision.path_scalar` | 0..1 | Show as percent. |
| Team proximity scalar | canonical team collision summary | `teams.<team>.collision.prox_scalar` | 0..1 | Show as percent if needed in diagnostics. |
| Team final speed scalar | canonical team collision summary | `teams.<team>.collision.final_scalar` | 0..1 | Large per-team percent readout. |
| Current pose free/collision | derived from team collision + first forward sample semantics | `teams.<team>.collision.in_collision` | bool | Team-level FREE / COLLISION badge. |
| Process age | `process_health.<proc>` | not available yet | milliseconds | White when healthy, orange when stale. |
| Process achieved FPS / Hz | not in canonical snapshot yet | available now on `heartbeat.<proc>.loop_hz` | Hz | If dashboard stays `state.full`-only, GC or a health aggregator must fold this in. Otherwise the dashboard can subscribe to `heartbeat.*` directly. |
| Collision checks per second | not in canonical snapshot yet | available now on `heartbeat.collision_worker_* .checks_per_sec` | checks/s | Already present on collision-worker heartbeats; no launcher-specific message is needed. |

## P4-specific display semantics

### Swimlane contents

Each of the 12 lanes should support these layers, from back to front:

1. Hard-range lane background from robot joint min to max.
2. Proximity strip centered on current pose, showing sampled collision results around that pose.
3. Current robot angle marker.
4. Command direction indicator from current position toward target position. Preferred form: arrow up or down.
5. Numeric actual angle readout near the lane header or footer.

If an axis is disconnected, dim the lane and replace active markers with a muted placeholder.

### Pause overlay

When `paused == true`, render a semi-transparent black overlay across the full screen with:

- Stage headline: `PAUSED`
- Reason line: friendly text derived from `pause_reason`
- Action line: `Game master intervention required`
- Optional secondary hint: `Check robot state, safety barrier, or admin controls`

### Diagnostics colors

- Normal text: white
- Warning text: orange
- Inactive / unavailable: grey

Proposed warning rules:

- FPS orange when achieved rate is below configured minimum.
- Age orange when stale above configured threshold.

### Process diagnostics source

Chosen approach for P4:

1. Dashboard subscribes to `state.full` for game state.
2. Dashboard subscribes to `heartbeat.*` for diagnostics.

This keeps the UI on a pure subscribe model for runtime reads. The only
publish path the dashboard needs is its own `heartbeat.gamemaster_ui`
back to the rest of the system.

The launcher is not inventing these values. It subscribes to `heartbeat.*`
and prints:

- `loop_hz` reported by each process in its heartbeat body
- `checks_per_sec` reported by collision workers in their heartbeat body
- heartbeat age and observed heartbeat rate computed locally from heartbeat arrival times

So with the chosen approach, the dashboard can already show:

- per-process reported loop rate from `heartbeat.<proc>.loop_hz`
- collision-worker throughput from `heartbeat.collision_worker_* .checks_per_sec`
- per-process age in ms computed locally from the latest heartbeat arrival time
- observed heartbeat rate computed locally from heartbeat arrival times if we want that as a secondary diagnostic

What this does not automatically provide:

- a single merged `state.full` diagnostics block
- per-process metrics that are not already present in heartbeat bodies
- dashboard render FPS itself, which the dashboard should measure locally

If later we want more diagnostics than `loop_hz` and collision-worker `checks_per_sec`, the clean extension is to add more fields to the relevant heartbeat bodies. Using the launcher as a new publisher is not required.

## Proposed config additions

These do not need to block the first mockup, but they define the dashboard color/warning behavior cleanly.

```yaml
tuning:
  dashboard:
    team_colors:
      a: "#2D6CDF"
      b: "#D94C3A"
    diagnostics:
      stale_warn_ms: 250
      process_min_hz:
        game_controller: 45
        robot_io: 90
        haptic_io: 45
        collision_broker: 1
      collision_checks_warn_cps: 30
```

## Layout option 1: Arena Mirror

Best when bucket scoring should echo the physical arena in the center.

```text
+----------------------------------------------------------------------------------+
| TEAM A HEADER             TIMER / STAGE / PAUSE              TEAM B HEADER       |
| score total A            large center block                 score total B        |
|                                                                              Hz  |
| A1  A2  A3                  central bucket arena              B1  B2  B3          |
| [lane][lane][lane][lane][lane][lane]  [diag spine]  [lane][lane][lane][lane]... |
| [lane][lane][lane][lane][lane][lane]  [proc table]  [lane][lane][lane][lane]... |
| team A path bar + final %                               team B path bar + final % |
+----------------------------------------------------------------------------------+
```

Why choose it:

- Strongest spectator read of left team vs right team.
- Center block is available for timer and the arena-like bucket layout.
- Easy to add a full-screen pause overlay.

## Layout option 2: Buckets Over Lanes

Best when the 12-lane data should dominate and bucket state can sit directly above each team.

```text
+----------------------------------------------------------------------------------+
| TEAM A total  A-left  A-mid  A-right | TIMER / STAGE | B-left  B-mid  B-right  TEAM B total |
|----------------------------------------------------------------------------------|
| A lane1  A lane2  A lane3  A lane4  A lane5  A lane6 | B lane1  B lane2 ... B lane6 |
| A lane1  A lane2  A lane3  A lane4  A lane5  A lane6 | B lane1  B lane2 ... B lane6 |
|----------------------------------------------------------------------------------|
| Team A FREE/COLLISION  final %  path bar     diagnostics     path bar  final % Team B |
+----------------------------------------------------------------------------------+
```

Why choose it:

- Cleanest mapping from player lanes to bucket outcomes.
- Most space-efficient for 12 lanes.
- Simple fixed-coordinate implementation.

## Layout option 3: Central Spine

Best when the timer, stage, and diagnostics need to feel like the command core of the screen.

```text
+----------------------------------------------------------------------------------+
| TEAM A PANEL          |         CENTRAL SPINE          |           TEAM B PANEL   |
| total score           |         TIMER LARGE            |           total score     |
| bucket triplet        |         STAGE LABEL            |           bucket triplet  |
| path bar              |         process table          |           path bar        |
| final %               |         pause reason           |           final %         |
| lanes lanes lanes     |         thin divider           |           lanes lanes     |
| lanes lanes lanes     |         thin divider           |           lanes lanes     |
+----------------------------------------------------------------------------------+
```

Why choose it:

- Strong audience focus on match state.
- Diagnostics can live in the spine without polluting the team panels.
- Good if the bucket graphic becomes more elaborate later.

## Layout option 4: Broadcast Stack

Best when you want a TV-style hierarchy: top for match state, middle for action, bottom for system status.

```text
+----------------------------------------------------------------------------------+
| TEAM A score | buckets |                 TIMER / STAGE                 | buckets | TEAM B score |
|----------------------------------------------------------------------------------|
| Team A lane 1..6                              | Team B lane 1..6                  |
| Team A lane 1..6                              | Team B lane 1..6                  |
|----------------------------------------------------------------------------------|
| Team A final % | Team A path bar | pause/help text | Team B path bar | Team B final % |
|----------------------------------------------------------------------------------|
| process health table | collision worker cps | haptic / robot age | misc diagnostics |
+----------------------------------------------------------------------------------+
```

Why choose it:

- Easiest spectator read from a distance.
- Diagnostics stay low-priority at the bottom.
- Good fit for 4k and 2560x1440 scaling.

## Recommendation

Start with option 1 or option 2.

- Option 1 if the center bucket/arena relationship matters most.
- Option 2 if the 12 swimlanes should be the dominant visual object.

Both options preserve the visual language from the existing player mockups while staying compatible with fixed-coordinate pygame layout.