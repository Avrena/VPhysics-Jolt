# September 12 SCPRP populated BRDM capture

This is a temporary, server-only **observer**, not another vehicle/solver fix.
It is separate from LVS and does not spawn entities, add bots, change physics,
wrap LVS methods, repair wheels, or send code/data to clients.

Deploy `lua/autorun/server/sv_ncg_lvs_incident_recorder.lua` as an optional addon.
It defaults on and expires at **2026-09-13 04:00 server local time** (SCPRP is
Asia/Shanghai). The native binary and LVS implementation remain unchanged.

## Public test: September 12, around 19:00 China time

1. After the final restart, spawn one BRDM in a clear, safe test area. The observer
   selects the first BRDM created, or the sole existing BRDM when loaded. It does
   not guess between multiple existing vehicles.
2. Check `ncg_lvs_recorder_status`. Confirm the target entity index matches the
   test vehicle, `last_tick` advances and `error` is absent. If necessary, select
   it with `ncg_lvs_recorder_watch <entity index>` (server console/superadmin only).
3. Establish a healthy low-population baseline: settle, start engine, briefly
   drive, brake/handbrake and release. Do not replace this vehicle as players join.
4. Repeat short, normal driving checks as the population grows, including around
   50, 75 and 100 real players. A parked vehicle alone does not exercise the known
   engine-start/brake path. No stressbots or NextBots are needed.
5. On the first visible failure, use `ncg_lvs_recorder_mark <short observation>`
   before repair/removal if safe, and note which action immediately preceded it.
   A `false` return means a capture is already pending, cooling down or capped;
   check status. The rolling checkpoint continues independently.

The same APIs are available to server-side inspection:
`NCG_LVS_RECORDER.Status()`, `.Watch(entity)`, `.Mark("observation")`.
Set `ncg_lvs_recorder_enabled 0` to stop; this does not remove/change the vehicle.

## Evidence and limits

- DATA directory: `garrysmod/data/ncg_lvs_incidents/`.
- One vehicle, at most eight wheels and 24 visited constraints per wheel.
  A sample is marked `truncated` if either limit is hit.
- Up to 25 Hz, hence every tick at the currently configured 22 Hz. A 256-frame
  ring covers about 11.6 seconds at full tickrate, longer under server slowdown.
- First observation, alternating rolling checkpoints every 15 seconds, and a
  final ring on entity removal, map cleanup, disable, expiry or orderly shutdown.
- Incident pre-window is saved immediately; post-window after five seconds.
  A 30-second capture cooldown prevents overlapping disk bursts. First engine
  start and first detected anomaly per 25-player population tier are captured;
  six anomaly, eight engine and eight manual captures maximum per watched target.
  An early persistent fault cannot consume all later population-tier slots.
- Files are capped at 4 MiB; all recorder JSON at 64 MiB. At capacity it reports
  an error and refuses additional writes. It never deletes old incident evidence.
  Copy evidence out and explicitly manage retention before another test session.
- On Lua failure it stops its tick observer and attempts to save its last ring.
  Status reports `stopped`/`error`. It does not try to repair or reset physics.
- A sudden crash/power loss may lose the newest checkpoint/event. Alternating
  checkpoints preserve an earlier ring; there is no claim of fsync durability.

Samples contain engine/brake/throttle/steer, player count, tick/wall timing,
chassis/wheel/master entity and physics transforms, local angular velocity,
mass/inertia/motion/sleep/collision state, wheel damage/repair/lock fields and
LVS's cached motion-controller commands. No player names or Steam IDs are stored.
`Force` is recorded as **angular-local** and `ForceAng` as **linear**, matching
the actual LVS return order. Wheel/master axis dot is `wheel.Forward · master.Right`.
Master translation intentionally stays at spawn; it is not a wheel-position target.

Constraint definitions are the **Lua-authored** values, not Jolt's internal
rest frames. Distance errors use each endpoint's physics-local anchor, matching
the constraint library. Springs may receive native `Fire` updates not reflected
in their Lua-authored length, so spring distance is recorded but is not used as
a failure trigger. Object-keyed UIDs preserve distinct HolyLib virtual constraints
even when all their `EntIndex()` values are zero. Geometry thresholds only trigger
evidence capture; they do not diagnose cause or alter any state.

`test_recorder.lua` is an isolated fixture: provide `SOURCE` with the observer
source. It replaces entity, hook, command, cvar, clock and disk APIs with fakes.
It checks buffer bounds/order, zero-index constraint identities, engine pre/post
captures, load-tier reservation, rod errors, nonfinite velocity serialization,
removal persistence, disable and expiry. It creates no real entities or hooks.
These tests do **not** establish real-vehicle behavior or populated-server overhead.
