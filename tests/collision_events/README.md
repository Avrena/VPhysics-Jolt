# Collision callback snapshots

`prepare.py` extracts the event classes, effective-velocity functions, pair
clock and event tracker from the production listener. `test_events.cpp` uses
real Jolt bodies with a small Source object-interface shim. It checks oblique
and angular contact velocities, approach versus separation, preservation of
contact-time data after solving and player-controller teardown, owned callback
data after copy/move, bounded pair timing at long uptime, deterministic substep
keys, and all 64 worker-queue slots.

Generate the include with `python3 tests/collision_events/prepare.py generated`,
then compile `test_events.cpp` with `-Igenerated`, the matching Jolt headers and
archive, and the same ABI and instruction-set definitions as the wrapper build.
The extraction does not replace implementation with a parallel model.

These checks do not replace game-engine tests of callback delivery, damage,
drag, or the cost of dispatching callbacks.
