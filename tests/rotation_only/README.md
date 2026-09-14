# Rotation-only constraint regressions

This fixture compiles the actual `InitialiseRagdoll` rotation-only branch,
`RagdollLimits_t`, and `PostSimulate` extracted by `prepare.cmake`. When testing
an older source revision it also extracts the actual delayed recapture method.
Missing source boundaries fail preparation rather than silently testing a copy
of the implementation.

The fixture supplies only the SDK input fields and out-of-scope engine services
needed by those functions. It links an existing, matching Jolt static library;
it does not require a Source SDK or Jolt rebuild. The shim is not an engine ABI
test. Contacts are deliberately disabled to isolate the angular constraint.

Coverage:

- Normal and inverted/mirrored LVS limits, clockwise asymmetric limits, and
  authored torque without an added friction floor.
- All translation axes and the wheel-spin axis remain free.
- A transient tilt after creation does not replace the joint or its rest frame.
- Small and larger tilts, and a later disturbance of the same joint, recover
  toward the authored alignment instead of a widened or recaptured rest pose.
- LVS's common deferred spawn rotation preserves the relative frame while the
  wheel can spin and translate independently of its static steering master.
- An all-angular-axis brake socket stops spin at the current spin phase;
  removing it permits spin again without changing the axle direction.

The simulation uses 22 Hz, two collision substeps, 10 velocity iterations,
two position iterations, and Baumgarte 0.01. Recovery checks allow 0.01 degrees
of solver residual; they do not assert exact floating-point zero or a live
vehicle recovery-time guarantee. No server, bots, or live entities are used.
Authored settings retain Jolt's own small-angle locking semantics; this does
not introduce exact enforcement of every tiny offset angular window.

## Run with an existing AVX2 Jolt archive

Use headers and compiler definitions matching the archive (including the
assertions, precision, debug-renderer, and instruction-set configuration).
For the existing Linux64 release AVX2 build:

```sh
cmake -DOUTPUT_DIR="$PWD/generated" -P tests/rotation_only/prepare.cmake
c++ -std=c++17 -O2 -DNDEBUG -DJPH_DEBUG_RENDERER \
  -DJPH_USE_SSE4_1 -DJPH_USE_SSE4_2 -DJPH_USE_AVX -DJPH_USE_AVX2 \
  -DJPH_USE_LZCNT -DJPH_USE_TZCNT -DJPH_USE_F16C -DJPH_USE_FMADD \
  -msse4.1 -msse4.2 -mavx2 -mlzcnt -mf16c -mfma -mbmi \
  -I"$JOLT_SOURCE" -Igenerated tests/rotation_only/test_rotation_only.cpp \
  "$JOLT_LIBRARY" -pthread -o rotation_only_tests
./rotation_only_tests
```

Set `JOLT_SOURCE` to the directory containing `Jolt/` and `JOLT_LIBRARY` to
the matching archive. Optional `-DSOURCE_FILE=/path/to/vjolt_constraints.cpp`
selects the baseline for preparation without changing the checkout.

Manual gates remain: BRDM/LAV spawn, engine start/stop, braking and brake release,
wheel damage/repair, rapid physgun rotation and airborne recovery, and populated
driving. The tests do not establish those client/server acceptance results.
