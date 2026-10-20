# Contact penetration recovery

`test_depenetration.cpp` links the matching Jolt archive and compiles the actual
`ConfigureContactRecovery` policy. It uses a convex hull initially embedded
0.8 Source units in a static floor, 22 Hz, two collision substeps, two position
iterations, ten velocity iterations, linear cast, Baumgarte 0.01 and slop 0.01.
The fixture uses an explicit mass and material; it does not load engine assets.

The default run checks the original failure, scoped recovery and its timing,
a global Baumgarte control, slower recovery at the original factor, unchanged
excluded and nonpenetrating bodies, cached-contact restore, repeated embedding,
sensor exclusion, material preservation, higher configured correction, zero
slop, a tilted embedded hull and multi-point correction without overshoot.
`test_eligibility.cpp` compiles the Source eligibility method extracted by
`prepare.cmake`; only engine storage and controller types are shimmed.

Build with the same headers, definitions and instruction set as the rebuilt
Linux64 release AVX2 archive:

```sh
cmake -DOUTPUT_DIR="$PWD/generated" -P tests/depenetration/prepare.cmake
c++ -std=c++17 -O2 -DNDEBUG -DJPH_DEBUG_RENDERER \
  -DJPH_USE_SSE4_1 -DJPH_USE_SSE4_2 -DJPH_USE_AVX -DJPH_USE_AVX2 \
  -DJPH_USE_LZCNT -DJPH_USE_TZCNT -DJPH_USE_F16C -DJPH_USE_FMADD \
  -msse4.1 -msse4.2 -mavx2 -mlzcnt -mf16c -mfma -mbmi \
  -I"$JOLT_SOURCE" -Ivphysics_jolt tests/depenetration/test_depenetration.cpp \
  "$JOLT_LIBRARY" -pthread -o depenetration_tests
c++ -std=c++17 -O2 -Igenerated tests/depenetration/test_eligibility.cpp \
  -o recovery_eligibility_tests
./depenetration_tests
./recovery_eligibility_tests
```

The Jolt contact settings and internal constraint layout have changed: a stale
archive is not ABI compatible, even if linking succeeds. Rebuild Jolt and every
dependent native translation unit together.

Recovery uses the stronger of the configured correction factor and 0.2,
targets zero contact penetration and prevents sleep above the configured slop.
Contact positions are refreshed after each point's correction to account for
both translation and rotation already applied at earlier points. The velocity
solver and global sleep thresholds retain their existing behavior.
The existing sleep timer establishes rest before a final translation-only
correction resolves contacts still deeper than the slop, using the usual maximum
penetration-distance cap. This correction resets the sleep test. Earlier
correction retains angular response so thin hulls can align with their contacts.

Deep contacts against nearly opposing static surfaces cannot both be resolved
by translation. Those contacts use the original slop and correction factor,
without a sleep veto or translation fallback. Classification uses all contacts
in an island before solver work is split, and is repeated each substep without
adding saved state. Other contacts retain recovery. The compact 16-unit cube
fixture overlaps two walls by 0.25, 1 and 4 units per side, observes it for
12 seconds, and checks sleep, wakes and motion after 2 seconds. It also checks
a fitting-gap control, both walls in one static compound body, and recovery
after removing one wall and waking the cube.

For a single diagnostic case, arguments are global Baumgarte, recovery 0/1,
shape 0/1/2 and yaw in degrees. Shape 0 is the embedded hull; 1 is a thin box;
2 is a thin tetrahedron at an offset from the world origin. The latter exposes
a separate narrow-phase limitation: the reported contact depth may be smaller
than the depth of the lowest hull vertex. Recovery acts on solver contact
points; these tests do not claim to repair missing contacts or hull geometry.

The eligibility shim and native floor fixture do not establish engine
integration, full vehicle behavior or player acceptance.
