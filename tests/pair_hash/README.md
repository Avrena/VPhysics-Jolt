# Collision-exclusion hash regression test

This narrow C++ target compiles `vjolt_objectpairhash.cpp` itself, replacing only
the engine PCH / allocator includes with STL headers and an interface-only SDK
stand-in. It does not build or run Jolt, the physics DLL, GMod, or bots.

```sh
cmake -S tests/pair_hash -B /tmp/vjolt-objectpairhash
cmake --build /tmp/vjolt-objectpairhash --config Debug
ctest --test-dir /tmp/vjolt-objectpairhash -C Debug --output-on-failure
```

The fixture deliberately finds distinct real pointer values in the same
1024-way object bucket. It checks cleanup of both a present and an absent
colliding key, partner enumeration, single-pair reference accounting, duplicate
operations, output capacity, and pairs with both endpoints in the same bucket.
The pre-fix implementation fails 12 of 23 checks with MSVC's checked iterators.

Source uses this interface for the collision-exclusion hash. In the stock game
path, `PhysDestroyObject` removes the object's pairs and (when deleting the
entity) its entity pairs. `PhysDisableEntityCollisions` adds entity pairs, and
the collision solver consults both entity and physics-object pairs:

- [Stock cleanup and exclusion creation](https://github.com/ValveSoftware/source-sdk-2013/blob/master/src/game/shared/physics_shared.cpp)
- [Stock collision solver](https://github.com/ValveSoftware/source-sdk-2013/blob/master/src/game/server/physics.cpp)

These tests establish the bookkeeping defect and its repair. They do not prove
that every observed LVS symptom has the same cause, validate the GMod ABI, or
replace a populated-server vehicle test. No wheel/chassis transforms, suspension
strengths, rotation limits, or recapture timing are changed by this fix.
