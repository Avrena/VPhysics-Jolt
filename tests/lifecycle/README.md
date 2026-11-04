# Environment lifecycle regressions

This target compiles the complete production `vjolt_environment_lifecycle.cpp`
translation unit against small SDK/Jolt interface stand-ins. It also extracts
and compiles the real object destructor, destroyed-listener methods, constraint
destructor and `OnJoltPhysicsObjectDestroyed`, plus the environment's object-list,
transfer and constraint-notification methods. There is no copied implementation
of the deletion algorithm in the fixture. The extraction script matches braces
while ignoring comments and strings, and fails configuration if a selected
method cannot be found.

The stand-ins provide body storage, event handlers, the validity registry and
collision-data lifetime observations. They do not implement a physics solver,
engine ABI or the collision decision cache itself. The latter has its own tests
in `tests/should_collide_cache`. The collision stand-in marks metadata as released
without freeing the fixture variable, allowing an ordering defect to be reported
without relying on an allocator's reuse behaviour.

```sh
cmake -S tests/lifecycle -B /tmp/vjolt-lifecycle -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/vjolt-lifecycle -j2
ctest --test-dir /tmp/vjolt-lifecycle --output-on-failure
```

Eleven independent scenarios cover:

- cleanup requests during simulation retaining marked wrappers until the safe boundary;
- repeated object and constraint deletion requests;
- callbacks growing the pending-object vector past its allocation;
- nested cleanup from an object destruction callback;
- the real constraint-destruction notification path deleting another listener;
- shared collision metadata surviving object callbacks and duplicate release requests;
- environment teardown with queued objects, helper bodies sharing a wrapper,
  a public deletion flag, and objects created by destruction callbacks;
- `GetObjectList` refreshing its cache during environment teardown;
- cache invalidation before body removal and again after destruction callbacks;
- source-environment cache invalidation during transfer.

The object-queue setting still does not defer a constraint outside simulation.
The corresponding scenario checks that deleting that constraint does not flush
pending objects. Constraint deletion also detaches listeners immediately outside
simulation; postponing it during an object destructor would change callback
behaviour and can leave a constraint referring to an already-destroyed object.

## Baseline reproduction

To run the identical scenarios against the pre-fix production methods, use a
separate build directory and the baseline ref. The script reads the methods
from git without modifying the checkout:

```sh
cmake -S tests/lifecycle -B /tmp/vjolt-lifecycle-baseline \
  -DVJOLT_BASELINE_REF=12db29f4c92c7a1622878cfc9c2ccfca694c6181 \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
  -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=address,undefined'
cmake --build /tmp/vjolt-lifecycle-baseline -j2
ctest --test-dir /tmp/vjolt-lifecycle-baseline --output-on-failure
```

On Linux x86-64 with GCC 13.3, all eleven baseline scenarios failed. AddressSanitizer
reported use-after-free for vector growth, nested cleanup and duplicate constraint
deletion. The teardown-cache scenario dereferenced a missing body after a callback
overwrote the traversal's shared snapshot. The listener-mutation scenario failed
the stand-in's checked vector bounds. The remaining failures demonstrate wrong
deletion timing, collision-data lifetime or missing invalidation hooks. The
updated implementation passes all eleven with AddressSanitizer and UBSan.

LeakSanitizer could not inspect processes in the qualification container, so that
local run used `ASAN_OPTIONS=detect_leaks=0`; address and undefined-behaviour
instrumentation remained enabled. This restriction is not required for an
ordinary Linux host. These fixtures establish lifecycle defects and their repair;
they do not replace a populated GMod server soak or validate engine loading.
