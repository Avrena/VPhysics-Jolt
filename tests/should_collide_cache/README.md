# Collision-filter cache and callback-lifetime regressions

This standalone C++17 target compiles the production `VJoltShouldCollideCache`
header and extracts the actual listener's `ShouldCollide`, invalidation,
solver replacement, event-liveness guard and `FlushCallbacks` methods. The
listener fixture supplies minimal engine/object interfaces and event storage;
it does not replace the production admission or dispatch methods with models.
It requires no Jolt archive or Source SDK.

## Run

```sh
cmake -S tests/should_collide_cache -B /tmp/vjolt-cache -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/vjolt-cache -j 4
ctest --test-dir /tmp/vjolt-cache --output-on-failure
```

For GCC/Clang address and undefined-behaviour checks, configure another build
with `-DVJOLT_CACHE_SANITIZERS=ON -DCMAKE_BUILD_TYPE=Debug`. LeakSanitizer may not
work in a ptraced container; `ASAN_OPTIONS=detect_leaks=0` disables only leak
detection when that host limitation is encountered. It is not needed on an
ordinary Linux host. A separate ThreadSanitizer run is:

```sh
c++ -std=c++17 -O1 -g -fsanitize=thread -fno-omit-frame-pointer \
  -Ivphysics_jolt tests/should_collide_cache/test_cache.cpp -pthread \
  -o /tmp/vjolt-cache-tsan
/tmp/vjolt-cache-tsan
```

## What is checked

The helper fixture checks symmetric pair identity, cached true and false
answers, duplicate insertion, independent neighbours, callback-time
invalidation, same-ID invalidation/recreation, full clears, admission at
capacity and admission after cleanup. It exercises shuffled high-degree
neighbour removal, capacity reclamation, mixed dense graphs against an
independent reference map, a synchronised cross-thread invalidation race and
four concurrent readers/writers/invalidators.

The listener fixture checks the actual callback integration: an invalidation
inside `ShouldCollide` cannot be undone by storing that callback's old answer;
a solver replacement inside the callback invalidates the old result without
taking the already-held callback mutex. The current invocation still returns
its solver's answer, including false. The next invocation consults the new
state. An atomic solver pointer does not provide ownership or make arbitrary
concurrent destruction of the game-owned solver safe.

The actual deferred event flush is checked with an object marked during
`PreCollision`: its matching `PostCollision` still runs, and all later
collision/touch/trigger/fluid/friction events involving that endpoint are
dropped. Friction callbacks are independent, so deletion during the first
suppresses the second. A real freed allocation is put into a later event to
check that registry membership is consulted before dereferencing its flags.
ASan checks that ordering. This fixture complements `tests/lifecycle`, which
checks the actual destruction queues and invalidation hooks.

The same fixture can compile the base listener as a negative control:

```sh
git show 12db29f4c92c7a1622878cfc9c2ccfca694c6181:vphysics_jolt/vjolt_listener_contact.h \
  > /tmp/vjolt-base-listener.h
cmake -S tests/should_collide_cache -B /tmp/vjolt-cache-base \
  -DVJOLT_LISTENER_SOURCE=/tmp/vjolt-base-listener.h -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/vjolt-cache-base --target should_collide_listener_tests
/tmp/vjolt-cache-base/should_collide_listener_tests
```

That base fails five of fourteen checks: callback-time stale reinsertion,
re-entrant replacement, continued queued callbacks after marking an endpoint,
callbacks to an already-marked endpoint, and the second friction callback.
Missing source boundaries fail extraction rather than silently testing a
copied implementation.

## Admission, complexity and storage

The production cache admits at most 100,000 pair entries. At capacity a new
answer is returned normally and is simply not cached; existing unrelated
entries remain intact. Invalidation, object destruction and transfer reclaim
space. This differs from the old policy, which allowed growth beyond 100,000
and cleared the entire map on a later invalidation. The new cap can mean
repeated callbacks for uncached pairs until space is released.

Each entry stores slots for both endpoint lists. Removing a pair uses swap-pop
and repairs the moved pair's slot, avoiding an additional linear neighbour
search for each deletion. Invalidation is proportional to the affected
degree on average, with the same expected constant-time hash assumptions as
the old cache. Endpoint vectors shrink geometrically when at most one
quarter full, preventing many formerly busy but still-live bodies from
retaining arbitrarily large historical adjacency allocations. Total reverse
storage remains proportional to admitted entries; the dense maps may retain
their bounded high-water capacities. The index costs more memory and more
work on cold insertion than the old boolean map.

## Focused cost comparison

```sh
/tmp/vjolt-cache/should_collide_cache_benchmark legacy
/tmp/vjolt-cache/should_collide_cache_benchmark indexed
```

The benchmark compiles the actual new helper and a labelled reproduction of
the old map/lock/full-scan algorithm. It is deliberately not a correctness
test or a game-server benchmark. Each mode builds 50,000 pairs, performs
1,000,000 cached lookups and invalidates 1,000 bodies. Its two graphs are a
25,000-body degree-four graph and a shared world body with 50,000 prop
neighbours, both within the backend's 65,536-body limit. It reports cold
insertion, hits and invalidation separately so the index's cost is visible.
On glibc, `heap_delta_bytes` is the allocator's used-block delta, including
allocation overhead and retained map capacity, not an exact payload-byte
count. Use separate fresh processes for the two modes and compare repeated
runs; timings are affected by the host scheduler and allocator.

These tests do not establish GMod ABI compatibility, loaded-server TPS,
collision damage, player behaviour or game-owned solver lifetime. They do
not change collision thresholds, solver tuning, Jolt's submodule revision,
physics callbacks' threading policy or saved physics formats.

## Provenance

The mutation epoch and reverse-index design were adapted from
[`vjolt_should_collide_cache.h` at schweiziske/GMod-Jolt-Physics
`a31992de4e801fdcc3ba8aacae5de668129f4283`](https://github.com/schweiziske/GMod-Jolt-Physics/blob/a31992de4e801fdcc3ba8aacae5de668129f4283/vphysics_jolt/vjolt_should_collide_cache.h).
This implementation adds constant-time endpoint removal, bounded admission
and geometric capacity reclamation. The upstream MIT notice is preserved in
`LICENSES/GMod-Jolt-Physics-MIT.txt`.
