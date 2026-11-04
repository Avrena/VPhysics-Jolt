# Linux x86-64 production review

Reviewed on 3 November 2026 for the Garry's Mod dedicated server target.

- Integration base: `Avrena/VPhysics-Jolt` `fixed` at
  `12db29f4c92c7a1622878cfc9c2ccfca694c6181`.
- Reference implementation: `schweiziske/GMod-Jolt-Physics` at
  `a31992de4e801fdcc3ba8aacae5de668129f4283`.
- Retained Jolt dependency: `Avrena/JoltPhysics`
  `68b3ee60ae6475e02eaa477706e79c16e0ed919d`.

## Decision and scope

Two changes have a concrete benefit for a long-running Linux server: rejecting
stale collision-filter cache writes and making object cleanup safe when game
callbacks mutate deletion queues. Both can be adapted independently of the
reference project's Windows SDK, extension modules and customised solver.

| Candidate | Decision | Production rationale |
| --- | --- | --- |
| Indexed, generation-checked `ShouldCollide` cache | Adapt | Prevent stale reinsertion after callback-time invalidation; avoid scanning unrelated pairs on every filter change. |
| Re-entrant deletion and teardown | Adapt | Destructors can enter game callbacks, grow queues, remove listeners and request cleanup again. |
| Synchronous `PreCollision` / solver response / `PostCollision` | Defer | Requires coordinated job-system, callback and solver changes. Its synchronisation cost and gameplay semantics need separate engine qualification. |
| Expanded save/restore and object-buffer format | Defer | Useful API coverage, but changes persistence and ownership contracts. A separate change needs round-trip and old-save compatibility decisions. |
| New constraint, trace and fluid implementations | Defer | Broad simulation changes are not justified by the narrow Linux production defects reproduced here. |
| Soft bodies, gravity fields, gears and rails | Defer | Additional gameplay features with native integration and platform work, not a demonstrated improvement to the existing server workload. |
| Windows CPU-affinity policy and packaging | Exclude | Does not improve the Linux target. |

The reference project is not a strict superset of this fork. In particular, its
bounding-box cache, disabled-fluid handling and player/shadow input protections
do not retain all of this fork's fixes. Those implementations stay in place.

## Collision-filter cache

The existing cache calls the game solver without holding the map lock, then
unconditionally stores its answer. A game callback can invalidate that very
pair before returning; the later store would undo the invalidation. A mutation
epoch now rejects that store. The current invocation still returns the game's
answer, and the next lookup recomputes rather than using a stale cached result.
Replacing the game solver also clears the cache and advances the epoch.

Each body indexes its cached pairs. Each pair stores its position in the two
endpoint lists, permitting swap-and-pop removal with backlink repair. Unlike
the reference implementation's linear neighbour search, deleting many leaves
connected to the same high-degree body does not make index maintenance
quadratic. A cache hit still takes a shared lock; game solver calls retain the
existing serialised callback section.

Admission stops at 100,000 pairs. At capacity the game's answer is returned
without caching a new pair. Existing entries remain usable, and body/filter
invalidation makes room again. This replaces the old conditional whole-cache
clear performed by an invalidation after the map exceeded 100,000 entries.
The extra indexes have a memory and cold-fill cost; this is a targeted
invalidation improvement, not a claim of universally faster physics.
Endpoint lists shrink geometrically when at most one-quarter full, so live
bodies do not retain excessive historical adjacency capacity after cleanup.

Deletion removes pairs before removing a body, and the object destructor clears
again after game callbacks, before Jolt can recycle the ID. Environment transfer
invalidates the source environment's old ID. These hooks keep the reverse index
from retaining dead or transferred pairs indefinitely.

The atomic solver pointer prevents an unsynchronised pointer read/write. It does
not transfer ownership of a game handler or permit another thread to destroy a
handler while its callback is executing; the existing engine lifetime contract
still applies.

## Object and constraint lifetime

Deletion methods live in `vjolt_environment_lifecycle.cpp`, which is included
in the existing VPC build. The changes address callback-driven mutation:

- A queue consumes its current slot before invoking a destructor and uses an
  index across callbacks, so vector growth cannot invalidate an iterator.
- Repeated requests for a still-live queued object or constraint are coalesced.
- Recursive cleanup cannot drain an object that is already being destroyed.
- Cleanup requested during simulation waits until the normal safe boundary;
  queued contact events skip marked endpoints. A collision event already
  started still receives its matched `PreCollision` / `PostCollision` pair.
- The second friction callback rechecks its endpoints after the first callback.
- Collision metadata remains alive until object destruction callbacks finish.
- Object-destruction listeners are consumed before calling game code, so a
  callback removing another listener cannot invalidate the traversal.
- Environment teardown owns its body-ID snapshot, deduplicates wrappers shared
  with player-controller bodies, and handles bodies created by destruction
  callbacks before releasing deferred collision data.

Constraint destruction retains this fork's existing timing: it is deferred
during simulation, but remains immediate outside simulation even when the
object delete queue is enabled or an object is being destroyed. Deferring it
more broadly can leave a constraint referencing an object whose destruction
has already removed the corresponding listener.

These are internal data-structure and lifetime changes. The Source interfaces,
saved formats, Jolt pin, solver iterations, CCD defaults, collision speed gates,
callback budgets and physics tuning are unchanged.

## Reproducing the native regression fixtures

The fixtures compile the actual cache, listener methods, lifecycle translation
unit and selected object/constraint methods. Small shims supply the engine
interfaces and record callback order; they do not replace the changed algorithms
with a separate implementation. See `tests/should_collide_cache/README.md` and
`tests/lifecycle/README.md` for negative controls and individual scenarios.

```sh
cmake -S tests -B build/native-regressions -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/native-regressions --parallel 2
ctest --test-dir build/native-regressions --output-on-failure

cmake -S tests -B build/native-sanitizers -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DVJOLT_SANITIZERS=ON
cmake --build build/native-sanitizers --parallel 2
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  ctest --test-dir build/native-sanitizers --output-on-failure
```

The Linux regression workflow runs both configurations. These fixtures establish
the reproduced bookkeeping and lifetime behaviour, not live GMod ABI acceptance,
addon compatibility, engine save/restore behaviour or server tick-rate gains.

## Qualification results

The aggregate suite passed all 14 CTest cases in both RelWithDebInfo and
AddressSanitizer + UndefinedBehaviourSanitizer configurations on Linux x86-64.
This includes the existing object-pair hash fixture, 22 cache checks, 14 listener
checks and 11 separately launched lifecycle scenarios. A separate ThreadSanitizer
run passed the 22 cache checks, including a synchronised invalidate/recreate
race and 48,000 concurrent operations. Local LeakSanitizer was disabled because
the execution host prevents its process inspection; address and undefined-behaviour
instrumentation remained enabled. The GitHub regression workflow enables leak
detection on its runner.

Negative controls compile the exact base revision through the same source
extractors. The old listener fails five of the fourteen checks. All eleven
lifecycle scenarios fail against the old code: some reproduce ASan use-after-free
or invalid memory access, while others detect the missing invalidation,
deduplication or callback-order guarantees. These are not eleven distinct
crash claims. In particular, queue growth, nested cleanup and repeated queued
constraint deletion reproduce memory-safety failures in the old implementation.

### Full native Linux x86-64 build

Both the exact baseline and candidate built as the GMod dedicated-server target
using GCC/G++ 13.3 on Ubuntu 24.04 and `RaphaelIT7/mini-source-sdk` at
`ba064cd7e560300103e1630ee90f7bad7f2a2224`. The candidate recompiled every adapter
translation unit in SSE2, SSE4.2 and AVX2 configurations, including the new
lifecycle source. The unchanged SDK/Jolt archives were reused. The SDK's i386
VPC generator ran under QEMU because the host cannot execute ELF32; compilation,
linking and all backend execution were native x86-64.

All three candidate backends loaded successfully and returned all four expected
interfaces: `VPhysics031`, `VPhysicsGMod`, `VPhysicsCollision007` and
`VPhysicsSurfaceProps001`. Their ELF dependencies, versioned-symbol requirements
and exported symbol names match the baseline. The wrapper is byte-identical.

A smoke harness compiled against the real SDK interfaces exercised each actual
backend through `Init`, 16 environment lifetimes, 512 sphere lifecycles, 64
simulation steps, finite falling motion, object/environment cleanup and
`Shutdown`. All three candidate variants passed; the baseline SSE2 backend
passed the same test. Across the candidate that is 48 environments, 1,536 sphere
lifecycles and 192 simulation steps. No physics implementations were mocked.
The reusable harness and exact compiler/loader commands are in
`tests/native_backend`. The existing `tests/collision_events` fixture also
passed all 33 checks against the pinned SSE2 Jolt archive, using extracted
production event/clock/tracker methods and real Jolt bodies with a narrow
Source-object stand-in.

This smoke uses the backend's supported initialisation path without `g_pCVar`.
It does not exercise engine connection, ConVar registration, GMod/Lua callbacks,
maps, vehicles or production load. The Ubuntu qualification binaries require
GLIBC 2.38 and GLIBCXX 3.4.29 and are not distribution artifacts. A successful
Bullseye Linux64 dedicated build remains the production artifact gate.

### Focused cache benchmark

Five-run medians on Linux x86-64 with GCC 13.3, glibc 2.39 and a Release build:

| Graph and measurement | Old scan cache | New indexed cache |
| --- | ---: | ---: |
| 25,000 bodies, degree four: insert 50,000 pairs | 3.683 ms | 9.541 ms |
| Same graph: 1,000,000 cached lookups | 39.614 ms | 41.061 ms |
| Same graph: invalidate 1,000 bodies | 50.428 ms | 1.029 ms |
| Same graph: allocator used-block delta | 1,585,264 bytes | 4,622,112 bytes |
| World + 50,000 props: insert 50,000 pairs | 2.774 ms | 7.728 ms |
| Same graph: 1,000,000 cached lookups | 15.174 ms | 16.266 ms |
| Same graph: invalidate 1,000 props | 61.539 ms | 0.286 ms |
| Same graph: allocator used-block delta | 1,572,896 bytes | 6,849,264 bytes |

Both graph shapes fit the backend's 65,536-body limit. The benchmark compiles
the actual new helper and a labelled reproduction of the old map/lock/full-scan
algorithm. It is a synthetic cost comparison, not a loaded-server TPS forecast.
The allocator measurement includes overhead and retained capacity; it is not
an exact payload count or total server memory usage. The complete benchmark
source and reproduction commands are in `tests/should_collide_cache`.

Invalidation is about 49 times faster in the first graph and 215 times faster
in the shared-world graph. Cold fills are about 2.6–2.8 times slower; indexed
storage adds about 3.0 MB and 5.3 MB respectively at 50,000 pairs. Cached lookup
times were slightly higher in these runs on a shared host. The value is therefore
the reproduced correctness fixes and reduced invalidation work, especially for
filter changes and object churn, rather than a blanket performance claim.

### Production rollout

The source review, negative controls, sanitised regressions and native backend
tests justify proposing this adaptation for production. Before deployment,
use the Bullseye Linux64 artifact in a staging server with the production addon
set. Exercise prop/ragdoll cleanup, collision-filter changes, constraint removal
from callbacks, map changes and prolonged object churn; compare crash behaviour,
tick-time tails, memory usage and callback load at cache saturation. The review
does not establish a live server soak result or a universal speed increase.

## Build maintenance

The previous Linux64 dedicated job failed during dependency installation before
compiling this code:
https://github.com/Avrena/VPhysics-Jolt/actions/runs/40612055305/job/122565710347

Its Bullseye security index referenced packages no longer served by the live
security mirror. The workflow now redirects only that source to
`archive.debian.org/debian-security`, preserving the original protocol, package
list and Bullseye compiler/runtime target. Apt signature and validity checks
remain enabled. The archived signed index and representative failing packages
were independently checked before making the change.

The existing development workflow could deploy to Pterodactyl on any branch
push when its secrets were configured. Its deployment step now requires
`refs/heads/fixed`; feature-branch builds cannot deploy the candidate.

## Attribution

The cache invalidation epoch/reverse-index approach and re-entrant lifecycle
work were adapted from:

- https://github.com/schweiziske/GMod-Jolt-Physics/blob/a31992de4e801fdcc3ba8aacae5de668129f4283/vphysics_jolt/vjolt_should_collide_cache.h
- https://github.com/schweiziske/GMod-Jolt-Physics/blob/a31992de4e801fdcc3ba8aacae5de668129f4283/vphysics_jolt/vjolt_environment_lifecycle.cpp

The upstream MIT copyright and permission notice is preserved in
`LICENSES/GMod-Jolt-Physics-MIT.txt`. The endpoint-slot removal, bounded admission,
integration with this fork's deferred callbacks, and regression fixtures are
specific to this adaptation.
