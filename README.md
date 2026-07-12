![VPhysics Jolt logo](assets/cube_base_nobg.png "VPhysics Jolt")

# VPhysics Jolt (Volt)

An NCG-maintained, Garry's Mod-focused fork of VPhysics Jolt. Volt replaces
Source's IVP/Havok-based VPhysics implementation with
[Jolt Physics](https://github.com/jrouwe/JoltPhysics), with an emphasis on
compatibility, predictable game behavior, crash containment, and high object counts.

> [!CAUTION]
> Volt replaces a core engine component. A successful build does not establish ABI,
> simulation, or save compatibility for every Source branch. Test on an isolated
> installation, keep backups, and retain the original VPhysics binaries for rollback.

## Fork status

The default `fixed` branch is the active integration branch. It carries the original
Volt implementation plus a substantial GMod parity and hardening pass.

| Area | Current focus |
|---|---|
| Primary target | Garry's Mod client and dedicated server |
| Additional CI targets | Source SDK 2013 SP/MP and Alien Swarm |
| Platforms | Windows and Linux, x86 and x64 where supported by the target SDK |
| Language/toolchain | C++20; MSVC or GCC |
| Jolt dependency | Pinned Git submodule at `joltphysics/src` |

Notable work in this fork includes:

- physics-object lifetime and validity tracking compatible with GMod/HolyLib;
- player-controller, ragdoll, constraint, collision-callback, and IVP behavior parity;
- contact-pair cleanup and callback hardening during deletion and environment transfer;
- non-finite/out-of-world body containment and safer allocator/thread behavior;
- object-list, collision-shape, and contact-query performance improvements;
- configurable worker-thread and physics tuning controls;
- CI builds for GMod client and dedicated-server targets on x86 and x64.

## Capability overview

| Capability | Status | Notes |
|---|:---:|---|
| Constraints and pulleys | ✅ | Breakable constraints remain incomplete |
| Motion and constraint motors | ✅ | Source-facing behavior preserved where implemented |
| Ragdolls | ✅ | Includes rotation-only constraint and jitter fixes |
| Triggers and touch callbacks | ✅ | Callback lifetime paths are hardened |
| Prop damage and breaking | ✅ | Game-driven behavior supported |
| Fluids and splash events | ✅ | Supported |
| Wheeled vehicles | ✅ | Supported |
| Raycast vehicles | ⚠️ | Airboat support exists; broad parity is incomplete |
| NPCs, doors, and shadow controllers | ✅ | Supported |
| Save/restore | ✅ | Treat cross-version saves as compatibility-sensitive |
| Portal integration | ✅ | Supported by the original implementation |
| Per-object collision filtering | ✅ | Includes game `ShouldCollide` behavior |
| Multithreaded simulation | ✅ | Worker count can be constrained for deployment needs |
| Large object counts | ✅ | Designed to avoid stock VPhysics scaling bottlenecks |

Volt is intentionally not presented as bug-free or behavior-identical on every engine
fork. Report missing features and parity differences with a minimal, deterministic
reproduction.

## Installation

Use a build made for the exact engine branch, operating system, architecture, and
client/server target being deployed.

1. Download a successful artifact from
   [GitHub Actions](https://github.com/Avrena/VPhysics-Jolt/actions), or build from source.
2. Stop the game or dedicated server completely.
3. Back up the existing VPhysics binaries and any important saves/server data.
4. Deploy the matching artifact without mixing files from another SDK or architecture.
5. Start on a staging map and validate startup, spawning, constraints, ragdolls,
   vehicles, save/restore, cleanup, and shutdown before wider use.

> [!IMPORTANT]
> GMod uses interfaces and SDK material that cannot be redistributed in this repository.
> The CI workflow obtains the compatible SDK layout separately. A generic SDK 2013 build
> is not a substitute for a GMod-targeted build.

## Building

Clone recursively so the pinned Jolt submodule is present:

```sh
git clone --recursive https://github.com/Avrena/VPhysics-Jolt.git
```

The supported workflow builds Volt inside a compatible mini Source SDK tree. See
[`build.md`](build.md) for the Windows and Linux procedure.

Minimum toolchain expectations:

- Visual Studio 2022 and a current Windows SDK on Windows;
- GCC/G++ 10 or newer plus multilib packages for Linux x86 builds;
- a C++20-capable compiler;
- the SDK branch matching the intended game target.

The repository workflows are the reference for GMod x86/x64 build layout and artifact
contents.

## Runtime configuration

This fork exposes targeted convars and `VJOLT_CVAR_*` environment overrides for
selected solver, worker-thread, contact, and compatibility controls. Keep production
settings explicit and record non-default values in bug reports.

Avoid changing several physics controls at once. Establish a baseline, change one
variable, and compare behavior and performance using a repeatable map/scenario.

## Reporting problems

Open an issue in this repository using the bug form. Include:

- the exact Volt commit;
- engine/SDK branch and commit;
- operating system, architecture, compiler, and build configuration;
- a minimal map/entity/constraint reproduction;
- whether the problem also occurs with stock VPhysics;
- redacted logs or stack traces.

Do not upload credentials, private server details, player data, or raw memory dumps.

## Contributing

Pull requests should stay focused and identify every engine/platform target actually
tested. Changes to ABI assumptions, collision behavior, object lifetime, threading,
serialization, determinism, save/restore, or the Jolt submodule require explicit risk
and validation notes.

External code and ideas must be attributed and license-compatible. Generated projects,
build output, and binaries do not belong in normal source pull requests.

## Lineage and license

Volt was created by [Joshua Ashton](https://github.com/Joshua-Ashton) and
[Josh Dowell](https://github.com/Slartibarty). This branch descends through
[`RaphaelIT7/VPhysics-Jolt`](https://github.com/RaphaelIT7/VPhysics-Jolt) and carries
additional NCG-focused GMod compatibility and hardening work.

The project is licensed under the MIT License. See [`LICENSE`](LICENSE).

<details>
<summary>Original demonstration videos</summary>

- [Lots of Melons + Dumpster](https://www.youtube.com/watch?v=gPDQkmfQCsc)
- [Physically Simulated Chain](https://www.youtube.com/watch?v=tVmQTmbSJM0)
- [Lots of Balls Test](https://www.youtube.com/watch?v=tYfiTyRtmz8)
- [Weld Car Dupe Test](https://www.youtube.com/watch?v=5_QbbXbIrg8)
- [Door + NPC physics-shadow test](https://www.youtube.com/watch?v=SdEj7HTuJmU)
- [Cubes, ragdolls, and funnel](https://www.youtube.com/watch?v=CLVnSwg33Dk)
- [Slow-motion cubes](https://www.youtube.com/watch?v=GzW_4bufwEk)
- [Propane in dumpster](https://www.youtube.com/watch?v=10vvRJVHGQc)

</details>
