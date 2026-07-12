## Summary

<!-- Explain what changed and the user-visible or engine-level reason for it. -->

## Related issue

<!-- Link an issue, for example: Closes #123. -->

## Engine targets tested

<!-- Check only targets you actually built and exercised. -->

- [ ] Source SDK 2013 single-player
- [ ] Source SDK 2013 multiplayer
- [ ] Alien Swarm
- [ ] Garry's Mod client
- [ ] Garry's Mod dedicated server
- [ ] Windows x86
- [ ] Windows x64
- [ ] Linux x86
- [ ] Linux x64

## Validation

<!--
List exact build/test commands and scenarios. For physics changes, include maps/entities,
object counts, save/restore coverage, and before/after performance data where relevant.
-->

## Physics compatibility and risk

<!--
Describe changes to Source/Jolt ABI assumptions, collision filtering, constraints, contact
callbacks, object lifetime, threading, determinism, serialization, or save compatibility.
Include a rollback plan for behavior-changing work.
-->

> [!CAUTION]
> VPhysics-Jolt replaces a core engine component. Test on an isolated installation with
> backups of saves and server data. A successful build does not prove runtime compatibility,
> stable simulation, or safe save/restore behavior on every engine branch and architecture.

## Checklist

- [ ] The change is focused and excludes unrelated generated projects, build output, and binaries.
- [ ] Any `joltphysics/src` submodule update is intentional, pinned to a reviewed commit, and explained above.
- [ ] Public interfaces and Source/Jolt ABI assumptions remain compatible or are documented.
- [ ] Thread safety, object lifetime, callbacks, and environment transfer were considered where relevant.
- [ ] Save/restore and cleanup behavior were tested or explicitly identified as untested.
- [ ] Hot-path changes include representative performance evidence or a reason it is unnecessary.
- [ ] External code or ideas are attributed and license-compatible.
- [ ] Logs, dumps, screenshots, and test data contain no credentials, private paths, or player data.
