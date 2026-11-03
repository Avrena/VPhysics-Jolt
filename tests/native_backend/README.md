# Native GMod Linux x86-64 backend smoke test

This harness loads the actual backend shared library and uses the matching
Source SDK's public interfaces. It checks four interface factories, initialises
the backend, then runs 16 environment lifetimes with 32 spheres and four
simulation steps each. It checks finite falling motion, live/deleted object
registry membership, environment cleanup and shutdown. No physics methods or
engine classes are mocked.

It requires a completed GMod Linux x86-64 backend build and its matching SDK
headers, `libtier0.so` and `libvstdlib.so`. The qualification used
`RaphaelIT7/mini-source-sdk` commit
`ba064cd7e560300103e1630ee90f7bad7f2a2224` (`gmod-changes`) and the repository's
unchanged Jolt submodule at `68b3ee60ae6475e02eaa477706e79c16e0ed919d`.
Use the SDK and compiler/runtime matching the binaries being tested.

From the repository root, set the SDK source directory and compile:

```sh
VJOLT_SDK_SRC=/path/to/mini-source-sdk/gmod/src
VJOLT_SMOKE_OUT="$(mktemp -d)"
g++ -std=gnu++20 -O2 -DNDEBUG -DGAME_GMOD -DGAME_SDK2013 \
  -DPLATFORM_64BITS -DPOSIX -D_POSIX -DLINUX -D_LINUX -DGNUC \
  -DCOMPILER_GCC -DRAD_TELEMETRY_DISABLED -DVPC -Wno-register -fpermissive \
  -I"$VJOLT_SDK_SRC/public" -I"$VJOLT_SDK_SRC/public/tier0" \
  -I"$VJOLT_SDK_SRC/public/tier1" -I"$VJOLT_SDK_SRC/common" \
  tests/native_backend/test_lifecycle.cpp \
  -ldl -L"$VJOLT_SDK_SRC/lib/public/linux64" -ltier0 -lvstdlib \
  -o "$VJOLT_SMOKE_OUT/test_native_backend"
```

Run each CPU-supported variant in a fresh process. SSE2 is the baseline;
only include SSE4.2/AVX2 on a CPU supporting the backend's instruction set.
The final argument can instead name a separately saved baseline or candidate
library:

```sh
for variant in sse2 sse42 avx2; do
  LD_LIBRARY_PATH="$VJOLT_SDK_SRC/lib/public/linux64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    "$VJOLT_SMOKE_OUT/test_native_backend" \
    "$VJOLT_SDK_SRC/../game/bin/vphysics_jolt_${variant}.so" || exit 1
done
```

Successful runs print the four factory checks, `backend Init OK`, and
`16 environments / 512 sphere lifecycles / 64 simulation steps / Shutdown OK`.
Failures return a nonzero exit status.

This is an isolated backend smoke test, separate from the SDK-free CMake
regressions. It uses the supported initialisation path with no `g_pCVar`, and
does not supply a synthetic game factory. It does not test Connect/ConVar
registration, GMod/Lua callbacks, maps, vehicles, production load, or deployment
compatibility with an older C/C++ runtime. Build distribution artifacts with
the project's deployment toolchain rather than distributing local smoke-test
builds.
