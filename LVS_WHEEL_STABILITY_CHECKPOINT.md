# LVS wheel stability investigation checkpoint

This branch consolidates the VJolt/LVS wheel-stability investigation in one
history without changing the production `fixed` branch.

## Baseline

- Production source checkpoint: `c356f3c14426daee3883f6b1cbaf06fe07f3a0ac`
- Live-proven AVX2 binary SHA-256:
  `7b4af3462af5b31935d502389f5645e3e96ae9012333cf31ecfdebee9b078a13`
- Default-off trace checkpoint retained as this branch's working tree:
  `19038ecc5261c5af7563d0b2d1e6af497ac67b58`

The live binary contains the `c356f3c` production behavior, but it was built
and staged before that source commit. Do not assume a later rebuild will be
byte-identical solely from the source commit.

## Consolidated alternatives

The umbrella merge records these former branch tips as parents:

- `644073ab95a7590f80d85675ff0b5afb2c071099`: pre-solve recapture and
  inverted-mirror diagnostic; rejected for production.
- `f25fbf0c451fd0c2e3a2ba0a9aa73337ed17ed21`: tiny-axis hardening;
  rejected by live testing.
- `4670b089a3bc19fa64499f9fe11512507c2d4ee0`: tiny-axis position-motor
  experiment; not production-proven.

These alternatives are intentionally history-only in the umbrella merge.
Their mutually incompatible runtime behavior is not combined in the final
tree. Future experiments should start from this branch, select one hypothesis,
and retain default-off bounded diagnostics.

The remaining player-visible upright/camber behavior is tracked in
Avrena/VPhysics-Jolt issue #11. This checkpoint is not a production deployment
or a request to merge into the upstream VPhysics-Jolt or LVS repositories.
