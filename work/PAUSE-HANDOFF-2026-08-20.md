# Pause handoff — 2026-08-20

## Status at pause

No production file was changed. The production simulator remains:

- `outputs/bite-simulator/app.js`
- SHA-256 `3A58BFB55ACA173DB63435F75C7FDF19BB0F6D68D42AEABF4E77FC1A7AB2D344`
- `TRACEABLE_CUTTING_CONTINUOUS_TOI_ENABLED = false`

The accepted work-only S4c + strict-DXF combined candidate remains:

- `work/s4c-private-toi-dxf-combined/app-combined-candidate.js`
- SHA-256 `06B6EC1F6E611DDD4122652DC9763D8F75129904D224ACCA3CABB1AF3D71D0AA`
- both continuous-material and main-transaction switches remain false

## Friend-version comparison and merge

The useful parts of the Downloads/friend AI revision were isolated and merged
only into the work-only combined candidate:

- strict DXF parsing and indexed coverage proof;
- holes and disconnected geometry gates;
- geometry-derived mass, COM, pivot inertia and central inertia;
- atomic rebuild and rollback;
- preset/import safety around manual inertia.

The imported revision also contained a synchronous fork/shovel import hang.
Three `flatMap(samplePathForCollision)` calls leaked the array index as the
sampler's optional chord length; index zero produced an infinite loop. All three
sites use an explicit unary wrapper in the accepted work-only delta. Real Edge
tests cover legal solid and offset-hole forks, the former hang, fifth-collider
partial failure, and continued use of the restored old world. Built-in CAD and
parameter modes each remain bit-exact with frozen S3 for 2000 steps.

Primary report:
`work/s4c-private-toi-dxf-combined/FINAL-REPORT.md`

## Newly frozen v2.1 numerical geometry checkpoint

Directory: `work/material-split-scale-mode-family-v2_1`

- 9F-v2.1 finite rotation:
  `6C55302E10BC1EECB0333D8549A3A55441A5047869CAE83E920B0F6A00C58449`
- EDD-v2.1 mode family:
  `41B1CB64449BD7585675F275856EBC7E9FC081F354AE9AC2F411283F755B40C6`
- 59-v2.1 exact mode family:
  `B6242837F332E0A86131E64DC180CFF14D7E8C6AE3374336CDF8277804F04DF1`
- freeze manifest:
  `CCE42ECC3AAB7EC1D0A8584E7699EF1910B724F72F05B78A39356707CE86C5DE`

Independent read-only audit verdict: **PASS**, P0=0, P1=0.

Evidence:

- manifest 15/15 exact;
- focused numerical tests 10/10;
- real three-mode fixture PASS with one canonical geometry signature, one
  common numerical-policy signature, one composite snapshot, and three path
  signatures;
- quick random audit 20/20 samples;
- all frozen 9F/EDD/BC/59/v3 regressions pass and their hashes remain fixed.

Real three-mode guard values:

- common final coordinate scale: `0.07017917700505896`
- transform guard: `7.478179990007675e-15`
- twice-transform diameter: `1.4956359980015354e-14`
- computed final scale: `0.07017917700507394`
- polygon guard: `4.228577815034578e-13`
- effective length floor: `4.3033596149346554e-13`
- area guard: `8.607506424316472e-12`
- remaining relative-cap margin: `1.379249357568353e-11`

## Intentional red lights

This checkpoint is not production authority:

- `trigProviderCertified = false`
- `productionNumericalAuthority = false`
- `productionWiring = false`
- `preparedRootAuthority = false`
- no S4d reconnection
- zero physical q writes, geometry commits, removals or MAIN calls

The prepared-root consumer still stops honestly because the nonzero
twice-transform-roundoff radius leaves an unresolved interval after the fixed
budget. No tolerance, guard or search budget was loosened to hide this result.

## Required continuation order

1. Freeze and independently audit a certified trig/norm provider, including
   argument reduction, large angles, subnormals and cross-instance rejection.
2. Prove a nonzero-radius prepared-root rule that either isolates the leftmost
   root or preserves a stable domain stop. Do not solve this by increasing the
   interval budget or relaxing guards.
3. Only then rebind through the exact S4d event-slice AE46 token, S4c owner and
   original prepared object using private WeakMap identities and exact rollback.
4. Keep that checkpoint dry: zero q write, zero commit, zero removal, MAIN false.
5. Geometry/chip ledger, multi-event remainder, full convergence and atomic
   publish remain later gates before any production enablement.

Detailed continuation plan:
`work/material-split-scale-mode-family-v2_1/NEXT-STAGE.md`
