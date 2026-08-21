# Friend DXF port — work-only prototype

This directory is an isolated, pure-ESM port study.  It is **not wired to the
production app**, S2, or S3 candidates.  It has no browser, DOM, filesystem,
network, child-process, dynamic-import, or third-party runtime dependency.

## Scope retained from friend commit `63812623d7616a654374190e92ec08a027345445`

- strict world-XY / zero-thickness DXF acceptance;
- closed-chain topology: exactly one outer ring plus disjoint ordinary holes;
- hole-aware triangulation preparation and area-quality verification;
- initial-overlap injection contract, exact solid-vs-rectangle penetration,
  and geometric-to-solver contact-index mapping;
- a ported/adapted DXF fixture catalogue and pure Node tests.

## Explicitly out of scope

- No change to the 80 um penetration gate. `PENETRATION_GATE_METRES` is
  exported solely so a future adapter can use the existing production constant.
- No friend manual `weaponInertia`, separate mass/inertia semantics, default
  phase, or material feature switch is present.
- No Rapier colliders, polygon-clipping implementation, UI, preset, or import
  transaction is present.  These need adapters at the integration anchors.

## Fail-closed contracts added after independent P0 review

`prepareSolidTriangulation` now requires a positive, unit-matched
`geometryFloor`. It stops with `DXF_UNCALIBRATED_GEOMETRY_FLOOR` if an adapter
does not bind one, or `DXF_FEATURE_BELOW_GEOMETRY_FLOOR` if an edge or
non-adjacent boundary clearance is thinner than that floor.

It does not trust Earcut's total area alone. Every output triangle must be
inside the canonical outer-minus-holes solid, may not cross a canonical
boundary or occupy a hole, and may not overlap another triangle. Canonical
boundary edges must appear exactly once and internal mesh edges exactly twice;
only then is total-area equality accepted as the final coverage proof.

`assertNoInitialOverlap` defaults to a strict zero-area allowance. A numerical
kernel tolerance can only be passed explicitly as `kernelAreaTolerance` in the
same squared unit. `matchGeometricContactsToSolver` throws by default on any
unmatched or ambiguous/duplicate witness; callers choosing `failClosed: false`
receive `{ ok: false, code }` and must treat it as a hard gate.

For the matcher, that opt-out is deliberately exact: only an own data property
whose value is boolean `false` enables it. Options must be a number or a plain
object; falsy values, missing fields, accessors, inherited fields, and invalid
option shapes cannot quietly select a soft path. See
`OFFICIAL-FAIL-CLOSED-REPORT.md` and `INDEPENDENT-FAIL-CLOSED-REVIEW.md`.

## Safe verification

After the static audit recorded in `AUDIT.md`, run only:

```powershell
node --check src/dxf-strict.mjs
node --test test/dxf-strict.test.mjs
```

The test invokes only this directory's source and fixture text. It does not
execute the Downloads project, start a browser, spawn a process, access the
network, or write outside test process output.
