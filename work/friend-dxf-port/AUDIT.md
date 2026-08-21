# Work-only DXF port audit

Date: 2026-08-20 (Asia/Shanghai)

## Boundary and provenance

- Reviewed provenance is friend commit
  `63812623d7616a654374190e92ec08a027345445`, parent
  `9e835c32703ce39350f0e04fa1affb71fa8a7d18`.
- Downloads original was neither edited nor executed. The source was inspected
  read-only through the already-audited `work/friend-version-review/isolated-copy`.
- Only files under this directory were created. No production file, S2/S3
  frozen candidate, friend isolated copy, or Downloads file was changed.
- Current production `outputs/bite-simulator/app.js` SHA-256 observed after
  this work: `3A58BFB55ACA173DB63435F75C7FDF19BB0F6D68D42AEABF4E77FC1A7AB2D344`.
  It was rechecked after the P0 hardening and remained the same.

Source hashes used for traceability:

| source at 6381262 | SHA-256 |
| --- | --- |
| `app.js` | `7B12CB1CFAF7C790A38EC216F31CA628A7E605D55F22E67B57B0F8279D2478C3` |
| `qa/dxf-fixtures.mjs` | `CD33804B6DB086216A219F128A7875FD5D6383063B1E46C5D836EB7AC140A528` |
| `qa/dxf-fixtures.test.mjs` | `BEBA8501F6EC35EC9DF0365747AEC7459FCBB3F1F17A4B9D47BA9E4830A29F1B` |

## Static safety review before execution

The newly written `src/*.mjs` and `test/*.mjs` were searched for child-process,
spawn/exec/fork, filesystem, HTTP/fetch/XHR/WebSocket/EventSource/beacon,
dynamic import/require/eval/Function, `process.exit`, and timer calls. No match
was found. The code uses only ESM imports, `node:assert/strict`, and `node:test`.
It contains no external dependency, DOM, browser launch, or write operation.

## Verification run (copy only)

With Node `v24.15.0` and cwd `work/friend-dxf-port`:

| command | result |
| --- | --- |
| `node --check src/dxf-strict.mjs` | pass |
| `node --check test/dxf-fixtures.mjs` | pass |
| `node --check test/dxf-strict.test.mjs` | pass |
| `node --test test/dxf-strict.test.mjs` | 13 pass / 0 fail, 108.9807 ms |

Output hashes:

| work file | SHA-256 |
| --- | --- |
| `src/dxf-strict.mjs` | `BE5DF7D7A2C4B5CF735073E4A653C55FD928F5E112F8F162B2E25F22327C1F1B` |
| `test/dxf-fixtures.mjs` | `75BCBA904C3DE7EC0EFB5B17D42FC699ADF76E1B52B8B7E49E4E046B1DFEFD84` |
| `test/dxf-strict.test.mjs` | `21B857DF6724C8640EDFC47FA0870771F774D1AD0DBEBE3AF6F589FA70C58E86` |

## Semantic limits

- The module holds the unchanged 80 um value as
  `PENETRATION_GATE_METRES = 0.00008`; it does not accept or apply a looser
  gate. Production rollback and gate ownership remain untouched.
- Initial-overlap detection takes an already-computed exact intersection area.
  An integration adapter must call the existing local polygon-clipping
  intersection and then `assertNoInitialOverlap`; this prototype intentionally
  does not ship clipping code.
- Triangulation takes a supplied local Earcut-compatible function and verifies
  triangle area against the outer-minus-holes area. The test uses a controlled
  hole-aware fixture triangulator, not a copied third-party library.
- Exact penetration is for a canonical solid against an oriented rectangle;
  the adapter must provide positions/angles/half-extents in one unit system.
- No automatic mass-properties implementation is included: S2 must remain the
  sole owner of automatic mass/COM/inertia, fed from the same validated rings.

## P0 independent-review hardening (same work-only directory)

The earlier area-only Earcut check could accept duplicate coverage of one half
of a solid while omitting another half, if the doubled triangle area happened
to equal the net solid area. It has been replaced by a proof-oriented gate:

1. Every non-degenerate indexed triangle has a strict-solid centroid and
   in-solid edge midpoint; it cannot cross canonical boundaries or fill a hole.
2. Pairwise triangle interior overlap and non-manifold/T-junction edge overlap
   are rejected.
3. Every canonical boundary edge is used exactly once; each non-boundary mesh
   edge exactly twice. This proves the canonical boundary is present and the
   indexed mesh has no open internal seam.
4. Only after containment, non-overlap, and edge multiplicity pass is summed
   triangle area compared to canonical outer-minus-holes area using a small
   floating-point arithmetic tolerance. Together these establish bidirectional
   coverage for the no-Steiner-vertex mesh; a triangulator unable to provide it
   is rejected rather than trusted.

The P0 regression set adds: duplicate-half/same-area, fill-hole plus
equal-area omitted patch, missing canonical boundary, required/too-thin
geometry floor, strict initial-overlap default/external tolerance, and
unmatched/ambiguous contact mapping. The current test count is 12.

`prepareSolidTriangulation` requires an external positive `geometryFloor` in
the same unit as its rings. This is deliberately not inferred from the 80 um
penetration gate. `assertNoInitialOverlap` defaults to zero; a clipping kernel
tolerance is explicit as `kernelAreaTolerance` in squared units. Contact
mapping throws `GeometryContractError` by default; `{ ok: false }` is available
only through explicit `failClosed: false` for a caller that immediately gates
its result.

## P0 fail-closed bypass remediation

Independent review identified that the prior object spread could copy falsey
`failClosed` values such as `0` or `''`; `contactFailure` then used a truthiness
test and could return softly. The narrow fix accepts only a numeric tolerance
or a plain object, rejects accessors/non-boolean supplied `failClosed`, ignores
inherited fields, and chooses a soft result only for an own data property
exactly equal to boolean `false`. `contactFailure` independently uses the same
exact-false condition.

The new mutation test asserts hard errors for missing, undefined, null, zero,
empty string, and true `failClosed` variants; for invalid top-level option
types; and for a getter. It separately proves that literal `false` remains the
sole soft diagnostic mode. Official and independent review records are in
`OFFICIAL-FAIL-CLOSED-REPORT.md` and `INDEPENDENT-FAIL-CLOSED-REVIEW.md`.
The complete post-remediation file set is frozen in `FREEZE-SHA256SUMS.txt`.
