# Source-level minimal merge map

## Authorities

The merge uses these immutable parents:

- common S3 ancestor: `2381261205DCF64E82439480DDF937DFCE7AD10872FF2509334900E0B63B89D7`
- sole current/base side: signed S4c `81953B2929DA15C4DBCCB821C81CBB15C8164C8BCFEB6812F2C1217CDBE56A1C`
- imported DXF delta side: fixed DXF candidate `AA04E8839B7C79E2E41E74C36B16135ECB289C96953C6899E6DC8BB7524E5638`

The combined source was created by applying only the DXF delta from `238... -> AA04...` to a byte copy of `81953...`. The old DXF candidate was never used as the destination or authority side.

## Minimal delta map

| Parent delta | Logical diff | Combined replay | Proof |
|---|---:|---:|---|
| S3 -> S4c | `+1365 / -3`, 1368 changed lines | DXF -> combined: `+1365 / -3` | all 1368 added/removed lines are sequence-identical |
| S3 -> fixed DXF | `+863 / -182`, 1045 changed lines | S4c -> combined: `+863 / -182` | all 1045 added/removed lines are sequence-identical |

`verify-combined-source.mjs` computes both comparisons directly with zero-context source diffs. This is stronger than matching totals: every added and removed source line, in order, must be identical on both sides.

## Conflict audit

The base-coordinate hunk sets overlap only at the insertion immediately after S3 line 4:

- S4c inserts its private pins, module references, token/capability ownership and checkpoint declarations.
- DXF inserts the private strict parser/topology/coverage port.
- The S3 base contributes no replaced text at that point.

Resolution is concatenation, not line selection:

1. preserve the S4c block first and unchanged;
2. append the strict DXF block unchanged;
3. retain the pre-existing private material-witness port after both.

The resulting top order is anchored at combined lines 5, 73 and 427. No conflict marker remains.

All other hunks merge without a textual overlap. S4c owns every private 78F/token/capability/checkpoint region, including:

- private module pin and require: combined lines 14-46;
- live Sfree capture: around line 9284;
- generation, exact-token and capability ownership, private witness and rollback logic: lines 12888-13965;
- fixed-argument QA boundary: lines 13966-14043.

The imported DXF delta owns only its S3-relative additions/replacements, including:

- frozen strict browser port: line 73 onward;
- legacy payload/manual inertia removal: line 1843;
- strict fork/weapon geometry and automatic mass properties: lines 2157-3041;
- strict staged rebuild/rollback and importer: lines 11972-12038;
- fixed public DXF QA bridge and `importDxfText` entry.

## P0 callback fix

The replayed DXF delta already contains the signed three-line hang fix. Combined anchors are:

- line 2161: `activeForkBodyPoints`
- line 2483: `getShovelBoundsAt`
- line 2495: `getShovelLeadingEdgeAt`

All use `flatMap((path) => samplePathForCollision(path))`. There are zero remaining `flatMap(samplePathForCollision)` calls and no diagnostic trace code.

## Domain ownership

Conflicts are resolved in favor of S4c private authority. The combined source still has:

- `TRACEABLE_CUTTING_CONTINUOUS_TOI_ENABLED = false` at line 1362;
- `MATERIAL_EVENT_MAIN_TRANSACTION_WIRING_ENABLED = false` at line 1373;
- signed 78F pin `78F53...` and AE46 pin `AE46...` unchanged;
- no new 80D/KKT/publication/removal authority.

The current combined raw source hash is `06B6EC1F6E611DDD4122652DC9763D8F75129904D224ACCA3CABB1AF3D71D0AA`.
