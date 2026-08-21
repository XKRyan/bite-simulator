# S4c private-TOI + strict DXF combined checkpoint

## Verdict

**PASS.** The combined candidate is an exact two-delta union built with signed S4c `81953...` as the sole destination/base. It preserves every S4c private 78F/token/capability change line, adds the fixed strict-DXF browser delta without modifying it, and keeps MAIN, 80D/KKT publication, material removal and fracture authority closed.

No production, S2, S3, signed S4c, signed 78F/AE46, signed strict port, or frozen DXF candidate file was modified.

## Frozen combined artifacts

| Artifact | SHA-256 |
|---|---|
| raw combined source | `06B6EC1F6E611DDD4122652DC9763D8F75129904D224ACCA3CABB1AF3D71D0AA` |
| deterministic private browser bundle | `C100E7FBF724448643D3CF6123EEDECF201A923321838E7B27299CFCC809A2A9` |
| bundle manifest | `27528C05B74BE1ACC777FF954B3104405586F48E3EC18EA2A1B596C0132853D2` |
| build-plan file | `3E2C3D91950E87408419A814CD93A41F202A88FC40F061514807ED14D865DB73` |
| canonical build-plan digest | `0250944AA850F97B4A75E46AC567B65E6EF4AABAC27FF542A53E7B087606ED30` |

The clean rebuild produces the same `C100...` bundle.

## Locked authorities

| Authority | SHA-256 |
|---|---|
| signed S4c unique base | `81953B2929DA15C4DBCCB821C81CBB15C8164C8BCFEB6812F2C1217CDBE56A1C` |
| fixed DXF delta parent | `AA04E8839B7C79E2E41E74C36B16135ECB289C96953C6899E6DC8BB7524E5638` |
| common frozen S3 | `2381261205DCF64E82439480DDF937DFCE7AD10872FF2509334900E0B63B89D7` |
| frozen S2 | `6E8E638407ED1B4F1A3A9F3C97A1E5C191AB6AE8D843CCB1622679E48312C320` |
| strict DXF port | `BE5DF7D7A2C4B5CF735073E4A653C55FD928F5E112F8F162B2E25F22327C1F1B` |
| private 78F | `78F53C3ACA51678BC3BA16336E97BD6F09084AE257F7C3C558450974E28D5166` |
| prepared AE46 | `AE46B3E461BFD5BE7641E71FD279BB2BF56F7743BFB3137ECB36215498F691BF` |
| production app | `3A58BFB55ACA173DB63435F75C7FDF19BB0F6D68D42AEABF4E77FC1A7AB2D344` |

## Source-level merge proof

The merge map is frozen in `SOURCE-DIFF-MAP.md`.

| Delta | Parent diff | Replay onto other parent | Exact result |
|---|---:|---:|---|
| strict DXF | S3 -> DXF: `+863 / -182` | S4c -> combined: `+863 / -182` | all 1045 changed lines sequence-identical |
| private S4c | S3 -> S4c: `+1365 / -3` | DXF -> combined: `+1365 / -3` | all 1368 changed lines sequence-identical |

There was one source-text conflict: both parents insert after S3 line 4. The base has no replaced text there. Resolution concatenates the unchanged S4c private block first, the unchanged strict-DXF port second, and the pre-existing material-witness port third. No private S4c line was selected from the older DXF side.

The three callback-arity fixes are retained at combined lines 2161, 2483 and 2495. There are zero direct `flatMap(samplePathForCollision)` calls and no diagnostic trace instrumentation.

## S4c/private authority results

| Gate | Result | Report SHA-256 |
|---|---:|---|
| real Edge S4c named checks | **22/22** | `099C193669C060D37F0A7EBA80150662E7FE4975C69DA51E4E0303D4A672799B` |
| combined checkpoint/supply-chain audit | **24/24** | `F85AF851941EF89588600B6F667E599EFA4DB65C6E12627A30619F9608931CF7` |
| independent S4c anchors | **20/20** | `4470BBEF72F2DB8FF334E77F1A904FB91FACDDA18019414D1D1AA00565A9A590` |

The real parameter-tip checkpoint still performs exactly one Rapier step, one force application, one AE46 Sfree capture and one private 78F call. Clone, cross-session, stale-token/capability and all five injected-fault cases remain rejected with exact root restoration. Fixed root/back/rotation/hole/no-contact cases preserve same-TOI point, normal and closing certificates. Raw module tables, tokens, capabilities and `s4cToiModule` remain private.

`MATERIAL_EVENT_MAIN_TRANSACTION_WIRING_ENABLED` and `TRACEABLE_CUTTING_CONTINUOUS_TOI_ENABLED` remain `false`. The checkpoint still domain-stops before hidden-trajectory authority, 80D, KKT, publication, writeback, fracture or removal.

## Strict DXF results

| Gate | Result |
|---|---:|
| combined source/union/private-boundary static checks | **6/6** |
| signed strict source port | **13/13** |
| real Edge strict/fault/rollback/overlap prefix | **5/5** |
| real Edge fork solid/hole/rollback/live-world suite | **4/4** |
| real Edge weapon solid/hole/mass/preset suite | **PASS** |
| real Edge frozen-S3 CAD/parameter tail | **2/2** |

The strict implementation embedded in the combined source is byte-equivalent to `BE5...` after removing only ESM `export` keywords. Equal-area repeated coverage, equal-area hole-fill/omission, missing boundary coverage, holes, T-junctions, the 80 µm feature floor, initial overlap and fail-closed contact matching remain covered.

### Weapon mass properties

For a `0.2 kg`, `80 x 60 mm` solid rectangle:

- area: expected/observed `0.0048 m²`;
- COM: expected/observed `(0, 0)`;
- pivot and central inertia: expected/observed `0.0001666666666666667 kg·m²`;
- proof triangles and Rapier colliders: `2 / 2`.

For the same outer with a `20 x 20 mm` hole offset to `x = 20 mm`:

- net area: expected/observed `0.004399999999999999 m²`;
- COM x: expected `-0.0018181818181818188 m`, observed canonical `-0.001818181818181818 m`, Rapier `-0.001818181830458343 m`;
- pivot inertia: expected/observed `0.00017333333333333336 kg·m²`;
- central inertia: expected/observed canonical `0.00017267217630853997 kg·m²`, Rapier `0.00017267218208871782 kg·m²`;
- proof triangles and Rapier colliders: `8 / 8`.

Changing mass to `0.25 kg` recomputes pivot inertia to `0.00021666666666666668 kg·m²`. A simultaneous manual `weaponInertia` payload is discarded. Applying the legacy `454g` preset migrates its historical value to a mass input of `0.04615384615384615 kg`; the resulting `0.00004 kg·m²` inertia remains explicitly geometry/mass-derived and the UI field stays read-only.

### Fork mass properties and P0 rollback

Both a legal solid fork and the same fork with an offset hole return well below the 1 s gate. At `0.03 kg`, the offset-hole Rapier COM is `(-0.019655171781778336, 0.006689655128866434) m` and central inertia is `0.00002539974775572773 kg·m²`. Changing mass to `0.045 kg` keeps COM fixed and scales central inertia to `0.0000380996243620757 kg·m²`, exactly `1.5x` within tolerance.

The accepted offset-hole fork completes 2000 ticks with a valid Rapier mass graph. The formerly hanging unmountable rectangle returns `DXF_REBUILD_FAILED` in about `2 ms` with an exact full snapshot rollback. A fault on the fifth collider create, after four successful creates, also rolls back exactly; the restored old world/handles then run 61 ticks bit-exact with an untouched control.

### Frozen runtime baselines

- rigid built-in CAD: full 1 s / 2000 steps is bit-exact with frozen S3 in normalized state, metrics, current frame, parameter controls and history DOM;
- rigid parameter weapon + parameter fork: the same 2000-step comparison is bit-exact;
- offset-hole fork: 2000-step accepted-world run passes;
- runtime material removal remains `false` throughout.

## Reproduction

From repository root:

```powershell
node work\s4c-private-toi-dxf-combined\build.js
node work\s4c-private-toi-dxf-combined\run-checkpoint-browser.js --candidate-sha=C100E7FBF724448643D3CF6123EEDECF201A923321838E7B27299CFCC809A2A9
node work\s4c-private-toi-dxf-combined\audit-combined-checkpoint.js
node work\s4c-private-toi-dxf-combined\run-independent-s4c-audit.js
node --test work\s4c-private-toi-dxf-combined\verify-combined-source.mjs
node --test work\friend-dxf-port\test\dxf-strict.test.mjs
node work\s4c-private-toi-dxf-combined\run-edge.js edge-final.html prefix
node work\s4c-private-toi-dxf-combined\run-edge.js edge-fork-fix.html
node work\s4c-private-toi-dxf-combined\run-edge.js edge-min-weapon.html
node work\s4c-private-toi-dxf-combined\run-edge.js edge-tail.html
```
