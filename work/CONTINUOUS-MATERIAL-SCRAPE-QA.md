# Continuous material scraping — independent acceptance contract

This is a work-only acceptance suite. It never edits `outputs/bite-simulator` and it never treats a rigid result as a material result.

## Current baseline (expected failure)

The source bytes are frozen by the runner before a browser is started. On the current 1.0.1 development source:

- `TRACEABLE_CUTTING_CONTINUOUS_TOI_ENABLED` is `false`.
- CAD cutting can be *requested* only when a traceable `h_min` and `Gc` are supplied, but it cannot be enabled.
- Parameter teeth are additionally excluded from `materialCuttingRequested`.
- The first load-bearing weapon contact is rolled back and ends in `modelDomainStopped`, with zero committed damage.
- The dormant sweep starts at `gap / closingSpeed`. That is a linear estimate for one reported contact, not the earliest compressive root over all tooth/target features.
- Therefore a safe transactional stop is a useful safety property, but it is an expected **material-acceptance failure**. It proves neither scraping, groove persistence, material conservation nor 4 s operation.

Historical frozen reports already demonstrate the safe-stop baseline for parameter teeth N=1/10/20 and both fork types (`work/final-independent-physics-audit-material.json`). The new `baseline` suite rechecks both CAD and parameter weapon paths against one newly frozen source hash.

## Required geometry matrix

The mandatory matrix is the Cartesian product below (96 cases):

| Axis | Values |
|---|---|
| Weapon geometry | parameter solid; imported closed CAD solid |
| Fork geometry | parameter zero-thickness line; imported/built-in CAD fork |
| Physical teeth | 1, 10, 20 |
| Transmission ratio | 0.5 (low), 2.1 (high) |
| Initial angle | 0°, 90°, 180°, 270° |

For the CAD branch, N is not a cosmetic parameter. The harness imports a deterministic closed star/lobe DXF containing N actual, separated lobes and rejects the case unless the active CAD detector reports exactly N teeth. Parameter N=10/20 uses the production non-overlap geometry validator.

Every case uses `contactModel: "material"`, a nonzero traceable QA `h_min`, a nonzero `Gc`, and real motor acceleration. A result from `contactModel: "rigid"` or runtime mode other than `traceable-cutting` fails even if its penetration and energy gates are green.

## Acceptance gates

All tolerances below are declared before results are viewed. A missing observable is `FAIL`, never an inferred pass.

1. **Continuous event and TOI order**
   - Every committed removal event identifies its physical cutting feature and tooth order.
   - It exposes all compressive candidate fractions for that substep, the selected fraction, and method `continuous-compressive-root`.
   - The selected fraction equals the minimum valid compressive candidate to `1e-9` in fractional time; committed events in one fixed tick are nondecreasing in TOI.
   - State is advanced to that TOI, the coupled contact/material response is solved there, geometry and mass are committed atomically, and the exact remainder is replayed. `remainderReplayed` must be true.
   - `gap / closingSpeed` alone fails this gate, including when it happens to be numerically close.

2. **Independent material geometry and no duplicate removal**
   - Remaining area is recomputed in the QA page from the returned even-odd MultiPolygon by an independent shoelace implementation.
   - At every accepted fixed tick:
     `delta(removedArea) == -delta(remainingArea)` and
     `removedArea == initialArea - remainingArea`.
   - `removedArea`, `removedVolume`, and `removedMass` are finite and monotone nondecreasing. Remaining area and target-body mass are monotone nonincreasing.
   - The sum of independently observed fresh-area deltas equals the final geometric complement. Re-entering an old groove cannot increase any removed-material counter. This is the production duplicate-removal gate; hit count and rigid impulse are not substitutes.
   - Area tolerance is `max(1e-12 m², initialArea × 1e-8)`.

3. **Mass/volume conservation**
   - `removedVolume == removedArea × effectiveWeaponZOverlap`.
   - `removedMass == density × removedVolume`.
   - `currentTargetBodyMass + removedMass == initialTargetBodyMass`.
   - Volume tolerance is `max(1e-15 m³, initialVolume × 1e-8)`; mass tolerance is `max(1e-10 kg, initialMass × 1e-8)`.
   - COM and inertia must remain finite with positive remaining mass/inertia. No rigid upper-bound impulse is accepted as evidence for these identities.

4. **Groove history across teeth**
   - N=10 and N=20 long-scrape sentinels must commit fresh area from at least two distinct tooth orders.
   - Cut history retains tooth identity, TOI and remaining-area version after later teeth arrive.
   - Earlier removal is never healed: all per-tick remaining-area samples are nonincreasing, and final geometry still accounts for the sum of every fresh delta.
   - A 256-entry display ring buffer is not an acceptable conservation ledger; the QA accumulates every accepted version delta independently.

5. **Momentum and energy ledgers**
   - Every tick which changes geometry exposes a closed material momentum ledger: pre-state, external/joint impulse, boundary/contact impulse, outgoing chip momentum, post-state, residual and a declared numerical tolerance. Linear and angular residuals must be within that declared tolerance.
   - Every such tick exposes a phase-separated energy ledger: pre mechanical energy, actuation work frozen before collision response, Rapier/joint exchange, finite material dissipation, mass-removal energy, post mechanical energy, residual and tolerance.
   - Material work, boundary work, mass-removal energy and constraint exchange are not double counted. Dissipation is nonnegative; numerical energy gain and ledger residual do not exceed the declared tolerance.
   - Every completed public impact episode must also have `energyConverged === true` and `numericalEnergyGain <= energyTolerance`.
   - Missing outgoing-chip momentum or a still-open episode without a per-tick ledger fails.

6. **Penetration**
   - Each role (`weapon`, `fork`, `floor`, `forkFloor`) satisfies accepted geometric penetration `<= 80 µm + 0.1 nm` for every accepted state.
   - Rejected penetration remains audit data; it never licenses an accepted state above 80 µm.

7. **Fixed-step convergence**
   - Representative cases are rerun from identical source bytes at 0.5, 0.25 and 0.125 ms. The runner changes only the work-served frozen copy of `FIXED_DT`.
   - Tooth order and event order are identical. Fine-pair event-time disagreement is at most 0.25 ms.
   - For removed area/volume/mass/material work, the 0.25→0.125 ms difference is no larger than 75% of the 0.5→0.25 ms difference (plus the invariant absolute tolerance), and the fine-pair relative difference is at most 1%.
   - Fine-pair target/weapon surface-pose disagreement at the scrape checkpoint is at most 80 µm. A run which merely stops at all three steps is not convergence.

8. **Transactional rollback**
   - The work-served copy can inject a failure immediately after a provisional removal commit.
   - The fixed tick must abort with a clear material-domain failure. Rapier world snapshot bytes, body pose/velocity/mass/COM/inertia, material geometry/stats/history, contact ownership, counters and event history equal the pre-tick values exactly.
   - Terminal failure text and rejected-attempt diagnostics are excluded from equality because they are intentionally written only after rollback.
   - The injection must actually fire; the ordinary current safety gate is not sufficient evidence.

9. **Duration semantics**
   - Valid sentinels run naturally to 4.000 s without solver/model stop.
   - A 1.250 s user horizon ends at 1.250 s, and its final physical/material state equals the 1.250 s prefix of the otherwise identical 4 s run.
   - Duration changes only the horizon; it must not change fixed-step dynamics or delete groove history.

## Suites and entry points

Run from the repository root:

```text
node work/run-continuous-material-scrape-qa.js --suite=baseline
node work/run-continuous-material-scrape-qa.js --suite=prototype --angle=90 --parallel=1
node work/run-continuous-material-scrape-qa.js --suite=matrix --parallel=2
node work/run-continuous-material-scrape-qa.js --suite=history --parallel=2
node work/run-continuous-material-scrape-qa.js --suite=convergence --parallel=2
node work/run-continuous-material-scrape-qa.js --suite=rollback --parallel=1
node work/run-continuous-material-scrape-qa.js --suite=duration --parallel=1
node work/run-continuous-material-scrape-qa.js --suite=all --parallel=2
```

Useful development options:

```text
--app=work/candidate-app.js
--single=0
--horizon=1
--output=work/custom-material-report.json
```

The runner freezes and hashes the selected app at startup, serves only an instrumented in-memory copy, records the end hash, and writes `work/continuous-material-scrape-report-<suite>.json`. It creates isolated browser profiles under `work/.edge-continuous-material-*` and removes only the exact profile it created after that process exits.

`prototype` (alias `single-tooth`) is the fast author loop: parameter weapon + parameter fork, N=1, ratio 0.5, default angle 90°, 0.4 s. It runs one normal continuous-removal case and one post-provisional-commit failure-injection case. The first must expose/validate the earliest-TOI audit and all conservation ledgers; the second must expose the same TOI audit and restore world bytes plus simulation bookkeeping exactly. `--angle=<deg>` changes only this quick suite's initial angle.

## Minimum structural model needed to turn the baseline green

Use one geometry-neutral `CuttingFeature` contract for both CAD and parameter solids: stable feature/tooth id, closed local removal loop, declared working face, provenance, edge radius/source and removal capability. For each substep:

1. Find the earliest **compressive** TOI over every cutting feature against the current damaged target MultiPolygon (including holes and disconnected components).
2. Advance all coupled bodies to that TOI.
3. Solve the weapon/target/fork/floor cluster once at the common event state, including finite constitutive resistance and an outgoing-chip momentum/energy ledger.
4. Commit the exact polygon difference and target mass/COM/inertia atomically.
5. Rebuild the material boundary and replay the remaining substep, repeating if a later feature reaches a new earliest TOI.

This is the smallest model that can answer the requested continuous-scrape questions. Enabling the dormant end-of-substep impulse path or relaxing the 80 µm gate cannot satisfy this contract.

## Parameter-groove history: minimum physically admissible interface

It is impossible to keep the pristine weapon/target boundary active in the rigid solver and also avoid a full-strength response every time a tooth revisits an already cut groove. A hit cooldown, tooth id cache, impulse clamp or event de-duplication changes telemetry only; the unchanged boundary still delivers a real rigid impulse.

The minimum admissible split is role-specific:

- Keep the untouched structural target shape for target/floor and target/fork support.
- Mask that pristine shape from the weapon pair while material mode owns the pair.
- Resolve weapon contact/TOI against one shared target-local `remainingGeometry` (or an exactly equivalent deformed surface state) used by every CAD/parameter tooth.
- Define `freshRegion = currentSweep ∩ remainingGeometry`, then atomically set `remainingGeometry := remainingGeometry − currentSweep`. Strength, fracture work and removed mass apply only to `freshRegion`.
- Key history by target-local material, not by tooth. Tooth identity is provenance for event order; it is not the state that prevents repeat removal.

For detached cutting, revisiting an empty groove has no material normal response until a surviving boundary is reached. Friction against a surface can exist only where a surviving surface actually exists. For plastic ploughing without detachment, a geometry-only void is insufficient: the state must retain permanent surface displacement plus a local internal variable such as maximum plastic indentation/equivalent plastic strain. Reloading to the stored maximum may be elastic, but it cannot book the full yield/plastic work again; only a new maximum deformation can add plastic work. If that residual-surface constitutive state is not implemented, sustained non-removal ploughing must remain a model-domain stop.

Minimum interface returned to the work audit for every accepted material event:

```text
CuttingFeature { featureId, toothOrder, closedLoopLocal, workingFace, provenance, edgeRadius }
MaterialEvent {
  fixedTick, sequence, featureId, toothOrder,
  toiAudit { method, candidateFractions, compressiveCandidateFractions,
             selectedFraction, remainderReplayed },
  oldGeometryVersion, newGeometryVersion,
  sweptArea, freshArea, alreadyRemovedOverlapArea,
  remainingAreaBefore, remainingAreaAfter,
  removedVolumeDelta, removedMassDelta,
  momentumLedger, energyLedger
}
```

Specific repeat-contact acceptance probes:

1. Replay an identical tooth sweep over an existing groove: `freshArea`, removed volume/mass and fracture/plastic work deltas are exactly zero within polygon tolerance.
2. Send a different tooth through the same groove: no new material response occurs until its continuous TOI reaches surviving material; history must not reset on tooth change.
3. Partially overlap old and virgin material: only the set difference is charged, and `freshArea + alreadyRemovedOverlapArea == sweptArea ∩ initialMaterial`.
4. For N=10/20 over multiple revolutions, cumulative removed area equals the union of all committed fresh regions, independent of the 256-entry display-history cap.
5. For a ploughing-only load/unload/reload cycle, a reload to the previous maximum creates zero new permanent work; a deeper pass charges only the incremental plastic state.

Model limits remain explicit: this is a monotone 2.5D removal/deformation slice over the actual Z overlap; it does not model chip re-deposition, thermal softening or strain-rate dependence without data. A full-width cut that creates independent target fragments remains a domain stop until a multi-body fragment model exists.
