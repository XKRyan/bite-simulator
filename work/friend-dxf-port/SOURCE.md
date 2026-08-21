# Provenance and integration notes

Source reviewed read-only: `C:\Users\Administrator\Downloads\bite-simulator-v1.0\bite-simulator-v1.0`

- selected commit: `63812623d7616a654374190e92ec08a027345445` (`6381262`,
  `fix: harden simulator geometry and presets`)
- parent: `9e835c32703ce39350f0e04fa1affb71fa8a7d18`
- source `app.js` SHA-256:
  `7B12CB1CFAF7C790A38EC216F31CA628A7E605D55F22E67B57B0F8279D2478C3`
- source fixture SHA-256 values are recorded in the parent work-only audit:
  `work/friend-version-review/AUDIT.md`.

The code here is an independently written, narrow ESM adaptation of the
following source concepts: `parseDxf`, planar validation, `drawingSolid`,
`prepareCadSolidTriangulation`, `initialSolidTargetOverlapArea`,
`exactCadSolidTargetPenetration`, and `matchGeometricContactsToSolver`.
The fixtures are rebuilt as concise ASCII equivalents; they are not imported
from or executed out of the Downloads tree.

## Production/S2 wiring anchors (do not patch as part of this prototype)

Current production file: `outputs/bite-simulator/app.js`.

1. Replace the body of its DXF parse/import validation path near
   `parseDxf` (currently line ~7221) and `createImportedDrawing`, after an
   adapter converts this module's `[x,y]` points to its `{x,y}` points and
   scales millimetres to metres.
2. Make `drawingSolid` consume only `validateSingleOuterSolid(...).solid`;
   retain one canonical `rings` value for collision, clipping, and automatic
   mass properties.
3. Adapt `prepareSolidTriangulation` at collider construction. Pass the
   already-loaded local Earcut function **and an explicit, calibrated,
   metre-valued geometry floor**; preserve the returned boundary edge keys and
   mark all other triangle edges as seams, never as tooth faces. An omitted or
   below-floor feature is a geometry-domain stop, not a fallback triangulation.
4. At initial rig construction, call `assertNoInitialOverlap` through the
   existing local polygon-clipping area adapter. A positive area is a setup
   failure, never an allowed penetration. The default is strict zero; bind any
   polygon-kernel numerical tolerance explicitly in m² at this adapter.
5. At the existing exact CAD penetration audit (near
   `exactCadSolidTargetPenetration`), use `exactSolidRectanglePenetration`
   or retain the current equivalent. Compare its result to the *existing*
   `0.00008 m` hard gate and rollback path unchanged.
6. At `collectRigContacts`, retain geometric index for impulses and mapped
   solver index for world witness/distance. The prototype's
   `matchGeometricContactsToSolver` is stricter than the friend helper: every
   contact needs exactly one counterpart; ambiguity, duplicate witnesses, or
   either kind of unmatched contact is a fail-closed stop.
   Its optional soft diagnostic return may be requested only with an own plain
   options field `failClosed: false`; never forward UI truthy/falsy values.

## Known merge conflicts / prerequisites

- S2's `automaticWeaponMassProperties()` owns the mass/COM/inertia contract.
  Do not copy friend `weaponInertia` input or preset assignments. Feed S2 the
  validated canonical rings and let it derive inertia.
- S2/S3 candidate files are frozen. This module does not import them, alter
  them, or select a phase/material capability.
- The production point representation and imported DXF unit scaling differ
  from this pure module. Write explicit adapters rather than mixing object
  shapes or raw-mm and metre values.
- Production already has local Earcut and polygon-clipping. Do not add a new
  dependency or copy a library into production; inject the existing local
  function at the two adapter boundaries.
- The mesh proof intentionally accepts only original canonical vertices:
  boundary edges must occur exactly once and all internal indexed edges exactly
  twice. If a future triangulator introduces Steiner vertices or cannot satisfy
  this proof, reject it until a separately audited proof adapter exists.
