# Continuous triangle / material boundary witness

This directory is an independent, work-only geometry proof. It does not modify
or load the simulator candidate and it does not use a browser.

`material-toi-witness.js` finds the globally earliest onset between one rigid
triangular tooth and every outer/hole boundary segment of the current material
`MultiPolygon`. Coordinates use the same nesting as `polygon-clipping`:

```text
MultiPolygon = [ Polygon ... ]
Polygon      = [ outerRing, holeRing ... ]
Ring         = [ [x,y] ... ]
```

## API

```js
const { findEarliestTriangleMaterialWitness } = require('./material-toi-witness');

const result = findEarliestTriangleMaterialWitness({
  feature: {
    vertices: [
      { id: 'root', x: 0, y: 0 },
      { id: 'tip',  x: 1, y: 0 },
      { id: 'back', x: 0, y: 0.2 },
    ],
    edges: [
      { id: 'working-face', working: true },
      { id: 'back-edge', working: false },
      { id: 'root-edge', working: false },
    ],
    workingVertexIds: ['tip'],
  },
  material: currentMultiPolygon,
  originalMaterial: optionalUndamagedMultiPolygon,
  dt,
  featureMotion: { start: featurePose0, end: featurePose1, angleDelta },
  materialMotion: { start: targetPose0, end: targetPose1, angleDelta },
  history: { hadPriorContact: false },
  options: { timeTolerance: 1e-8, distanceTolerance: 1e-10 },
});
```

`angleDelta` is optional for rotations in `[-pi, pi]`; when supplied it is the
unwrapped finite rotation and therefore supports rotations whose endpoint
angles alone would be ambiguous.

The witness contains feature edge id/index/fraction, optional vertex id,
material polygon/ring/segment id/index/fraction, the two world points and their
common midpoint, material-outward world normal, signed closing velocity, and a
working/non-working classification. A non-working earliest onset always returns
`removalAllowed: false` and `domainAction: stop-non-working-boundary`.

For the named triangle `[root, tip, back]`, shared vertices have deterministic
physical ownership by their incoming edge:

- `tip` -> `working-face` (cutting is permitted when compressive)
- `root` -> `root-edge` (material removal is refused)
- `back` -> `back-edge` (material removal is refused)

This prevents the root endpoint shared by the working face from being
accidentally accepted as a cutting witness.

## Conservative time search

At each queried time, the helper exactly evaluates both rigid poses. It computes
the minimum distance `d(t)` across all 3 feature edges and all current material
boundary segments. A global bound on relative surface speed is

```text
V = |v_feature_origin - v_material_origin|
    + |omega_feature| R_feature
    + |omega_material| R_material .
```

For an interval of width `h`, evaluated at its midpoint `tm`, the distance
function is Lipschitz. The whole interval is contact-free only when

```text
d(tm) - V h/2 > distanceTolerance .
```

Otherwise the interval is recursively subdivided, earliest half first. Thus a
brief rotational contact that enters and leaves between clear endpoints and
clear fixed samples cannot be skipped. Angular cells are only an efficiency
partition; they are never a collision certificate. An unresolved leaf reports
its `[tLower,tUpper]` and

```text
maximumUnresolvedSpatialError
  = distanceTolerance + V (tUpper - tLower) .
```

The full width is deliberate: the returned actual-contact sample can be at an
endpoint of the bracket, so a midpoint `/2` bound would underestimate error.

Every world point, normal and signed closing velocity is reconstructed from the
same leaf/refined TOI pose and the corresponding constant rigid-motion twist:

```text
v_point = v_origin + omega x R(angle_TOI) r_local
closing = -(v_feature_point - v_material_point) dot n_material_outward .
```

No endpoint body state is used for a TOI witness.

The built-in motion model is exact only for constant origin translation and a
constant angular rate over the supplied, explicitly unwrapped `angleDelta`.
When integrating Rapier or another nonlinear solver, the caller must first
rollback/replay into intervals for which this motion representation and the
surface-speed bound are valid. Merely linearly interpolating two nonlinear
prefix endpoint states is not a continuous-motion certificate.

Outer and hole normals do not rely on input winding. Ring 0's normal points from
its interior material to the exterior; later rings point from surrounding solid
into the hole.

## Contact class

- `initial-overlap`: positive-area overlap with current material at `t=0`;
  invalid and never removable.
- `re-entry`: starts clear of current material but inside `originalMaterial`, or
  `history.hadPriorContact` is true.
- `virgin-contact`: all other first onsets.

An initial zero-area boundary touch with non-positive closing is classified as
`separating-boundary`, not as a material event. With the default
`continueAfterSeparatingBoundary: true`, the helper advances only through a
prefix certified non-compressive for every feature/material segment pair and
continues looking for the later first compressive onset. The option can be set
false to return the separating observation immediately. Consequently, neither
a working nor a non-working boundary that is merely being left can mask a later
compressive event.

Run the deterministic Node suite with:

```text
node work/material-toi-witness/run-tests.js
```

It writes `test-report.json` with per-case observables and SHA-256 hashes.
