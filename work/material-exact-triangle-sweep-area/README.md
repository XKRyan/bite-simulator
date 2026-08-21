# Exact triangle/sweep area kernel

This work-only module computes the area shared by:

- an interior-disjoint triangle cover of the current remaining material; and
- the exact union of convex cells from the declared discrete working-face sweep.

All input `Number` coordinates are decoded to their exact IEEE-754 dyadic values. Convex clipping, area, first moments, and polar second moments use reduced `BigInt` rationals. The public scalars are accompanied by outward-rounded one-ULP bounds derived by comparing each result with the exact rational. This is intended to support a later chip mass/COM/inertia ledger without reconstructing those quantities from target pre/post deltas.

Positive-area overlap in the material triangle cover is a domain stop. Sweep cells may overlap under finite rotation: each new convex cell is exactly differenced against the accumulated union with rational half-plane clipping, producing interior-disjoint convex pieces before any material intersection or moment sum. A hard piece-count cap stops pathological arrangements. Shared edges and points are allowed.

Every declared convex cell must also be simple in its supplied vertex order. All non-adjacent edge pairs are checked with exact rational orientation and on-segment tests, including collinear overlap. This prevents a bow-tie or more complex self-intersection from being silently replaced by its convex hull even when its local turn signs and shoelace area happen to look consistent.

`prepareTriangleCover()` performs the expensive exact disjointness check once and returns an identity-bound, immutable capability. A cloned or foreign capability is rejected. This is a numerical integrity mechanism, not a physics authority.

This is not an authority. It does not establish triangle-cover completeness, working-face ownership, TOI identity, the Rapier state, non-folding motion, or production removal. Those must be supplied by a private, signed composition before this kernel can be used beyond numerical falsification.

Run:

```text
node work/material-exact-triangle-sweep-area/run-tests.js
```
