# S4b-1A pure material-geometry kernel

This directory contains a disconnected, work-only mathematical kernel. It is
not wired to the simulator, does not enable material removal, and does not
authenticate a path, tooth face, collision witness, body, or Sfree state.

## Critical authority boundary

**PATH / OWNER AUTHORITY IS NOT SIGNED BY THIS MODULE.**

`createTranslationSweepKernel(config)` accepts an already-authorised
mathematical descriptor. Its `pathDigest`, `geometrySnapshotDigest`, payload
digests, and interval digests are integrity checks only; they are not trust
credentials. A later B-layer wrapper must own an opaque live-session capability
and capture, from the same Rapier Sfree transaction, at least:

- step `h`, body handles, body poses and local COM/material frames;
- full tooth vertices and immutable working/non-working edge ownership;
- current authoritative remaining MultiPolygon;
- the material row, frozen KKT mode family, TOI witness, and their common frame.

Until that B layer is independently signed, this kernel is not production
ready and its raw config must not be reachable from the simulator main loop.

## Proven mathematical domain

The kernel accepts only:

- the current remaining material as a complete polygon-clipping MultiPolygon,
  retaining outer rings, holes, and disconnected components;
- one nonzero fixed-orientation line segment;
- an affine translation `d(p) = d0 + d1*p` over a finite `pDomain`;
- exactly zero constant and p-dependent angular travel.

Finite rotation, an unresolved boundary feature, a numerically unresolved
positive sweep, an invalid/self-crossing polygon, or an area-error enclosure
above the fixed relative cap returns `solver-domain-stop`.

## Exact samples and numerical enclosure

For the supported domain, the continuous sweep is exactly

`S(p) = workingSegment MinkowskiSum [0, d(p)]`.

It is represented by the convex hull of its four endpoints. The fresh area is
computed as `area(G intersection S(p))`, and checked against
`area(G)-area(G difference S(p))`. These are two consistency paths through the
same pinned polygon-clipping kernel, not independent numerical kernels.

Every published sample includes an area interval. The internal length guard is
derived from coordinate scale, operation count, and machine epsilon; a caller
may only enlarge it. Its area enclosure is dimensionally consistent:

`E_area = 32*(P_total*eps_length + N_edges*pi*eps_length^2) + E_roundoff`.

`E_roundoff` scales as coordinate-scale squared times machine epsilon and an
operation-count factor. The kernel refuses the domain if this guard exceeds a
scale-independent `1e-8` fraction of the reference area. Geometry must also
have feature separation at least `128*eps_length`, providing the topology
resolution assumed by the boundary-tube bound.

## Conservative p-interval

For midpoint `m` and every `p` in `[lo,hi]`,

`Hausdorff(S(p),S(m)) <= delta = |d1|*(hi-lo)/2`.

Each sweep is convex and has generalized perimeter bounded by

`Pmax = 2*(faceLength + max(|d(lo)|,|d(hi)|))`.

The two Steiner inclusions give

`area(S(p) symmetricDifference S(m)) <= 2*Pmax*delta + 2*pi*delta^2`.

Intersection with a fixed `G` contracts symmetric difference. The published
bound is therefore the midpoint's complete numerical area enclosure expanded
by this geometric variation. The endpoint numerical enclosures are unioned
explicitly, so returned endpoints cannot escape through an implicit tolerance.

## Capability boundary for the simulator

The parameter-tooth domain remains only a geometry/transaction control fixture.
It does not establish detached-fragment physics. Production parameter macros
must keep `materialRemovalDefined=false` and `fracturePathDefined=false`.
Detached removal remains restricted to a model with a defined working edge and
sourced `h_min`, `Gc`, and `Uc`; otherwise use a mass-conserving plastic-groove
model or stop the material response.

Run:

```text
node work/material-geometry-interval-oracle/run-tests.js
```
