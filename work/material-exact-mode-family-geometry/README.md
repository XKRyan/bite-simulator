# Exact mode-family triangle geometry

This work-only composition replaces the fixed polygon-boolean sample guard in the finite-rotation mode-family prototype with exact dyadic-rational triangle/cell intersections.

The signed Coulomb solver still selects the complete affine active-mode envelope. Each exact `p` sample is first solved by that private mode family, then overlapping endpoint-chord cells are reduced to an exact interior-disjoint union and intersected with an independently interior-disjoint triangle cover. Area, centroid, and polar moments come from exact `BigInt` fractions; the public area enclosure is one output ULP. Interval variation remains the conservative finite-rotation analytic bound, split at every Coulomb mode transition.

The module cross-checks the exact area against the independent polygon enclosure and requires the triangle-cover total area to match the frozen remaining-geometry area. Equal area is not a proof of equal support, so triangle-cover completeness remains an upstream authority obligation.

For zero-angular constant-twist translation, the time cells partition one translated strip. A rotating interval additionally proves every cell keeps a strict common orientation throughout the `p` interval and every non-adjacent pair stays separated using midpoint distance minus certified endpoint displacement radii. If that stronger proof is unresolved, the oracle returns the conservative full range `[0,currentArea]`; the left-first root search may then subdivide. It never substitutes a sampled rotating topology for an interval proof.

Both exact samples and interval-midpoint topology checks preserve the declared time-cell vertex order and reject self-intersection; they never convex-hull-fill a folded cell. A folded interval therefore receives the conservative full-range enclosure, while a folded exact endpoint is a geometry-domain stop.

This module does **not** authorize Sfree, TOI, path, working-face ownership, or material removal.
