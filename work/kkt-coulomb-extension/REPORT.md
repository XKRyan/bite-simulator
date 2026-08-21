# Coulomb KKT prototype audit report

Status: **reference tests pass; work-only; not production-integrated**.

## Result

`run-tests.js` passes 13/13 named tests. The last test contains 120 additional
fixed-seed coupled systems, so failures remain exactly reproducible.

The deterministic two-contact witness has these exact mode boundaries:

| `p` interval | fork-target | target-floor | material work |
| --- | --- | --- | ---: |
| 0 to 0.5 | inactive | inactive | 1.550 |
| 0.5 to 0.6 | `-slide` | inactive | 0.145 |
| 0.6 to 1.0 | `-slide` | `-slide` | 0.336 |

Therefore `D(1) = 2.031`. Both slide impulses lie on their input Coulomb
boundaries, their signs oppose post-contact slip, and the remaining structural
friction dissipation is non-negative. At the prepared `D(p)=2` constitutive
root, the energy ledger closes to floating-point zero.

## Covered invariants

- stick, positive slide, and negative slide;
- simultaneous multi-contact KKT mode switches;
- common contact point for each normal/tangent impulse pair;
- `|pt| <= mu*pn` and maximum-dissipation slip sign;
- minimum post-event kinetic energy across feasible modes;
- no post-event kinetic-energy gain over the free state;
- exact affine-segment integration of `D(p)`;
- same-`p` binding of impulse, velocity, mode, contact points, and geometry;
- explicit no-feasible-solution and candidate-cap domain stops;
- rollback after an injected partial-commit fault;
- byte-deterministic replay and signature;
- explicit zero material friction when no material tangent law exists.

## Production gate

This result is necessary but not sufficient for production use. Before any
merge, the caller must generate the normal and tangent rows from the same real
manifold point, preserve the fixed candidate cap, bind the result into the
existing TOI transaction, close the real chip mass/momentum/energy ledger, and
repeat timestep convergence and browser fault-injection tests. Until then the
current production material safety stop remains the honest behavior.

