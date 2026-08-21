# KKT Coulomb extension (work-only)

This directory is an isolated reference prototype. It does not modify or run
the production simulator.

## Model

For a prescribed material-normal impulse parameter `p >= 0`, the solver uses

```text
q(p) = qFree - Minv * materialRow^T * p
       + Minv * sum(normalRow_i^T * pn_i + tangentRow_i^T * pt_i)
```

Each structural contact is enumerated in exactly one of four modes:

- `inactive`: `pn = pt = 0`, post normal velocity is non-negative;
- `stick`: post normal and tangent velocities are zero and
  `|pt| <= mu*pn`;
- `+slide`: `pt = +mu*pn` and post tangent velocity is non-positive;
- `-slide`: `pt = -mu*pn` and post tangent velocity is non-negative.

All active normal and tangent equations are solved in one KKT system. Among
all feasible modes at the same `p`, the selected state has minimum post-event
kinetic energy (Coulomb maximum dissipation). Impulse norm and a code-point
mode key are deterministic tie-breakers only. A candidate that raises kinetic
energy above the free state is a solver-domain stop.

`fork-target`, `target-floor`, and `fork-floor` contacts must each supply their
own `mu`; there is no fallback coefficient. Normal and tangent rows live in
one contact record with one public `point`, so they cannot be committed from
different contact points. The binding signature covers that point, both rows,
`mu`, the selected modes, `p`, `qPost`, and the geometry payload.

The material contact keeps one constitutive normal row. This prototype has no
material tangential law. Omitting it is reported as
`{defined:false, coefficient:0, mode:"none-explicit"}`. Supplying material
`mu`, `friction`, or `tangentRow` is refused instead of guessing a model.

## Piecewise material work

Within a fixed mode, `q(p)` and material speed are affine. The implementation
constructs all feasibility endpoints and pairwise kinetic-energy crossings of
the feasible modes, then integrates

```text
D(p) = integral from 0 to p of max(materialRow*q(s), 0) ds
```

analytically on each resulting interval. The material-work root, KKT state,
contact points, modes, and fresh-area payload are committed at the same `p`.
There is no endpoint averaging, sequential projection, impulse scaling, speed
cap, or feedback-controller tuning.

## Explicit domain limits

- At most 6 structural contacts by default.
- The exhaustive upper bound is `4^N`; it must not exceed 4096 by default.
- The energy-envelope pair count is also bounded.
- Exceeding a bound, finding no feasible mode, requesting undefined material
  friction, returning structural friction work on a prepared loading path, or
  failing the same-`p` work root produces an explicit `solver-domain-stop`.
  Candidates are never silently truncated.

## Run

```powershell
node work\kkt-coulomb-extension\run-tests.js
```

The generated `test-report.json` is the machine-readable report.

## Scope limitations

- This is a 2-D, one-tangent-direction impulse model; it is not a 3-D friction
  cone, rolling-friction, compliance, restitution, or position-correction
  model.
- Contact Jacobian rows are supplied by the caller. The prototype binds their
  shared public point but does not reconstruct or validate geometry itself.
- Candidate enumeration is exponential and deliberately stops at its declared
  cap.
- This is not wired to Rapier, continuous TOI replay, chip geometry, or the
  production material ledger. Those integrations require their own browser
  and timestep-convergence validation.

