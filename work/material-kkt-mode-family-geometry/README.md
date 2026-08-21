# Coulomb mode-family geometry composition (work-only)

This prototype joins the pinned 8B Coulomb solver's exact
maximum-dissipation envelope to the finite-rotation polygonal sweep kernel.
It is a mathematical composition only: real Sfree, TOI/working-owner,
remaining-geometry and prepared-session authority are absent.

The signed six-DOF order is fixed as:

```text
[rail/hinge pivot vx, weapon omega, fork omega,
 target COM vx, target COM vy, target omega]
```

The module builds one private Coulomb problem, enumerates its complete ordered
maximum-dissipation envelope and retains exact mode object identities. Each
mode's `qIntercept/qSlope` is mapped to a separate finite-rotation geometry
kernel. An exact sample must match the independent endpoint trial's `p`, mode,
`qPost`, active IDs, contact states, impulses, common points and rows exactly.

An area interval is split at every envelope transition. Each subinterval is
certified by its own affine mode kernel; the outer range combines the minimum
and maximum and explicitly evaluates every transition under the solver's
deterministic tie-breaking rule. Ordered coverage, inner certificate digests,
mode identity and upstream module pins are retained in the proof.

Run:

```text
node work/material-kkt-mode-family-geometry/run-tests.js
```

The tests embed the existing two-contact fixture into the real six-DOF order.
They cover four envelope cells (`inactive`, one-contact sliding, two-contact
sliding, stick/slide), exact transition states, cross-mode dense containment,
forged trial rejection and a rotating folded-cell domain stop.

Current blockers remain deliberate:

- The code uses a pinned work-only `_reference.buildProblem`; production must
  keep it inside the same private bundle/session rather than exporting it.
- The finite-rotation provider's conservative polygon numerical uncertainty
  can still prevent the v3 least-root proof at tighter constitutive tolerances.
- No authority, material commit, mass/collider rebuild, chip ledger, remainder
  replay or production switch is supplied.

