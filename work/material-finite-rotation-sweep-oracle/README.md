# Finite-rotation sweep oracle (work-only)

This directory contains a pure mathematical prototype for a finite-rotation
material sweep. It does not authenticate a Rapier state, a contact owner, a
prepared KKT root, or a remaining-material object. It is not wired to the app
and cannot enable removal.

## Defined numerical model

For a fixed impulse parameter `p`, feature and material translational and
angular velocities are affine within one already-frozen active-mode cell. Both
bodies drift from one common start state over `h`. At every declared time node,
the working segment is transformed into target-local coordinates. An adjacent
pair of endpoint segments forms one convex polygonal cell; the discrete sweep
is the union of those cells.

At every exact sample the ordered cell is required to be strictly convex (or
exactly stationary). A folded, moving-collinear, or numerically unresolved cell
stops the domain and must be time-refined. This makes the convex cell equal to
the swept set of the two linearly interpolated endpoint chords.

The module also records a second-derivative enclosure for the distance between
the true constant-twist endpoint curve and its time chord. The fixed maximum is
5 micrometres. This is only a local spatial-error certificate; production must
still prove the private path/source binding and `dt`, `dt/2`, `dt/4`
convergence.

## Area and interval certificate

For each sample, the pinned polygon kernel computes:

`fresh = G intersection sweep`

`remaining = G difference sweep`

The result must satisfy the independent fresh/difference area identity within a
dimensionally correct boundary-tube plus roundoff enclosure. Thin or unstable
topology stops safely.

For `p` intervals, endpoint derivatives bound corresponding cell Hausdorff
motion. Each convex cell uses a Steiner symmetric-difference bound, and the
union uses the sum of component bounds. A global planar-arrangement edge budget
adds a boolean-kernel enclosure valid between, not just at, lo/mid/hi. The
published interval explicitly contains the lo/mid/hi samples and their numeric
guards.

## Verification

Run:

```text
node work/material-finite-rotation-sweep-oracle/run-tests.js
node work/material-finite-rotation-sweep-oracle/run-random-audit.js
```

The deterministic suite covers analytic translation, finite and counter
rotation, holes and disconnected islands, empty geometry, dense interval
containment, time-chord enclosure, refinement, scaling, malformed inputs, and
folded cells. The fixed-seed randomized audit covers 160 independent motion and
geometry configurations and 30,400 interior interval samples.

## Deliberate blockers

- `authority` is explicitly `none`; digests are integrity only.
- The current conservative boolean-area uncertainty can prevent the signed v3
  solver from proving a least constitutive root at its much tighter work
  tolerance. That result is a correct domain stop, not permission to widen the
  root tolerance silently.
- No non-working-edge ordering, real private 78F ownership, mode-cell source,
  geometry lifecycle, mass update, chip ledger, remainder replay, or commit is
  provided here.
- The convex sweep is the declared numerical path, never a claim about an
  opaque within-step Rapier trajectory.

