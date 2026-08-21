# Split-scale finite-rotation / mode-family v2.1 checkpoint

## Frozen scope

This directory is a work-only mathematical checkpoint.  It does not modify or
wire production, S4c, S4d, Rapier state, prepared-root publication, material
removal, or MAIN.  The production app remains `3A58...D344` and the accepted
combined raw S4c base remains `06B6...D0AA`.

The v2.1 chain contains three new-source copies only:

1. `finite-rotation-sweep-oracle-v2_1.js`
2. `mode-family-geometry-v2_1.js`
3. `exact-mode-family-geometry-v2_1.js`

The exact triangle-cover source remains the frozen `BC128...A77B` source.

## Numerical change

The old single conservative coordinate scale is split into:

- `finalCoordinateScaleUpperBound`: remaining-geometry coordinates and every
  fixed-time-node working-segment endpoint over the full p interval.  Each
  endpoint-pair component maximum is inflated by an outward Lipschitz bound
  times half the p width, so an interior rotational component maximum is not
  missed.
- `transformOperandScaleUpperBound`: a cancellation-aware upper bound on the
  world-space and affine intermediate operands.  It contributes only through a
  fixed 96-operation `gamma_n` transform guard.
- `computedFinalCoordinateScaleUpperBound`: the final/discrete bound plus twice
  the transform guard.  Polygon length and area roundoff use this value; the raw
  transform operand scale is never squared into an area guard.

The operational length floor is

`max(requested, transformLengthGuard + polygonLengthGuard)`.

Interval path radii use the outward sum

`derivativeBound * halfWidth + 2 * transformLengthGuard`.

Positive multiplication underflow rounds upward to at least
`Number.MIN_VALUE`; gamma overflow or `n * epsilon >= 1` stops the domain.

EDD-v2.1 measures both native bounds for every complete 8B envelope mode,
takes two independent maxima, and rebuilds every mode with both common values.
Caller-supplied common or per-mode bounds are forbidden.  The family binds one
canonical-geometry signature, one common-policy signature, and separate mode
path signatures.

## Real three-mode result

The fixed real S4d numerical fixture passes the v2.1 geometry chain:

- common final/discrete bound: `0.07017917700505896`
- common computed-final bound: `0.07017917700507394`
- common transform-operand bound: `0.35082008975424306`
- polygon length guard: `4.228577815034578e-13`
- transform length guard: `7.478179990007675e-15`
- effective summed length floor: `4.3033596149346554e-13`
- all three modes share canonical/common/composite signatures (`1/1/1`)
- the three paths remain distinct (`3` path signatures)

The authoritative values and each mode's relative-area margin are in
`real-s4d-v2_1-direct-report.json`.

## Frozen tests

- focused split-scale, negative, dense-p, cancellation, floating-step and
  determinism tests: `10/10 PASS`
- v2.1 fixed-seed quick exact random subset: `2 configurations`, `4 intervals`,
  `20/20 exact samples`, `0 safe stops`
- real S4d three-mode direct fixture: `3/3 PASS`
- unchanged frozen regressions: 9F `13/13`, EDD `9/9`, BC `13/13`, exact union
  random `256/256`, 59E `10/10`, v3 `8/8` plus its frozen `12/12` mutants

`run-freeze-tests.js` records these results without writing any upstream frozen
directory.

## Hard limits

- `Math.sin`, `Math.cos`, and `Math.hypot` have no platform-independent ULP
  guarantee in ECMAScript.  Therefore `trigProviderCertified=false`, numerical
  production authority is false, and this checkpoint remains pinned-runtime,
  work-only evidence.
- The inherited v3 prepared-root consumption test honestly domain-stops under
  the nonzero twice-transform-roundoff interval-radius floor.  Its interval
  budget was not enlarged and no guard was relaxed.  No prepared-root authority
  is claimed.
- No S4d bundle or private composition was rebuilt.  Reconnection is explicitly
  deferred until a separately reviewed consumer can handle the nonzero numeric
  interval floor.
