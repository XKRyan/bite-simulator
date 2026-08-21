# Independent mutation review — fail-closed option boundary

This is a separately recorded adversarial review of the work-only port's
public matcher boundary. It is based on the executed mutation cases in
`test/dxf-strict.test.mjs`, not on a claim that a soft result is safe.

## Question

Can any JavaScript truthy/falsy coercion, invalid options shape, accessor, or
prototype field turn a failing contact mapping into `{ ok: false }` without an
explicit caller opt-out?

## Result: no covered bypass remains

The normalization path rejects non-number/non-plain-object inputs. For a plain
object it reads own property descriptors, rejects accessors and non-boolean
supplied `failClosed` values, ignores inherited fields, and computes the soft
path only from `own failClosed === false`. The common dangerous variants
`undefined`, `null`, `0`, `''`, `true`, and a missing field all throw on the
same unmatched-contact failure. A primitive numeric `0` remains a valid zero
tolerance, but it still throws on failure; it cannot select the soft path.

The lower-level `contactFailure` repeats the exact-false test, so a future
caller that bypasses normalization cannot weaken the default by passing a
falsey non-boolean value.

## Boundaries retained

- An explicit own boolean `false` is intentionally supported for a caller that
  immediately handles an inspectable `{ ok: false, code }`; it is the only
  soft mode.
- This report covers the pure module. Production remains unwired and unchanged.
- The executed test report is 13/13 pass; source and test hashes are recorded
  in `AUDIT.md`.
