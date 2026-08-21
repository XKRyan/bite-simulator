# Next stage after the v2.1 split-scale checkpoint

This is a planning note only.  It does not change the frozen v2.1 sources,
manifest, reports, production, S4c/S4d, or any authority flag.

## Frozen starting point

- 9F-v2.1: `6C55302E10BC1EECB0333D8549A3A55441A5047869CAE83E920B0F6A00C58449`
- EDD-v2.1: `41B1CB64449BD7585675F275856EBC7E9FC081F354AE9AC2F411283F755B40C6`
- 59-v2.1: `B6242837F332E0A86131E64DC180CFF14D7E8C6AE3374336CDF8277804F04DF1`
- freeze manifest: `CCE42ECC3AAB7EC1D0A8584E7699EF1910B724F72F05B78A39356707CE86C5DE`
- production remains `3A58BFB55ACA173DB63435F75C7FDF19BB0F6D68D42AEABF4E77FC1A7AB2D344`
- accepted combined S4c raw remains
  `06B6EC1F6E611DDD4122652DC9763D8F75129904D224ACCA3CABB1AF3D71D0AA`

All subsequent implementation must use a new directory and must not edit these
frozen bytes.

## Stage 1: certify the trig/norm provider

The current fixed-operation `gamma_n` bound covers basic binary64 arithmetic,
but ECMAScript does not promise a platform-independent ULP bound for
`Math.sin`, `Math.cos`, or `Math.hypot`.  A later authority-bearing chain must
therefore provide one of the following, with fixed source bytes and independent
tests:

1. an interval trig/norm implementation whose returned intervals are proved to
   contain the exact result, including argument reduction; or
2. a pinned runtime/provider with an explicit, independently verified ULP bound
   over the complete admitted angle and coordinate domain.

The provider certificate must bind its source hash, argument domain, range
reduction, subnormal/overflow behavior, and error constant into the common
numerical-policy signature.  Large-angle inputs outside the certified reduction
domain must stop.  Until this gate passes,
`trigProviderCertified`, `productionNumericalAuthority`, and production wiring
must remain false.

Required negatives include large start angles plus tiny increments, quadrant
boundaries, near-zero norms, subnormals, overflow, cross-provider/cross-instance
objects, cloned certificates, and a provider hash mismatch.

## Stage 2: handle the prepared-root nonzero-radius floor

The inherited v3 consumer currently stops honestly with:

`least-root interval proof is unresolved: an interval contains zero but cannot be refined to a bound endpoint sample`

This occurs because a sound path interval retains a nonzero
`2 * transformLengthGuard` radius even as the p interval narrows.  The next
consumer design must preserve that floor.  It must not:

- increase `maximumCertifiedIntervals` merely to hide the same terminal state;
- relax the transform, polygon, work, or residual guards;
- substitute endpoint samples for the whole-interval certificate;
- rewrite the exact sample or signed work value to force a crossing; or
- publish an ambiguous interval as a prepared root.

A successful design needs a proof rule that either isolates the leftmost root
despite the persistent numerical enclosure, or returns a stable explicit
domain stop.  Every accepted prefix leaf must still exclude a root using finite
recomputed lower and upper work bounds.  The accepted root bracket must remain
left-first, ordered, identity-bound to the original v3 prepared object, and
robust against floating-p steps, V-notches, nonmonotone stopping endpoints,
clones, resigned fakes, cross-module objects, and stale sessions.

The v2.1 expected stop is a regression fixture.  It may change to PASS only
when a new mathematical certificate proves the stronger result; search-budget
or tolerance changes alone are not acceptable evidence.

## Stage 3: reconnect only through the same event-slice private owner

After Stages 1 and 2 are independently frozen, a new private composition may
reconnect them to S4d.  It must use the already selected numerical event slice,
not the coarse whole-step endpoint:

1. restore every trial from the same parent root;
2. replay exactly one final `h_event` world step from that parent;
3. re-run private 78F and require the same tip/working-face contact at the right
   endpoint with the same common point, normal, row, feature owner, material
   version, and triangle cover;
4. capture the event-slice post-world/pre-KKT `qFree`, `Minv`, structural rows,
   geometry, h/time partition, and remaining cover from that exact replay; and
5. bind the exact AE46 returned token object, the S4c owner record, and the
   original v3 prepared object as three private WeakMap identities in the same
   module composition.

No caller-provided working-face string, reconstructed token, cloned prepared
root, cross-module capability, cross-session record, or stale material version
may pass.  The incompatible AE46 `geometrySnapshotSignature` path must continue
to stop; no synthetic signature bridge is allowed.

Every geometry callback and dry trial must check the complete root, counters,
aliases, groups, damage, geometry versions, and capability identities before
and after the call.  Any exception or mutation requires exact rollback and an
explicit `completeExact` check.  The final checkpoint remains dry:

- zero physical q writes;
- zero geometry commits;
- zero removals;
- MAIN false; and
- no AE46 finish/commit call.

## Required exit order

1. Freeze and independently audit the trig/norm provider.
2. Freeze and independently audit the nonzero-radius prepared-root consumer,
   including the preserved domain-stop path.
3. Rebase a new S4d private composition on the exact frozen hashes.
4. Run the fixed real three-mode event-slice fixture plus clone, cross-module,
   stale, mutation, rollback-failure, and owner/version negatives.
5. Freeze only if all successful paths preserve zero-write/zero-commit scope;
   otherwise publish the exact domain-stop evidence and leave authority false.

