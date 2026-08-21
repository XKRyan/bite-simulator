# Material free-cluster contract, version 1

This contract is the activation gate for the free-state/contact-cluster boundary. It is intentionally stricter than a numerical end-state comparison: it constrains ownership, state provenance, exception recovery, and the exact data accepted by the material branch.

## Locked basis

The runner refuses source drift from these reviewed inputs:

- S2 candidate: `6E8E638407ED1B4F1A3A9F3C97A1E5C191AB6AE8D843CCB1622679E48312C320`.
- Coulomb KKT: `8B3058E5833E96D7C83971B6D0C234E6641D0CB98044012DFFFA5DA2268E1A3E`.
- Normal-only KKT: `866BE26C0A0DF18D114B1026EEC08C943E6D9F672265F7DB0B49C10C28836719`.

The normal-only and zero-friction Coulomb solvers must agree on their common domain. This guards the signed-solver handoff in addition to checking file hashes.

## Required module surface

The module under test exports:

```js
createFreeClusterSession(adapter, config)
solveClusterFromFree(input)
canonicalExposed(value)
faultStages // or FAULT_STAGES
```

The session exports:

```js
advanceFree({ h, event })
finishNoMaterial(token)
trialMaterial(token, { lambda, dry: true })
finishMaterial(token, { acceptedTrial })
abort(token)
```

Passing `materialContact`, `structuralContacts`, rows, modes, or points to either finish method or to `trialMaterial` is forbidden. Those values may only come from the contact frame captured at the same free state.

## Adapter boundary

The adapter supplies:

```js
snapshot()
restore(snapshot)
advanceFree({ h, event, maskedPairs })
readExposedState()
readTrace()
readSolverGroups()
restoreSolverGroups(groups) // writeSolverGroups(groups) is an accepted fallback
writeQPost(qPost)
```

`adapter.advanceFree` owns resetting/applying the interval's external forces and actuation, and exactly one world step. It returns:

```js
{
  qFree,
  Wact,
  contactFrame,
  stepEpoch,
  forceEpoch
}
```

`contactFrame` contains the inverse mass operator, material row/point, structural row/point/gap/friction records, onset/persistent classification, restitution source, cutting-energy source, and width needed by the signed solve. It is captured from the same `Sfree`; it is not reconstructed after a dry solve.

## Normative invariants

### 1. Exactly one free advance

For a successful call, `stepEpoch` and `forceEpoch` each increase by exactly one. The adapter performs exactly one world step, one external-force integration, and one actuation-work integration. Trials and either finish path perform none of these operations.

`qFree`, `Wact`, `h`, `stepEpoch`, `forceEpoch`, the solver-group mask epoch, and the contact-frame signature are bound into the returned token. `Wact` is observably immutable for the token's lifetime and is never recomputed from `qPost`.

### 2. One token, one finish branch

One `Sfree` token is consumed by exactly one of:

- `finishNoMaterial`, which performs the structural solve at material impulse zero; or
- `finishMaterial`, which writes the accepted simultaneous material/structural `qPost`.

After either succeeds, the other must refuse before any mutation. A failed dry trial, domain stop, rejected binding, or injected exception does not consume the token.

### 3. Dry trial purity

`trialMaterial(..., {dry:true})` is a total read-only operation over the adapter. Before and after it, the complete exposed state, trace, caches, counters, clock, solver groups, and geometry are byte-identical under `canonicalExposed`. Repeating the same trial produces an identical result.

### 4. Frozen contact-frame provenance

All points, normal/tangent rows, gaps, friction coefficients, contact phases, and restitution data come from the token's signed `contactFrame`. Caller-supplied replacements are refused. This prevents a root at one manifold from being committed at another.

### 5. Accepted same-parameter binding

`finishMaterial` independently validates the accepted dry trial and preserves, exactly:

- `p === lambda`;
- `h` and `qFree`;
- material and structural point/row bindings;
- selected contact states and `modeKey`;
- `qPost`;
- exact work segments and dissipated work;
- geometry payload; and
- the accepted binding signature.

The result and commit ticket also carry the token's original `Wact`; neither may recompute it from the accepted post-contact state.

Changing any accepted `p`, row, point, mode, `qPost`, work segment, geometry payload, or signature is a pre-mutation refusal. The successful result supplies a commit ticket carrying the same bindings. This stage does **not** publish or rebuild geometry; the later atomic material transaction consumes the ticket.

### 6. One interval `h`

One positive finite `h` is bound to the token and used by every calculation:

```text
persistent: b_n = max(gap, 0) / h
onset:      b_n = max(gap, 0) / h + e * min(0, v_n^-)
constraint: J_n q_post + b_n >= 0
```

Only positive gap supplies an endpoint closure budget. Existing negative gap is not projected away through an energy-injecting separating velocity; it contributes zero Moreau bias and remains penetration/domain telemetry. The same `h` is reported by the free advance, Moreau row construction, work integration, and geometry payload. There is no hidden substep-specific `h` in one accepted event.

The inverse mass operator is frozen inside the same signed `contactFrame`. A trial may not fall back to later live body/adapter mass state. Missing `contactFrame.Minv` is a pre-solve refusal; changing live mass after `advanceFree` cannot change trials made from that token.

Restitution is applied only to an onset contact. Once the persistent-contact owner takes over, restitution is zero and only the Moreau gap term remains. Thus a persistent closing speed is stopped, not launched again.

### 7. Solver-group restoration

The original solver groups are restored exactly on success and on every declared exception edge: before/after masking, before/after the world step, free capture, dry solve, accepted validation, velocity writeback, ticket creation, and token consumption. An advance failure restores the root state. A later operation failure restores its pre-operation `Sfree` state and leaves the token retryable.

### 8. Explicit finite candidate domain

For `N` structural Coulomb contacts, the theoretical mode count is `4^N`. The upper bound is checked before enumeration. If the contact count or candidate count exceeds the configured limit, the only valid outcome is a pure:

```js
{ ok: false, status: 'solver-domain-stop', reason: '...candidate upper bound...' }
```

Contacts or modes may not be dropped, truncated, greedily selected, or solved sequentially to avoid the stop.

### 9. Abort and recovery

`abort(token)` restores the pre-advance root state exactly and consumes the token. It is valid after a dry domain stop. A rejected mixed binding and an injected finish fault preserve `Sfree` so the caller can retry or abort deliberately.

## Required fault stages

The exact stage list is exported by the reference module and checked by the runner. It covers partial mask writes, partial group restoration, advance/world-step, dry solve, material validation/writeback/ticket, and structural solve/writeback/consumption. Missing a stage is a conformance failure; this prevents an untested catch/finally edge from being added silently.

## Scope limit

Passing this pure-JavaScript contract does not prove that Rapier extracts the correct production contact frame, that a browser world actually advances once, or that the main switch is enabled safely. It proves the transaction boundary and catches state-machine violations before the frozen integration module is exercised. The integration module must pass the same black-box suite under its own SHA before production wiring can be signed.
