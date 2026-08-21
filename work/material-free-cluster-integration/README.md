# Material free-cluster integration

`free-cluster.js` is the executable boundary between one actual free world advance and one simultaneous structural/material Coulomb solve. It directly reuses the signed solver in `work/kkt-coulomb-extension/event-kkt-coulomb.js`.

## Ownership

One session performs this sequence:

1. Snapshot the root state and original solver groups.
2. Mask the owned contact pairs.
3. Ask the adapter for exactly one force integration/world step.
4. Freeze `qFree`, `Wact`, `h`, step/force epochs, and the same-state contact-frame signature in an `Sfree` token.
5. Restore solver groups.
6. Either finish the structural branch at material impulse zero, or run pure dry material trials and finish one accepted simultaneous state.

The two finish branches are mutually exclusive. An injected failure restores the pre-operation state and leaves the token retryable. `abort` restores the root snapshot.

## Contact and time formulas

Structural rows are derived only from the frozen contact frame:

```text
persistent: normalBias = gap / h
onset:      normalBias = gap / h + restitution * min(0, preNormalVelocity)
```

Restitution is therefore never replayed after persistent-contact ownership begins. The same `h` is carried through free-state, Moreau, work, and geometry bindings.

## Geometry boundary

`finishMaterial` writes only the accepted `qPost`. It does not mutate material geometry. It returns a frozen commit ticket containing the `Sfree`, contact-frame, accepted-state, `Wact`, `h`, row/mode, `qPost`, and geometry-payload bindings. A later root transaction must consume this ticket while staging geometry/body/chip publication atomically.

## Adapter API

The session expects an adapter with root snapshot/restore, solver-group read/mask/restore, one `advanceFree`, trace reads, and `writeQPost`. `adapter.advanceFree({h,event,maskedPairs})` returns:

```js
{ qFree, Wact, contactFrame, stepEpoch, forceEpoch }
```

An event-specific `freshArea` callback may be supplied as `event.freshArea`; the configuration callback remains a compatibility fallback for the current black-box fixture.

## Exports

```js
createFreeClusterSession(adapter, config)
solveClusterFromFree(input)
canonicalExposed(value)
faultStages
```

The session exports `advanceFree`, `trialMaterial`, `finishNoMaterial`, `finishMaterial`, and `abort`.

## Verification

```powershell
node work/material-free-cluster-qa/run-tests.js --module work/material-free-cluster-integration/free-cluster.js
```

The frozen run passes 14/14 reference-contract checks, catches 10/10 deliberate negative mutants, and passes 14/14 tests against this module. This module is not production-wired by itself; real Rapier contact-frame extraction, remainder replay, geometry/body/chip staging, and the main activation switch remain separate gates.
