# Official fail-closed remediation report

Scope: work-only `matchGeometricContactsToSolver` option normalization. No
production, S2/S3 candidate, friend copy, or Downloads file was edited.

## Contract now enforced

The fourth argument is accepted only as either:

1. a primitive number, interpreted as `tolerance` and always fail-closed; or
2. a plain object whose prototype is `Object.prototype` or `null`.

Within a plain object, only an **own data property** exactly equal to boolean
`false` enables a soft `{ ok: false }` return. Everything else is fail-closed:

| caller input | failure behaviour |
| --- | --- |
| omitted / `{}` / `{ failClosed: undefined }` | throws `GeometryContractError` |
| `{ failClosed: true }` | throws `GeometryContractError` |
| `{ failClosed: null }`, `0`, or `''` | rejects invalid options and throws |
| top-level `null`, boolean, string, array, Date | rejects invalid options and throws |
| numeric tolerance (including `0`) | hard gate; failures throw |
| `{ failClosed: false }` | only permitted soft return |

Accessors are rejected without invoking them. Inherited fields are ignored, so
a prototype cannot disable the hard gate. `contactFailure` itself now throws
unless its internal flag is exactly `false`, preventing a secondary truthy /
falsy bypass.

## Evidence

- Static safety scan: pass (no process, network, dynamic-code, filesystem, or
  timer patterns in source/test ESM files).
- Syntax checks: pass.
- `node --test test/dxf-strict.test.mjs`: **13 pass, 0 fail**, 118.0831 ms on
  Node v24.15.0.
- The P0 mutation test covers missing, undefined, null, zero, empty string,
  true, invalid top-level values, an accessor, and literal false.

Frozen implementation SHA-256:

`BE5DF7D7A2C4B5CF735073E4A653C55FD928F5E112F8F162B2E25F22327C1F1B`
