# Freeze limits

- Work-only numerical geometry checkpoint; authority is `none`.
- `trigProviderCertified=false` and `productionNumericalAuthority=false`.
- `productionWiring=false`, `preparedRootAuthority=false`, and
  `s4dReconnected=false`.
- Zero physical q writes, geometry commits, material removals, or MAIN calls.
- The prepared-root integration path stops with
  `least-root interval proof is unresolved: an interval contains zero but cannot be refined to a bound endpoint sample`.
  This is preserved as negative evidence.  No search budget, tolerance, guard,
  or frozen v3 source was changed to make it pass.
- Frozen upstream and production hashes are asserted by `run-freeze-tests.js`.

