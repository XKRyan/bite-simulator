# bite-simulator

Browser-based bite and rigid-body simulation, plus frozen work-only research
checkpoints for traceable material-contact modelling.

## Status

The runnable simulator is in `outputs/bite-simulator/`. The research artifacts
under `work/` are intentionally fail-closed: continuous material removal and
production wiring remain disabled until the remaining authority and prepared-root
proofs are completed. They should not be treated as production cutting logic.

The combined checkpoint includes the strict DXF import improvements developed in
the comparison branch: holes and disconnected geometry, automatic mass/COM/inertia,
atomic rollback, and the three callback-arity fixes that removed the fork-import
hang. The signed rigid baseline remains unchanged.

## Run the simulator

Serve the directory with any local static web server, then open the shown URL:

```powershell
python -m http.server 8000 --directory outputs/bite-simulator
```

Open `http://localhost:8000/` in a browser. Opening `index.html` directly may not
load all browser assets correctly.

## Repository layout

- `outputs/bite-simulator/` — runnable browser simulator.
- `work/s4c-private-toi-dxf-combined/` — frozen combined S4c + strict-DXF
  candidate, deterministic private bundle, reports, and source-union proof.
- `work/material-split-scale-mode-family-v2_1/` — frozen split-scale
  finite-rotation/mode-family mathematical checkpoint.
- `work/PAUSE-HANDOFF-2026-08-20.md` — current handoff, limitations, and next
  work items.
- Other small `work/` directories are pinned dependencies needed to inspect or
  rerun the current Node-side checkpoint tests.

The combined browser bundle is published as a frozen artifact and is verified by
its 30-entry `FREEZE-SHA256SUMS.txt`. Its historical build plan pinned an older
v3 prepared-root source, while the dependency directory here contains the newer
identity-hardened v3 used by the v2.1 tests. Do not regenerate the combined bundle
from the current dependency tree and treat it as the same frozen artifact.

## Integrity anchors

| Artifact | SHA-256 |
| --- | --- |
| Production `app.js` | `3A58BFB55ACA173DB63435F75C7FDF19BB0F6D68D42AEABF4E77FC1A7AB2D344` |
| Combined candidate | `06B6EC1F6E611DDD4122652DC9763D8F75129904D224ACCA3CABB1AF3D71D0AA` |
| Combined private bundle | `C100E7FBF724448643D3CF6123EEDECF201A923321838E7B27299CFCC809A2A9` |
| v2.1 freeze manifest | `CCE42ECC3AAB7EC1D0A8584E7699EF1910B724F72F05B78A39356707CE86C5DE` |
| Handoff document | `6C10405FA11865934C0C3B6048F4A30126A48D1A5C8EB77911A5ABBF0D28A34D` |

Detailed test evidence and scope limits are recorded in the two checkpoint
directories. In particular, v2.1 is a work-only mathematical result: trig/norm
authority, prepared-root consumption, material removal, and production wiring
remain disabled.
