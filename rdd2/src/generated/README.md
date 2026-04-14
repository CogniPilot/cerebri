Generated control artifacts live in this directory.

Rules:
- Do not edit these files by hand.
- Handwritten flight-stack code should wrap them from normal `src/` modules.
- If behavior needs to change, prefer regenerating or replacing these artifacts instead of patching them in place.

Current status:
- `rdd2.*` is an imported generated source artifact.
- `PIDAxis.*` is retained only as a historical reference and is no longer wired into the flight build.
- This directory is transitional and is expected to be replaced by a local Rumoca-based generation flow.
