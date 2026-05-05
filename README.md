# Grbly

Small GRBL sender and visualizer for Linux desktop use.

```
cargo run
```

## Current Shape

- Serial connection to GRBL controllers.
- Live jog, feed/spindle overrides, homing/unlock/zero actions, soft reset, and job streaming.
- G-code preview with job bounds and soft-limit line highlighting.
- Separate simulation mode that does not mutate the live job status.
- Compact tabs for Run, Jog, Setup, and Probe controls.
- Probe-based Z autoleveling: builds a heightmap over the loaded job's bbox and applies it to every motion line at stream time.

## Probe + Heightmap

For PCB milling and similar work where the workpiece isn't perfectly flat. Wire one alligator clip to the bit (or spindle body), one to the copper, into GRBL's probe input pin (`A5` on UNO, varies by board). The PROBE tab shows a live `PROBE PIN: CONNECTED / OPEN` indicator — touch bit to copper to verify wiring before running anything.

Workflow:

1. Jog the bit to the bottom-left corner of the board, gently touching the surface.
2. `ZERO XYZ` (Setup tab) — sets WPos to `(0, 0, 0)` at that point.
3. Load gcode. The 3D preview shows amber crosses at every probe candidate point inside the gcode bbox.
4. PROBE tab → pick grid `N×M` (default 5×5), pick mode:
   - **AUTO**: drives `G38.3` at each point. The active point is cyan, finished ones turn green at the probed Z (so the warp is visible in 3D).
   - **MANUAL**: rapids to each point, exposes Z± jog buttons + `DONE` for hand probing on non-conductive substrate.
5. `START PROBE`. Result is saved to `~/.cache/grbly/heightmap.txt` and auto-loaded next session.

When a heightmap is attached, every `start_job` and `step_line` runs the gcode through a modal-aware transform that subdivides each motion in 1mm chunks and adds the bilinear `dz(x, y)` to every emitted Z. Single-codepath; no separate "apply" toggle.

There's also a single-point `PROBE HERE` button for spot-checks and a `PROBE → ZERO Z` button that probes and zeros work-Z exactly at the probed surface.

### Probe safety

- `G38.3` (no alarm on miss). On miss the engine returns the error cleanly instead of dropping the machine into ALARM.
- `max_depth` (default 0.3mm) caps how far below `Z=0` the probe is allowed to plunge — protects the bit if the alligator clip falls off.
- Every exit path retracts to `safe_z` before returning.

## Safety Notes

- Live job start and spindle start require a second click.
- Jog and setup controls are disabled when the machine is disconnected or in an incompatible GRBL state.
- Soft-limit checks use GRBL travel settings `$130`, `$131`, `$132`, WCO, and `$20`.
- This is still a hobby controller; verify motion on your machine before trusting unattended cuts.

## Known Limits

- G-code preview supports common linear moves, XY arcs with `I/J` or `R`, absolute/incremental coordinates, and inch/mm scaling.
- More advanced modal behavior, non-XY planes, and cutter compensation are not complete.
- The 3D toolpath shows the un-warped path; the heightmap correction is applied at stream time, not in the visual preview.

![a](screenshots/a.png)
![b](screenshots/b.png)
