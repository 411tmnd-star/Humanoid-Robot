# Phase Spec: Warehouse Environment — Phase 1 (Shell, Debris, Discovery Lighting)

## Scope

Environment/set-dressing work for the portfolio film, built entirely inside
`blender/robot_armature_v12.blend` — the same file as the rig, not a
separate asset. `RobotArmature` stands at world origin (confirmed live:
`matrix_world.translation == (0,0,0)`, `DEF_Head` at world height ~1.905m).
Everything below lives in two collections, isolated from every rig object
(`RobotArmature`, `DEF_*`/`MECH_*`/`CTRL_*` bones, `411_*` meshes) so
environment work can never collide with rig work:

- `ENV_Warehouse` — the building shell
- `ENV_Debris` — set dressing + camera

This doc covers docs/ENGINEERING_RULES.md-adjacent territory (a new domain, not rig
mechanics) — none of docs/ENGINEERING_RULES.md's DEF/MECH/CTRL/joint-classification rules
apply here; this doc follows the same phase-doc *format* as
`docs/phase_*.md` for consistency, not its rig-specific content rules.

Scripts: `scripts/build/warehouse_env_v1.py`, `scripts/build/warehouse_debris_v1.py`.
Both are idempotent (`run()` refuses to duplicate if their collection
already exists; `teardown()` removes it cleanly) and self-contained — no
external asset dependency required to run.

## ENV_Warehouse (27 objects)

- **Dimensions**: 24m (X) × 40m (Y) shell, 9m eaves height.
- **Floor + 4 walls**: concrete. The west wall is split into two segments
  plus a lintel, leaving an 8m × 6m broken loading-dock opening centered on
  that wall — the story's one light source into an otherwise sealed space.
- **Roof + 12 steel trusses** at 4m spacing, corrugated-metal material.
- **Lighting**: 6 warm sodium point lights (800W, color `(1.0, 0.65, 0.25)`)
  along the ceiling spine, plus one cold spot light (`ENV_DockShaft`,
  color `(0.75, 0.82, 1.0)`) standing outside the dock opening — the two
  color temperatures are deliberately contrasting (dying warm fixtures vs.
  the one true daylight source).
- **Atmosphere**: EEVEE volumetric fog via a bounded `ENV_FogVolume` cube
  (Volume Scatter shader, not the world's global volume, so density stays
  localized to the interior).
- **Materials — real procedural PBR node graphs, not flat colors**:
  PolyHaven/Sketchfab/Hyper3D are all disabled in this session's Blender-side
  MCP panel (checkbox + reconnect required — not something this session can
  flip remotely), so:
  - `build_procedural_concrete()`: layered noise drives damp/dry base-color
    blotching plus a separate large-scale noise for darker stain streaks,
    with a fine noise bump map for surface roughness.
  - `build_procedural_rusted_metal()`: a Wave texture gives real geometric
    corrugation via Bump; a Voronoi×Noise mask blends rust-orange into base
    steel and locally kills metallic/roughness so rust reads as matte oxide,
    not shiny steel.
  - `run()` accepts `concrete_material_name`/`metal_material_name` overrides
    for real downloaded textures later, once PolyHaven is enabled.

## ENV_Debris (30 objects)

- **8 "decaying robot" husks** — mesh-data duplicates of the hero robot's
  own parts (`411_Torso`, `411_UpperArm_R/L`, `411_Wrist_R/L`,
  `411_WaistRing_New`, `411_ThumbJoint_R`, `411_Visor_Patch_backup`),
  rusted-metal material override, scattered and tumbled outside a 4.5m clear
  "stage" radius around the origin. Narrative reasoning stated up front in
  the script: this makes the warehouse a graveyard of the robot's own kind
  rather than generic scrap — directly supporting a self-discovery beat.
  Deterministic scatter (`random.Random(411)` seed) — same layout every
  rebuild.
- **16 crates/barrels + 5 cable coils** — generic industrial debris,
  procedural wood/steel/rubber materials.
- **One camera** (`ENV_EstablishingCam`), currently framed for the discovery
  beat (see Lighting Composition below), lens 35mm.

## Bugs found and fixed during this build (with live evidence, not just claimed)

1. **Floating husks** (first render). Duplicated mesh data kept the source
   object's original on-body vertex coordinates; repositioning the new
   object's *origin* to floor level didn't move the *geometry*, which stayed
   floating at its original body height (a torso/visor chunk hovering at
   head height against a wall). Fixed with `_recenter_mesh_to_floor()`: a
   bmesh pass that shifts every vertex down by the mesh's own local-space
   minimum Z before the object is placed.
2. **Tumble lift** (second render, same husks). The Z-recenter alone only
   guarantees the lowest point sits at Z=0 *before* rotation; a tumbled
   husk's random X/Y tilt then rotates around that point and lifts the rest
   of the panel off the floor. Fixed with `_settle_on_floor()`: transforms
   every vertex by the actual rotation+scale that will be applied, finds the
   resulting minimum Z, and back-solves the object's location.z so that
   minimum lands exactly on the target floor height.
3. **Operator context failure** (`bpy.ops.mesh.primitive_cube_add` +
   `transform_apply` → `"context is incorrect"`). These operators require an
   active 3D viewport in Blender's window/screen context, which an
   MCP-driven, non-interactive script can't guarantee call-to-call. Fixed by
   rewriting every primitive creator (cube, cylinder, torus) as direct
   `bmesh` construction — `bmesh.ops.create_cube`, `bmesh.ops.create_cone`
   for cylinders, and a hand-built parametric torus for cable coils (bmesh
   has no built-in torus op). None of this depends on viewport context.
4. **Camera embedded in solid wall geometry** → fully black render. A
   reframe attempt placed the camera at `x=12.0`, which is *exactly*
   `WIDTH/2` — the east wall's center plane — so the camera sat inside solid
   concrete and rendered the mesh's interior surface at point-blank range.
   Fixed by moving to `x=9.0`, 3m of real clearance from that wall.
5. **Fog/light tuned for the wrong camera distance** (not a code bug, a
   parameter mismatch). Fog density (0.05) and dock-shaft energy (18000W)
   read fine from the original ~14–17m establishing-camera distance but
   completely washed out the frame into flat haze once the camera moved to
   ~10m for the discovery close-up — the volumetric scattering from that
   much light through that much fog overwhelmed the shot. Retuned to
   density 0.012 / energy 7000W for the closer framing.

## Lighting Composition (resolved this pass)

**Original gap**: the dock-shaft spot was aimed at the floor just past the
opening (`x = -WIDTH/2 + 2, y=0, z=0`), not at the robot — so the beam
landed to the robot's left instead of on it.

**Fix**: `ROBOT_AIM_POINT = (0, 0, 1.3)` (chest/head height at the robot's
actual world position, confirmed live) is now the aim target for both the
dock-shaft spot and the camera. The spot's cone was also narrowed
(40°→18°) so it reads as a directed beam on the subject rather than a wide
flood, with a geometric check in `build_lighting()` confirming the beam
axis still clears the 8m×6m opening at the correct height (verified: beam
crosses the opening plane at 5.39m, within the 0–6m opening) rather than
being clipped by the lintel.

**Camera**: repositioned twice this pass (once into the wall by mistake,
corrected) to `(9.0, -5.0, 2.2)` aimed at `(0, 0, 1.5)`, 35mm lens. Final
render (`blender/warehouse_preview_v6.png`) confirms the robot standing
clearly inside a visible, focused light shaft with the fog reading as thin
atmosphere rather than blanket haze.

## Status

**VALIDATED against a real render**, not just described: `warehouse_preview_v6.png`
is the current reference frame. Iterated v1→v6 with each fix confirmed by
re-rendering and visually comparing, not assumed fixed from reading the code.

## Known gaps, stated rather than hidden

- Dust-mote particle system intentionally not added — fog + light shaft
  carry depth for now; flag if a future pass wants particles layered in.
- PolyHaven/Sketchfab/Hyper3D remain disabled — real scanned textures and
  Sketchfab industrial-debris props are a follow-up once those are enabled
  in the Blender-side MCP panel + reconnected.
- The wide establishing-shot framing (`warehouse_preview_v3.png`, showing
  the full shell + trusses) was superseded by the closer discovery framing
  for this pass — both are valid shots for different beats in the film, not
  a strict replacement. Re-run `warehouse_debris_v1.py`'s camera placement
  logic with different coordinates if the wide shot is needed again for the
  final cut.
