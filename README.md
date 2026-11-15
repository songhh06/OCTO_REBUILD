# OCTO_REBUILD — octopus simulator (main)

C++ shared-world XPBD physics with a Godot 4.6.2 render client. This
repository is independent of the preserved historical `../OCTO`
project. The untracked original GameReady model remains a local asset
at `game/assets/Octopus_GameReady_v5.glb`.

## Windows launch

```powershell
.\tools\build.ps1
.\PLAY_DEMO.cmd
```

Godot must be installed at `C:\Tools\Godot\godot.exe`.
Click the Godot window to focus it. Tab switches between octopuses;
left mouse drag applies bounded force, right mouse drag orbits the
camera, and F5 resets.

## Game controls v1.0

| Torso-relative arm | Extend (hold) | Stroke backward (hold) |
|---|---|---|
| Left front | F | D |
| Right front | J | K |
| Left side-front | S | A |
| Right side-front | L | ; |
| Left side-rear | R | E |
| Right side-rear | U | I |
| Left rear | W | Q |
| Right rear | O | P |

Let go of extend to retract/curl, let go of stroke to return
toward the front. Both inputs are independent, so F+D together
extends and paddles the same arm.

**Space** immediately releases every suction contact. While held,
automatic gripping stays disabled. Upon release a short re-grip
cooldown applies. Retracting a floor-attached arm peels contacts;
a gripped movable object remains held during retraction unless
Space is used or the attachment breaks. Release does not reset
the object's velocity.

See [the controls specification and simulation caveats](docs/design/CONTROL_V1.md).

The main branch does **not** include the abandoned expensive jelly
shader/membrane experiment. Original skeleton skin is retained.
The automatic suction/contact implementation uses coarse discrete rod
sites, not full anatomical sucker surface mechanics.

## Tests

```powershell
.\tools\build.ps1
ctest --test-dir build/release-final -C Release --output-on-failure
```

GitHub Actions also tests Linux, Windows, and a native Godot
synthetic-key input path. The original GameReady model is not stored
in Git, so final visual/gameplay acceptance and FPS with the original
asset require a local Windows demo run.


## C++ bridge version mismatch (`bad_operation`)

Controls v1.0 sends `step2`; an `octo_server.exe` compiled before this
protocol existed will reply `bad_operation` even if `reset` succeeds.
`PLAY_DEMO.cmd` / `tools/run-demo.ps1` now detects outdated native
binaries, rebuilds them when necessary, and probes a disposable native
server using a real `step2` request before launching Godot.

For a manual rebuild or when launching Godot independently:

```powershell
.\tools\build.ps1
.\tools\check-bridge.ps1 -Server .\build\release-final\octo_server.exe
.\PLAY_DEMO.cmd
```

Close any older running Godot/server process first, since Windows may
prevent replacing an executable while it is running. The expected
probe output begins with `BRIDGE_V1_PREFLIGHT_PASS`. If validation
fails even after recompilation, check `OCTO_SERVER` is pointing to
the repository's newly built executable.

## Stronger octopus arms / grips

Default C++ arm stroke motor force/gain is now **4×** the earlier
baseline (`WorldConfig::arm_motor_strength=4.0`), and the automatic
sucker/legacy tip break thresholds are **3×** stronger
(`WorldConfig::sucker_strength=3.0`). The 16-key controls are unchanged.
These are force/threshold multipliers, **not guaranteed 4× travel**;
contact, mass, friction and the number of attached suckers also matter.
The values are bounded in `WorldConfig` and checked by the native
control regression suite. Rebuild the native server with
`./tools/build.ps1 -NoTests` before running the Godot demo.

## Slime surface, progressive uncoiling, anti-flip (2026-11-15)

- Imported GameReady GLB materials receive a **lightweight wet PBR
  finish** (roughness 0.20, clearcoat 0.68, clearcoat roughness 0.10).
  Original albedo/normal textures stay intact; there is no animated
  jelly shader, per-frame re-meshing or simulated mucus fluid.
- Pressing an arm's extension key advances a continuous **base-to-tip
  uncoil front**. Releasing the key coils the arm back progressively.
  The distal sections retain target curvature until the front reaches
  them, rather than instantly turning the whole rod straight.
- A contacted rigid object/floor supplies a **contact-local frame**.
  Nearby ungripped suction sites are steered toward the local surface;
  adhesion itself still requires actual C++ contact and can break.
- Torso upright bias applies capped, equal-and-opposite forces to
  real torso mass nodes, reducing unintended flips without locking the
  body or overriding collisions.
- The previous overpowered arm motor 4.0 and suction 3.0 multipliers
  are reduced to **2.2 / 2.2**. The eight dual-key pairs and Space
  release semantics remain unchanged.

**Fidelity note:** wrapped contact uses sphere-like local curvature
around sampled rigid particles, not a true arbitrary-mesh surface
geodesic; the wet appearance is PBR lighting, not a fluid layer.
Real GameReady GLB visuals and feel remain subject to local testing.

## Low-resolution viscoelastic flesh / mantle (2026-11-15)

The C++ solver now allocates **12 dynamic shell particles per animal**
(two rings of six) around the original seven-vertex torso cage.
They have physical mass, contact with ground and props, and
compliant strands to the body. The original tetrahedral body volume
constraints remain active.

Each of the 16 segments of each arm additionally carries a
**damped dynamic transverse strain** driven by measured contact
compression and smoothed along neighboring sections. The two
principal axis scales are inverse pairs so their product remains
one; axial stretch also changes area through the existing
section-volume correction. The physical strain has a slow recovery
and local viscous damping rather than snapping back in one frame.
Strain is applied through the existing GPU-skinned GameReady arm
bones, with the previous restrictive ±1.8 visual scale expanded
to 0.40–2.5. **No new jelly shader or 5k-vertex CPU mesh rebuild**.

The bridge transmits all 19 body mass-node positions and 2-axis
section strain data. If the imported GLB exposes a **separate named
mantle/head/hood mesh**, Godot uses the physical shell bounds to
rescale its existing mesh transform (no rebuilt vertices); a
combined/unnamed GLB cannot receive that mantle visual deformation
until its model is split or painted with suitable vertex weights.
This is a coarse shape/viscoelastic proxy, **not full FEM or
Navier–Stokes slime fluid**; its discrete shell particles do not
provide exact triangle-to-triangle surface collision.

**GameReady combined mesh correction:** original recorded GLB has **193
bones** (192 arm bones plus a shared torso root). In addition to
separately named mantle meshes, the renderer now scales the **shared
torso root bone** from the low-resolution shell's measured bounding
axes and compensates the child arm bones with parent-space targets.
This makes root-weighted mantle vertices deform even when the GLB is
one combined mesh. Other vertices only deform according to their
original skin weights, so full local mantle folds still require a
re-rig or a multi-point weighted soft skin.

A momentum-conserving internal dashpot damps shell-to-cage relative
velocity before integrating particle positions, reducing rubbery
oscillation without injecting external forces. The shell is only
12 colliding mass points (not a watertight volumetric tetra mesh).

## Delayed material creep and stress relaxation

When you hold an arm against the terrain or pull it with the mouse,
C++ now slowly adapts the stress-free arm length/bend shape and the
mantle-shell strand rest lengths. This reduces the mechanical tendency
to snap immediately back to the originally rigged rest pose.
It is distinct from the previous viscous damping: **viscosity** removes
oscillations; **creep** changes internal rest shape under sustained
strain, and **stress relaxation** reduces the remaining elastic strain
under constant deformation. On release, slow recovery acts instead
of permanently freezing the octopus's shape. Per-segment measurements
are available in the bridge as `creep` and `elastic_strain`.
All calculations remain in the shared C++ world; GPU skinning remains
unchanged. This is an approximate and bounded gameplay material,
not a measured biological viscoelastic constitutive model.

## Experimental true 3D arm — isolated preview

Run `PLAY_VOLUMETRIC_DEMO.cmd` after pulling `main` and placing the
private GameReady GLB in `game/assets/Octopus_GameReady_v5.glb`.
The normal `PLAY_DEMO.cmd` is unchanged and keeps the experimental
feature **off**. A new `OCTO_VOLUME_ARM=1` environment flag enables
one physical arm (arm index 0) on each octopus via an optional fourth
argument to the native `reset` protocol. See
`tests/volumetric_arm_tests.cpp` for the live C++ stability fixture.

The new physical arm contains **nine rings of four independent 3D mass
nodes**, connected to its original rod by compliant cross-sectional,
longitudinal and diagonal strands. Every pair of adjacent rings has
XPBD signed-volume tetrahedra. Ring masses collide with the world;
separately enforced projected area constraints allow irregular
flattening with sideways bulging. The two other existing systems
(Cosserat muscle drive and C++ sucker contacts) still use the same
shared physics world. The native bridge transmits real surface points
in `arms[0].volume_rings`. The Godot pilot overlays a translucent,
**36-vertex dynamically built surface** on the original GLB arm so
these true non-elliptical section movements can be seen independently
of its existing bone weights. This small CPU mesh update is **only an
experimental visualization** and is NOT the final GPU-skinned surface.

**Important limitation:** this prototype is not ready for global
default enablement or expansion to eight arms. In the current two-animal
rigid-prop/floor CI fixture, the original torso volume residual stays
bounded (maximum about 0.016), but the maximum **individual arm
soft-tetra relative volume residual is still about 3.41**. In other
words, there is real volumetric geometry and contacts but high
local volume errors: local incompressibility and inversion stability
have *not* been solved. Further work must improve tet quality and
contact-constrained volume projection before replacing the production
rendering. Neither the GameReady GLB frame time nor realistic slime
appearance has been verified by the CI environment.

The instrumented Linux CI run for **two octopuses** reported physics
step mean 2.45 ms with the pilot versus 1.97 ms without (same runner;
not a hardware-independent claim). The global Godot frame time,
rendering and original GameReady visual realism remain unmeasured.

## Biologically grounded living GameReady skin (2026-11-15)

The standard `PLAY_DEMO.cmd` now applies **live deformation to the
original imported GLB mesh vertices** through Godot GPU skinning and
bone-weighted physical tissue pressure from the shared C++ simulator.
The shader retains source color, UV, normal, metallic and roughness
textures, adds small physical-contact flattening and sideways skin
bulging, subtle nonoscillating papillae and slow pigment modulation.
Arms now rest **asymmetrically and passively** rather than all eight
retracting/actively sweeping in unison; a gentle mantle respiration
also helps avoid the frozen cooked-octopus appearance.

The unstable tetrahedral trial is still off by default. The new GPU
skin does **not** create full surface-level collisions or validated
incompressible material behavior. The actual private GameReady GLB is
not tracked in the repository and still requires on-device visual
validation. Use `$env:OCTO_SKIN_MODE="legacy"` before launching to
compare the old appearance. See
[`docs/design/REALISTIC_OCTOPUS_SURFACE.md`](docs/design/REALISTIC_OCTOPUS_SURFACE.md).
Run `python tools/audit_gameready_mesh.py game/assets/Octopus_GameReady_v5.glb`
on the local asset to check vertex/face, skin and topological details.

## Gravity-settled octopus rest pose (2026-11-15)

The standard `PLAY_DEMO.cmd` now uses a distinct **grounded** native
physics profile (the final optional field in `reset` is 1). Its spawn
height is 0.27 m instead of 0.75 m, torso upright torque multiplier
0.08 instead of 0.45, and uncommanded arm activation and curvature
are both zero. Unpressed arms have long, slightly asymmetric lengths
rather than actively bending into eight similar raised curls. The
C++ bend and director compliance is higher when muscles are off,
particularly toward the distal ends. At rest, sucker contact does not
automatically glue every arm to the floor; pressing an arm control
still activates its force-limited motion and suction.

A native C++ 6-second gravity/floor regression measured torso center
height **0.2247 m** and **100%** of sampled distal arm *centerline*
nodes below 0.115 m. This is evidence of a physically grounded pose,
**not proof that all triangles of the original GLB have collision
contact**. Existing shared world collision and user input remain active.

For an A/B comparison with the old posture, run PowerShell:
```powershell
$env:OCTO_REST_MODE = "legacy"
.\\PLAY_DEMO.cmd
Remove-Item Env:\\OCTO_REST_MODE
```
Actual GameReady GUI rendering is still unverified in CI because the
private source GLB is not stored in Git. Screenshots should be checked
for residual skinning mismatches against the now-correct floor-laying
centerline physics.

## Flexible arm-to-mantle junction (2026-11-15)

The prior solver anchored each arm root to **all seven stiff body
cage vertices**, effectively creating a rigid internal hub even if
arm tissue parameters were soft. Default grounded gameplay now uses
an outer **deformable mantle-shell attachment**: three neighboring
shell particles plus a softer one-node deep tissue connection and a
secondary strand from the next arm segment. The arm's load now
physically deforms its attached mantle region. The core 7-node volume
cage continues to conserve body volume. The original world and 16-key
input protocol are unchanged; the older root constraints remain for
legacy non-grounded physics regression scenarios only.

A high-force native pull fixture confirms that the arm root moves
relative to the body cage and carries the actual mantle shell along.
This improvement does **not** imply that the private GLB's skin weights
now create a completely smooth arm-to-body visual junction; vertex
weight painting/re-rigging may still be needed after inspecting the
source asset on the user's PC.

## Collision repair and tissue-specific materials (2026-11-15)

**Arm versus dynamic props:** the C++ collision solver now checks
not just eight prop-corner spheres but also the **complete oriented
box faces**, including the swept paths of arm nodes and interpolated
segment midpoints. Corrections apply equal/opposite position impulses
to the actual arm and eight connected rigid-corner masses in the
single C++ world. A final contact projection occurs **after** arm
radius changes during the step, preventing late floor penetration
from axial expansion. A cheap swept bound skips distant segments.

**Mantle is now modeled separately as a low-resolution water balloon.**
Its 12 dynamic outer-ring masses and two end-cap nodes enclose a
24-triangle shell. An XPBD **single global volume constraint** holds
the shell's total volume approximately constant while allowing the
surface to flatten and bulge under pressure; the internal seven-node
body cage is more compliant in grounded gameplay mode. This is
not a fully sampled FEM balloon or a per-vertex fluid pressure solver.

**Arms have distinct chewy, viscous tissue.** Sustained floor contact
now supplies pressure even after solver penetration is removed,
driving gradual transverse flattening and reciprocal lateral
expansion rather than springy one-frame impulses. Unloaded arms
recover slowly, while the existing Cosserat muscles and suckers
remain the motor/adhesion controls. The material parameter
`arm_radial_yield=0.48` bounds the local thinner transverse scale.

Native regression now pulls an arm through the middle of a dynamic
box and checks that no sampled arm centerline enters its interior;
measured minimum axis-aligned clearance was 0.2003 m for a box with
0.16 m half-size and 1772+ animal/prop contact observations.
Under a strong sustained load the mantle relative volume deviation
was ~0.7%; at grounded rest its volume residual was ~0.014% and
an arm's thinnest transverse ratio fell to ~0.552. These are
**physics-state tests**, not confirmation that the original private
GameReady GLB's full visible surface cannot clip geometry.

The user's asset is intentionally untracked; verify real mesh collision
and material behavior with `PLAY_DEMO.cmd` after pulling the changes.
A fully watertight triangle-based mesh contact and calibrated
muscular hydrostat constitutive model remain outside this patch.
