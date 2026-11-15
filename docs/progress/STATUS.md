# Current status — 2026-11-15 (Asia/Seoul)

G0 baseline PARTIAL: old source inspected read-only; original dirty tree retained.
New independent repo OCTO_REBUILD, base cf42649, separate Sol/Terra worktrees.
No code is copied from the old solver. Original GameReady art will be reused locally.

Implementation underway, no phase declared complete. Sol owns XPBD world,
Terra owns bridge/render/input, Astra owns API/build/integration/performance.
2026-11-15 Sol takeover: initial Sol physics and Terra bridge/Godot work integrated into this repository. Astra stopped at user request; no agents continue working. Build: PASS (MSVC Release via tools/build.ps1). CTest: 2/2 PASS; independent 2-animal 10,000 tick soak PASS. Godot 4.6.2 original GameReady asset loaded with 193 bones and 2/10 animals advanced 60 ticks; visual screenshots captured. GPU scene appears but root/skin physical alignment is unmeasured. Native 10-animal active spread physics mean/p95 4.29/5.10 ms, dense 5.50/5.93 ms on Ryzen 5 5600X. Godot RPC observed ~55.8 ms (2 animals) and ~264.8 ms (10); 60 FPS FAIL. See docs/validation/INITIAL_RESULTS.md and evidence.
Targets: 1-animal real native/Godot vertical slice, then 2 shared-world animals,
then measured 1/2/5/10 load. Full 10-animal 60 FPS and physical feature completeness
are separate, unproven gates. First prototype is not full Cosserat or anatomical tissue.

Original failure lessons: global nonlinear solve had high active-input latency;
normal selection and endpoint contact caused regressions; body/skin mapping hid
physical gaps. New benchmarks include active inputs, reports expose contact/skin
limitations, and deterministic scenarios replace incompatible old binary checkpoints.

2026-11-15 follow-up: main-thread pipe polling caused multi-frame response delay.
Independent reader thread, compact render-only JSON and cached skeleton lookup
reduce 10-animal last-response RPC from ~265 ms to ~19 ms. Actual visible Godot
60-tick run took 1540 ms for 59 ticks (~38.3 simulation ticks/s). Ten-animal
60 Hz remains FAIL. Read docs/validation/BRIDGE_PROFILE.md for measurements.

2026-11-15 torsion slice: each rod segment now owns a material roll angle and
angular velocity with a tapered section inertia. Neighboring relative roll
is solved by XPBD compliance, anchored at the root. Z/X plus an arm key commands
opposite torsion; Godot receives and applies solved material roll. Build and
CTest 2/2 PASS, 10-animal active twist native p95 4.87 ms over 600 measured
ticks, 10-animal Godot passive smoke 60 ticks PASS but ~37.5 simulation ticks/s.
This is not full orientation-based Cosserat: no coupled shear/bending material
frames or torque reaction into torso. See docs/validation/TORSION_SLICE.md.

2026-11-15 bending follow-up: accumulated XPBD multiplier per arm/inner node
for three-component centerline curvature constraint, reset each actual substep.
Four vs sixteen solver sweeps targeted response fixture PASS, Release CTest 2/2,
2-animal 10,000-tick soak PASS. Ten active twist+bend native mean/p95
4.48/4.86 ms; visible Godot 10-animal 60-tick scene PASS but 37.9 ticks/s.
Two-animal active original-model scene showed arm bend and nonzero material
roll ~1.35 rad after 60 ticks. Full rotational Cosserat and 60 Hz still open.
See docs/validation/BENDING_XPBD.md.

2026-11-15 torso torsion coupling: first arm material-roll constraint now
uses the deforming torso up-axis and applies opposing mass-weighted position
corrections to two torso nodes. Isolated opposite-twist fixture body-node
difference ~1.15 mm and last-tick total angular momentum residual
~3.17e-9 kg m²/s. CTest 2/2, 10,000-tick soak PASS; 10-animal active twist
native p95 5.04 ms. Godot two-animal active input PASS, ten passive display
60 ticks PASS at 34.4 simulation ticks/s. This is still an approximate
scalar torsion coupling, not full orientation Cosserat or 60 Hz.
See docs/validation/TORSO_TORQUE.md.

2026-11-15 asynchronous bridge update: Godot reader thread now sends the next
native step on a 16.667 ms clock using the latest main-thread input, and the
renderer consumes completed authoritative C++ snapshots. Render-only JSON uses
six significant digits; C++ physics remains double. GPU 10-animal spread-active
600-tick test: 59.965 simulation ticks/s, 1191 visible frames/9.99 s, p95
frame interval 8.333 ms. Three dense-active 600-tick repeats: 59.766, 59.964,
60.001 ticks/s; frame p95 8.639/8.333/8.333 ms. Earlier dense run also
59.713 ticks/s. One additional run ended after tick 1 without a completion
record; cause unresolved. Runtime performance is PARTIAL for the tested scene,
not a 10-animal worst-case guarantee. Reset during dense-active run observed
and 600 further ticks completed. Build and CTest 3/3 PASS including bridge
protocol errors/dense reset. Read docs/validation/ASYNC_10_PERFORMANCE.md.
Follow-up: watchdog now uses time since last validated response, and a forced
native-child termination at tick 20 produced an explicit Godot exit code 2.
The earlier incomplete run remains unexplained, but future silence is bounded
by a 15-second timeout. Normal two-animal smoke still exits zero.

2026-11-15 swept contact checkpoint: shared C++ particle contact now tests
relative linear sphere sweeps over the substep and caches swept broadphase
bounds per solver pass. Analytic crossing and 200 N s two-prop regression
PASS; 400 N s order retention FAIL, so general CCD is not validated. Release
build and CTest 3/3 PASS; 2-animal dense 10,000-tick soak 0 nonfinite.
Ten-animal dense active native 600-tick mean/p95 6.587/7.024 ms; Godot GPU
600 ticks exited zero with screenshot, but 56.081 simulation ticks/s misses
the 60 Hz target. See docs/validation/SWEPT_CONTACT.md. Next: robust TOI
response/whole-shape collision, dense contact/skin optimization, tissue model.

2026-11-15 dynamic prop sweep guard: an enclosing-sphere center sweep and
post-constraint center separation now make the 400 N s two-prop straight-line
regression PASS for order, center gap and total momentum. Parallel near miss
PASS, Release CTest 3/3 PASS, 2-animal 10,000-tick and 10-animal 600-tick
native runs have zero nonfinite. Godot 10-animal dense active 600-tick runs
twice succeeded at 58.611 and 58.544 ticks/s, but two earlier runs hit the
15-second response watchdog. Runtime reliability and 60 Hz remain open.
Watchdog errors now include last tick/request/response IDs. This proxy does
not constitute exact oriented-box or deforming-surface CCD. See
docs/validation/RIGID_SWEEP_GUARD.md.

2026-11-15 bridge stall trace: 4/4 additional 10-animal 600-tick GPU runs
exited zero at 57.544–58.772 ticks/s. A 200 microsecond polling trial had
one 15-second watchdog failure at displayed tick 433 (request 435, response
434) and one successful run; polling change reverted. Blocking `get_line()`
trial failed at two-animal reset and was reverted. Three direct C++ server
600-tick active streams passed (~97 ticks/s). Reader and child liveness are
now included in future watchdog errors. Root cause remains unresolved; see
docs/validation/BRIDGE_STALL_TRACE.md.

Additional bridge evidence: one later 10-animal 600-tick run timed out at
tick 276, with request 278 outstanding and both the Godot reader thread and
C++ child alive. Pending bookkeeping was corrected; reader phase and byte
availability are recorded on future watchdog failures. Six instrumented
repeats then passed, 56.833–58.547 ticks/s. No phase-level failure captured;
reliability and sustained 60 Hz both remain failing gates in these tests.

2026-11-15 input path follow-up: headless Godot `game.input` CTest injects
Tab/Q and mouse pick/motion for animal 2, then checks C++ snapshot motion.
Display-only UI mouse interception was fixed. Release CTest 4/4 PASS;
120-tick run measured arm relative-tip divergence 0.164 m and horizontal
body motion during drag 0.774 m. This covers the synthetic two-animal path,
not a manual playability/attachment audit. See docs/validation/GODOT_INPUT_PATH.md.

2026-11-15 bridge bad_operation regression: after controls v1 main
adoption, an older pre-step2 executable at build/release-final/octo_server.exe
can be selected by the PowerShell demo launcher (previously checked only
Test-Path). Added tools/check-bridge.ps1 to send a real step2 request and
validate tick, fields, and protocol; run-demo.ps1 now rebuilds on source
staleness or failed protocol preflight before Godot launches. Direct Godot
bad_operation now emits actionable rebuild guidance. Windows CI exercises
preflight against an actual compiled octo_server.exe.

2026-11-15 force tuning on main: boosted physics-world sweep actuator
force gain and per-site motor cap to 4x prior baseline, suction-break
thresholds to 3x (both automatic sucker nodes and legacy tip grips).
Exposed bounded C++ WorldConfig multipliers and added a weak-vs-strong
stroke displacement regression test. This does not modify rendering or
control key bindings. Real GameReady play-feel/60Hz with new forces
remains to be validated locally; CI physics and integration pass.

2026-11-15 softer body/arms and seek-to-grip pass: XPBD structural
compliance multiplied by 5 for torso edges and arm axial, root,
bending/shear/twist constraints; torso volume loosened by sqrt(5).
Godot held extend now requests rest-length 1.48 instead of 1.28,
release 0.58 instead of 0.70; C++ clamp extended to 0.55..1.50.
No new shader or full volumetric mantle skin is implemented. When
an arm is extended with no active sucker, distal nodes receive
bounded downward forces towards reachable floor, then adhesive
contacts are created ONLY after physical contact. Automatic grip
prefers movable rigid props first and floor second, with a shorter
dwell threshold. Absence of reachable surfaces or held Space prevents
attachment. New control regression exercises extended-floor search,
actual grip and global Space release.

2026-11-15 uncoiling/slime/upright follow-up on main: added per-arm
uncoil front sampled in both centerline bending and director targets,
base-to-tip release and reverse recoiling, plus snapshot 'uncoil'
progress for Godot/bridge inspection. Contacted suckers record local
surface frames; a C++ surface-follow motor guides adjacent suction
candidate nodes around a contact rather than teleporting grip. Added
weak capped physical upright body torque (two dynamic torso nodes),
reduced arm_motor_strength 4.0->2.2 and sucker_strength 3.0->2.2.
A lightweight imported StandardMaterial3D PBR finish with low roughness
and clearcoat makes the existing model look wet without a jelly shader.
No full mucus simulation, arbitrary mesh wrapping, or GLB hardware
render benchmark claimed. CI regression: uncoil progress/recoil,
forces, grip/Space, old director fixtures and Godot shader-free PBR.

2026-11-15 tissue volume prototype on main: original 7 torso cage nodes
now coupled to 12 additional physically simulated mantle shell
particles (2x6 ring). Shell collides with floor/objects, is joined by
soft internal springs, and does not bypass C++ world. Arm section
transverse strain is its own damped, viscous state, pressure-responsive
and smoothed to neighboring segments, with reciprocal principal
scales and axial volume coupling. Snapshot/bridge exports 19 body
nodes and transverse strain per section. GameReady arm skeleton now
shows larger nonuniform transverse deformation, no shader/per-frame
mesh rebuild. A separately named mantle mesh is optionally scaled
from physical shell extents; merged GLB mantle remains visually
unresolved. Native 19-node/suction/tissue tests and bridge contract
added; Windows, Linux and Godot CI should gate regressions.

2026-11-15 combined GameReady torso fix: all 193 GLB bones include a
shared torso root; its root-weighted surface receives C++ 12-shell
bulk deformation via Godot root bone pose, with arm child transforms
recomputed against the deformed parent. A synthetic Godot Skeleton3D
root test verifies finite/nonidentity scaling without private GLB.
Internal mantle-to-center velocity dashpots now exchange equal and
opposite impulses to damp bouncy oscillation while conserving total
linear momentum in free space. Renderer still uses existing GPU skin,
not the former CPU rebuilt jelly skin or a high-cost custom shader.

2026-11-15 rubbery-bounce correction after live-play feedback: the former
arm tissue transversal value followed a second-order spring/velocity
integrator, while XPBD distance/director projection regenerated rod
particle velocities every tick. Switched the local strain to an
overdamped first-order deformation state, with faster flattening and
slower recovery. Added configurable physical rod-relative viscous damping
(default 14/s) using equal-and-opposite pair impulses, preserving linear
momentum and leaving contacts in the native solver. Baseline native
regression compared identical applied impulses without vs with this
viscosity (integrated kinetic energy 7.65415 vs 5.7151 over sampled
ticks on CI). This validates damping, not the subjective on-screen
'goo' feeling. GameReady live visuals still require local play test.

2026-11-15 delayed material creep and stress relaxation on main:
introduced rest-shape memory on arm axial strands and mantle shell,
plus slow geometric bend residual memory. Once each fixed physics
substep, a bounded standard-linear-solid-inspired internal variable
adapts toward sustained measured deformation; no changes inside
individual XPBD iteration loops. An explicit slow recovery term brings
rest shape back after unloading, and analytical exponential stepping
keeps parameters independent of substep dt. Original volume-constrained
torso cage and rigid-box edges are intentionally excluded. C++ bridge
now reports per-segment internal creep and remaining elastic strain.
Dedicated numerical tests verify under held 20% deformation that
stress relaxes over seconds, and after release memory recovers;
an integration fixture loads the actual physical arm tip. Tests must
be verified against Linux+Windows CI and local GameReady visuals.

2026-11-15 first genuine 3D volumetric arm pilot on main:
Added 9 cross-sectional rings x 4 C++ dynamic surface particles for
arm 0 only (per octopus), longitudinal/perimeter/diagonal constraints,
32 signed-volume tetrahedra and local area-conserving XPBD projection.
All ring particles participate in shared floor/object/octopus C++
contact. Rest of the arms retain old centerline implementation; pilot
is opt-in by environment OCTO_VOLUME_ARM=1 and optional native reset
parameter. Added an explicit PLAY_VOLUMETRIC_DEMO.cmd; ordinary
PLAY_DEMO.cmd is unchanged. The experiment overlays a lightweight
36-vertex translucent procedural shell driven by actual ring coordinates,
which demonstrates non-elliptical shape changes but is not final
weighted GPU skin. The cross section radius of the existing GameReady
arm is also sampled from the physical cage when enabled.
The original first unguarded rollout broke existing tests, so the
feature was moved behind an off-by-default option and a separate
6-test native regression including 2 octopuses, floor and boxes.
**Known blocker:** test maximum individual tetra relative volume
residual ~3.41463 under two-animal shared-world contact; unacceptable
for a calibrated incompressible soft-body. No claim of complete
volumetric soft-body stability or production gameplay fidelity.
Linux benchmark two octopuses: prototype 2.45 ms mean vs baseline
1.97 ms mean physics step (runner-dependent). No full GameReady GUI
benchmark. Deformation/contact tests and demo-input path checked
through CI.

2026-11-15 living source-GLB skin pass: direct vertex shader deformation
of the original GameReady mesh before its existing GPU skeletal
skinning, driven by native arm tissue compression mapped to actual
GLB bone indices. Source albedo/normal/roughness/metallic textures and
UVs are preserved; unsupported transparent/custom materials stay
unchanged. Stable micro-papillae, roughness variation, and slow
chromatophore-like pigment changes added without per-frame CPU mesh
rebuild. The player command path now releases unpressed arms to a
lightly coiled asymmetric passive posture with zero idle sweep,
instead of eight identical tight active coils. Added slow mantle
respiratory skin scale. Godot synthetic skin/real shader test covers
texture preservation, bone mapping, baseline control, and breathing.
Original private GLB absent from GitHub, so full model screenshot/FPS
and exact source surface-to-C++ contact accuracy remain unverified.
Physics 3D tetra pilot remains deliberately opt-in due high local
volume errors. Source GLB audit script and design report added.

2026-11-15 physics posture correction after screenshot feedback: old
GameReady demo was spider-like with a raised torso and 8 tense arms.
Added opt-in `WorldConfig::grounded_rest` and a fourth optional native
reset argument (0/1), and enabled grounded mode by default in the
Godot demo. Native gameplay start height .27 m (was .75), anti-upright
strength .08 (was .45); rest commands have zero motor activation,
zero curvature, zero sweep, small asymmetric length variations. At
zero muscle activation, distal centerline bending and director
constraints are more compliant so gravity dominates rather than
imposing curled spring shapes. Inactive suction contacts no longer
create persistent floor anchors. Controls and Space remain intact.
Isolated 360-tick / six-second physics regression: torso center height
.224737 m, distal-node ground fraction 1.0 (<.115 m), with native
floor contact. These measured metrics do not test the private
GameReady model's visual triangle-to-floor clearance. Old mode can be
enabled temporarily via `OCTO_REST_MODE=legacy` for A/B.

2026-11-15 arm-mantle junction correction: confirmed the original solver
hard-tied every arm's root to **all 7 torso cage nodes** at
`1e-7 * softness` compliance. In grounded gameplay this is now
replaced by spatially local, distributed XPBD constraints from the arm
root to 3 closest lower mantle-shell vertices, from the second arm
node to the nearest shell vertex, and one **very soft** central
connective-tissue tether. The outer mantle-shell masses and arm nodes
remain fully dynamic and exchange forces in the shared C++ world.
Unmodified rigid-root constraints remain only for legacy isolated
physics fixtures (`grounded_rest=false`) so established unit tests
are not silently reinterpreted. Config exposes
`arm_root_shell_compliance=3e-5`,
`arm_root_center_compliance=2e-4` before softness scaling.
New real physical arm-drag regression verifies displacement of the
mantle shell and relative motion of arm root vs cage, in addition to
previous floor contact and no-penetration checks. Linux CI measured
maximum shell displacement 0.555837 m and relative root deformation
0.11756 m in a sustained high-force stress fixture, not typical idle
movement and not an acceptable calibration for final real-animal
fidelity. Existing idle case: torso y=0.224709, 94.79% distal nodes
near ground; visual proxy floor clipping 0, lower belly contact 1.
Actual private GLB skin continuity still requires local visual review.

2026-11-15 material split / box collision pass: fixed gap in previous
prop collision model (only 8 corner spheres were checked) by adding
continuous swept sphere-vs-oriented-box face contacts for rod nodes and
segment midpoints, with mass-weighted reactions to all 8 corners.
After axial-length correction updates arm radius, perform a final
floor/prop contact pass. Swept broadphase rejects distant sections,
reducing the CPU regression from the first unoptimized contact pass.
Separately, mantle shell 12 nodes + two caps now have a closed
24-triangle oriented volume with one XPBD global pressure-like
constraint. The 7-node inner cage is more compliant in grounded mode.
Arms use slow nonoscillatory, pressure-loaded 'chewy' transverse
flattening with reciprocal lateral bulge; resting contact pressure
persists without requiring deep floor penetration. All physical
interactions remain in the same C++ solver. Added explicit diagnostic
max_mantle_volume_error and a real arm-vs-box interior CTest.
CI Linux tests: face-center push stayed outside box, centerline
minimum axis distance ~0.2003 m, mantle max relative volume deviation
~0.00714 under that load, ground rest ~0.000134; resting arm minimum
thickness ratio ~0.552. Private GLB not in CI: rendered triangle
penetration and visual 'water-balloon/noodle' fidelity unverified.
