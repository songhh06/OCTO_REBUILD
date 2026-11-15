#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace octo {
struct Vec3 { double x=0, y=0, z=0; };
struct Quat { double x=0, y=0, z=0, w=1; };
struct MuscleCommand {
    std::uint32_t animal=0, arm=0;
    double activation=0;          // [0,1], filtered by solver
    double curvature=0;          // rad/m, signed curl in arm's vertical plane
    double extension=1;          // rest-length ratio, clamp [0.65,1.35]
    double twist=0;              // rad/m
    double stiffness=0;          // [0,1]
    bool attach=false;           // legacy explicit tip attachment
    double sweep=0;              // [-1,1] body-relative anterior/posterior muscle drive
};
struct DragCommand {
    enum class Kind { None, Animal, ArmTip, Rigid };
    Kind kind=Kind::None;
    std::uint32_t index=0, arm=0;
    Vec3 target{};               // metres in world coordinates
    double stiffness=30;        // N/m, force-limited, never teleport
    double max_force=20;         // N
};
struct WorldConfig {
    std::uint32_t animals=2, segments=16, substeps=4, iterations=8;
    Vec3 gravity{0,-9.81,0};
    double fixed_dt=1.0/60.0, damping=0.4, friction=0.65;
    double spacing=2.2, spawn_height=0.75;
    // Gameplay actuator and adhesion strength multipliers. Applied in the
    // C++ shared world (not Godot forces or visual animation).
    double arm_motor_strength=2.2;      // moderated from the overpowered 4.0 profile
    double sucker_strength=2.2;         // grips persist but do not lock the whole body rigidly
    // Greater compliance = softer, more shape change under the same load.
    // Keep volume compliance lower than body shape compliance so the
    // octopus deforms without turning into an unconstrained point cloud.
    double softness=5.0;                // preserve highly deformable XPBD flesh
    double upright_strength=0.45;       // capped anti-roll/anti-pitch torso torque
    double spawn_roll=0.0;            // radians: initial roll about world Z (test fixture; 0 by default)
    double uncoil_speed=2.0;            // length-fraction/second, progressive release front
    double coil_curvature=8.0;          // radians/metre in the coiled distal section
    // Viscoelastic tissue; stronger dissipation means less rubbery bounce.
    double tissue_relaxation=3.2;       // s^-1, slow local flesh recovery
    double tissue_viscosity=10.0;       // s^-1, strain-rate dissipation
    double arm_internal_viscosity=14.0;  // s^-1, relative-velocity dissipation along rods
    // Standard-linear-solid-inspired internal rest-shape adaptation.
    // A sustained strain gradually moves the stress-free shape toward
    // the loaded configuration; unloaded tissue recovers much more slowly.
    double creep_rate=0.70;            // s^-1, delayed deformation/relaxation
    double creep_recovery=0.06;        // s^-1, return toward anatomical rest
    double max_creep_strain=0.25;      // fractional rest-length adaptation
    double bend_creep_rate=0.45;       // s^-1, slow curvature memory
    // Staged volumetric arm prototype, enabled for physical arm 0 only.
    bool volumetric_arm_prototype=false;
    double volumetric_compliance=2e-10; // per-tetra XPBD volume compliance
    bool grounded_rest=false;           // opt-in gravity-settled octopus posture (gameplay)
    double visual_arm_floor_radius=0.065; // world m: conservative GameReady skin/sucker envelope
    double visual_mantle_floor_radius=0.065; // world m: cage surface clearance
    double arm_root_shell_compliance=3e-5; // m/N XPBD: local mantle-to-arm tissue junction
    double arm_root_center_compliance=2e-4; // softer deep-tissue tether, avoids rigid root hub
    double mantle_balloon_compliance=8e-8; // low-compliance global shell volume, pressure-like
    double arm_radial_yield=0.48; // passive chewy arm flattened thickness / diameter
    bool reach_ground_on_extend=true;   // guide ungripped extended tips to contact
    std::uint32_t rigid_count=3;
    bool auto_suction=false;      // enabled for gameplay; original native tests retain passive baseline
    bool collisions=true, floor=true;
};
struct ArmSnapshot {
    double uncoil_progress=0;      // base-to-tip unwound fraction [0,1]
    std::vector<Vec3> nodes;     // segments+1, root -> tip, world metres
    std::vector<Quat> frames;    // segments, local +Z is tangent
    std::vector<Vec3> angular_velocities; // 3D director angular velocity, world rad/s
    std::vector<double> material_roll; // segments, physically solved torsion angle (radians)
    std::vector<std::array<double,2>> tissue_strain; // two dynamical transverse semiaxis scale factors
    std::vector<double> creep_strain; // segment axial rest-shape adaptation
    std::vector<double> elastic_strain; // instantaneous length/rest mismatch (stress proxy)
    std::vector<std::array<Vec3,4>> volume_rings; // 4 real 3D surface masses per ring; not a rendered GLB mesh
    std::vector<std::array<double,2>> radii; // segments, ellipse semiaxes metres
    std::vector<bool> attached;  // segments; physical attachment, not decoration
};
struct AnimalSnapshot {
    Vec3 center{};
    Quat orientation{};
    std::vector<Vec3> body_nodes; // first 7 cage vertices, then low-resolution tissue shell
    std::array<ArmSnapshot,8> arms;
};
struct RigidSnapshot { Vec3 center{}; Quat orientation{}; Vec3 half_extents{0.16,0.16,0.16}; };
struct Diagnostics {
    double step_ms=0, constraints_ms=0, collision_ms=0;
    double max_penetration=0, max_volume_error=0, max_soft_volume_error=0, max_mantle_volume_error=0, max_shear_error=0, kinetic_energy=0;
    Vec3 linear_momentum{};
    Vec3 angular_momentum{}; // about world origin, includes material-roll spin
    std::uint64_t contact_count=0, candidate_pairs=0, attachments=0, broken_attachments=0, nonfinite_count=0;
    std::uint64_t cross_animal_contacts=0, animal_prop_contacts=0;
    std::uint64_t swept_contacts=0;
};
struct RenderSnapshot {
    std::uint64_t tick=0;
    double time=0;
    std::vector<AnimalSnapshot> animals;
    std::vector<RigidSnapshot> rigids;
    Diagnostics diagnostics{};
};
// Single-thread owner. Inputs persist until replaced; snapshots are owned copies.
// All dynamics and contacts belong here, never in Godot RigidBody3D.
class PhysicsWorld {
public:
    explicit PhysicsWorld(const WorldConfig& config = {});
    ~PhysicsWorld();
    PhysicsWorld(PhysicsWorld&&) noexcept;
    PhysicsWorld& operator=(PhysicsWorld&&) noexcept;
    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;
    void reset(const WorldConfig& config);
    bool command(const MuscleCommand& command);
    bool drag(const DragCommand& command);
    void release_all_suckers(bool held); // Space overrides all grips while held
    bool impulse_rigid(std::uint32_t index, Vec3 impulse); // N·s at the prop center, for controlled physical fixtures
    bool step();                 // exactly config.fixed_dt; false on invalid state
    RenderSnapshot snapshot() const;
    const WorldConfig& config() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace octo
