#include "octo/world.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace octo;

static void require(bool ok,const char* msg){if(!ok)throw std::runtime_error(msg);}
static double distance(Vec3 a,Vec3 b){return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z);}

int main(){try{
  WorldConfig isolated;
  isolated.animals=1;isolated.rigid_count=0;isolated.gravity={0,0,0};
  isolated.floor=false;isolated.collisions=false;isolated.damping=.4;
  PhysicsWorld forward(isolated),backward(isolated);
  MuscleCommand m;m.arm=0;m.activation=1;m.curvature=0;m.extension=1.28;m.sweep=-1;
  auto initial=forward.snapshot();
  require(forward.command(m),"forward paddle accepted");
  m.sweep=1;require(backward.command(m),"backward paddle accepted");
  require(distance(initial.animals[0].arms[0].nodes.back(),
                   backward.snapshot().animals[0].arms[0].nodes.back())<1e-12,
          "command must not teleport arm");
  for(int i=0;i<90;++i){
    require(forward.step(),"forward paddle stable");
    require(backward.step(),"backward paddle stable");
  }
  auto a=forward.snapshot(),b=backward.snapshot();
  const double delta=distance(a.animals[0].arms[0].nodes.back(),b.animals[0].arms[0].nodes.back());
  std::cout<<"CONTROL_V1_MOTOR opposite_sweep_delta_m="<<delta<<"\n";
  require(delta>.003,"opposite strokes physically distinguishable");
  require(a.diagnostics.nonfinite_count==0&&b.diagnostics.nonfinite_count==0,"finite sweep state");

  // Actuator power must produce more arm motion than the previous
  // conservative baseline under identical physical initial conditions.
  WorldConfig weak_cfg=isolated, strong_cfg=isolated;
  weak_cfg.arm_motor_strength=1.0;
  strong_cfg.arm_motor_strength=4.0;
  PhysicsWorld weak(weak_cfg),strong(strong_cfg);
  MuscleCommand push;m.arm=0;
  push.arm=0;push.activation=1;push.curvature=0;push.extension=1.0;
  push.sweep=1.0;
  require(weak.command(push)&&strong.command(push),"test stroke command accepted");
  const Vec3 start_tip=weak.snapshot().animals[0].arms[0].nodes.back();
  for(int i=0;i<16;++i){
    require(weak.step(),"weak baseline remains finite");
    require(strong.step(),"strong motor remains finite");
  }
  const auto weak_snapshot=weak.snapshot(),strong_snapshot=strong.snapshot();
  const double weak_travel=distance(start_tip,weak_snapshot.animals[0].arms[0].nodes.back());
  const double strong_travel=distance(start_tip,strong_snapshot.animals[0].arms[0].nodes.back());
  std::cout<<"CONTROL_V1_POWER weak_tip_motion_m="<<weak_travel
           <<" strong_tip_motion_m="<<strong_travel<<"\n";
  require(strong_travel>weak_travel*1.05,
          "increased actuator strength must measurably increase stroke motion");
  require(strong_snapshot.diagnostics.nonfinite_count==0,
          "strong motor state remains finite");

  // Invalid / unbounded forces cannot silently enter the solver.
  {auto invalid=isolated; invalid.arm_motor_strength=15.;
   bool rejected=false;try{PhysicsWorld bad(invalid);}catch(const std::invalid_argument&){rejected=true;}
   require(rejected,"reject unsafe actuator strength");}
  WorldConfig floor;
  floor.animals=1;floor.rigid_count=0;floor.floor=true;floor.collisions=false;
  floor.gravity={0,-9.81,0};floor.spawn_height=.14;
  floor.auto_suction=true;floor.damping=.6;
  PhysicsWorld sticky(floor);
  std::uint64_t grip=0;
  for(int i=0;i<240;++i){
    require(sticky.step(),"automatic grip stable");
    grip=std::max(grip,sticky.snapshot().diagnostics.attachments);
  }
  std::cout<<"CONTROL_V1_GRIP max_auto_floor_contacts="<<grip<<"\n";
  require(grip>0,"stable real-contact floor probes should grip");
  sticky.release_all_suckers(true);
  require(sticky.step(),"Space release stable");
  require(sticky.snapshot().diagnostics.attachments==0,"Space releases every grip");
  for(int i=0;i<10;++i){
    require(sticky.step(),"held Space stable");
    require(sticky.snapshot().diagnostics.attachments==0,"held Space forbids new grip");
  }
  sticky.release_all_suckers(false);
  for(int i=0;i<5;++i){
    require(sticky.step(),"cooldown stable");
    require(sticky.snapshot().diagnostics.attachments==0,"no instantaneous regrip");
  }
  // Extension must seek an actual floor contact, then acquire an
  // ordinary suction constraint; no distant/teleported attachment.
  {auto reach_cfg=floor;reach_cfg.spawn_height=.75;reach_cfg.softness=5.;
   PhysicsWorld reaching(reach_cfg);
   MuscleCommand extend;extend.arm=0;extend.activation=1;
   extend.extension=1.48;extend.curvature=0;extend.sweep=-1;
   require(reaching.command(extend),"reach-for-floor command accepted");
   std::uint64_t acquired=0;
   for(int i=0;i<240;++i){
    require(reaching.step(),"reach-for-floor physics stays finite");
    acquired=std::max(acquired,reaching.snapshot().diagnostics.attachments);
   }
   std::cout<<"CONTROL_V1_REACH_GROUND max_contacts="<<acquired<<"\n";
   require(acquired>0,"extending arm eventually grips contacted floor");
   reaching.release_all_suckers(true);
   require(reaching.step(),"reach grip Space release stable");
   require(reaching.snapshot().diagnostics.attachments==0,
           "Space overrides reach-for-floor adhesion");
  }
  // Coil/uncoil progresses through the rod over time instead of
  // instantaneously resizing a straight tube.
  {auto c=isolated;c.auto_suction=false;c.coil_curvature=8.0;c.uncoil_speed=2.0;
   PhysicsWorld coiling(c);
   const double start=coiling.snapshot().animals[0].arms[0].uncoil_progress;
   require(start==0.0,"arm starts with an unreleased coil");
   MuscleCommand extend;extend.arm=0;extend.activation=1;
   extend.extension=1.48;extend.curvature=0;extend.sweep=0;
   require(coiling.command(extend),"uncoil command accepted");
   for(int i=0;i<15;++i)require(coiling.step(),"uncoiling is finite");
   const double mid=coiling.snapshot().animals[0].arms[0].uncoil_progress;
   require(mid>.35&&mid<.65,"base-to-tip uncoil wave travels gradually");
   for(int i=0;i<20;++i)require(coiling.step(),"uncoil reaches distal end");
   const double extended=coiling.snapshot().animals[0].arms[0].uncoil_progress;
   require(extended>.98,"whole arm uncoils after sufficient time");
   extend.extension=.58;extend.curvature=3.6;
   require(coiling.command(extend),"recoil command accepted");
   for(int i=0;i<15;++i)require(coiling.step(),"recoiling stays finite");
   const double returned=coiling.snapshot().animals[0].arms[0].uncoil_progress;
   require(returned<extended-.35,"recoil front retreats when key released");
   std::cout<<"UNCOIL_PASS start="<<start<<" mid="<<mid
            <<" extended="<<extended<<" returned="<<returned<<"\n";
  }
  // Upright correction is a physical field, not a graphics transform.
  // It must stay finite with collisions and gravity in the shared world.
  {auto c=floor;c.upright_strength=.45;c.arm_motor_strength=2.2;
   PhysicsWorld stable(c);
   for(int i=0;i<240;++i)require(stable.step(),"upright field remains stable");
   require(stable.snapshot().diagnostics.nonfinite_count==0,
           "upright constraint has finite physical state");
  }
  {auto invalid=isolated;invalid.upright_strength=3.0;
   bool rejected=false;try{PhysicsWorld no(invalid);}catch(const std::invalid_argument&){rejected=true;}
   require(rejected,"unbounded stabilization torque must be rejected");
  }
  // Start genuinely rolled over, not merely upright and motionless.
  // Both worlds have the same initial physical cage/arms. The unassisted
  // one keeps its initial roll in zero gravity; the corrected one must
  // develop a more upward top-to-bottom torso axis.
  {WorldConfig passive=isolated;
   passive.spawn_roll=2.65;passive.coil_curvature=0;
   passive.upright_strength=0;passive.damping=.5;
   WorldConfig assisted=passive;assisted.upright_strength=.45;
   PhysicsWorld untreated(passive),restored(assisted);
   auto cos_tilt=[](const RenderSnapshot& s)->double{
    const auto& body=s.animals[0].body_nodes;
    return (body[3].y-body[4].y)/distance(body[3],body[4]);
   };
   const double initial_tilt=cos_tilt(restored.snapshot());
   double best_assisted=initial_tilt,best_passive=initial_tilt;
   for(int i=0;i<240;++i){
    require(untreated.step()&&restored.step(),"tilted-body recovery finite");
    best_assisted=std::max(best_assisted,cos_tilt(restored.snapshot()));
    best_passive=std::max(best_passive,cos_tilt(untreated.snapshot()));
   }
   std::cout<<"UPRIGHT_RECOVERY initial="<<initial_tilt
            <<" best_assisted="<<best_assisted
            <<" best_passive="<<best_passive<<"\n";
   require(best_assisted>best_passive+.10,
           "upright motor measurably corrects a rolled-over body");
  }
  // Low-resolution mantle tissue is dynamic geometry with 12 genuine
  // colliding mass nodes, not a decorative rubber shader.
  {auto c=floor;c.spawn_height=.12;c.tissue_viscosity=10.0;
   PhysicsWorld tissue_world(c);
   auto start=tissue_world.snapshot();
   require(start.animals[0].body_nodes.size()==19,
           "mantle includes seven cage and twelve physical shell nodes");
   require(start.animals[0].arms[0].tissue_strain.size()==c.segments,
           "each arm section maintains two transverse material strains");
   double max_strain=0;
   for(int i=0;i<180;++i){
    require(tissue_world.step(),"viscoelastic mantle remains finite");
    auto snapshot=tissue_world.snapshot();
    for(auto strain:snapshot.animals[0].arms[0].tissue_strain){
     require(std::isfinite(strain[0])&&std::isfinite(strain[1]),
             "section strains always finite");
     require(strain[0]>=.49&&strain[0]<=2.51 &&
             strain[1]>=.39&&strain[1]<=1.51,
             "section strain stays inside mechanical yield bounds");
     require(std::abs(strain[0]*strain[1]-1.)<1e-9,
             "elliptic tissue redistributes transverse area");
     max_strain=std::max(max_strain,std::abs(strain[1]-1.));
    }
    for(const auto& node:snapshot.animals[0].body_nodes)
     require(node.y>=-.001,"colliding mantle nodes do not cross ground");
   }
   require(max_strain>.10,
           "floor contact must cause at least 10% local transverse deformation");
   std::cout<<"TISSUE_PASS mantle_nodes="<<
     tissue_world.snapshot().animals[0].body_nodes.size()<<
     " max_transverse_strain="<<max_strain<<"\n";
  }
  {auto invalid=isolated;invalid.tissue_viscosity=-3;
   bool rejected=false;try{PhysicsWorld no(invalid);}catch(const std::invalid_argument&){rejected=true;}
   require(rejected,"negative tissue viscosity is not accepted");
  }
  // Viscoelastic damping must remove oscillatory energy without a
  // global/kinematic position lock. Same impulse, same initial world;
  // only relative rod viscosity differs. Both are closed systems.
  {auto c=isolated;c.auto_suction=false;c.damping=0;
   c.coil_curvature=0;c.upright_strength=0;
   c.arm_internal_viscosity=0;
   PhysicsWorld spring(c);
   c.arm_internal_viscosity=14;
   PhysicsWorld viscous(c);
   DragCommand d;d.kind=DragCommand::Kind::ArmTip;d.index=0;d.arm=0;
   d.target={0.3,1.35,1.6};d.max_force=8;d.stiffness=50;
   require(spring.drag(d)&&viscous.drag(d),"matched impulse fixtures");
   require(spring.step()&&viscous.step(),"matched impulse applied");
   d.kind=DragCommand::Kind::None;
   require(spring.drag(d)&&viscous.drag(d),"external impulse removed");
   double spring_energy=0,viscous_energy=0;
   for(int i=0;i<180;++i){
    require(spring.step()&&viscous.step(),"viscosity fixtures finite");
    if(i>60){
     spring_energy+=spring.snapshot().diagnostics.kinetic_energy;
     viscous_energy+=viscous.snapshot().diagnostics.kinetic_energy;
    }
   }
   std::cout<<"VISCOUS_DAMPING spring_energy="<<spring_energy
            <<" damped_energy="<<viscous_energy<<"\n";
   require(viscous_energy<spring_energy,
           "viscous arms must dissipate more energy than spring-only rods");
  }
  // Native world integration: a held load must evolve actual rod
  // rest-shape memory rather than merely changing a render material.
  {WorldConfig c=isolated;
   c.coil_curvature=0;c.upright_strength=0;c.gravity={0,0,0};
   c.auto_suction=false;c.creep_rate=1.2;c.creep_recovery=.04;
   PhysicsWorld loaded(c);
   DragCommand drag;drag.kind=DragCommand::Kind::ArmTip;
   drag.index=0;drag.arm=0;drag.target={0,.75,2.0};
   drag.stiffness=100;drag.max_force=35;
   require(loaded.drag(drag),"constant-load creep drag accepted");
   double max_creep=0,max_elastic=0;
   for(int tick=0;tick<240;++tick){
    require(loaded.step(),"native creep under sustained load stays finite");
    const auto snapshot=loaded.snapshot();
    const auto& arm=snapshot.animals[0].arms[0];
    require(arm.creep_strain.size()==c.segments &&
            arm.elastic_strain.size()==c.segments,
            "creep state exposed for each rod segment");
    for(unsigned j=0;j<c.segments;++j){
     require(std::isfinite(arm.creep_strain[j])&&
             std::isfinite(arm.elastic_strain[j]),"stress/creep state finite");
     require(std::abs(arm.creep_strain[j])<=c.max_creep_strain+1e-10,
             "material creep stays bounded");
     max_creep=std::max(max_creep,std::abs(arm.creep_strain[j]));
     max_elastic=std::max(max_elastic,std::abs(arm.elastic_strain[j]));
    }
   }
   std::cout<<"NATIVE_CREEP max_memory="<<max_creep
            <<" max_elastic="<<max_elastic<<"\n";
   require(max_creep>1e-5,"sustained force must change physical rest-shape memory");
  }
  // Experimental 3D flesh is a real collision/volume simulation, and
  // opt-in until coupled geometry passes all long stress fixtures.
  {auto c=isolated;c.volumetric_arm_prototype=true;c.coil_curvature=0;
   c.creep_rate=0;c.bend_creep_rate=0;c.upright_strength=0;
   PhysicsWorld volumetric(c);
   auto initial=volumetric.snapshot();
   require(initial.animals[0].arms[0].volume_rings.size()==c.segments/2+1,
           "3D arm prototype allocates four physical shell nodes per ring");
   for(int tick=0;tick<120;++tick)
    require(volumetric.step(),"3D volumetric arm remains finite at rest");
   auto state=volumetric.snapshot();
   require(state.animals[0].arms[0].volume_rings.size()==c.segments/2+1,
           "3D ring positions survive simulation");
   double displacement=0;
   for(size_t j=0;j<state.animals[0].arms[0].volume_rings.size();++j)
    for(int k=0;k<4;++k)
     displacement=std::max(displacement,distance(initial.animals[0].arms[0].volume_rings[j][k],
                                                state.animals[0].arms[0].volume_rings[j][k]));
   std::cout<<"VOLUMETRIC_ARM_PROTOTYPE rings="<<state.animals[0].arms[0].volume_rings.size()
            <<" rest_motion="<<displacement<<"\n";
  }
  // Real gravity-settled resting pose: no arm can continuously sweep
  // or hold the prior prescribed cooked-octopus curl at zero input.
  {WorldConfig rest;
   rest.animals=1;rest.rigid_count=0;rest.auto_suction=true;
   rest.grounded_rest=true;rest.spawn_height=.27;rest.upright_strength=.08;
   PhysicsWorld resting(rest);
   for(unsigned arm=0;arm<8;++arm){
    MuscleCommand idle;idle.arm=arm;idle.activation=0;
    idle.curvature=0;idle.extension=1.10;idle.sweep=0;
    require(resting.command(idle),"idle gravitational settling command accepted");
   }
   for(int tick=0;tick<360;++tick)
    require(resting.step(),"resting octopus remains finite with floor contacts");
   auto snapshot=resting.snapshot();
   unsigned close=0,total=0;
   for(const auto& ar:snapshot.animals[0].arms){
    for(size_t k=ar.nodes.size()/3;k<ar.nodes.size();++k){
     ++total;
     if(ar.nodes[k].y<.115)++close;
    }
   }
   double fraction=double(close)/total;
   std::cout<<"GROUNDED_REST torso_y="<<snapshot.animals[0].center.y
            <<" distal_floor_fraction="<<fraction
            <<" contact_count="<<snapshot.diagnostics.contact_count<<"\n";
   require(snapshot.animals[0].center.y<.43,
           "passive mantle must rest low instead of standing on arms");
   require(fraction>.50,
           "majority of distal passive arm nodes must lie near the ground");
   unsigned visible_clip=0,belly_contacts=0;
   const auto& body=snapshot.animals[0].body_nodes;
   for(unsigned j=0;j<7;++j)
    if(body[j].y <rest.visual_mantle_floor_radius-1e-4)++visible_clip;
   for(const auto& ar:snapshot.animals[0].arms)
    for(unsigned j=0;j<ar.nodes.size();++j){
     const double t=double(j)/(ar.nodes.size()-1);
     const double visual_radius=rest.visual_arm_floor_radius*(1.-.38*t);
     if(ar.nodes[j].y<visual_radius-1e-4)++visible_clip;
    }
   if(body[4].y-rest.visual_mantle_floor_radius<.025)++belly_contacts;
   std::cout<<"VISIBLE_SURFACE_CONTACT clipping="<<visible_clip
            <<" belly_contact="<<belly_contacts<<"\n";
   require(visible_clip==0,
           "resting visible arm and mantle envelopes must clear floor");
   require(belly_contacts>0,
           "mantle lower cage must physically contact the ground");
   require(snapshot.diagnostics.attachments==0,
           "uncommanded resting octopus cannot glue every arm to the floor");
  }
  // Two animals and three colliding props must also settle without
  // losing the shared native world or silently raising the bodies.
  {WorldConfig scene;
   scene.animals=2;scene.rigid_count=3;scene.auto_suction=true;
   scene.grounded_rest=true;scene.spawn_height=.27;
   scene.upright_strength=.08;
   PhysicsWorld world(scene);
   for(int tick=0;tick<240;++tick)
    require(world.step(),"grounded two-octopus prop world remains finite");
   const auto state=world.snapshot();
   double max_body=0;
   unsigned near_ground=0,total=0;
   for(const auto& animal:state.animals){
    max_body=std::max(max_body,animal.center.y);
    for(const auto& arm:animal.arms)
     for(size_t k=arm.nodes.size()/3;k<arm.nodes.size();++k){
      ++total;
      if(arm.nodes[k].y<.15)++near_ground;
     }
   }
   std::cout<<"GROUNDED_SHARED_WORLD max_body_y="<<max_body
            <<" near_floor_fraction="<<double(near_ground)/total<<"\n";
   require(state.diagnostics.nonfinite_count==0,
           "grounded shared world state finite");
   require(max_body<.48,"two passive bodies cannot become spider-like towers");
   require(double(near_ground)/total>.55,
           "most passive arm nodes must stay close to the floor even with props");
  }
  // Physical arm roots attach to deformable mantle tissue, not a
  // fixed seven-strut torso hub. Pulling one arm must move the nearby
  // mantle shell and allow the junction to deform relative to the cage.
  {WorldConfig c;c.animals=1;c.rigid_count=0;c.grounded_rest=true;
   c.auto_suction=false;c.spawn_height=.27;c.upright_strength=.08;
   PhysicsWorld world(c);
   const auto rest=world.snapshot();
   DragCommand pull;pull.kind=DragCommand::Kind::ArmTip;
   pull.index=0;pull.arm=0;
   const auto tip=rest.animals[0].arms[0].nodes.back();
   pull.target={tip.x+.25,tip.y+.48,tip.z+.24};
   pull.stiffness=130;pull.max_force=28;
   require(world.drag(pull),"arm pull accepted by shared solver");
   double max_shell_shift=0,max_root_relative=0;
   for(int i=0;i<120;++i){
    require(world.step(),"compliant mantle joint remains finite under load");
    const auto snap=world.snapshot();
    for(size_t j=7;j<snap.animals[0].body_nodes.size();++j)
     max_shell_shift=std::max(max_shell_shift,
       distance(snap.animals[0].body_nodes[j],rest.animals[0].body_nodes[j]));
    const auto& b=snap.animals[0];
    const auto& original=rest.animals[0];
    const Vec3 r{b.arms[0].nodes[0].x-b.center.x,
                 b.arms[0].nodes[0].y-b.center.y,
                 b.arms[0].nodes[0].z-b.center.z};
    const Vec3 r0{original.arms[0].nodes[0].x-original.center.x,
                  original.arms[0].nodes[0].y-original.center.y,
                  original.arms[0].nodes[0].z-original.center.z};
    max_root_relative=std::max(max_root_relative,distance(r,r0));
   }
   std::cout<<"ARM_MANTLE_JUNCTION shell_motion="<<max_shell_shift
            <<" relative_root_motion="<<max_root_relative<<"\n";
   require(max_shell_shift>.003,
           "arm movement must deform the mantle shell through its junction");
   require(max_root_relative>.003,
           "arm root may not remain welded to the torso cage");
  }
  // Complete dynamic prop faces, not just the eight corner spheres.
  // Pull an active arm tip through the geometric middle of a cube.
  // If the gap between corners is unprotected the rod ends up inside.
  {WorldConfig scene;scene.animals=1;scene.rigid_count=3;
   scene.grounded_rest=true;scene.spawn_height=.27;
   scene.upright_strength=.08;scene.auto_suction=false;
   PhysicsWorld world(scene);
   DragCommand move;move.kind=DragCommand::Kind::ArmTip;
   move.index=0;move.arm=0;move.target={0,.32,1.35};
   move.stiffness=125;move.max_force=30;
   require(world.drag(move),"arm-box face push command accepted");
   double worst_intrusion=1e9,max_balloon_error=0;
   uint64_t body_contacts=0;
   for(int t=0;t<240;++t){
    require(world.step(),"face-colliding octopus world remains finite");
    auto snap=world.snapshot();
    max_balloon_error=std::max(max_balloon_error,
                               snap.diagnostics.max_mantle_volume_error);
    body_contacts+=snap.diagnostics.animal_prop_contacts;
    for(const auto& box:snap.rigids)for(const auto& arm:snap.animals[0].arms)
     for(const auto& pt:arm.nodes){
      const double extent=std::max({std::abs(pt.x-box.center.x),
                                    std::abs(pt.y-box.center.y),
                                    std::abs(pt.z-box.center.z)});
      worst_intrusion=std::min(worst_intrusion,extent);
      require(extent>.115,"arm centerline entered interior of rigid cube");
     }
   }
   std::cout<<"BOX_FACE_COLLISION minimum_axis_distance="<<worst_intrusion
            <<" animal_prop_contacts="<<body_contacts
            <<" mantle_balloon_relative_error="<<max_balloon_error<<"\n";
   require(body_contacts>0,"live arm must collide against full box faces");
   require(std::isfinite(max_balloon_error)&&max_balloon_error<.8,
           "water-balloon mantle conserves bounded global volume");
  }
  std::cout<<"control v1 tests passed\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
