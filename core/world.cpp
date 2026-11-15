#include "octo/world.hpp"
#include "octo/collision.hpp"
#include "octo/rod_math.hpp"
#include "octo/viscoelastic.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
namespace octo {
namespace {
Vec3 operator+(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec3 operator-(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec3 operator*(Vec3 a,double s){return {a.x*s,a.y*s,a.z*s};}
Vec3 operator/(Vec3 a,double s){return a*(1/s);}
double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
Vec3 cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double norm(Vec3 a){return std::sqrt(dot(a,a));}
Vec3 unit(Vec3 a){double n=norm(a);return n>1e-12?a/n:Vec3{0,1,0};}
bool finite(Vec3 a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
Quat frame(Vec3 z){z=unit(z);Vec3 x=unit(cross(std::abs(z.y)<.95?Vec3{0,1,0}:Vec3{1,0,0},z)),y=cross(z,x);double t=x.x+y.y+z.z;Quat q;if(t>0){double s=2*std::sqrt(1+t);q={(y.z-z.y)/s,(z.x-x.z)/s,(x.y-y.x)/s,s/4};}else{ // robust tangent-aligned reference frame; physical material roll is applied separately
 double d=std::clamp(z.z,-1.,1.);if(d<-.999999)q={0,1,0,0};else {Vec3 c=cross({0,0,1},z);double s=std::sqrt(2*(1+d));q={c.x/s,c.y/s,c.z/s,s/2};}}
return q;}
Quat roll_frame(Quat q,double angle){const double half=.5*angle,s=std::sin(half),c=std::cos(half);return {q.x*c+q.y*s,q.y*c-q.x*s,q.z*c+q.w*s,q.w*c-q.z*s};}
}
struct PhysicsWorld::Impl {
struct P{Vec3 x,old,v;double inv,r;int animal,arm,node,rigid;double compression=0;};
struct Bounds { Vec3 lo, hi; };
struct D{int a,b;double rest,compliance,lambda=0,base_rest=0,creep=0;bool adaptive=false,arm_axial=false;};
struct T{std::array<int,4> p;double rest,lambda=0;bool soft_volume=false;};
struct Roll{double angle=0,old=0,omega=0,inv_inertia=0,lambda=0;};
struct Director{Quat q{},old{};Vec3 omega{},bend_lambda{},shear_lambda{};double inv_inertia=0;};
struct Sucker{
 unsigned node=0;int attached=-1;Vec3 anchor{};double rest=0,dwell=0,cooldown=0;
 // Surface-local frame, updated on attachment and used to lay remaining
 // ungripped suckers against the contacted terrain or moving object.
 Vec3 contact_normal{0,1,0},contact_tangent{1,0,0},contact_binormal{0,0,1};
 double wrap_radius=0; // local object curvature estimate (metres)
};
struct TissueSection{double transverse=1,rate=0,pressure=0;};
struct Arm{std::vector<TissueSection> tissue;std::vector<Sucker> suckers;double peel_clock=0;double uncoil_front=0;unsigned arm_index=0;std::vector<int> p;std::vector<double> radius;std::vector<double> squash;std::vector<Roll> roll;std::vector<Director> directors;std::vector<Vec3> bend_lambda,bend_creep;std::vector<int> axial_edges;std::vector<std::array<int,4>> volume_rings;MuscleCommand target{},active{};int attached=-1;Vec3 anchor{};double attach_rest=0;};
struct Animal{std::vector<int> body;std::vector<int> shell;std::array<Arm,8> arms;double shell_rest_volume=0,shell_lambda=0;};
WorldConfig cfg;bool release_held=false;std::vector<P> p;std::vector<D> d;std::vector<T> tetra;std::vector<Animal> animals;std::vector<std::vector<int>> rigids;std::vector<Bounds> contact_bounds;std::vector<int> contact_order;DragCommand drag;Diagnostics diag;std::uint64_t tick=0;double time=0;
int particle(Vec3 x,double mass,double radius,int animal=-1,int arm=-1,int node=-1,int rigid=-1){p.push_back({x,x,{},1/mass,radius,animal,arm,node,rigid});return int(p.size()-1);}
void edge(int a,int b,double compliance){const double rest=norm(p[a].x-p[b].x);d.push_back({a,b,rest,compliance,0,rest,0,false,false});}
double volume(const T&t)const{auto a=p[t.p[0]].x;return dot(p[t.p[1]].x-a,cross(p[t.p[2]].x-a,p[t.p[3]].x-a))/6;}
explicit Impl(WorldConfig c):cfg(c){
if(c.animals>64||c.segments<2||c.segments>64||c.substeps<1||c.substeps>64||c.iterations<1||c.iterations>100||!finite(c.gravity)||!std::isfinite(c.fixed_dt)||c.fixed_dt<=0||c.fixed_dt>.1||!std::isfinite(c.damping)||c.damping<0||!std::isfinite(c.friction)||c.friction<0||!std::isfinite(c.spacing)||c.spacing<0||!std::isfinite(c.spawn_height)||c.rigid_count>128||
   !std::isfinite(c.arm_motor_strength)||c.arm_motor_strength<0.1||c.arm_motor_strength>12.0||
   !std::isfinite(c.sucker_strength)||c.sucker_strength<0.1||c.sucker_strength>12.0||
   !std::isfinite(c.softness)||c.softness<0.5||c.softness>10.0||
   !std::isfinite(c.upright_strength)||c.upright_strength<0||c.upright_strength>2.0||
   !std::isfinite(c.spawn_roll)||std::abs(c.spawn_roll)>std::acos(-1.0)||
   !std::isfinite(c.uncoil_speed)||c.uncoil_speed<0.1||c.uncoil_speed>5.0||
   !std::isfinite(c.coil_curvature)||c.coil_curvature<0||c.coil_curvature>14.0||
   !std::isfinite(c.tissue_relaxation)||c.tissue_relaxation<.2||c.tissue_relaxation>20||
   !std::isfinite(c.tissue_viscosity)||c.tissue_viscosity<.5||c.tissue_viscosity>40||
   !std::isfinite(c.arm_internal_viscosity)||c.arm_internal_viscosity<0||c.arm_internal_viscosity>40||
   !std::isfinite(c.creep_rate)||c.creep_rate<0||c.creep_rate>5||
   !std::isfinite(c.creep_recovery)||c.creep_recovery<0||c.creep_recovery>2||
   !std::isfinite(c.max_creep_strain)||c.max_creep_strain<0||c.max_creep_strain>.5||
   !std::isfinite(c.bend_creep_rate)||c.bend_creep_rate<0||c.bend_creep_rate>5||
   !std::isfinite(c.volumetric_compliance)||c.volumetric_compliance<0||c.volumetric_compliance>1e-4||
   !std::isfinite(c.visual_arm_floor_radius)||c.visual_arm_floor_radius<0||c.visual_arm_floor_radius>.2||
   !std::isfinite(c.visual_mantle_floor_radius)||c.visual_mantle_floor_radius<0||c.visual_mantle_floor_radius>.2||
   !std::isfinite(c.arm_root_shell_compliance)||c.arm_root_shell_compliance<0||c.arm_root_shell_compliance>.005||
   !std::isfinite(c.arm_root_center_compliance)||c.arm_root_center_compliance<0||c.arm_root_center_compliance>.005||
   !std::isfinite(c.mantle_balloon_compliance)||c.mantle_balloon_compliance<0||c.mantle_balloon_compliance>.001||
   !std::isfinite(c.arm_radial_yield)||c.arm_radial_yield<.2||c.arm_radial_yield>1.)throw std::invalid_argument("Invalid WorldConfig");
for(unsigned n=0;n<c.animals;++n){Animal a;Vec3 center{(double(n)-double(c.animals-1)/2)*c.spacing,c.spawn_height,0};
const double cr=std::cos(c.spawn_roll),sr=std::sin(c.spawn_roll);
const auto rotate_spawn=[&](Vec3 v)->Vec3{
 return {v.x*cr-v.y*sr,v.x*sr+v.y*cr,v.z};
};
for(Vec3 off:std::array<Vec3,7>{{{0,0,0},{.2,0,0},{-.2,0,0},{0,.24,0},{0,-.16,0},{0,0,.2},{0,0,-.2}}})a.body.push_back(particle(center+rotate_spawn(off),.12,.065,int(n)));
for(int i=0;i<7;++i)for(int j=i+1;j<7;++j)edge(a.body[i],a.body[j],2e-5*c.softness);
for(int x:{1,2})for(int y:{3,4})for(int z:{5,6}){T t{{a.body[0],a.body[x],a.body[y],a.body[z]},0};t.rest=volume(t);tetra.push_back(t);}
 // Low-resolution deforming mantle tissue: two rings of six dynamic
 // particles. Each particle collides with terrain/props and reacts
 // through compliant strands to the existing volume-conserving cage.
 // No per-frame render mesh allocation; cage remains the source of
 // stable orientation, while the shell drives a mantle scale field.
 for(int level=0;level<2;++level){
  const double y=level==0?-.06:.19;
  const double radius=level==0?.22:.16;
  for(int k=0;k<6;++k){
   const double phi=2*std::acos(-1.)*k/6.;
   const Vec3 offset=rotate_spawn({radius*std::cos(phi),y,radius*std::sin(phi)});
   const int idx=particle(center+offset,.010,.033,int(n),-1,7+level*6+k);
   a.shell.push_back(idx);
   edge(idx,a.body[0],8e-5*c.softness);
   edge(idx,a.body[3],1.2e-4*c.softness);
   edge(idx,a.body[4],1.2e-4*c.softness);
   if(k>0)edge(idx,a.shell[level*6+k-1],1.2e-4*c.softness);
  }
  edge(a.shell[level*6],a.shell[level*6+5],1.2e-4*c.softness);
 }
 for(int k=0;k<6;++k)edge(a.shell[k],a.shell[6+k],1.0e-4*c.softness);
 // Rest volume of the 12-node mantle outer shell. This is separate
 // from the rigid-ish seven-point torso cage; the shell can change shape
 // while maintaining a water-balloon-style global volume.
 // Six quads connect two rings and two pole caps close the surface.
 // Signed orientation is corrected when evaluating the volume.
 auto shell_vol=[&](const Animal& animal)->double{
  const Vec3 o=p[animal.body[0]].x;
  double total=0;
  auto tri=[&](int ia,int ib,int ic){
   Vec3 u=p[ia].x-o,v=p[ib].x-o,w=p[ic].x-o;
   total+=dot(u,cross(v,w))/6.0;
  };
  for(int k=0;k<6;++k){
   int next=(k+1)%6;
   tri(animal.shell[k],animal.shell[6+k],animal.shell[6+next]);
   tri(animal.shell[k],animal.shell[6+next],animal.shell[next]);
   tri(animal.body[4],animal.shell[k],animal.shell[next]);
   tri(animal.body[3],animal.shell[6+next],animal.shell[6+k]);
  }
  return total;
 };
 a.shell_rest_volume=shell_vol(a);
for(int k=0;k<8;++k){auto& ar=a.arms[k];ar.arm_index=k;ar.uncoil_front=0;double angle=k*std::acos(-1.)/4;Vec3 radial=rotate_spawn({std::sin(angle),0,std::cos(angle)});for(unsigned j=0;j<=c.segments;++j){double u=double(j)/c.segments;double radius=.036*(1-.8*u);ar.radius.push_back(radius);ar.p.push_back(particle(center+radial*(.2+.9*u),.012,radius,int(n),k,int(j)));if(j){edge(ar.p[j-1],ar.p[j],3e-7*c.softness);ar.axial_edges.push_back(int(d.size()-1));}}
ar.squash.assign(c.segments,1.0);
ar.tissue.resize(c.segments);
for(unsigned j=0;j<c.segments;++j){const double radius=ar.radius[j],mass=.012;ar.roll.push_back({0,0,0,j?2.0/(mass*radius*radius):0,0});}
for(unsigned j=0;j<c.segments;++j){const double length=.9/c.segments,radius=ar.radius[j];const double inertia=.012*(length*length+3*radius*radius)/12;Quat q=frame(radial);ar.directors.push_back({q,q,{},{},{},1/inertia});}
ar.bend_lambda.resize(c.segments-1);ar.bend_creep.resize(c.segments-1);
const unsigned spacing=std::max(3u,c.segments/6);for(unsigned j=spacing;j<=c.segments;j+=spacing)ar.suckers.push_back({j});if(ar.suckers.empty()||ar.suckers.back().node!=c.segments)ar.suckers.push_back({c.segments});
// Staged, independent tetrahedral arm flesh. Four real 3D masses
// per cross-section (every two rod segments) and four signed-volume
// tetrahedra between adjacent rings. The original centerline commands
// these through compliant strands, never by teleporting the ring.
if(c.volumetric_arm_prototype&&k==0&&c.segments>=4){
 const unsigned stride=2;
 const Vec3 outward=radial;
 const Vec3 vertical=rotate_spawn({0,1,0});
 const Vec3 sideways=unit(cross(outward,vertical));
 for(unsigned j=0;j<=c.segments;j+=stride){
  std::array<int,4> ring{};
  const double rad=.036*(1-.8*double(j)/c.segments);
  const Vec3 midpoint=p[ar.p[j]].x;
  const Vec3 directions[4]={vertical,sideways,vertical*(-1),sideways*(-1)};
  for(int v=0;v<4;++v){
   const Vec3 pos=midpoint+directions[v]*rad;
   ring[v]=particle(pos,.003,.008,int(n),k,int(1000+j*4+v));
   // Finite shear compliance allows true non-elliptical squeeze.
   edge(ring[v],ar.p[j],8e-7*c.softness);
   if(!ar.volume_rings.empty()){
    const int before=ar.volume_rings.back()[v];
    edge(before,ring[v],1.8e-6*c.softness);
    edge(ar.volume_rings.back()[(v+1)%4],ring[v],2.0e-6*c.softness);
   }
  }
  // Structural perimeter permits contact patches to flatten locally
  // while opposed nodes move outward to conserve tetrahedral volume.
  for(int v=0;v<4;++v)
   edge(ring[v],ring[(v+1)%4],3e-6*c.softness);
  if(!ar.volume_rings.empty()){
   const auto& last=ar.volume_rings.back();
   const int previous_center=ar.p[j-stride],current_center=ar.p[j];
   for(int v=0;v<4;++v){
    T t{{previous_center,last[v],last[(v+1)%4],current_center},0};
    t.rest=volume(t);
    if(std::abs(t.rest)>1e-9){t.soft_volume=true;tetra.push_back(t);}
   }
  }
  ar.volume_rings.push_back(ring);
 }
}
// Preserve original rigid-root physics fixtures when the new
// grounded gameplay tissue profile is disabled. The player's octopus
// uses the compliant mantle junction below.
if(!c.grounded_rest){
 for(int body_vertex:a.body)edge(ar.p[0],body_vertex,1e-7*c.softness);
}else{
// Hydrostatically deformable arm-to-mantle junction. Previously root
// node 0 was tied to all seven cage nodes with nearly rigid links.
// Only the local outer mantle shell should transmit attachment stresses.
unsigned nearest=0;
double best=-std::numeric_limits<double>::infinity();
for(unsigned t=0;t<6;++t){
 const Vec3 offset=p[a.shell[t]].x-center;
 const double alignment=dot(offset,radial);
 if(alignment>best){best=alignment;nearest=t;}
}
const int shell_center=a.shell[nearest];
const int shell_left=a.shell[(nearest+5)%6];
const int shell_right=a.shell[(nearest+1)%6];
edge(ar.p[0],shell_center,c.arm_root_shell_compliance*c.softness);
edge(ar.p[0],shell_left,c.arm_root_shell_compliance*c.softness*1.8);
edge(ar.p[0],shell_right,c.arm_root_shell_compliance*c.softness*1.8);
// Widen the attachment across one adjoining rod segment; no hard hinge.
edge(ar.p[1],shell_center,c.arm_root_shell_compliance*c.softness*3.0);
edge(ar.p[0],a.body[0],c.arm_root_center_compliance*c.softness);
}
}
animals.push_back(a);}
// Adapt only arm axial and soft mantle-shell edges. Rigid prop
 // edges and volume-constrained torso cage must not acquire creep.
 for(auto& e:d){
  const auto& a=p[e.a];const auto& b=p[e.b];
  if(a.animal<0||a.animal!=b.animal)continue;
  if(a.arm>=0&&a.arm==b.arm&&std::abs(a.node-b.node)==1){
   e.adaptive=true;e.arm_axial=true;
  }else if(a.arm==-1&&b.arm==-1&&(a.node>=7||b.node>=7)){
   e.adaptive=true;
  }
 }
 for(unsigned k=0;k<c.rigid_count;++k){std::vector<int> ids;Vec3 center{double(k)*.65-.65,.32,1.35};for(int x:{-1,1})for(int y:{-1,1})for(int z:{-1,1})ids.push_back(particle(center+Vec3{x*.16,y*.16,z*.16},.125,.08,-1,-1,-1,int(k)));for(int i=0;i<8;++i)for(int j=i+1;j<8;++j)edge(ids[i],ids[j],1e-9);rigids.push_back(ids);}}
double mantle_volume(const Animal& animal,std::vector<Vec3>* gradients=nullptr)const{
 // Oriented closed 24-triangle shell (12 ring-side + 12 polar-cap).
 // A single XPBD volume constraint avoids the local tetrahedral
 // overconstraint that made the earlier prototype springy/inverted.
 std::array<std::array<int,3>,24> faces{};
 int t=0;
 for(int k=0;k<6;++k){
  const int next=(k+1)%6;
  faces[t++]={animal.shell[k],animal.shell[6+k],animal.shell[6+next]};
  faces[t++]={animal.shell[k],animal.shell[6+next],animal.shell[next]};
  faces[t++]={animal.body[4],animal.shell[k],animal.shell[next]};
  faces[t++]={animal.body[3],animal.shell[6+next],animal.shell[6+k]};
 }
 const int origin=animal.body[0];
 const Vec3 center=p[origin].x;
 if(gradients)gradients->assign(p.size(),Vec3{});
 double volume=0;
 for(const auto& face:faces){
  const Vec3 a=p[face[0]].x-center;
  const Vec3 b=p[face[1]].x-center;
  const Vec3 c=p[face[2]].x-center;
  volume+=dot(a,cross(b,c))/6.;
  if(gradients){
   const Vec3 ga=cross(b,c)/6.;
   const Vec3 gb=cross(c,a)/6.;
   const Vec3 gc=cross(a,b)/6.;
   (*gradients)[face[0]]=(*gradients)[face[0]]+ga;
   (*gradients)[face[1]]=(*gradients)[face[1]]+gb;
   (*gradients)[face[2]]=(*gradients)[face[2]]+gc;
   (*gradients)[origin]=(*gradients)[origin]-ga-gb-gc;
  }
 }
 return volume;
}
void balloon_volume_constraints(double h){
 if(!cfg.grounded_rest)return;
 // Maintain total mantle content but allow local side-wall yielding,
 // bulging, and cap flattening under gravity/rigid object contacts.
 for(auto& animal:animals){
  std::vector<Vec3> gradient;
  double volume=mantle_volume(animal,&gradient);
  if(std::abs(animal.shell_rest_volume)<1e-9)continue;
  // Negative initial shell winding is legitimate: preserve its sign.
  const double reference=animal.shell_rest_volume;
  double denom=cfg.mantle_balloon_compliance/(h*h);
  for(size_t i=0;i<p.size();++i)
   if(dot(gradient[i],gradient[i])>0)denom+=p[i].inv*dot(gradient[i],gradient[i]);
  if(denom<1e-16)continue;
  const double alpha=cfg.mantle_balloon_compliance/(h*h);
  double dl=(-(volume-reference)-alpha*animal.shell_lambda)/denom;
  // Avoid pressure blowup after violent prop impacts.
  dl=std::clamp(dl,-.003,.003);
  animal.shell_lambda+=dl;
  for(size_t i=0;i<p.size();++i)
   if(dot(gradient[i],gradient[i])>0)
    p[i].x=p[i].x+gradient[i]*(p[i].inv*dl);
 }
}
void distance(D& e,double h){auto&a=p[e.a];auto&b=p[e.b];Vec3 delta=b.x-a.x;double l=norm(delta);if(l<1e-12)return;double alpha=e.compliance/(h*h),dl=(-(l-e.rest)-alpha*e.lambda)/(a.inv+b.inv+alpha);e.lambda+=dl;Vec3 impulse=delta*(dl/l);a.x=a.x-impulse*a.inv;b.x=b.x+impulse*b.inv;}
void volumes(double h){for(auto&t:tetra){auto a=p[t.p[0]].x,b=p[t.p[1]].x,c=p[t.p[2]].x,e=p[t.p[3]].x;std::array<Vec3,4> g;g[1]=cross(c-a,e-a)/6;g[2]=cross(e-a,b-a)/6;g[3]=cross(b-a,c-a)/6;g[0]=(g[1]+g[2]+g[3])*(-1);double alpha=(t.soft_volume?cfg.volumetric_compliance:1e-10*std::sqrt(cfg.softness))/(h*h),den=alpha;for(int i=0;i<4;++i)den+=p[t.p[i]].inv*dot(g[i],g[i]);double dl=(-(volume(t)-t.rest)-alpha*t.lambda)/den;t.lambda+=dl;for(int i=0;i<4;++i){auto&v=p[t.p[i]];v.x=v.x+g[i]*(v.inv*dl);}}}
void section_area_constraints(double h){
 // Four independently moving perimeter masses enclose each local
 // 3D cross-section. Constrain only the projected *area*, not the
 // shape or aspect ratio, so floor contact can create a flat face
 // and push the remaining flesh sideways.
 if(!cfg.volumetric_arm_prototype)return;
 for(auto& animal:animals)for(auto& arm:animal.arms){
  if(arm.volume_rings.empty())continue;
  for(size_t j=0;j<arm.volume_rings.size();++j){
   const auto& ring=arm.volume_rings[j];
   const unsigned center_index=unsigned(j)*2;
   const unsigned next_index=std::min(cfg.segments,center_index+2);
   const unsigned previous_index=center_index>0?center_index-2:0;
   const Vec3 axis=p[arm.p[next_index]].x-p[arm.p[previous_index]].x;
   const Vec3 tangent=unit(axis);
   Vec3 points[4];
   for(int k=0;k<4;++k)points[k]=p[ring[k]].x;
   double area=0;
   for(int k=0;k<4;++k)
    area+=.5*dot(cross(points[k],points[(k+1)%4]),tangent);
   const double u=double(center_index)/cfg.segments;
   const double r0=.036*(1-.8*u);
   const double anatomical_area=2*r0*r0;
   // Nominal axial stretch of the centerline changes the necessary
   // section area for approximately conserved material volume.
   const double rest_span=.9/double(cfg.segments)*2*arm.active.extension;
   const double actual_span=norm(p[arm.p[next_index]].x-p[arm.p[previous_index]].x);
   const double target=anatomical_area*std::clamp(rest_span/std::max(actual_span,1e-5),.55,1.8);
   Vec3 gradients[4];
   double weight=0;
   for(int k=0;k<4;++k){
    gradients[k]=cross(points[(k+1)%4]-points[(k+3)%4],tangent)*.5;
    weight+=p[ring[k]].inv*dot(gradients[k],gradients[k]);
   }
   const double alpha=2e-11/(h*h);
   if(weight+alpha<=1e-20)continue;
   const double correction=(target-area)/(weight+alpha);
   for(int k=0;k<4;++k)
    p[ring[k]].x=p[ring[k]].x+gradients[k]*(correction*p[ring[k]].inv);
  }
 }
}
Vec3 coil_direction(const Arm& ar,unsigned joint)const{
 // A physically held prop gives a local bend-in direction; terrain
 // stays free to curl upwards. This is not a remote grasp or skin force.
 const Sucker* nearest=nullptr;
 unsigned closest=cfg.segments+1;
 for(const auto& s:ar.suckers){
  if(s.attached==-1)continue;
  unsigned sep=unsigned(std::abs(int(s.node)-int(joint)));
  if(sep<closest){closest=sep;nearest=&s;}
 }
 if(!nearest||closest>std::max(3u,cfg.segments/3)||nearest->attached==-2)
  return {0,1,0};
 return nearest->contact_normal*(-1);
}
void bending(double h){for(auto&a:animals){Vec3 up=unit(p[a.body[3]].x-p[a.body[4]].x);for(auto&ar:a.arms){double length=.9/cfg.segments*ar.active.extension;for(unsigned j=1;j<cfg.segments;++j){auto&l=p[ar.p[j-1]];auto&m=p[ar.p[j]];auto&r=p[ar.p[j+1]];const double u=double(j)/double(cfg.segments);
 // Release a smooth front root->tip. Coiled distal sections store
 // curvature even while the proximal arm has already unwound.
 const double phase=std::clamp((u-ar.uncoil_front+.10)/.20,0.,1.);
 const double coil=phase*phase*(3.-2.*phase);
 const double wanted=(ar.active.curvature+cfg.coil_curvature*coil)*ar.active.activation;
 const Vec3 curl=coil>0.02?coil_direction(ar,j):up;
 Vec3 target=curl*(wanted*length*length)+ar.bend_creep[j-1];Vec3 error=l.x-m.x*2+r.x-target;// Muscle-off hydrostat bending offers very little resistance under
 // gravity: the *inactive* arm must not behave like a loaded spring.
 const double passive=cfg.grounded_rest?1.0+7.0*(1.0-ar.active.activation)*u*u:1.0;
 double alpha=(2e-5*cfg.softness*(1-.9*ar.active.stiffness)*passive)/(h*h);auto&lambda=ar.bend_lambda[j-1];Vec3 dl=(error+lambda*alpha)*(-1/(l.inv+4*m.inv+r.inv+alpha));lambda=lambda+dl;l.x=l.x+dl*l.inv;m.x=m.x-dl*(2*m.inv);r.x=r.x+dl*r.inv;}}}}
void update_creep_memory(double h){
 // Internal SLS-style delayed rest-shape adaptation: q relaxes toward
 // the current total strain under sustained load, with a slower
 // anatomical recovery term. This also relaxes residual spring force
 // at a constant imposed deformation. Updated ONCE per substep.
 const double rate=cfg.creep_rate+cfg.creep_recovery;
 for(auto& e:d){
  if(!e.adaptive||rate<=0)continue;
  double nominal=e.base_rest;
  if(e.arm_axial){
   const auto& ar=animals[p[e.a].animal].arms[p[e.a].arm];
   nominal*=ar.active.extension;
  }
  const double length=norm(p[e.a].x-p[e.b].x);
  const double total=std::clamp(length/std::max(nominal,1e-7)-1.,-.5,.5);
  e.creep=viscoelastic::update_creep(e.creep,total,h,cfg.creep_rate,
                                     cfg.creep_recovery,cfg.max_creep_strain);
 }
 // Geometric curvature also holds a delayed stress-free correction.
 // Do not overwrite the user's active motor command, which remains
 // independent of accumulated passive deformation.
 const double bend_rate=cfg.bend_creep_rate+cfg.creep_recovery;
 if(bend_rate<=0)return;
 for(auto& animal:animals)for(auto& ar:animal.arms){
  const double segment=.9/double(cfg.segments)*ar.active.extension;
  for(unsigned j=1;j<cfg.segments;++j){
   const double u=double(j)/double(cfg.segments);
   const double phase=std::clamp((u-ar.uncoil_front+.10)/.20,0.,1.);
   const double coil=phase*phase*(3.-2.*phase);
   const Vec3 curl=coil>.02?coil_direction(ar,j):
       unit(p[animal.body[3]].x-p[animal.body[4]].x);
   const Vec3 commanded=curl*((ar.active.curvature+cfg.coil_curvature*coil)*
                               ar.active.activation*segment*segment);
   const Vec3 measured=p[ar.p[j-1]].x-p[ar.p[j]].x*2+p[ar.p[j+1]].x-commanded;
   auto& memory=ar.bend_creep[j-1];
   const Vec3 equilibrium=measured*(cfg.bend_creep_rate/bend_rate);
   memory=equilibrium+(memory-equilibrium)*std::exp(-bend_rate*h);
   const double limit=.8*segment;
   const double magnitude=norm(memory);
   if(magnitude>limit)memory=memory*(limit/magnitude);
  }
 }
}
void torsion(double h){for(auto&a:animals)for(auto&ar:a.arms){const double desired=ar.active.activation*ar.active.twist*(.9/cfg.segments)*ar.active.extension;const double alpha=(1e-5*cfg.softness*(1-.9*ar.active.stiffness))/(h*h);
// The first material section is attached to a deformable torso orientation.
// Opposite displacements at two cage nodes transmit its torque into the world.
const Vec3 axis=unit(p[ar.p[1]].x-p[ar.p[0]].x),up{0,1,0};Vec3 side=cross(axis,up);if(norm(side)<1e-5)side=cross(axis,Vec3{1,0,0});side=unit(side);
auto&top=p[a.body[3]];auto&bottom=p[a.body[4]];Vec3 span=top.x-bottom.x;double vertical=dot(span,up),lateral=dot(span,side),den=vertical*vertical+lateral*lateral;
if(den>1e-8){const double body_angle=std::atan2(lateral,vertical);Vec3 grad=(side*vertical-up*lateral)/den;auto&first=ar.roll[1];double error=first.angle-body_angle-desired;double weight=first.inv_inertia+(top.inv+bottom.inv)*dot(grad,grad);double delta=(-error-alpha*first.lambda)/(weight+alpha);first.lambda+=delta;first.angle+=first.inv_inertia*delta;top.x=top.x-grad*(top.inv*delta);bottom.x=bottom.x+grad*(bottom.inv*delta);span=top.x-bottom.x;ar.roll[0].angle=std::atan2(dot(span,side),dot(span,up));}
for(size_t j=2;j<ar.roll.size();++j){auto&left=ar.roll[j-1];auto&right=ar.roll[j];const double error=right.angle-left.angle-desired;const double delta=(-error-alpha*right.lambda)/(left.inv_inertia+right.inv_inertia+alpha);right.lambda+=delta;left.angle-=left.inv_inertia*delta;right.angle+=right.inv_inertia*delta;}}}
void director_constraints(double h){namespace rm=rod_math;
for(auto&a:animals)for(auto&ar:a.arms){const double segment_length=.9/cfg.segments*ar.active.extension;
const double shear_alpha=2e-6*cfg.softness/(h*h);
 const double muscle_off=cfg.grounded_rest?1.0+7.0*(1.0-ar.active.activation):1.0;
 const double bend_alpha=2e-5*cfg.softness*(1-.9*ar.active.stiffness)*muscle_off/(h*h);
for(size_t j=0;j<ar.directors.size();++j){auto&director=ar.directors[j];auto&left=p[ar.p[j]];auto&right=p[ar.p[j+1]];
Vec3 e=rm::rotate(director.q,{0,0,1}),span=right.x-left.x;
Vec3 error=rm::sub(span,rm::mul(e,rm::dot(span,e)));
double den=left.inv+right.inv+segment_length*segment_length*director.inv_inertia+shear_alpha;
Vec3 correction=rm::mul(rm::add(error,rm::mul(director.shear_lambda,shear_alpha)),1/den);
director.shear_lambda=rm::sub(director.shear_lambda,correction);
left.x=left.x+correction*left.inv;right.x=right.x-correction*right.inv;
director.q=rm::integrate(director.q,rm::mul(rm::cross(e,correction),segment_length*director.inv_inertia),1);
}
for(size_t j=1;j<ar.directors.size();++j){auto&left=ar.directors[j-1];auto&right=ar.directors[j];
Vec3 tangent=rm::rotate(left.q,{0,0,1});
 const Vec3 curl=coil_direction(ar,unsigned(j));
 Vec3 axis=rm::cross(tangent,curl);double axis_len=rm::length(axis);
if(axis_len<1e-8)axis=rm::cross(tangent,{1,0,0}),axis_len=rm::length(axis);
axis=rm::mul(axis,1/axis_len);
const double u=double(j)/double(cfg.segments);
 const double phase=std::clamp((u-ar.uncoil_front+.10)/.20,0.,1.);
 const double coil=phase*phase*(3.-2.*phase);
 const double wanted=ar.active.activation*(ar.active.curvature+cfg.coil_curvature*coil);
 Quat target=rm::multiply(rm::rotation(rm::mul(axis,wanted*segment_length)),left.q);
Vec3 error=rm::log_rotation(rm::multiply(target,rm::inverse(right.q)));
double den=left.inv_inertia+right.inv_inertia+bend_alpha;
Vec3 correction=rm::mul(rm::sub(error,rm::mul(right.bend_lambda,bend_alpha)),1/den);
right.bend_lambda=rm::add(right.bend_lambda,correction);
left.q=rm::integrate(left.q,rm::mul(correction,-left.inv_inertia),1);
right.q=rm::integrate(right.q,rm::mul(correction,right.inv_inertia),1);
}}}
bool excluded(const P&a,const P&b){if(a.rigid>=0&&a.rigid==b.rigid)return true;if(a.animal>=0&&a.animal==b.animal){if(a.arm<0&&b.arm<0)return true;if(a.arm==b.arm&&a.arm>=0&&(a.node>=1000||b.node>=1000))return true;
 if(a.arm==b.arm&&std::abs(a.node-b.node)<=2)return true;if(a.arm<0&&b.node<=2)return true;if(b.arm<0&&a.node<=2)return true;if(a.node<=2&&b.node<=2)return true;}return false;}
// A coarse whole-prop guard catches center paths that cross before individual
// corner constraints can resolve the coherent body's translation.
void rigid_center_sweeps(){if(!cfg.collisions)return;const double diameter=2*.16*std::sqrt(3.0);
for(size_t i=0;i<rigids.size();++i)for(size_t j=i+1;j<rigids.size();++j){Vec3 a0{},a1{},b0{},b1{};
for(int id:rigids[i]){a0=a0+p[id].old/8;a1=a1+p[id].x/8;}
for(int id:rigids[j]){b0=b0+p[id].old/8;b1=b1+p[id].x/8;}
auto hit=swept_sphere_toi(a0,a1,b0,b1,diameter);
Vec3 n=hit?hit->normal:unit(b1-a1);
if(!hit&&norm(b1-a1)>=diameter)continue;
double correction=diameter-dot(b1-a1,n);if(correction<=0)continue;
for(int id:rigids[i])p[id].x=p[id].x-n*(correction*.5);
for(int id:rigids[j])p[id].x=p[id].x+n*(correction*.5);
++diag.swept_contacts;
}}
double floor_surface_radius(const P& particle)const{
 double radius=particle.r;
 if(!cfg.grounded_rest||particle.animal<0)return radius;
 if(particle.arm>=0&&particle.node>=0&&particle.node<=int(cfg.segments)){
  // The GameReady surface, including protruding suckers, extends beyond
  // the thin XPBD centerline. Floor collision needs that *visual*
  // envelope rather than solely the nominal .036m axial radius.
  const double t=double(particle.node)/cfg.segments;
  radius=std::max(radius,cfg.visual_arm_floor_radius*(1.-.38*t));
 }else if(particle.arm==-1&&particle.node>=0&&particle.node<7){
  // Preserve genuine mantle/cage contact instead of passing a tiny
  // centerline sphere through the seabed.
  radius=std::max(radius,cfg.visual_mantle_floor_radius);
 }
 return radius;
}
void contacts(){if(cfg.floor)for(auto&a:p){
 const double surface_radius=floor_surface_radius(a);
 if(a.x.y<surface_radius){double depth=surface_radius-a.x.y;a.compression=std::max(a.compression,depth);diag.max_penetration=std::max(diag.max_penetration,depth);++diag.contact_count;a.x.y=surface_radius;Vec3 tangent{a.x.x-a.old.x,0,a.x.z-a.old.z};double l=norm(tangent);if(l>0)a.x=a.x-tangent*(std::min(l,cfg.friction*depth)/l);}
}
if(!cfg.collisions)return;
// Swept AABB broadphase includes both endpoints so fast crossing paths remain candidates.
contact_bounds.resize(p.size());contact_order.resize(p.size());
for(size_t i=0;i<p.size();++i){const auto&v=p[i];contact_bounds[i]={{std::min(v.old.x,v.x.x)-v.r,std::min(v.old.y,v.x.y)-v.r,std::min(v.old.z,v.x.z)-v.r},{std::max(v.old.x,v.x.x)+v.r,std::max(v.old.y,v.x.y)+v.r,std::max(v.old.z,v.x.z)+v.r}};contact_order[i]=int(i);}
std::sort(contact_order.begin(),contact_order.end(),[&](int a,int b){return contact_bounds[a].lo.x<contact_bounds[b].lo.x;});
for(size_t i=0;i<contact_order.size();++i){auto&a=p[contact_order[i]];for(size_t j=i+1;j<contact_order.size();++j){auto&b=p[contact_order[j]];
const auto&ba=contact_bounds[contact_order[i]],&bb=contact_bounds[contact_order[j]];
if(bb.lo.x>ba.hi.x)break;
if(excluded(a,b)||ba.lo.y>bb.hi.y||bb.lo.y>ba.hi.y||ba.lo.z>bb.hi.z||bb.lo.z>ba.hi.z)continue;
++diag.candidate_pairs;
Vec3 end=b.x-a.x;double l=norm(end),depth=a.r+b.r-l;Vec3 n{};
if(depth>0){n=l>1e-10?end/l:Vec3{1,0,0};diag.max_penetration=std::max(diag.max_penetration,depth);}
else{auto hit=swept_sphere_toi(a.old,a.x,b.old,b.x,a.r+b.r);if(!hit)continue;
n=hit->normal;depth=a.r+b.r-dot(end,n);if(depth<=1e-12)continue;++diag.swept_contacts;}
++diag.contact_count;
a.compression=std::max(a.compression,depth*.5);b.compression=std::max(b.compression,depth*.5);
if(a.animal>=0&&b.animal>=0&&a.animal!=b.animal)++diag.cross_animal_contacts;
if((a.animal>=0&&b.rigid>=0)||(b.animal>=0&&a.rigid>=0))++diag.animal_prop_contacts;
double w=a.inv+b.inv;a.x=a.x-n*(depth*a.inv/w);b.x=b.x+n*(depth*b.inv/w);
Vec3 relative=(b.x-b.old)-(a.x-a.old);Vec3 tangent=relative-n*dot(relative,n);double tl=norm(tangent);
if(tl>0){Vec3 corr=tangent*(std::min(tl,cfg.friction*depth)/tl);a.x=a.x+corr*(a.inv/w);b.x=b.x-corr*(b.inv/w);}
}}}



void upright_motors(double h){
 // A damped restoring *torque*: opposite forces on two existing torso
 // particles, not a hidden transform snap. Contacts still determine
 // whether an octopus can stand on a wall or roll under large loads.
 if(cfg.upright_strength<=0)return;
 for(auto& animal:animals){
  auto& top=p[animal.body[3]];
  auto& bottom=p[animal.body[4]];
  const Vec3 axis=top.x-bottom.x;
  const double span=norm(axis);
  if(span<1e-8)continue;
  const Vec3 world_up{0,1,0};
  const Vec3 axis_unit=axis/span;
  // Project the desired up direction onto the local rotation tangent.
  // Unlike simply dampening lateral offset, this makes inverted poses
  // unstable instead of accidentally stabilizing an upside-down body.
  Vec3 turn=world_up-axis_unit*dot(world_up,axis_unit);
  if(dot(axis_unit,world_up)<-.97 && norm(turn)<.24){
   Vec3 heading=p[animal.body[5]].x-p[animal.body[6]].x;
   Vec3 side=cross(heading,world_up);
   if(norm(side)<1e-6)side={1,0,0};
   turn=unit(side)*.24;
  }
  const Vec3 relative_v=top.v-bottom.v;
  const Vec3 lateral_v=relative_v-axis_unit*dot(relative_v,axis_unit);
  Vec3 force=(turn*(9.0*span)-lateral_v*1.8)*cfg.upright_strength;
  const double magnitude=norm(force);
  const double limit=2.5*cfg.upright_strength;
  if(magnitude>limit)force=force*(limit/magnitude);
  top.x=top.x+force*(top.inv*h*h);
  bottom.x=bottom.x-force*(bottom.inv*h*h);
 }
}
// Choose a surface frame at contact. This frame is descriptive,
// not an invented physical joint: its tangent tracks neighbouring
// rod nodes and its binormal points around the contacted object.
void record_sucker_frame(Arm& ar,Sucker& s){
 const P& site=p[ar.p[s.node]];
 const Vec3 axial=p[ar.p[std::min(unsigned(cfg.segments),s.node+1)]].x-
                  p[ar.p[s.node-1]].x;
 Vec3 normal{0,1,0};
 double radius=0;
 if(s.attached>=0){
  normal=unit(site.x-p[s.attached].x);
  radius=p[s.attached].r+site.r;
 }else if(s.attached==-2){
  normal={0,1,0};
 }
 Vec3 tangent=axial-normal*dot(axial,normal);
 if(norm(tangent)<1e-6){
  tangent=cross(normal,{0,0,1});
  if(norm(tangent)<1e-6)tangent=cross(normal,{1,0,0});
 }
 tangent=unit(tangent);
 s.contact_normal=normal;
 s.contact_tangent=tangent;
 s.contact_binormal=unit(cross(normal,tangent));
 s.wrap_radius=radius;
}
void follow_surface(double h){
 if(!cfg.auto_suction||release_held)return;
 for(auto& animal:animals)for(auto& ar:animal.arms){
  if(ar.target.activation<.5||ar.target.extension<1.0)continue;
  const Sucker* latest=nullptr;
  for(const auto& s:ar.suckers)
   if(s.attached!=-1&&(!latest||s.node>latest->node))latest=&s;
  if(!latest)continue;
  Vec3 guide=latest->contact_tangent;
  Vec3 normal=latest->contact_normal;
  for(auto& candidate:ar.suckers){
   if(candidate.attached!=-1||candidate.cooldown>0)continue;
   // Following the already attached distal site, gradually bring
   // the next nearby sucker onto the same tangent surface.
   if(candidate.node+cfg.segments/3<latest->node)continue;
   P& site=p[ar.p[candidate.node]];
   const double along=(.9/cfg.segments)*(int(candidate.node)-int(latest->node));
   Vec3 desired=latest->anchor+guide*along;
   if(latest->attached>=0){
    // Local tangent/normal/binormal of the first real sucker contact
    // approximates wrapping along the curved target's surface.
    const Vec3 center=p[latest->attached].x;
    const double radius=std::max(.025,p[latest->attached].r+site.r);
    const double angle=std::clamp(along/radius,-1.5,1.5);
    const Vec3 normal=unit(latest->contact_normal*std::cos(angle)
                          +guide*std::sin(angle));
    desired=center+normal*radius;
   }else{
    desired.y=site.r;
   }
   Vec3 force=(desired-site.x)*3.0;
   const double magnitude=norm(force);
   if(magnitude>.8)force=force*(.8/magnitude);
   site.x=site.x+force*(site.inv*h*h);
  }
 }
}

void sweep_motors(double h){
 // Muscle drive is force-limited and applies equal/opposite torso reactions.
 // At -1 target the arm sweeps anterior, at +1 it sweeps posterior.
 // ±60 deg is a *target*; rigid body contact can limit achieved angle.
 const double pi=std::acos(-1.0);
 for(auto&animal:animals){
  Vec3 forward=unit(p[animal.body[5]].x-p[animal.body[6]].x);
  Vec3 right=unit(cross({0,1,0},forward));
  for(auto&ar:animal.arms){
   if(std::abs(ar.active.sweep)<1e-5)continue;
   const double angle=ar.arm_index*pi/4.0;
   const Vec3 home=right*std::sin(angle)+forward*std::cos(angle);
   const double sign=ar.arm_index<4?1.0:-1.0;
   const double turn=sign*(pi/3.0)*ar.active.sweep;
   const Vec3 goal=home*std::cos(turn)+cross({0,1,0},home)*std::sin(turn);
   const Vec3 root=p[ar.p[0]].x;
   for(unsigned j=std::max(2u,cfg.segments/3);j<=cfg.segments;++j){
    const double u=double(j)/cfg.segments;
    auto&site=p[ar.p[j]];
    Vec3 force=(root+goal*(.9*u*ar.active.extension)-site.x)*(1.3*cfg.arm_motor_strength);
    const double mag=norm(force);
    const double max_motor_force=.085*cfg.arm_motor_strength;
    if(mag>max_motor_force)force=force*(max_motor_force/mag);
    site.x=site.x+force*(site.inv*h*h);
    for(int b:animal.body)p[b].x=p[b].x-force*(p[b].inv*h*h/animal.body.size());
   }
  }
 }
}
void seek_floor_contact(double h){
 if(!cfg.auto_suction||!cfg.floor||!cfg.reach_ground_on_extend||release_held)return;
 for(auto&animal:animals)for(auto&ar:animal.arms){
  if(ar.target.activation<.5||ar.target.extension<1.0)continue;
  bool has_grip=false;
  for(const auto& s:ar.suckers)if(s.attached!=-1){has_grip=true;break;}
  if(has_grip||ar.attached!=-1)continue;
  // Move distal arm nodes down via bounded forces. Their rod constraints
  // react on the rest of the arm/body; no fake remote floor attachment.
  for(unsigned j=std::max(2u,cfg.segments/2);j<=cfg.segments;++j){
   auto& tip=p[ar.p[j]];
   if(tip.x.y-tip.r<.015)continue;
   const double weight=double(j)/double(cfg.segments);
   const double force=std::min(2.5,8.0*(tip.x.y-tip.r))*weight;
   tip.x.y-=force*tip.inv*h*h;
  }
 }
}
void suction_constraints(double h){
 if(!cfg.auto_suction||release_held)return;
 for(auto&animal:animals)for(auto&ar:animal.arms)for(auto&s:ar.suckers){
  if(s.attached==-1)continue;
  auto&node=p[ar.p[s.node]];
  const Vec3 target=s.attached==-2?s.anchor:p[s.attached].x;
  const Vec3 delta=target-node.x;
  const double len=norm(delta);
  if(len<1e-10)continue;
  const double error=len-(s.attached==-2?0:s.rest);
  const double w=node.inv+(s.attached>=0?p[s.attached].inv:0);
  const double lambda=error/(w+4e-7/(h*h));
  const double area_scale=std::max(.002,node.r/.035);
  if(std::abs(lambda)/(h*h)>12.0*cfg.sucker_strength*area_scale*area_scale){
   s.attached=-1;s.dwell=0;s.cooldown=.18;++diag.broken_attachments;continue;
  }
  const Vec3 move=delta*(lambda/len);
  node.x=node.x+move*node.inv;
  if(s.attached>=0)p[s.attached].x=p[s.attached].x-move*p[s.attached].inv;
 }
}
void damp_mantle_tissue(double h){
 // Internal Kelvin-Voigt-like dashpot: exchange equal-and-opposite
 // impulses between shell and central cage. This removes jiggle while
 // preserving total linear momentum in a free, isolated octopus.
 const double fraction=1.0-std::exp(-cfg.tissue_viscosity*h);
 for(auto& animal:animals){
  auto& center=p[animal.body[0]];
  for(int index:animal.shell){
   auto& shell=p[index];
   const Vec3 relative=shell.v-center.v;
   const Vec3 impulse=relative*(fraction/(shell.inv+center.inv));
   shell.v=shell.v-impulse*shell.inv;
   center.v=center.v+impulse*center.inv;
  }
 }
}
void damp_arm_relative_velocities(double h){
 // XPBD rest-length/bend projections otherwise regenerate a large
 // relative particle velocity every tick, felt as a stiff spring.
 // Internal pairwise impulses suppress only relative motion and cancel
 // exactly, preserving the total linear momentum of the whole world.
 const double factor=1.-std::exp(-cfg.arm_internal_viscosity*h);
 if(factor<=0)return;
 for(auto& animal:animals)for(auto& arm:animal.arms){
  for(size_t j=1;j<arm.p.size();++j){
   auto& a=p[arm.p[j-1]];
   auto& b=p[arm.p[j]];
   const Vec3 relative=b.v-a.v;
   const Vec3 impulse=relative*(factor/(a.inv+b.inv));
   a.v=a.v+impulse*a.inv;
   b.v=b.v-impulse*b.inv;
  }
 }
}
void tissue_dynamics(double h){
 // Each rod section stores independent, damped transverse strain. Contact
 // pressures push strain; neighboring sections share it gradually. Axis
 // product still follows the axial volume correction, so compression
 // creates a lateral bulge rather than an unphysical volume loss.
 for(auto& animal:animals)for(auto& arm:animal.arms){
  std::vector<double> pressure(arm.tissue.size(),0);
  for(size_t j=0;j<arm.tissue.size();++j){
   const auto& a=p[arm.p[j]];
   const auto& b=p[arm.p[j+1]];
   const double r=std::max(.004,arm.radius[j]);
   pressure[j]=std::clamp(std::max(a.compression,b.compression)/r,0.,2.5);
  }
  for(size_t j=0;j<arm.tissue.size();++j){
   auto& tissue=arm.tissue[j];
   const double before=j?arm.tissue[j-1].transverse:tissue.transverse;
   const double after=j+1<arm.tissue.size()?arm.tissue[j+1].transverse:tissue.transverse;
   // Contact acts on material state, not just on rendered thickness.
   const double material_load=std::clamp(pressure[j],0.,1.8);
   const double neighbor=before+after-2*tissue.transverse;
   // Overdamped Maxwell-like relaxation instead of a mass-on-spring:
   // compression changes local shape without storing oscillatory
   // 'tissue.rate' energy. Recovery is deliberately much slower than
   // pressure-driven flattening, so contact releases do not ping back.
   const double desired=std::clamp(1.0-1.6*material_load+
                              .16*neighbor,.40,1.50);
   const double rate=desired<tissue.transverse
                         ?cfg.tissue_viscosity:cfg.tissue_relaxation*.35;
   const double response=1.0-std::exp(-rate*h);
   const double previous=tissue.transverse;
   tissue.transverse=std::clamp(previous+(desired-previous)*response,.40,1.50);
   tissue.rate=(tissue.transverse-previous)/h;
   tissue.pressure=material_load;
   arm.squash[j]=tissue.transverse;
  }
 }
}
void update_suckers(double h){
 if(!cfg.auto_suction)return;
 for(auto&animal:animals)for(auto&ar:animal.arms){
  if(ar.active.extension<.87)ar.peel_clock+=h;
  else ar.peel_clock=0;
  for(auto&s:ar.suckers){
   s.cooldown=std::max(0.0,s.cooldown-h);
   if(release_held){s.attached=-1;s.dwell=0;s.cooldown=.18;continue;}
   if(s.attached==-2&&ar.active.extension<.87){
    const double delay=double(cfg.segments-s.node)/cfg.segments*.22;
    if(ar.peel_clock>=delay){s.attached=-1;s.cooldown=.18;s.dwell=0;}
   }
   if(s.attached!=-1){
    record_sucker_frame(ar,s);
    continue;
   }
   if(s.cooldown>0)continue;
   // An octopus resting under its own weight can make contact without
   // automatically welding every inactive sucker to the seabed. Gripping
   // remains a separate, active behavior and Space still releases.
   if(cfg.grounded_rest&&ar.target.activation<.1&&ar.active.activation<.15){
    s.dwell=0;
    continue;
   }
   auto&node=p[ar.p[s.node]];
   Vec3 tangent=unit(p[ar.p[std::min(unsigned(cfg.segments),s.node+1)]].x-p[ar.p[s.node-1]].x);
   const Vec3 down{0,-1,0};
   Vec3 face=down-tangent*dot(down,tangent);
   if(norm(face)<.25){s.dwell=0;continue;}
   face=unit(face);
   if(norm(node.v)>.75){s.dwell=0;continue;}
   int target=-1;
   Vec3 anchor{};
   double rest=0;
   // Prefer an actually contacted movable object to terrain.
   if(cfg.collisions){
    double best=.020;
    for(size_t j=0;j<p.size();++j){
     if(p[j].rigid<0)continue;
     const Vec3 to=p[j].x-node.x;
     const double gap=norm(to)-(p[j].r+node.r);
     if(gap<best&&norm(p[j].v-node.v)<1.2&&dot(face,unit(to))>.15){
      target=int(j);rest=norm(to);best=gap;
     }
    }
   }
   // Only make a floor anchor after the sucker has really reached it.
   if(target==-1&&cfg.floor&&node.x.y-node.r<.020&&dot(face,down)>.30){
    target=-2;anchor={node.x.x,node.r,node.x.z};
   }
   if(target==-1||std::acos(-1.0)*node.r*node.r<.000035){s.dwell=0;continue;}
   s.dwell+=h;
   if(s.dwell<.020)continue;
   s.attached=target;s.anchor=anchor;s.rest=rest;s.dwell=0;
   record_sucker_frame(ar,s);
  }
 }
}

void attachments(double h){for(auto&a:animals)for(auto&ar:a.arms){auto&tip=p[ar.p.back()];if(release_held||!ar.target.attach){ar.attached=-1;continue;}if(ar.attached==-1){if(cfg.floor&&tip.x.y-tip.r<.025){ar.attached=-2;ar.anchor={tip.x.x,tip.r,tip.x.z};}else for(size_t i=0;i<p.size();++i)if(p[i].rigid>=0&&norm(p[i].x-tip.x)<p[i].r+tip.r+.025){ar.attached=int(i);ar.attach_rest=norm(p[i].x-tip.x);break;}}
if(ar.attached==-1)continue;Vec3 target=ar.attached==-2?ar.anchor:p[ar.attached].x;Vec3 delta=target-tip.x;double l=norm(delta);double rest=ar.attached==-2?0:ar.attach_rest;double w=tip.inv+(ar.attached>=0?p[ar.attached].inv:0);double impulse=(l-rest)/(w+1e-7/(h*h));if(std::abs(impulse)/(h*h)>8*cfg.sucker_strength){ar.attached=-1;++diag.broken_attachments;continue;}if(l>1e-10){Vec3 correction=delta*(impulse/l);tip.x=tip.x+correction*tip.inv;if(ar.attached>=0)p[ar.attached].x=p[ar.attached].x-correction*p[ar.attached].inv;}}}
bool step(){auto start=std::chrono::steady_clock::now();diag={};double h=cfg.fixed_dt/cfg.substeps;
for(unsigned sub=0;sub<cfg.substeps;++sub){for(auto&a:animals)for(auto&ar:a.arms){
 const double uncoil_target=(ar.target.activation>.5&&ar.target.extension>=1.0)?1.0:0.0;
 const double max_change=cfg.uncoil_speed*h;
 ar.uncoil_front+=std::clamp(uncoil_target-ar.uncoil_front,-max_change,max_change);
 double f=1-std::exp(-h*12);auto blend=[&](double&v,double t){v+=(t-v)*f;};blend(ar.active.activation,ar.target.activation);blend(ar.active.curvature,ar.target.curvature);blend(ar.active.extension,ar.target.extension);blend(ar.active.stiffness,ar.target.stiffness);blend(ar.active.twist,ar.target.twist);blend(ar.active.sweep,ar.target.sweep);}
for(auto&particle:p)particle.compression=0;
for(auto&e:d){
 e.lambda=0;
 if(e.adaptive){
  const double nominal=e.base_rest*(e.arm_axial?
      animals[p[e.a].animal].arms[p[e.a].arm].active.extension:1.);
  e.rest=nominal*(1.+e.creep);
 }
}
for(auto&t:tetra)t.lambda=0;
for(auto&animal:animals)animal.shell_lambda=0;
for(auto&a:animals)for(auto&ar:a.arms)std::fill(ar.bend_lambda.begin(),ar.bend_lambda.end(),Vec3{});
for(auto&a:animals)for(auto&ar:a.arms)for(auto&roll:ar.roll){roll.lambda=0;roll.old=roll.angle;roll.omega*=std::exp(-cfg.damping*h);roll.angle+=roll.omega*h;}
for(auto&a:animals)for(auto&ar:a.arms)for(auto&director:ar.directors){director.old=director.q;director.bend_lambda={};director.shear_lambda={};director.omega=director.omega*std::exp(-cfg.damping*h);director.q=rod_math::integrate(director.q,director.omega,h);}
int dragged=-1;if(drag.kind==DragCommand::Kind::Animal)dragged=animals[drag.index].body[0];if(drag.kind==DragCommand::Kind::ArmTip)dragged=animals[drag.index].arms[drag.arm].p.back();if(drag.kind==DragCommand::Kind::Rigid)dragged=rigids[drag.index][0];
for(size_t i=0;i<p.size();++i){auto&a=p[i];a.old=a.x;Vec3 force{};if(int(i)==dragged){force=(drag.target-a.x)*drag.stiffness;double l=norm(force);if(l>drag.max_force)force=force*(drag.max_force/l);}a.v=(a.v+(cfg.gravity+force*a.inv)*h)*std::exp(-cfg.damping*h);}
damp_mantle_tissue(h);
for(auto& particle:p)particle.x=particle.x+particle.v*h;
sweep_motors(h);
upright_motors(h);
follow_surface(h);
seek_floor_contact(h);
rigid_center_sweeps();
for(unsigned it=0;it<cfg.iterations;++it){auto c0=std::chrono::steady_clock::now();for(auto&e:d)distance(e,h);volumes(h);balloon_volume_constraints(h);section_area_constraints(h);bending(h);torsion(h);if(it%4==0||it+1==cfg.iterations)director_constraints(h);attachments(h);suction_constraints(h);diag.constraints_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-c0).count();auto t0=std::chrono::steady_clock::now();contacts();diag.collision_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();}
rigid_center_sweeps();
update_creep_memory(h);
tissue_dynamics(h);
update_suckers(h);
for(auto&a:animals)for(auto&ar:a.arms)for(unsigned j=0;j<cfg.segments;++j){double l=norm(p[ar.p[j+1]].x-p[ar.p[j]].x);double pressure=std::max(p[ar.p[j]].compression,p[ar.p[j+1]].compression);(void)pressure;p[ar.p[j]].r=ar.radius[j]*std::sqrt((.9/cfg.segments)/std::max(l,1e-5));p[ar.p[j]].r=std::min(.1,p[ar.p[j]].r);}
for(auto&a:p){a.v=(a.x-a.old)/h;if(!finite(a.x)||!finite(a.v)){++diag.nonfinite_count;return false;}}
damp_arm_relative_velocities(h);
for(auto&a:animals)for(auto&ar:a.arms)for(auto&roll:ar.roll){roll.omega=(roll.angle-roll.old)/h;if(!std::isfinite(roll.angle)||!std::isfinite(roll.omega)){++diag.nonfinite_count;return false;}}
for(auto&a:animals)for(auto&ar:a.arms)for(auto&director:ar.directors){director.q=rod_math::normalize(director.q);director.omega=rod_math::mul(rod_math::log_rotation(rod_math::multiply(director.q,rod_math::inverse(director.old))),1/h);if(!finite(director.omega)||!std::isfinite(director.q.w)){++diag.nonfinite_count;return false;}}
}
for(auto&a:p){diag.linear_momentum=diag.linear_momentum+a.v/a.inv;diag.angular_momentum=diag.angular_momentum+cross(a.x,a.v/a.inv);diag.kinetic_energy+=.5*dot(a.v,a.v)/a.inv;}for(auto&a:animals)for(auto&ar:a.arms){Vec3 axis=unit(p[ar.p[1]].x-p[ar.p[0]].x);for(auto&roll:ar.roll)if(roll.inv_inertia>0){diag.kinetic_energy+=.5*roll.omega*roll.omega/roll.inv_inertia;diag.angular_momentum=diag.angular_momentum+axis*(roll.omega/roll.inv_inertia);}for(size_t j=0;j<ar.directors.size();++j){auto&director=ar.directors[j];diag.kinetic_energy+=.5*dot(director.omega,director.omega)/director.inv_inertia;diag.angular_momentum=diag.angular_momentum+director.omega/director.inv_inertia;Vec3 tangent=unit(p[ar.p[j+1]].x-p[ar.p[j]].x),material=rod_math::rotate(director.q,{0,0,1});diag.max_shear_error=std::max(diag.max_shear_error,norm(cross(tangent,material)));}}for(auto&t:tetra){
 const double error=std::abs((volume(t)-t.rest)/t.rest);
 if(t.soft_volume)diag.max_soft_volume_error=std::max(diag.max_soft_volume_error,error);
 else diag.max_volume_error=std::max(diag.max_volume_error,error);
}for(auto&a:animals)for(auto&ar:a.arms){if(ar.attached!=-1)++diag.attachments;for(const auto& suction:ar.suckers)if(suction.attached!=-1)++diag.attachments;}++tick;time+=cfg.fixed_dt;diag.step_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();return true;}
RenderSnapshot snapshot()const{RenderSnapshot s;s.tick=tick;s.time=time;s.diagnostics=diag;for(auto&a:animals){AnimalSnapshot as;as.center=p[a.body[0]].x;as.orientation=frame(p[a.body[5]].x-p[a.body[6]].x);for(int b:a.body)as.body_nodes.push_back(p[b].x);for(int shell:a.shell)as.body_nodes.push_back(p[shell].x);for(int k=0;k<8;++k){auto&ar=a.arms[k];auto&out=as.arms[k];out.uncoil_progress=ar.uncoil_front;for(int id:ar.p)out.nodes.push_back(p[id].x);for(unsigned j=0;j<cfg.segments;++j){Vec3 tangent=p[ar.p[j+1]].x-p[ar.p[j]].x;out.frames.push_back(roll_frame(ar.directors[j].q,ar.roll[j].angle));out.angular_velocities.push_back(ar.directors[j].omega);out.material_roll.push_back(ar.roll[j].angle);double r=std::min(.12,ar.radius[j]*std::sqrt((.9/cfg.segments)/std::max(norm(tangent),1e-6)));if(!ar.volume_rings.empty()){
  // The four material surface masses determine the displayed section,
  // rather than prescribing an ellipse from the centerline. The
  // imported bone weights still limit the visual approximation.
  const auto& cage=ar.volume_rings[std::min(size_t(j/2),ar.volume_rings.size()-1)];
  const double radius_a=.5*norm(p[cage[0]].x-p[cage[2]].x);
  const double radius_b=.5*norm(p[cage[1]].x-p[cage[3]].x);
  out.radii.push_back({std::clamp(radius_a,.002,.12),
                       std::clamp(radius_b,.002,.12)});
 }else{
  out.radii.push_back({std::min(.16,r/ar.squash[j]),r*ar.squash[j]});
 }
 out.tissue_strain.push_back({1.0/ar.squash[j],ar.squash[j]});
 const auto& strand=d[ar.axial_edges[j]];
 out.creep_strain.push_back(strand.creep);
 const double axial=norm(p[ar.p[j+1]].x-p[ar.p[j]].x);
 out.elastic_strain.push_back(axial/std::max(strand.rest,1e-7)-1.);

bool held=j==cfg.segments-1&&ar.attached!=-1;for(const auto& suction:ar.suckers)if(suction.attached!=-1&&suction.node==j+1)held=true;out.attached.push_back(held);}
 for(const auto& ring:ar.volume_rings){
  std::array<Vec3,4> world_ring{};
  for(int v=0;v<4;++v)world_ring[v]=p[ring[v]].x;
  out.volume_rings.push_back(world_ring);
 }
 }s.animals.push_back(as);}for(auto&ids:rigids){RigidSnapshot rs;for(int id:ids)rs.center=rs.center+p[id].x/8;rs.orientation=frame(p[ids[1]].x-p[ids[0]].x);s.rigids.push_back(rs);}return s;}
};
PhysicsWorld::PhysicsWorld(const WorldConfig& c):impl_(std::make_unique<Impl>(c)){}
PhysicsWorld::~PhysicsWorld()=default;PhysicsWorld::PhysicsWorld(PhysicsWorld&&) noexcept=default;PhysicsWorld&PhysicsWorld::operator=(PhysicsWorld&&) noexcept=default;
void PhysicsWorld::reset(const WorldConfig&c){auto fresh=std::make_unique<Impl>(c);impl_=std::move(fresh);}
const WorldConfig&PhysicsWorld::config()const{return impl_->cfg;}
bool PhysicsWorld::command(const MuscleCommand&c){if(c.animal>=impl_->animals.size()||c.arm>=8||!std::isfinite(c.activation)||!std::isfinite(c.curvature)||!std::isfinite(c.extension)||!std::isfinite(c.twist)||!std::isfinite(c.stiffness)||!std::isfinite(c.sweep))return false;auto v=c;v.activation=std::clamp(v.activation,0.,1.);v.curvature=std::clamp(v.curvature,-15.,15.);v.extension=std::clamp(v.extension,.55,1.50);v.twist=std::clamp(v.twist,-20.,20.);v.stiffness=std::clamp(v.stiffness,0.,1.);v.sweep=std::clamp(v.sweep,-1.,1.);impl_->animals[c.animal].arms[c.arm].target=v;return true;}
void PhysicsWorld::release_all_suckers(bool held){impl_->release_held=held;if(held)for(auto&a:impl_->animals)for(auto&ar:a.arms){ar.attached=-1;for(auto&s:ar.suckers){s.attached=-1;s.dwell=0;s.cooldown=std::max(s.cooldown,.18);}}}
bool PhysicsWorld::drag(const DragCommand&c){if(!finite(c.target)||!std::isfinite(c.stiffness)||!std::isfinite(c.max_force)||c.stiffness<0||c.max_force<0||c.stiffness>1e5||c.max_force>1e4)return false;switch(c.kind){case DragCommand::Kind::None:break;case DragCommand::Kind::Animal:if(c.index>=impl_->animals.size())return false;break;case DragCommand::Kind::ArmTip:if(c.index>=impl_->animals.size()||c.arm>=8)return false;break;case DragCommand::Kind::Rigid:if(c.index>=impl_->rigids.size())return false;break;default:return false;}impl_->drag=c;return true;}
bool PhysicsWorld::impulse_rigid(std::uint32_t index,Vec3 impulse){if(index>=impl_->rigids.size()||!finite(impulse))return false;const auto&ids=impl_->rigids[index];const Vec3 share=impulse/double(ids.size());for(int id:ids)impl_->p[id].v=impl_->p[id].v+share*impl_->p[id].inv;return true;}
bool PhysicsWorld::step(){return impl_->step();}RenderSnapshot PhysicsWorld::snapshot()const{return impl_->snapshot();}
}

