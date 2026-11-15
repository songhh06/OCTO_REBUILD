#include "octo/world.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

double percentile(std::vector<double> values, double p) {
    std::sort(values.begin(),values.end());
    return values[std::min(values.size()-1, static_cast<std::size_t>(std::ceil(p*values.size()))-1)];
}
int main(int argc,char** argv) {
    const unsigned count = argc>1 ? static_cast<unsigned>(std::atoi(argv[1])) : 1;
    const unsigned ticks = argc>2 ? static_cast<unsigned>(std::atoi(argv[2])) : 600;
    const bool dense = argc>3 && std::string(argv[3])=="dense";
    const bool twist_load = argc>3 && std::string(argv[3])=="twist";
    const bool volume_load = argc>3 && std::string(argv[3])=="volumetric";
    const bool grounded = argc>3 && std::string(argv[3])=="grounded";
    if(count<1 || count>10 || ticks<10 || ticks>100000) return 2;
    octo::WorldConfig cfg; cfg.animals=count; cfg.spacing=dense?1.0:2.2;
    cfg.volumetric_arm_prototype=volume_load;
    cfg.grounded_rest=grounded;
    if(grounded){cfg.spawn_height=.27;cfg.upright_strength=.08;cfg.auto_suction=true;}
    octo::PhysicsWorld world(cfg);
    std::vector<double> times, snapshot_times, constraint_times, collision_times;
    double max_penetration=0,max_volume_error=0,max_shear_error=0,max_energy=0;
    std::uint64_t max_contacts=0,max_pairs=0,nonfinite=0,attachment_samples=0;
    constexpr unsigned warmup=120;
    for(unsigned tick=0;tick<ticks+warmup;++tick) {
        const double time=tick*cfg.fixed_dt;
        for(unsigned animal=0;animal<count;++animal) for(unsigned arm=0;arm<8;++arm) {
            octo::MuscleCommand cmd; cmd.animal=animal;cmd.arm=arm;
            cmd.activation=0.45+0.35*std::sin(time*2.0+arm*0.8+animal);
            cmd.curvature=2.0*std::sin(time*1.3+arm);
            cmd.extension=1.0+0.15*std::sin(time*1.7+arm*0.4);
            cmd.stiffness=0.5;
            cmd.twist=twist_load ? 3.0*std::sin(time*1.1+arm*0.6) : 0.0;
            cmd.attach=((tick/90+arm)%3==0);
            if(!world.command(cmd)) { std::cerr<<"command rejected\n"; return 3; }
        }
        const auto start=std::chrono::steady_clock::now();
        if(!world.step()) { std::cerr<<"step failed tick="<<tick<<"\n";return 4; }
        const auto stepped=std::chrono::steady_clock::now();
        const auto state=world.snapshot();
        const auto snapped=std::chrono::steady_clock::now();
        if(tick<warmup) continue;
        times.push_back(std::chrono::duration<double,std::milli>(stepped-start).count());
        snapshot_times.push_back(std::chrono::duration<double,std::milli>(snapped-stepped).count());
        constraint_times.push_back(state.diagnostics.constraints_ms);
        collision_times.push_back(state.diagnostics.collision_ms);
        max_penetration=std::max(max_penetration,state.diagnostics.max_penetration);
        max_volume_error=std::max(max_volume_error,state.diagnostics.max_volume_error);
        max_shear_error=std::max(max_shear_error,state.diagnostics.max_shear_error);
        max_energy=std::max(max_energy,state.diagnostics.kinetic_energy);
        max_contacts=std::max(max_contacts,state.diagnostics.contact_count);
        max_pairs=std::max(max_pairs,state.diagnostics.candidate_pairs);
        nonfinite+=state.diagnostics.nonfinite_count;
        attachment_samples+=state.diagnostics.attachments;
    }
    const auto mean=[](const std::vector<double>& v){return std::accumulate(v.begin(),v.end(),0.0)/v.size();};
    std::cout<<"animals,scenario,segments,substeps,iterations,warmup,measured_ticks,mean_ms,p50_ms,p95_ms,p99_ms,max_ms,snapshot_mean_ms,constraints_mean_ms,collision_mean_ms,max_penetration_m,max_volume_error,max_shear_sine,max_contacts,max_pairs,attachment_samples,max_energy_J,nonfinite\n";
    std::cout<<std::setprecision(9)<<count<<','<<(dense?"dense_active":twist_load?"spread_twist_active":volume_load?"volume_active":grounded?"grounded_active":"spread_active")<<','<<cfg.segments<<','<<cfg.substeps<<','<<cfg.iterations<<','<<warmup<<','<<ticks<<','<<mean(times)<<','<<percentile(times,.5)<<','<<percentile(times,.95)<<','<<percentile(times,.99)<<','<<*std::max_element(times.begin(),times.end())<<','<<mean(snapshot_times)<<','<<mean(constraint_times)<<','<<mean(collision_times)<<','<<max_penetration<<','<<max_volume_error<<','<<max_shear_error<<','<<max_contacts<<','<<max_pairs<<','<<attachment_samples<<','<<max_energy<<','<<nonfinite<<'\n';
    return nonfinite==0?0:5;
}
