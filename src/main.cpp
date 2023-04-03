

#include "cpm/Logging.hpp"                      //->cpm_lib->include->cpm
#include "cpm/CommandLineReader.hpp"            //->cpm_lib->include->cpm
#include "cpm/init.hpp"                         //->cpm_lib->include->cpm
#include "cpm/Writer.hpp"
#include "cpm/dds/VehicleCommandTrajectoryPubSubTypes.h"
#include "VehicleStateListPubSubTypes.h"
#include "VehicleCommandTrajectoryPubSubTypes.h"
#include "cpm/Participant.hpp"
#include "cpm/HLCCommunicator.hpp"

#include <iostream>
#include <sstream>
#include <memory>
#include <mutex>
#include <thread>
#include <set>
#include <stdexcept>
#include <random>

#include <Planner/CBSPlanner.hpp>
#include <nlohmann/json.hpp>


using json = nlohmann::json;


void write_pose_with_time_information(std::string filename,std::map<uint32_t,std::vector<dynamics::data::Pose2WithTime>> actual, std::map<uint32_t,std::vector<dynamics::data::Pose2WithTime>> ref, uint64_t time_ref_ms){
    json jresult;
    jresult["time_ref_ms"] = time_ref_ms;
    
    for(auto veh: actual){
        json jpath;
        jpath["id"] = veh.first;
        for(auto pose: veh.second){
            json jnode;
            jnode["px"] = pose.pos[0];
            jnode["py"] = pose.pos[1];
            jnode["ph"] = pose.h;
            jnode["pv"] = pose.vel;
            jnode["time_ms"] = pose.time_ms;
            jpath["path"].push_back(jnode);
        }
        jresult["actual"].push_back(jpath);
    }

    for(auto veh: ref){
        json jpath;
        jpath["id"] = veh.first;
        for(auto pose: veh.second){
            json jnode;
            jnode["px"] = pose.pos[0];
            jnode["py"] = pose.pos[1];
            jnode["ph"] = pose.h;
            jnode["pv"] = pose.vel;
            jnode["time_ms"] = pose.time_ms;
            jpath["path"].push_back(jnode);
        }
        jresult["ref"].push_back(jpath);
    }

    //dump file to disc
    std::ofstream o(filename);
    o << jresult << std::endl;
    o.close();
}

bool compare_pose_time(dynamics::data::Pose2WithTime d1, dynamics::data::Pose2WithTime d2)
{
    return (d1.time_ms < d2.time_ms);
}

int main(int argc, char *argv[])
{   //////////////////Set logging details///////////////////////////////////////////////////////////
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("G2FHLC");
    int32_t loglevel = Config::getInstance().get<int32_t>({"hlc_main_log_level"});

    const std::vector<int> vehicle_ids_int = cpm::cmd_parameter_ints("vehicle_ids", {4}, argc, argv);
    std::vector<uint8_t> vehicle_ids;
    for(auto i:vehicle_ids_int)
    {
        assert(i>0);
        assert(i<255);
        vehicle_ids.push_back(i);
    }

    // const std::vector<double> vehicle_ids_int = cpm::cmd_parameter_double("vehicle_ids", {4}, argc, argv);

    ////////////////Outstream in shell which vehicles were selected/////////////////////////////////
    std::stringstream vehicle_ids_stream;
    vehicle_ids_stream << "Vehicle IDs: ";
    for (uint8_t id : vehicle_ids)
    {
        vehicle_ids_stream << static_cast<uint32_t>(id) << "|"; //Cast s.t. uint8_t is not interpreted as a character
    }
    std::string vehicle_ids_string = vehicle_ids_stream.str();

    std::cout << vehicle_ids_string << std::endl;

    ////////////// Initialization for trajectory planning /////////////////////////////////
    const uint64_t dt_nanos = 400000000ull;
    HLCCommunicator hlc_communicator(vehicle_ids);
    cpm::Writer<VehicleCommandTrajectoryPubSubType> writer_vehicleCommandTrajectory(
            hlc_communicator.getLocalParticipant()->get_participant(), 
            "vehicleCommandTrajectory");

    ////////////// Set up CBS Planner /////////////////////////////////
    CBSPlanner planner;
    planner.mp_comp.loadGraphFromDisk(Config::getInstance().get<std::string>({"mp_file"}));
    constraint_node result;

    cpm::Logging::Instance().write(loglevel,"Startup, done preparing -> ready to go");

    ////////////// Trajectory Storage /////////////////////////////////
    std::map<uint32_t,std::vector<dynamics::data::Pose2WithTime>> reference_pose;
    std::map<uint32_t,std::vector<dynamics::data::Pose2WithTime>> actual_pose;
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});

    /////////////////////////////////Trajectory planner//////////////////////////////////////////
    hlc_communicator.onFirstTimestep([&](VehicleStateList vehicle_state_list) {
        
        uint64_t t_start_plan_ns = vehicle_state_list.t_now();
        uint64_t t_now_s = t_start_plan_ns / 1000000000;
        
        cpm::Logging::Instance().write(loglevel,"[G2F] Callback onFirst Timestep at %ull", t_now_s);

           
        uint32_t index = 0;
        std::vector<dynamics::data::PoseByIndex> start_positions;
        std::vector<dynamics::data::PoseByIndex> target_positions;

        auto test_target_locations = Config::getInstance().get<std::vector<std::vector<double>>>({"test_target_locations"});
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distr(0, test_target_locations.size());

        
        for(auto vehicle_state:vehicle_state_list.state_list())
        {
        
            cpm::Logging::Instance().write(loglevel,"[G2F]Got vehicle: %u with position %lf:%lf and heading %lf", vehicle_state.vehicle_id(), vehicle_state.pose().x(),vehicle_state.pose().y(),vehicle_state.pose().yaw());

            // Compute available vehicle start positions (just for logging)
            dynamics::data::Pose2D start;
            start.pos = {vehicle_state.pose().x() * 100,vehicle_state.pose().y() * 100};
            start.h = vehicle_state.pose().yaw();
            start.vel = 0;

            // Log Start state to velocity recording
            dynamics::data::Pose2WithTime start_pose;
            start_pose = start;
            start_pose.time_ms = t_start_plan_ns / 1000000;
            actual_pose[vehicle_state.vehicle_id()].push_back(start_pose);

            // Compute representation in Discretised State Representation
            auto start_pbi = planner.findNearestPoseByIndex(start);
            for (const auto& existing_pbi : start_positions) {
                if (start_pbi.x == existing_pbi.x && start_pbi.y == existing_pbi.y) {
                    start_pbi.x += 1; //Collision Hack :)
                }
            }

            //Setup start and target position vectors
            cpm::Logging::Instance().write(loglevel,"[G2F]Vehicle %u was assigned start position %ld:%ld and heading %ld", vehicle_state.vehicle_id(), start_pbi.x,start_pbi.y,start_pbi.a);
            start_positions.push_back(start_pbi);
            
            int32_t target_index = std::min(distr(gen), static_cast<int>(test_target_locations.size() - 1));
            auto chosen_target = test_target_locations.at(target_index);
            test_target_locations.erase(test_target_locations.begin() + target_index);
            dynamics::data::PoseByIndex target_pbi = {chosen_target[0], chosen_target[1], chosen_target[2], zero_velocity_level};
            cpm::Logging::Instance().write(loglevel,"[G2F]Vehicle %u was assigned target position %ld:%ld and heading %ld", vehicle_state.vehicle_id(), target_pbi.x,target_pbi.y,target_pbi.a);
            target_positions.push_back(target_pbi);
                
            index ++;
        }

        //Execute the single shot planning
        cpm::Logging::Instance().write(loglevel,"[G2F] CBS starting planning process...");
        result = planner.cbs(start_positions, target_positions);
        
        if(!result.feasible){
            cpm::Logging::Instance().write(loglevel,"[G2F] CBS Infeasible! Terminating!");
            return;
        }

        std::string name = std::to_string(t_now_s) + ".json";
        planner.writeMultiplePathsToDisk(result, name);
        cpm::Logging::Instance().write(loglevel,"[G2F] CBS finished planning, now executing plan");

        // Log predicted path to file for later comparison
        for(auto vehicle_path: result.result){
            for(auto ref_pose_with_time: vehicle_path.second.spline ){
                reference_pose[vehicle_path.first].push_back(ref_pose_with_time);
            }
        }    
    });

    cpm::Logging::Instance().write(loglevel,"[G2F] Successfully set up planner and callbacks");

    // Some internal state flags and internal state keeping
    bool initialized = false;
    int64_t t_delay_to_start_ms = 1000; 
    bool wrote_trajectory_data_to_disc = false;
    int64_t t_ref_start_ms = 0;

    TrajectoryPoint trajectory_point;
    hlc_communicator.onEachTimestep([&](VehicleStateList vehicle_state_list) {
            
        int64_t t_now_ns = vehicle_state_list.t_now();
        int64_t t_now_ms = t_now_ns / 1000000;
        
        //First iteration is used to set internal time referece after compute
        if(!initialized){
            initialized = true;
            t_ref_start_ms = t_now_ms;
            cpm::Logging::Instance().write(loglevel,"[G2F] Initialised  at %ull, starting to send trajectories... ", t_now_ms);
            return;
        }

        // If CBS Plan is infeasible, then there is nothing to do
        if(!result.feasible){
            return;
        }
        
    
        uint32_t index = 0;
        bool all_plans_finished = true;
        for(auto vehicle_state:vehicle_state_list.state_list())
        {

            // Log true vehicle state to vector for later comparison
            dynamics::data::Pose2WithTime current_pose;
            current_pose.pos = {vehicle_state.pose().x() * 100,vehicle_state.pose().y() * 100};
            current_pose.h = vehicle_state.pose().yaw();
            current_pose.vel = vehicle_state.speed() * 100;
            current_pose.time_ms = t_now_ms - t_ref_start_ms - t_delay_to_start_ms;
            actual_pose[vehicle_state.vehicle_id()].push_back(current_pose);


            // If the plan has ended, note down for each 
            auto plan_for_vehicle = result.result[index].spline;
            if(!wrote_trajectory_data_to_disc && (t_now_ms > (static_cast<uint64_t>(plan_for_vehicle.at(plan_for_vehicle.size() - 1).time_ms) + t_ref_start_ms + t_delay_to_start_ms))){
                write_pose_with_time_information(std::to_string(t_now_ms) + "_traj.json", actual_pose, reference_pose, t_ref_start_ms + t_delay_to_start_ms);
                wrote_trajectory_data_to_disc = true;
                cpm::Logging::Instance().write(loglevel,"[G2F] Saved driving vehicle recording to disk");
            }
            
            // Send the vehicles their complete plans (testing -> dont send half, all is better with sparser points)
            std::vector<TrajectoryPoint> trajectory_points;
            for(uint32_t i = 0; i < plan_for_vehicle.size(); i ++){ 

                    auto pose = plan_for_vehicle.at(i);

                    // Transform back from internal coordinate system to Lab system
                    TrajectoryPoint trajectory_point;
                    trajectory_point.px(pose.pos[0] / 100.f);
                    trajectory_point.py(pose.pos[1] / 100.f);
                    trajectory_point.vx((pose.vel / 100.f) * cos(pose.h));
                    trajectory_point.vy((pose.vel / 100.f) * sin(pose.h));
                    trajectory_point.t().nanoseconds((pose.time_ms + t_ref_start_ms + t_delay_to_start_ms) * 1000000);
                    trajectory_points.push_back(trajectory_point);
            }

            // Send the trajecory using the inbuild functions
            VehicleCommandTrajectory vehicle_command_trajectory;
            vehicle_command_trajectory.vehicle_id(vehicle_state.vehicle_id());
            vehicle_command_trajectory.trajectory_points(trajectory_points);
            vehicle_command_trajectory.header().create_stamp().nanoseconds(t_now_ns);
            vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(t_now_ns + 1000000000ull);
            writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);

            index ++;
            cpm::Logging::Instance().write(loglevel,"[G2F] Sent plan to vehicle %u with element count %u", vehicle_state.vehicle_id(), trajectory_points.size());
        }
    });

    cpm::Logging::Instance().write(loglevel,"[G2F] Startup :D");

    hlc_communicator.start();
}
