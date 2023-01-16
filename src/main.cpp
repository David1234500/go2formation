

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

#include <Planner/CBSPlanner.hpp>
#include <nlohmann/json.hpp>


using std::vector;
using json = nlohmann::json;

//Description for bash files
/**
 * \defgroup central_routing_files Additional Files
 * \ingroup central_routing
 */

/**
 * \page central_routing_files_page Additional Files for Central Routing
 * \subpage central_routing_build <br>
 * \subpage central_routing_run <br>
 * \ingroup central_routing_files
*/

/**
 * \page central_routing_build build.bash
 * \brief Build script for central_routing
 * \ingroup central_routing_files
 */

/**
 * \page central_routing_run run.bash
 * \brief Run script for central_routing
 * \ingroup central_routing_files
 */

/**
 * \brief Main function of the central_routing scenario
 * This tutorial is also described at https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Central+Routing+Example
 * \ingroup central_routing
 */

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
    cpm::Logging::Instance().set_id("central_routing");
    ////////////////Set vehicle IDs for the vehicles selected in the command line or the LCC////////
    const std::vector<int> vehicle_ids_int = cpm::cmd_parameter_ints("vehicle_ids", {4}, argc, argv);
    std::vector<uint8_t> vehicle_ids;
    for(auto i:vehicle_ids_int)
    {
        assert(i>0);
        assert(i<255);
        vehicle_ids.push_back(i);
    }

    ////////////////Outstream in shell which vehicles were selected/////////////////////////////////
    std::stringstream vehicle_ids_stream;
    vehicle_ids_stream << "Vehicle IDs: ";
    for (uint8_t id : vehicle_ids)
    {
        vehicle_ids_stream << static_cast<uint32_t>(id) << "|"; //Cast s.t. uint8_t is not interpreted as a character
    }
    std::string vehicle_ids_string = vehicle_ids_stream.str();

    std::cout << vehicle_ids_string << std::endl;

    //////////////Initialization for trajectory planning/////////////////////////////////
    // Definition of a timesegment in nano seconds and a trajecotry planner for more than one vehicle
    const uint64_t dt_nanos = 400000000ull;
    // MultiVehicleTrajectoryPlanner planner(dt_nanos);
    HLCCommunicator hlc_communicator(vehicle_ids);
    ///////////// writer and reader for sending trajectory commands////////////////////////
    //the writer will write data for the trajectory for the position of the vehicle (x,y) and the speed for each direction vecotr (vx,vy) and the vehicle ID
    cpm::Writer<VehicleCommandTrajectoryPubSubType> writer_vehicleCommandTrajectory(
            hlc_communicator.getLocalParticipant()->get_participant(), 
            "vehicleCommandTrajectory");

    CBSPlanner planner;
    planner.m_proxGraph.loadGraphFromDisk("/home/docker/dev/software/go2formation/build/NH-ICBS-HLC/proxy_state_graph.json");

    constraint_node result;

    cpm::Logging::Instance().write(
                    1,
                    "[G2F] STARTUP"
                    );

    
    std::map<uint32_t,std::vector<dynamics::data::Pose2WithTime>> reference_pose;
    std::map<uint32_t,std::vector<dynamics::data::Pose2WithTime>> actual_pose;

    /////////////////////////////////Trajectory planner//////////////////////////////////////////
    hlc_communicator.onFirstTimestep([&](VehicleStateList vehicle_state_list) {
        
        uint64_t t_start_plan_ns = vehicle_state_list.t_now();
        uint64_t t_now_s = t_start_plan_ns / 1000000000;
        
        cpm::Logging::Instance().write(
                    1,
                    "[G2F] Callback onFirst Timestep at %ull", t_now_s
                );

           
            uint32_t index = 0;
            std::vector<dynamics::data::PoseByIndex> start_positions;
            std::vector<dynamics::data::PoseByIndex> target_positions;
            for(auto vehicle_state:vehicle_state_list.state_list())
            {
            
             

                cpm::Logging::Instance().write(
                    1,
                    "[G2F]Got vehicle: %u with position %lf:%lf and heading %lf", vehicle_state.vehicle_id(), vehicle_state.pose().x(),vehicle_state.pose().y(),vehicle_state.pose().yaw()
                );
                dynamics::data::Pose2D start;
                start.pos = {vehicle_state.pose().x() * 100,vehicle_state.pose().y() * 100};
                start.h = vehicle_state.pose().yaw();
                start.vel = 0;

                // Log actual state to vector
                dynamics::data::Pose2WithTime start_pose;
                start_pose = start;
                start_pose.time_ms = t_start_plan_ns / 1000000;
                actual_pose[vehicle_state.vehicle_id()].push_back(start_pose);

                cpm::Logging::Instance().write(
                    1,
                    "[G2F]Got vehicle: %u with position %lf:%lf and heading %lf", vehicle_state.vehicle_id(), vehicle_state.pose().x(),vehicle_state.pose().y(),vehicle_state.pose().yaw()
                );
    
                auto start_pbi = planner.findNearestPoseByIndex(start);
                start_positions.push_back(start_pbi);
                std::cout << "START " << start_pbi.x << ":" << start_pbi.y << ":" << start_pbi.a << ":" << start_pbi.s << std::endl; 
                dynamics::data::PoseByIndex target_pbi = {6 + 20 * index,6,6,zero_velocity_level}; 
                target_positions.push_back(target_pbi);
                 std::cout << "TARGET " << target_pbi.x << ":" << target_pbi.y << ":" << target_pbi.a << ":" << target_pbi.s << std::endl; 
                index ++;
            }

            
            cpm::Logging::Instance().write(
                    1,
                    "[G2F] Planner plan"
                    );

            
            result = planner.cbs(start_positions, target_positions);
            std::string name = std::to_string(t_now_s) + ".json";
            //planner.writeMultiplePathsToDisk(result, name);

            //Log target/reference to file
            for(auto vehicle_path: result.result){
                for(auto ref_pose_with_time: vehicle_path.second.interprimitive){
                    
                    reference_pose[vehicle_path.first].push_back(ref_pose_with_time);
                    // cpm::Logging::Instance().write(1,"[G2F] Successfully set up planner and callbacks");
                
                }
            }    

             cpm::Logging::Instance().write(
                    1,
                    "[G2F] Planner done"
                    );
    });

    

    cpm::Logging::Instance().write(
                    1,
                    "[G2F] Successfully set up planner and callbacks"
                    );

    bool initialized = false;
    bool initialized_start_pose = false;
    int64_t t_ref_start_ms = 0;
    int64_t t_delay_to_start_ms = 2000; // 1sec to start
    bool sent_data = false;
    bool wrote_trajectory_data_to_disc = false;

    TrajectoryPoint trajectory_point;
    hlc_communicator.onEachTimestep([&](VehicleStateList vehicle_state_list) {

            int64_t t_now_ns = vehicle_state_list.t_now();
            int64_t t_now_ms = t_now_ns / 1000000;
            
            if(!initialized){
                initialized = true;
                t_ref_start_ms = t_now_ms;
                return;
            }

            // cpm::Logging::Instance().write(
            //         3,
            //         "[G2F] Callback on Each Timestep at %ull", t_now_ms);
        
            uint32_t index = 0;
            bool all_plans_finished = true;
            for(auto vehicle_state:vehicle_state_list.state_list())
            {

                // Log actual state to vector
                dynamics::data::Pose2D current_state;
                current_state.pos = {vehicle_state.pose().x() * 100,vehicle_state.pose().y() * 100};
                current_state.h = vehicle_state.pose().yaw();
                current_state.vel = vehicle_state.speed() * 100;

                dynamics::data::Pose2WithTime current_pose;
                current_pose = current_state;
                current_pose.time_ms = t_now_ms - t_ref_start_ms - t_delay_to_start_ms;
                actual_pose[vehicle_state.vehicle_id()].push_back(current_pose);
                


                auto plan_for_vehicle = result.result[index].spline;

                
                // cpm::Logging::Instance().write(
                //     1,
                //     "[G2F]Now: %s End: %s ", std::to_string(t_now_ms).c_str(), std::to_string(static_cast<uint64_t>(plan_for_vehicle.at(plan_for_vehicle.size() - 1).time_ms) + t_ref_start_ms + t_delay_to_start_ms).c_str()
                // );

                //  cpm::Logging::Instance().write(
                //     1,
                //     "[G2F]Plan: %s Ref: %s Delay: %s", std::to_string(static_cast<uint64_t>(plan_for_vehicle.at(plan_for_vehicle.size() - 1).time_ms)).c_str(), std::to_string(t_ref_start_ms).c_str(), std::to_string(t_delay_to_start_ms).c_str()
                // );

                vector<TrajectoryPoint> trajectory_points;
                if((t_now_ms > (static_cast<uint64_t>(plan_for_vehicle.at(plan_for_vehicle.size() - 1).time_ms) + t_ref_start_ms + t_delay_to_start_ms))){
                    
                    write_pose_with_time_information(std::to_string(t_now_ms) + "_traj.json", actual_pose, reference_pose, t_ref_start_ms + t_delay_to_start_ms);
                    wrote_trajectory_data_to_disc = true;
                
                    cpm::Logging::Instance().write(
                    1,
                    "[G2F]Plan: WROTE TO DISC"
                );

                    exit(0);

                }
                

                uint32_t start_index = 0;
                float min_dist = 1000000.f;
                for(uint32_t i = 0; i < plan_for_vehicle.size(); i ++){ 
                    dynamics::data::Vector2Df current_pos = {vehicle_state.pose().x() * 100,vehicle_state.pose().y() * 100};

                    if((current_pos - plan_for_vehicle.at(i).pos).norm() < min_dist){
                        min_dist = (current_pos - plan_for_vehicle.at(i).pos).norm();
                        start_index = i;
                    }

                }

                for(uint32_t i = start_index; i < plan_for_vehicle.size() && i < start_index + 40; i ++){ 

                        auto pose = plan_for_vehicle.at(i);

                        TrajectoryPoint trajectory_point;
                        trajectory_point.px(pose.pos[0] / 100.f);
                        trajectory_point.py(pose.pos[1] / 100.f);

                        trajectory_point.vx((pose.vel / 100.f) * cos(pose.h));
                        trajectory_point.vy((pose.vel / 100.f) * sin(pose.h));

                        // cpm::Logging::Instance().write(
                        // 3,
                        // "[G2F] Plan vehicle %u  %lf:%lf v%lf h%lf", vehicle_state.vehicle_id(), pose.pos[0],pose.pos[1],pose.vel, pose.h
                        // );
                        
                        uint32_t pose_time_ms = static_cast<uint32_t>(pose.time_ms); // 50 * trajectory_points.size();
                        trajectory_point.t().nanoseconds((pose_time_ms + t_ref_start_ms + t_delay_to_start_ms) * 1000000); //millisec to nanosec
                        // cpm::Logging::Instance().write(
                        // 3,
                        // "[G2F] Planning for %u time ms %llu", vehicle_state.vehicle_id(), (pose_time_ms + t_ref_start_ms + t_delay_to_start_ms) * 1000000
                        // ); 
                        trajectory_points.push_back(trajectory_point);
                   

                }

                // Send the current trajectory 
                VehicleCommandTrajectory vehicle_command_trajectory;
                vehicle_command_trajectory.vehicle_id(vehicle_state.vehicle_id());
                vehicle_command_trajectory.trajectory_points(trajectory_points);
                vehicle_command_trajectory.header().create_stamp().nanoseconds(t_now_ns);
                vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(t_now_ns + 1000000000ull);
                writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);

                index ++;
                // cpm::Logging::Instance().write(
                //     3,
                //     "[G2F] Sent plan to vehicle %u with element count %u", vehicle_state.vehicle_id(), trajectory_points.size()
                //     );

                if(all_plans_finished && !wrote_trajectory_data_to_disc){
                   
                    
                }   

            }


    });

    cpm::Logging::Instance().write(
                    1,
                    "[G2F] start hlc communicator"
                    );

    hlc_communicator.start();
}
