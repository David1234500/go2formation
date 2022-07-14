

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

#include <Planner/SimplePlanner.hpp>



using std::vector;

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

bool compare_pose_time(dynamics::data::Pose2DD d1, dynamics::data::Pose2DD d2)
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

    plan::SimpleRoutePlanner planner;

    cpm::Logging::Instance().write(
                    1,
                    "[G2F] STARTUP"
                    );

    /////////////////////////////////Trajectory planner//////////////////////////////////////////
    hlc_communicator.onFirstTimestep([&](VehicleStateList vehicle_state_list) {
        
        uint64_t t_start_plan_ns = vehicle_state_list.t_now();
        uint64_t t_now_s = t_start_plan_ns / 1000000000;
        
        cpm::Logging::Instance().write(
                    1,
                    "[G2F] Callback onFirst Timestep at %ull", t_now_s
                );

           

            for(auto vehicle_state:vehicle_state_list.state_list())
            {
            
                cpm::Logging::Instance().write(
                    1,
                    "[G2F]Got vehicle: %u with position %lf:%lf and heading %lf", vehicle_state.vehicle_id(), vehicle_state.pose().x(),vehicle_state.pose().y(),vehicle_state.pose().yaw()
                );
                dynamics::VehicleState state;
                state.pose.pos = {vehicle_state.pose().x() * 100,vehicle_state.pose().y() * 100};
                state.pose.h = vehicle_state.pose().yaw();
                state.pose.vel = 0;
                state.pose.time_ms = 0;

                state.target.pos = {200.f,200.f};
                state.target.vel = 0;
                state.target.h = 0;

                planner.vehicle_initial_states[vehicle_state.vehicle_id() - 1] = state;
            }

            auto config = util::Config::getInstance();
            config->key2Integer_map["vehicle_count"] = vehicle_state_list.state_list().size();

            cpm::Logging::Instance().write(
                    1,
                    "[G2F] Planner plan"
                    );

            planner.plan2();



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
    uint64_t t_ref_start_ms = 0;
    uint64_t t_delay_to_start_ms = 1000; // 1sec to start
    bool sent_data = false;

    hlc_communicator.onEachTimestep([&](VehicleStateList vehicle_state_list) {

            uint64_t t_now_ns = vehicle_state_list.t_now();
            uint64_t t_now_ms = t_now_ns / 1000000;
            
            auto config = util::Config::getInstance();
            uint32_t timestep_ms =  config->getIntegerByKey("timestep");

            if(!initialized){
                initialized = true;
                t_ref_start_ms = t_now_ms;
            }

            auto plan = planner.get_results_as_pose_list();

            cpm::Logging::Instance().write(
                    1,
                    "[G2F] Callback on Each Timestep at %ull", t_now_ms);
        
            for(auto vehicle_state:vehicle_state_list.state_list())
            {

                cpm::Logging::Instance().write(
                    3,
                    "[G2F]Got vehicle: %u with position %lf:%lf and heading %lf", vehicle_state.vehicle_id(), vehicle_state.pose().x(),vehicle_state.pose().y(),vehicle_state.pose().yaw()
                );

                bool found_start = false;
                vector<TrajectoryPoint> trajectory_points;
                sort(plan[vehicle_state.vehicle_id() - 1].begin(),plan[vehicle_state.vehicle_id() - 1].end(),compare_pose_time);
                
                for(auto pose: plan[vehicle_state.vehicle_id() - 1]){ 

                        TrajectoryPoint trajectory_point;
                        trajectory_point.px(pose.pos[0] / 100);
                        trajectory_point.py(pose.pos[1] / 100);

                        dynamics::data::Vector2Df v_vel = {(-pose.vel / 100.f), 0.f}; //cm/s -> m/s
                        Eigen::Rotation2Df m_rot_h(pose.h );
                        auto v_h = m_rot_h * v_vel;

                        // trajectory_point.vx(v_h[0]);
                        // trajectory_point.vy(v_h[1]);

                        // trajectory_point.vx(((-pose.vel / 100.f) * static_cast<float>(timestep_ms)/1000.f) * cos(pose.h));
                        // trajectory_point.vy(((-pose.vel / 100.f) * static_cast<float>(timestep_ms)/1000.f) * sin(pose.h));

                        cpm::Logging::Instance().write(
                        3,
                        "[G2F] Plan vehicle %u  %lf:%lf v%lf h%lf", vehicle_state.vehicle_id(), pose.pos[0],pose.pos[1],pose.vel, pose.h
                        );

                        cpm::Logging::Instance().write(
                        3,
                        "[G2F] Plan vel vec %u  %lf:%lf", vehicle_state.vehicle_id(), v_h[0], v_h[1]
                        );
                        
                        trajectory_point.t().nanoseconds((pose.time_ms + t_ref_start_ms + t_delay_to_start_ms) * 1000000); //millisec to nanosec
                        cpm::Logging::Instance().write(
                        3,
                        "[G2F] Planning for %u time ms %llu", vehicle_state.vehicle_id(), (pose.time_ms + t_ref_start_ms + t_delay_to_start_ms) * 1000000
                        ); 
                        trajectory_points.push_back(trajectory_point);
                   

                }

                // Send the current trajectory 
                VehicleCommandTrajectory vehicle_command_trajectory;
                vehicle_command_trajectory.vehicle_id(vehicle_state.vehicle_id());
                vehicle_command_trajectory.trajectory_points(trajectory_points);
                vehicle_command_trajectory.header().create_stamp().nanoseconds(t_now_ns);
                vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(t_now_ns + 1000000000ull);
                writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);

                cpm::Logging::Instance().write(
                    3,
                    "[G2F] Sent plan to vehicle %u with element count %u", vehicle_state.vehicle_id(), trajectory_points.size()
                    );

            }


    });

    cpm::Logging::Instance().write(
                    1,
                    "[G2F] start hlc communicator"
                    );

    hlc_communicator.start();
}
