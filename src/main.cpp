

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

    uint64_t t_start_ns = 0;
           
    /////////////////////////////////Trajectory planner//////////////////////////////////////////
    hlc_communicator.onFirstTimestep([&](VehicleStateList vehicle_state_list) {
        
        t_start_ns = vehicle_state_list.t_now();
        uint64_t t_now_s = t_start_ns / 1000000000;
        
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
                state.pose.pos = {vehicle_state.pose().x(),vehicle_state.pose().y()};
                state.pose.h = vehicle_state.pose().yaw();
                state.pose.vel = 0;
                state.pose.time_ms = 0;
                planner.vehicle_initial_states[vehicle_state.vehicle_id()] = state;
            }
    });

    planner.plan2();

    auto plan = planner.get_results_as_pose_list();

    cpm::Logging::Instance().write(
                    1,
                    "[G2F] Successfully computed routes for all vehicles..."
                    );

    hlc_communicator.onEachTimestep([&](VehicleStateList vehicle_state_list) {

            uint64_t t_now_ns = vehicle_state_list.t_now();
            uint64_t t_now_ms = t_now_ns / 1000000;
            uint64_t t_now_s = t_now_ns / 1000000000;

            cpm::Logging::Instance().write(
                    1,
                    "[G2F] Callback on Each Timestep at %ull", t_now_s
                    );
         

            for(auto vehicle_state:vehicle_state_list.state_list())
            {
            
                cpm::Logging::Instance().write(
                    1,
                    "[G2F]Got vehicle: %u with position %lf:%lf and heading %lf", vehicle_state.vehicle_id(), vehicle_state.pose().x(),vehicle_state.pose().y(),vehicle_state.pose().yaw()
                );

                bool found_start = false;
                vector<TrajectoryPoint> trajectory_points;
                for(auto pose: plan[vehicle_state.vehicle_id()]){
                    
                    if(pose.time_ms - t_now_ms < 100){ //look for the first pose within 100ms, then begin to build the trajectory
                        found_start = true;
                    }
                   
                    TrajectoryPoint trajectory_point;
                    trajectory_point.px(pose.pos[0]);
                    trajectory_point.py(pose.pos[1]);

                        
                    dynamics::data::Vector2Df v_vel = {pose.vel, 0.f};
                    Eigen::Rotation2Df m_rot_h(pose.h);
                    auto v_h = m_rot_h * v_vel;

                    trajectory_point.vx(v_h[0]);
                    trajectory_point.vy(v_h[1]);
                    
                    trajectory_point.t().nanoseconds(pose.time_ms * 1000000); 

                    trajectory_points.push_back(trajectory_point);

                }

                // Send the current trajectory 
                VehicleCommandTrajectory vehicle_command_trajectory;
                vehicle_command_trajectory.vehicle_id(vehicle_state.vehicle_id());
                vehicle_command_trajectory.trajectory_points(trajectory_points);
                vehicle_command_trajectory.header().create_stamp().nanoseconds(t_now_ns);
                vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(t_now_ns + 1000000000ull);
                writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);

            }


    });

    hlc_communicator.start();
}
