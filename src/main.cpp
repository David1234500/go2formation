

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

#include <planner/SimplePlanner.hpp>



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
                state.pose.time = 0;
                planner.vehicle_initial_states[vehicle_state.vehicle_id] = state;
            }
    });

    planner.plan2();

    

    cpm::Logging::Instance().write(
                    1,
                    "[G2F] Successfully computed routes"
                    );

    hlc_communicator.onEachTimestep([&](VehicleStateList vehicle_state_list) {

            uint64_t t_now_ns = vehicle_state_list.t_now();
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
               


            }


    });

    hlc_communicator.start();
}
