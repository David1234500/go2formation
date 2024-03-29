

#include "cpm/Logging.hpp"                      //->cpm_lib->include->cpm
#include "cpm/CommandLineReader.hpp"            //->cpm_lib->include->cpm
#include "cpm/init.hpp"                         //->cpm_lib->include->cpm
#include "cpm/Writer.hpp"
#include "cpm/dds/VehicleCommandTrajectoryPubSubTypes.h"
#include "cpm/dds/VehicleObservationPubSubTypes.h"
#include "VehicleStateListPubSubTypes.h"
#include "VehicleCommandTrajectoryPubSubTypes.h"
#include "cpm/Participant.hpp"
#include "cpm/HLCCommunicator.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include "cpm/Timer.hpp"                        

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

void appendPosesToFile(const dynamics::data::PoseByIndex& startPose, const dynamics::data::PoseByIndex& targetPose, const std::string& filename) {
    json jsonPoseArray;

    // Read existing data from the file
    std::ifstream inputFile(filename);
    if (inputFile.good()) {
        inputFile >> jsonPoseArray;
        inputFile.close();
    }

    // Serialize startPose and targetPose
    json jsonStartPose = {{"x", startPose.x}, {"y", startPose.y}, {"a", startPose.a}, {"s", startPose.s}, {"t", startPose.t}};
    json jsonTargetPose = {{"x", targetPose.x}, {"y", targetPose.y}, {"a", targetPose.a}, {"s", targetPose.s}, {"t", targetPose.t}};
    json comb = {{"start", jsonStartPose}, {"target", jsonTargetPose}};

    // Append the poses to the JSON array
    jsonPoseArray.push_back(jsonStartPose);

    // Write the JSON array back to the file
    std::ofstream outputFile(filename, std::ios::trunc);
    outputFile << jsonPoseArray.dump(4); // 4 spaces for indentation
    outputFile.close();
}

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
    const std::string config = cpm::cmd_parameter_string("config", "", argc, argv);
    cpm::Logging::Instance().set_id("G2FHLC");
    int32_t loglevel = Config::getInstance(config).get<int32_t>({"hlc_main_log_level"});

    const std::vector<int> vehicle_ids_int = cpm::cmd_parameter_ints("vehicle_ids", {4}, argc, argv);
    std::vector<uint8_t> vehicle_ids;
    for(auto i:vehicle_ids_int)
    {
        assert(i>0);
        assert(i<255);
        vehicle_ids.push_back(i);
    }

    const std::string vehicle_poses_json = cpm::cmd_parameter_string("poses", "", argc, argv);

    ////////////// Initialization for trajectory planning /////////////////////////////////
    cpm::Writer<VehicleCommandTrajectoryPubSubType> writer_vehicleCommandTrajectory(
            "vehicleCommandTrajectory");
    cpm::MultiVehicleReader<VehicleObservationPubSubType> ips_reader("vehicleObservation", vehicle_ids);

    // std::this_thread::sleep_for(std::chrono::seconds(3));

    ////////////// Set up CBS Planner /////////////////////////////////
    CBSPlanner planner;
    planner.mp_comp.loadGraphFromDisk(Config::getInstance().get<std::string>({"mp_state_graph"}));
    constraint_node result;


    ////////////// Parse Target Positions from the Commandline /////////////////////////////////
    std::map<uint32_t,dynamics::data::PoseByIndex> start_positions;
    std::map<uint32_t,dynamics::data::Pose2D> start_poses;
    std::map<uint32_t,dynamics::data::PoseByIndex> target_positions;
    std::map<uint32_t,dynamics::data::Pose2D> target_poses;


    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    bool send_off_map_edge = Config::getInstance().get<bool>({"test_yeet"});
    bool use_example_targets = true;

    if(vehicle_poses_json != ""){
        std::cout << "[G2F] Got target positions from the commandline. " << vehicle_poses_json << std::endl;
        auto pose_information = json::parse(vehicle_poses_json);
        for(auto pose: pose_information){
            dynamics::data::Pose2D target;
            std::cout << "[G2F] Got target pose " << pose << ":" << pose["yaw"] << ":" << pose["x"] << ":" << pose["y"]<< std::endl;
            
            target.h = pose["yaw"].get<float>();
            target.pos[0] = pose["x"].get<float>() * 100;
            target.pos[1] = pose["y"].get<float>() * 100;
            target.vel = 0;
            target_poses[pose["id"]] = target;

            cpm::Logging::Instance().write(loglevel,"[G2F]Got vehicle with position %lf:%lf and heading %lf", pose["x"],pose["y"], pose["yaw"]);
            auto target_tbi = planner.findNearestPoseByIndex(target);
            cpm::Logging::Instance().write(loglevel,"[G2F]Vehicle was assigned target position %ld:%ld and heading %ld", target_tbi.x,target_tbi.y,target_tbi.a);
            
            target_positions[pose["id"]] = target_tbi;
        }
        use_example_targets = false;
    }

    cpm::Logging::Instance().write(loglevel,"Startup, done preparing -> ready to go");

    ////////////// Trajectory Storage /////////////////////////////////
    
    std::map<uint32_t,std::vector<dynamics::data::Pose2WithTime>> reference_pose;
    std::map<uint32_t,std::vector<dynamics::data::Pose2WithTime>> actual_pose;
    
    uint64_t completion_time_ms = 0;

    auto now = std::chrono::system_clock::now();
    uint64_t t_now = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

    /////////////////////////////////Trajectory planner//////////////////////////////////////////
    
    std::map<uint8_t, VehicleObservation> ips_sample;
    std::map<uint8_t, uint64_t> ips_sample_age;
    ips_reader.get_samples(t_now, ips_sample, ips_sample_age);
    //check for vehicles if online
    bool all_vehicles_online = true;
    for (auto e : ips_sample_age) {
        std::cout << "Waiting for " << int(e.first) << std::endl;
        if (e.second > 1000000000ull) {
            
            auto data = ips_sample.at(e.first);
            auto new_id = data.vehicle_id();
            std::cout << "Waiting for r " << int(new_id) << std::endl;
            if(new_id != 0){
                all_vehicles_online = false;
            }
        }
    }

    if (!all_vehicles_online) {
        cpm::Logging::Instance().write(
            3,
            "Waiting for %s ...",
            "vehicles"
        );
        return -1;
    }
    
    cpm::Logging::Instance().write(loglevel,"[G2F] Callback onFirst Timestep at %ull", (t_now / 1000000000));

    uint32_t index = 0;
    auto test_target_locations = Config::getInstance().get<std::vector<std::vector<double>>>({"test_target_locations"});
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(0, test_target_locations.size());

    
    for (auto e : ips_sample) {
        auto data = e.second;
        auto new_id = data.vehicle_id();
        auto new_pose = data.pose();
        cpm::Logging::Instance().write(loglevel,"[G2F]Got vehicle: %u with position %lf:%lf and heading %lf", new_id, new_pose.x(),new_pose.y(),new_pose.yaw());

        // Compute available vehicle start positions (just for logging)
        dynamics::data::Pose2D start;
        start.pos = {new_pose.x() * 100.f,new_pose.y() * 100.f};
        start.h = new_pose.yaw();
        start.vel = 0;

        // Log Start state to velocity recording
        dynamics::data::Pose2WithTime start_pose;
        start_pose = start;
        start_pose.time_ms = t_now / 1000000;
        actual_pose[new_id].push_back(start_pose);
        start_poses[new_id] = start;

        // Compute representation in Discretised State Representation
        auto start_pbi = planner.findNearestPoseByIndex(start);
        for (const auto& existing_pbi : start_positions) {
            if (start_pbi.x == existing_pbi.second.x && start_pbi.y == existing_pbi.second.y) {
                start_pbi.x += 1; //Collision Hack :)
            }
        }

        //Setup start positions vectors 
        cpm::Logging::Instance().write(loglevel,"[G2F]Vehicle %u was assigned start position %ld:%ld and heading %ld", new_id, start_pbi.x,start_pbi.y,start_pbi.a);
        start_positions[new_id] = start_pbi;
        
        //Setup target positions vector -> if no positions have been set, then use demo ones
        dynamics::data::PoseByIndex target_pbi;
        if(send_off_map_edge){
            auto target_off_map = Config::getInstance().get<std::vector<double>>({"test_target_locations_drive_off"});
            target_pbi = {target_off_map[0], target_off_map[1], target_off_map[2], 2};
            target_positions[new_id] = target_pbi;

        }else if(use_example_targets){
            int32_t target_index = std::min(distr(gen), static_cast<int>(test_target_locations.size() - 1));
            auto chosen_target = test_target_locations.at(target_index);
            test_target_locations.erase(test_target_locations.begin() + target_index);
            target_pbi = {chosen_target[0], chosen_target[1], chosen_target[2], 2};
            cpm::Logging::Instance().write(loglevel,"[G2F]Vehicle %u was assigned target position %ld:%ld and heading %ld", new_id, target_pbi.x,target_pbi.y,target_pbi.a);
            target_positions[new_id] = target_pbi;
            target_poses[new_id] = planner.indexToPose(target_pbi);
        }

        appendPosesToFile(start_pbi, target_pbi, "../pose_data.json");
        index ++;
    }

    //Execute the single shot planning
    cpm::Logging::Instance().write(loglevel,"[G2F] CBS starting planning process...");
    std::vector<dynamics::data::PoseByIndex> start;
    std::vector<dynamics::data::PoseByIndex> target;
    std::map<uint32_t,uint32_t> results_map;
    uint32_t index2 = 0;
    for(auto start_pose: start_positions){
        auto id = start_pose.first;
        start.push_back(start_positions[id]);
        target.push_back(target_positions[id]);
        results_map[index2] = id;
        index2 += 1;
    }
    result = planner.cbs(start, target, false, send_off_map_edge);

    if(!result.feasible){
        result = planner.cbs(start, target, true, send_off_map_edge);
    }

    if(!result.feasible){
        cpm::Logging::Instance().write(loglevel,"[G2F] CBS Infeasible! Terminating!");
        return -1;
    }

    cpm::Logging::Instance().write(loglevel,"[G2F] CBS finished planning, now executing plan");

    // Log predicted path to file for later comparison

    std::map<int32_t, std::vector<dynamics::data::Pose2WithTime>> corrected_result;
    for(auto& vehicle_path: result.result){
        int64_t last_pose_time_ms = static_cast<int64_t>(vehicle_path.second.interprimitive.at(vehicle_path.second.interprimitive.size() - 1).time_ms);
        completion_time_ms = (last_pose_time_ms > completion_time_ms ? last_pose_time_ms : completion_time_ms);

        for(auto ref_pose_with_time: vehicle_path.second.interprimitive ){
            reference_pose[vehicle_path.first].push_back(ref_pose_with_time);
        }

    
    }    

    // Some internal state flags and internal state keeping
    bool initialized = false;
    int64_t t_delay_to_start_ms = 3000; 
    bool wrote_trajectory_data_to_disc = false;
    int64_t t_ref_start_ms = 0;

    TrajectoryPoint trajectory_point;
    const uint64_t dt_nanos = 500000000ull;
    auto timer = cpm::Timer::create("G2FHLC", dt_nanos, 0, false, true, false);
    timer->start([&](uint64_t t_now_ns) {
            
        
        int64_t t_chrono_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        int64_t t_now_ms = t_now_ns / 1000000;
        
        cpm::Logging::Instance().write(loglevel,"[G2F] Clock Delay %lld", (long long) (t_chrono_now_ns - t_now_ns));
        // This HLC takes forever so we skip all the old vehiclestates and get back to a somewhat recent state 
        if(std::abs(t_chrono_now_ns - static_cast<int64_t>(t_now_ns)) > 110000000 ){
            cpm::Logging::Instance().write(loglevel,"[G2F] Catching up in time %lld, starting to send trajectories... ", (long long) (t_chrono_now_ns - t_now_ns));
            return;
        }

        //First iteration is used to set internal time referece after compute
        if(!initialized){
            initialized = true;
            t_ref_start_ms = t_now_ms;
            cpm::Logging::Instance().write(loglevel,"[G2F] Initialised  at %ull, starting to send trajectories... ", t_now_ms);
            return;
        }
    
        uint32_t index = 0;
        bool all_plans_finished = true;
         std::map<uint8_t, VehicleObservation> ips_sample;
        std::map<uint8_t, uint64_t> ips_sample_age;
        ips_reader.get_samples(t_now, ips_sample, ips_sample_age);
        //check for vehicles if online
        bool all_vehicles_online = true;
        for (auto e : ips_sample) {
                auto data = e.second;
                auto new_id = data.vehicle_id();
                auto new_pose = data.pose();

                // Log true vehicle state to vector for later comparison
                dynamics::data::Pose2WithTime current_pose;
                current_pose.pos = {new_pose.x() * 100,new_pose.y() * 100};
                current_pose.h = new_pose.yaw();
                current_pose.time_ms = t_now_ms - t_ref_start_ms - t_delay_to_start_ms;
                actual_pose[new_id].push_back(current_pose);

                double threshold = 20.0; 
                auto plan_for_vehicle = result.result[index].interprimitive;

                // Find the closest point in the plan for the current vehicle state
                double min_distance = std::numeric_limits<double>::max();
                dynamics::data::Pose2WithTime closest_pose;
                for (const auto& pose : plan_for_vehicle) {
                    double distance = (current_pose.pos - pose.pos).norm();
                    if (distance < min_distance) {
                        min_distance = distance;
                        closest_pose = pose;
                    }
                }

                // Check if the vehicle is diverging more than the threshold from the plan
                if (min_distance > threshold) {
                    cpm::Logging::Instance().write(
                        loglevel, "[G2F] Vehicle %u is diverging from the plan by %f units",
                        new_id, min_distance
                    );

                    // TODO potentially recompute trajectory and try again
                }


                // If the plan has ended, note down for each 
                if(!wrote_trajectory_data_to_disc && (t_now_ms > (completion_time_ms + t_ref_start_ms + t_delay_to_start_ms))){
                    write_pose_with_time_information(std::to_string(t_now_ms) + "_traj.json", actual_pose, reference_pose, t_ref_start_ms + t_delay_to_start_ms);
                    wrote_trajectory_data_to_disc = true;
                    cpm::Logging::Instance().write(loglevel,"[G2F] Saved driving vehicle recording to disk");
                    return;
                }
                
                std::vector<TrajectoryPoint> trajectory_points;
                TrajectoryPoint trajectory_point;
                trajectory_point.px(start_poses[new_id].pos[0] / 100.f);
                trajectory_point.py(start_poses[new_id].pos[1] / 100.f);
                trajectory_point.vx((plan_for_vehicle.front().vel / 100.f) * cos(plan_for_vehicle.front().h));
                trajectory_point.vy((plan_for_vehicle.front().vel / 100.f) * sin(plan_for_vehicle.front().h));
                trajectory_point.t().nanoseconds((t_ref_start_ms + t_delay_to_start_ms) * 1000000);
                trajectory_points.push_back(trajectory_point);

                // Send the vehicles their complete plans (testing -> dont send half, all is better with sparser points)
                // std::vector<TrajectoryPoint> trajectory_points;
                uint32_t step_size = 7;
                for(uint32_t i = 0; i < plan_for_vehicle.size(); i += step_size){ 

                        auto pose = plan_for_vehicle.at(i);

                        // Transform back from internal coordinate system to Lab system
                        TrajectoryPoint trajectory_point;
                        
                        trajectory_point.px(pose.pos[0] / 100.f);
                        trajectory_point.py(pose.pos[1] / 100.f);
                        
                        trajectory_point.vx((pose.vel / 100.f) * cos(pose.h));
                        trajectory_point.vy((pose.vel / 100.f) * sin(pose.h));

                        int64_t pose_time = static_cast<int64_t>(pose.time_ms);
                        trajectory_point.t().nanoseconds((pose_time + t_ref_start_ms + t_delay_to_start_ms) * 1000000);
                        trajectory_points.push_back(trajectory_point);
                }

                // Add precise final target
                TrajectoryPoint trajectory_point2;
                trajectory_point2.px(target_poses[new_id].pos[0] / 100.f);
                trajectory_point2.py(target_poses[new_id].pos[1] / 100.f);
                cpm::Logging::Instance().write(loglevel,"[G2F] Adding target pose %lf:%lf", target_poses[new_id].pos[0] / 100.f, target_poses[new_id].pos[1] / 100.f);
                trajectory_point.vx((plan_for_vehicle.back().vel / 100.f) * cos(plan_for_vehicle.back().h));
                trajectory_point.vy((plan_for_vehicle.back().vel / 100.f) * sin(plan_for_vehicle.back().h));
                // cpm::Logging::Instance().write(loglevel,"[G2F] Adding target pose %lf:%lf", (plan_for_vehicle.back().vel / 100.f) * cos(plan_for_vehicle.back().h), (plan_for_vehicle.back().vel / 100.f) * sin(plan_for_vehicle.back().h));
                trajectory_point2.t().nanoseconds((static_cast<int64_t>(plan_for_vehicle.back().time_ms)  + t_ref_start_ms + t_delay_to_start_ms) * 1000000);
                trajectory_points.push_back(trajectory_point2);


                // Send the trajecory using the inbuild functions
                VehicleCommandTrajectory vehicle_command_trajectory;
                vehicle_command_trajectory.vehicle_id(new_id);
                vehicle_command_trajectory.trajectory_points(trajectory_points);
                vehicle_command_trajectory.header().create_stamp().nanoseconds(t_now_ns);
                vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(t_now_ns + 1000000000ull);
                writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);

                index ++;
                cpm::Logging::Instance().write(loglevel,"[G2F] Sent plan to vehicle %u with element count %u", new_id, trajectory_points.size());
            }
        
    });

    cpm::Logging::Instance().write(loglevel,"[G2F] Startup :D");
}