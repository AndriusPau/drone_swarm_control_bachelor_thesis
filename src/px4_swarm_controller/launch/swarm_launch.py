#!/usr/bin/env python3

import json
import launch
import launch_ros.actions
import launch.substitutions
from ament_index_python.packages import get_package_share_directory
import os

package = 'px4_swarm_controller'

# Supported drone models: iris plane standard_vtol rover r1_rover typhoon_h480
# Comments for swarm_config_1.json
# model is the drone model name
# model_counts counts how many drones exist of each drone model
# script is sent to the Gazebo simulation to create the initial drones
def parse_swarm_config(config_file):
    # Extract unique models and their counts
    model_counts = {}
    swarm_config = config_file["swarm"]
    for key, item in swarm_config.items():
        model = item["model"]                                   # model = "iris"
        model_counts[model] = model_counts.get(model, 0) + 1    # model_counts["iris"] = 3
    script = ""
    for key, item in model_counts.items():
        script += key + ":" + str(item) + ","   # script = "iris:3"
    script = script[:-1]
    # Extract all initial poses
    initial_poses_string = "\""
    initial_poses_dict = dict()
    destination_poses_string = "\""
    destination_poses_dict = dict()
    for idx, item in enumerate(swarm_config.values()):
        initial_pose = item["initial_pose"]
        initial_poses_dict["px4_" + str(idx + 1)] = initial_pose
        initial_poses_string += str(initial_pose["x"]) + "," + str(initial_pose["y"]) + "|"
        destination_pose = item["destination_pose"]
        destination_poses_dict["px4_" + str(idx + 1)] = destination_pose
        destination_poses_string += str(destination_pose["x"]) + "," + str(destination_pose["y"]) + "|"
    initial_poses_string = initial_poses_string[:-1] + "\""
    destination_poses_string = destination_poses_string[:-1] + "\""
    # initial_poses_string = "\"0.0,1.0|1.0,0.0|3.0,0.0\""
    # initial_poses_dict = {'px4_1':{'x':0.0,'y':1.0},'px4_2':{'x':1.0,'y':0.0},'px4_3':{'x':3.0,'y':0.0}}

    # return drone_count, drone_model_count, initial_coords_string, initial_coords_dict
    return len(swarm_config), script, initial_poses_string, initial_poses_dict, destination_poses_dict
    


def generate_launch_description():
    ld = launch.LaunchDescription()
    package_dir = get_package_share_directory(package)
    
    # Swarm information
    with open(os.path.join(package_dir, 'config', 'swarm_config_5.json'), 'r') as swarm_file:
        swarm_config_data = json.load(swarm_file)
    nb_drones, script, initial_poses, initial_poses_dict, destination_poses_dict = parse_swarm_config(swarm_config_data)
    swarm_file.close()

    ld.add_action(
        launch_ros.actions.Node(
            package=package,
            executable='simulation_node.py',
            name='simulation_node',
            parameters=[{'script': script, 'initial_pose': initial_poses, 'world': "empty"}]
        )
    )
    
    for drone_id in destination_poses_dict.keys():

        drone_number = int(drone_id.split("_")[-1])
        drone_naming = "px4_"
        drone_namespace = drone_id
        drone_initial_coord_x = initial_poses_dict[drone_id]["x"]
        drone_initial_coord_y = initial_poses_dict[drone_id]["y"] 
        drone_destination_coord_x = destination_poses_dict[drone_id]["x"]
        drone_destination_coord_y = destination_poses_dict[drone_id]["y"]

        ld.add_action(launch_ros.actions.Node(
            package=package,
            executable='swarm_drone',
            name='swarm_drone',
            namespace=drone_namespace,
            parameters=[
                {"drone_count": len(destination_poses_dict),
                 "drone_naming": drone_naming,
                 "drone_number": drone_number,
                 "initial_x": drone_initial_coord_x,
                 "initial_y": drone_initial_coord_y,
                 "destination_x": drone_destination_coord_x,
                 "destination_y": drone_destination_coord_y}]
        ))

        ld.add_action(launch_ros.actions.Node(
            package=package,
            executable='px4_position_logger.py',
            name='px4_position_logger',
            parameters=[
                {"drone_naming": drone_naming,
                 "drone_number": drone_number}]
        ))

    # Arming drones (each drone need to receive offboard command before arming)
    ld.add_action(
        launch_ros.actions.Node(
            package=package,
            executable='arming',
            name='arming',
            namespace='simulation',
            parameters=[{"nb_drones": nb_drones}]
        ))

    return ld
