#include "SwarmDrone.hpp"
#include <math.h>
#include <vector>
#include <string>

/**
 * 
 */
SwarmDrone::SwarmDrone() : Node("swarm_drone") {
    const std::string name_space{this->get_namespace()};
    RCLCPP_INFO(this->get_logger(), "namespace: %s", name_space.c_str());

    this->declare_parameter<std::string>("drone_naming");
    std::string drone_naming = this->get_parameter("drone_naming").as_string();

    this->declare_parameter<int>("drone_count");
    int total_drone_count = this->get_parameter("drone_count").as_int();
    this->declare_parameter<int>("drone_number");
    drone_number = this->get_parameter("drone_number").as_int();

    this->declare_parameter<double>("initial_x");
    this->declare_parameter<double>("initial_y");
    position_initial.x = this->get_parameter("initial_x").as_double();
    position_initial.y = this->get_parameter("initial_y").as_double();
    position_initial.z = 0;

    this->declare_parameter<double>("destination_x");
    this->declare_parameter<double>("destination_y");
    position_destination.x = this->get_parameter("destination_x").as_double();
    position_destination.y = this->get_parameter("destination_y").as_double();
    position_destination.z = 0;

    takeoff_complete = false;

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
        name_space + "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
        name_space + "/fmu/in/trajectory_setpoint", 10);
    collision_publisher = this->create_publisher<TrajectorySetpoint>(
        name_space + "/drone_collision", 10);
    
    
    current_position_publisher_ = this->create_publisher<VehicleLocalPosition>(
        name_space + "/fmu/out/vehicle_local_position_custom", 10);

    // See PX4 documentation on subscriber
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
    current_position_subscriber = this->create_subscription<VehicleLocalPosition>(
        name_space + "/fmu/out/vehicle_local_position", qos,
        [=](const typename VehicleLocalPosition::SharedPtr msg) {
            drone_position_subscriber_callback(msg);
        });

    RCLCPP_INFO(this->get_logger(), "from (%f,%f) to (%f,%f)", 
    position_initial.x, 
    position_initial.y, 
    position_destination.x, 
    position_destination.y);

    for (int i = 1; i <= total_drone_count; i++) {
        if (i == drone_number) {
            continue;
        }
        std::string subscription_name = "/" + drone_naming + std::to_string(i) + "/fmu/out/vehicle_local_position_custom";
        RCLCPP_INFO(this->get_logger(), subscription_name.c_str());
        other_drone_position_subscribers.push_back(this->create_subscription<VehicleLocalPosition>(
            subscription_name, qos,
            [=](const typename VehicleLocalPosition::SharedPtr msg) {
                other_drone_position_subscriber_callback(i, msg);
            }));
    }
}

/**
 * @brief Callback function for the vehicle local position subscriber.
 * @param pose The received vehicle local position message.
 */
void SwarmDrone::drone_position_subscriber_callback(const VehicleLocalPosition::SharedPtr &pose) {
    // For all the following nodes, we will switch the x-axis and the y-axis to follow the North East Down
    // convention. In fact, to spawn the drones, we need to follow Gazebo's frame convention East North Up.
    // Therefore, we need to switch the x-axis and the y-axis if we want to respect PX4's conventions.

    writeCP(pose);
    if (!takeoff_complete) {
        takeoff();
    } else {
        flight_main();
    }

    publish_offboard_control_mode();
    trajectory_setpoint_publisher_->publish(waypoint_message);
    current_position_publisher_->publish(current_position_message);
}

/**
 * @brief Callback function for the vehicle local position subscriber.
 * @param pose The received vehicle local position message.
 */
void SwarmDrone::other_drone_position_subscriber_callback(int target_drone, const VehicleLocalPosition::SharedPtr &pose) {
    other_drone_positions[target_drone] = XYZ{pose->x, pose->y, pose->z};
    other_drone_directions[target_drone] = pose->heading;
}

/**
 * @brief Publish the offboard control mode.
 */
void SwarmDrone::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Write the waypoint to be published based on the specified index.
 */
void SwarmDrone::writeWP(double x, double y, double z) {
    waypoint.x = x;
    waypoint.y = y;
    waypoint.z = z;
    // TODO: Check use_sim_time
    waypoint_message.timestamp = static_cast<uint64_t>(this->get_clock()->now().seconds());
    // North-East-Down frame (different from Gazebo)
    // The drones use their starting location as (0,0), so the global coordinates are
    // different from the drone perceived coordinates. 
    waypoint_message.position = {static_cast<float>(y-position_initial.y),
                         static_cast<float>(x-position_initial.x), 
                         static_cast<float>(-z)};

    waypoint_message.yaw = static_cast<float>(3.14);
}

/**
 * @brief Write the waypoint to be published based on the specified index.
 */
void SwarmDrone::writeCP(const VehicleLocalPosition::SharedPtr &pose) {
    // TODO: Check use_sim_time
    current_position_message.timestamp = static_cast<uint64_t>(this->get_clock()->now().seconds());
    // North-East-Down frame (different from Gazebo)
    // The drones use their starting location as (0,0), so the global coordinates are
    // different from the drone perceived coordinates. 
    angle_to_destination = direction_to_destination();
    position_local.x = pose->y;
    position_local.y = pose->x;
    position_local.z = pose->z;
    position_global.x = position_local.x + position_initial.x;
    position_global.y = position_local.y + position_initial.y;
    position_global.z = -position_local.z;
    current_position_message.x = static_cast<float>(position_global.x);
    current_position_message.y = static_cast<float>(position_global.y);
    current_position_message.z = static_cast<float>(position_global.z);
    current_position_message.heading = angle_to_destination;
}

void SwarmDrone::takeoff() {
    writeWP(position_initial.x, position_initial.y, takeoff_altitude);

    if (distance_to_waypoint() < takeoff_accuracy) {
        takeoff_complete = true;
        return;
    }
}

void SwarmDrone::flight_main() {
    XYZ attraction_destination = get_attraction_destination();
    XYZ attraction_drones = get_attraction_drone();
    XYZ repulsion = get_repulsion();
    XYZ repulsion_ground = get_repulsion_ground();

    XYZ total_force;
    total_force.x = (
        (attraction_destination.x * attraction_destination_force_multipier) + 
        (attraction_drones.x * attraction_drone_force_multiplier) + 
        (repulsion_ground.x * repulsion_ground_force_multiplier) + 
        (repulsion.x * repulsion_force_multiplier)) * 
        force_multiplier;
    total_force.y = (
        (attraction_destination.y * attraction_destination_force_multipier) + 
        (attraction_drones.y * attraction_drone_force_multiplier) + 
        (repulsion_ground.y * repulsion_ground_force_multiplier) + 
        (repulsion.y * repulsion_force_multiplier)) * 
        force_multiplier;
    total_force.z = (
        (attraction_destination.z * attraction_destination_force_multipier) + 
        (attraction_drones.z * attraction_drone_force_multiplier) + 
        (repulsion_ground.z * repulsion_ground_force_multiplier) + 
        (repulsion.z * repulsion_force_multiplier)) * 
        force_multiplier;

    // if (drone_number == 100) {
    //     RCLCPP_INFO(this->get_logger(), "%f %f %f | %f %f %f | %f %f %f | %f %f %f | %f %f %f", 
    //     position_global.x, position_global.y, position_global.z,
    //     // position_local.x, position_local.y, position_local.z,
    //     attraction_destination.x, attraction_destination.y, attraction_destination.z,
    //     repulsion.x, repulsion.y, repulsion.z,
    //     total_force.x, total_force.y, total_force.z,
    //     waypoint.x, waypoint.y, waypoint.z
    //     // position_destination.x, position_destination.y, position_destination.z
    //     );
    // }

    if (distance_to_destination() < target_precision) {
        writeWP(
            position_destination.x, 
            position_destination.y,
            position_global.z + total_force.z);
        return;
    }

    writeWP(
        position_global.x + total_force.x, 
        position_global.y + total_force.y, 
        position_global.z + total_force.z);
}

double SwarmDrone::distance_to_destination() {
    return get_distance(
        position_global.x,
        position_global.y,
        position_global.z,
        position_destination.x,
        position_destination.y,
        position_global.z);
}

double SwarmDrone::distance_to_waypoint() {
    return get_distance(
        position_global.x, 
        position_global.y, 
        position_global.z, 
        waypoint.x, 
        waypoint.y, 
        waypoint.z);
}

double SwarmDrone::direction_to_destination() {
    double radians = std::atan2(
        position_global.y - position_destination.y, 
        position_global.x - position_destination.x);
    
    double degrees = radians * (180.0 / M_PI);
    if (degrees < 0) {
        degrees += 360.0;
    }

    return degrees;
}

double SwarmDrone::get_distance(double x1, double y1, double z1, double x2, double y2, double z2) {
    return sqrt(
        pow(x2 - x1, 2) + 
        pow(y2 - y1, 2) + 
        pow(z2 - z1, 2));
}

XYZ SwarmDrone::get_attraction_destination() {
    XYZ attraction_destination;
    attraction_destination.x = 0;
    attraction_destination.y = 0;
    attraction_destination.z = 0;
    
    // We want the drones to reach the destination in the x and y axis, but the z axis does not matter
    // Therefore, we will use the current altitude as the target altitude.
    // Hence, replacing the `position_destination.z` with `position_global.z`.

    // Get vector from current position to destination.
    attraction_destination.x = position_destination.x - position_global.x;
    attraction_destination.y = position_destination.y - position_global.y;
    attraction_destination.z = position_global.z - position_global.z;

    // Get distance from current position to destination.
    double distance_to_destination_ = distance_to_destination();
    
    // Normalize the destination attraction vector by dividing the values from the distance. 
    attraction_destination.x = attraction_destination.x / distance_to_destination_;
    attraction_destination.y = attraction_destination.y / distance_to_destination_;
    attraction_destination.z = attraction_destination.z / distance_to_destination_;

    // Get attraction value from the distance to target. 
    double attraction_value = attraction_destination_formula(distance_to_destination_);

    // Multiply the normalized vector by the attraction value, depending on the distance to destination. 
    attraction_destination.x = attraction_destination.x * attraction_value;
    attraction_destination.y = attraction_destination.y * attraction_value;
    attraction_destination.z = attraction_destination.z * attraction_value;

    return attraction_destination;
}

XYZ SwarmDrone::get_attraction_drone() {
    XYZ total_drone_attraction;
    total_drone_attraction.x = 0;
    total_drone_attraction.y = 0;
    total_drone_attraction.z = 0;
    double drones_in_range = 0;
    double attraction_value = 0;

    for (auto drone : other_drone_positions) {

        // Get distance from current position to other drone.
        double distance_to_drone = get_distance(
            position_global.x,
            position_global.y,
            position_global.z,
            drone.second.x,
            drone.second.y,
            drone.second.z
        );

        if (distance_to_drone <= attraction_drone_max_distance && 
            abs(angle_to_destination - other_drone_directions[drone.first]) < angle_allignment_max) {
            XYZ drone_attraction;
            drones_in_range = drones_in_range + 1;
    
            // Get vector from current position to other drone.
            drone_attraction.x = drone.second.x - position_global.x;
            drone_attraction.y = drone.second.y - position_global.y;
            drone_attraction.z = drone.second.z - position_global.z;

            // Normalize the drone attraction vector by dividing the values from the distance. 
            drone_attraction.x = drone_attraction.x / distance_to_drone;
            drone_attraction.y = drone_attraction.y / distance_to_drone;
            drone_attraction.z = drone_attraction.z / distance_to_drone;

            // Get attraction value from the distance to target. 
            attraction_value = attraction_drone_formula(distance_to_drone);

            // Multiply the normalized vector by the attraction value, depending on the distance to destination. 
            drone_attraction.x = drone_attraction.x * attraction_value;
            drone_attraction.y = drone_attraction.y * attraction_value;
            drone_attraction.z = drone_attraction.z * attraction_value;

            total_drone_attraction.x = total_drone_attraction.x + drone_attraction.x;
            total_drone_attraction.y = total_drone_attraction.y + drone_attraction.y;
            total_drone_attraction.z = total_drone_attraction.z + drone_attraction.z;
        }
    }

    total_drone_attraction.x = total_drone_attraction.x / std::max(drones_in_range, 1.0);
    total_drone_attraction.y = total_drone_attraction.y / std::max(drones_in_range, 1.0);
    total_drone_attraction.z = total_drone_attraction.z / std::max(drones_in_range, 1.0);

    return total_drone_attraction;
}

XYZ SwarmDrone::get_repulsion() {
    XYZ total_drone_repulsion;
    total_drone_repulsion.x = 0;
    total_drone_repulsion.y = 0;
    total_drone_repulsion.z = 0;
    double drones_in_range = 0;
    double repulsion_value = 0;

    for (auto drone : other_drone_positions) {

        // Get distance from current position to other drone.
        double distance_to_drone = get_distance(
            position_global.x,
            position_global.y,
            position_global.z,
            drone.second.x,
            drone.second.y,
            drone.second.z
        );

        if (distance_to_drone <= collision_distance) {
            TrajectorySetpoint collision;
            collision.timestamp = static_cast<uint64_t>(this->get_clock()->now().seconds());
            collision.yaw = drone_number;
            collision.yawspeed = drone.first;
            collision.position[0] = position_global.x;
            collision.position[1] = position_global.y;
            collision.position[2] = position_global.z;
            collision.velocity[0] = drone.second.x;
            collision.velocity[1] = drone.second.y;
            collision.velocity[2] = drone.second.z;
            collision.acceleration[0] = distance_to_drone;
            collision_publisher->publish(collision);
            RCLCPP_INFO(this->get_logger(), "%d %d %f", 
                drone_number, drone.first, distance_to_drone);
        }

        if (distance_to_drone <= repulsion_max_distance) {
            XYZ drone_repulsion;
            drones_in_range = drones_in_range + 1;
    
            // Get vector from current position to other drone.
            drone_repulsion.x = position_global.x - drone.second.x;
            drone_repulsion.y = position_global.y - drone.second.y;
            drone_repulsion.z = position_global.z - drone.second.z;

            // Normalize the drone attraction vector by dividing the values from the distance. 
            drone_repulsion.x = drone_repulsion.x / distance_to_drone;
            drone_repulsion.y = drone_repulsion.y / distance_to_drone;
            drone_repulsion.z = drone_repulsion.z / distance_to_drone;
    
            // Get repulsion value from the distance to target. 
            repulsion_value = repulsion_obstacle_formula(distance_to_drone);
    
            // Multiply the normalized vector by the attraction value, depending on the distance to destination. 
            drone_repulsion.x = drone_repulsion.x * repulsion_value;
            drone_repulsion.y = drone_repulsion.y * repulsion_value;
            drone_repulsion.z = drone_repulsion.z * repulsion_value;
    
            total_drone_repulsion.x = total_drone_repulsion.x + drone_repulsion.x;
            total_drone_repulsion.y = total_drone_repulsion.y + drone_repulsion.y;
            total_drone_repulsion.z = total_drone_repulsion.z + drone_repulsion.z;
        }
    }
    total_drone_repulsion.x = total_drone_repulsion.x / std::max(drones_in_range, 1.0);
    total_drone_repulsion.y = total_drone_repulsion.y / std::max(drones_in_range, 1.0);
    total_drone_repulsion.z = total_drone_repulsion.z / std::max(drones_in_range, 1.0);

    return total_drone_repulsion;
}

XYZ SwarmDrone::get_repulsion_ground() {
    XYZ total_drone_repulsion;
    total_drone_repulsion.x = 0;
    total_drone_repulsion.y = 0;
    total_drone_repulsion.z = 0;

    double repulsion = repulsion_ground_formula(position_global.z);

    total_drone_repulsion.z = repulsion;

    return total_drone_repulsion;
}

double SwarmDrone::attraction_destination_formula(double distance) {
    if (distance > target_slowdown_distance) {
        return 1;
    }

    if (distance < target_precision) {
        return 0;
    }

    return ((log(distance/target_precision))/log(target_slowdown_distance/target_precision));   
}

double SwarmDrone::attraction_drone_formula(double distance) {
    if (distance <= attraction_drone_min_distance || 
        distance >= attraction_drone_max_distance) {
        return 0;
    }

    return (log(distance) / log(attraction_drone_max_distance * 2));
}

double SwarmDrone::repulsion_obstacle_formula(double distance) {
    if (distance <= repulsion_min_distance) {
        return 1;
    }

    if (distance >= repulsion_max_distance) {
        return 0;
    }

    return (1 - pow(((distance - repulsion_min_distance) / (repulsion_max_distance - repulsion_min_distance)), 2));
}

double SwarmDrone::repulsion_ground_formula(double z) {
    if (z <= 0) {
        return 1;
    }

    if (z >= 5) {
        return 0;
    }

    return ((-0.2 * z) + 1);
}

/**
 * @brief Main function to start the SwarmDrone node.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Exit code.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmDrone>());
    rclcpp::shutdown();
    return 0;
}