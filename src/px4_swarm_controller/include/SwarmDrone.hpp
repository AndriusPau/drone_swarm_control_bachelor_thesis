#pragma once

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

struct XYZ {
    double x;
    double y;
    double z;
};

/**
 * @class SwarmDrone
 * @brief asdf
 */
class SwarmDrone : public rclcpp::Node {
    using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
    // using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
    // using Odometry = nav_msgs::msg::Odometry;
    // using Path = nav_msgs::msg::Path;
    // using PoseStamped = geometry_msgs::msg::PoseStamped;
    // using Pose = geometry_msgs::msg::Pose;
    // using Point = geometry_msgs::msg::Point;

    public:
    SwarmDrone();

    private:
        /**
         * @brief Publish the offboard control mode.
         */
        void publish_offboard_control_mode();

        /**
         * @brief Callback function for the vehicle local position subscriber.
         * @param pose The received vehicle local position message.
         */
        void drone_position_subscriber_callback(const VehicleLocalPosition::SharedPtr &pose);

        /**
         * @brief Callback function for the vehicle local position subscriber.
         * @param pose The received vehicle local position message.
         */
        void other_drone_position_subscriber_callback(int target_drone, const VehicleLocalPosition::SharedPtr &pose);

        /**
         * @brief Write the waypoint to be published based on the specified index.
         * @param idx Index of the waypoint.
         */
        void writeWP(double x, double y, double z);

                /**
         * @brief Write the waypoint to be published based on the specified index.
         * @param idx Index of the waypoint.
         */
        void writeCP(const VehicleLocalPosition::SharedPtr &pose);

        void takeoff();

        void flight_main();

        double distance_to_destination();

        double distance_to_waypoint();

        double direction_to_destination();

        double get_distance(double x1, double y1, double z1, double x2, double y2, double z2);

        XYZ get_attraction_destination();

        XYZ get_attraction_drone();

        XYZ get_repulsion();

        XYZ get_repulsion_ground();

        double attraction_destination_formula(double distance);

        double attraction_drone_formula(double distance);

        double repulsion_obstacle_formula(double distance);

        double repulsion_ground_formula(double z);

        // void rviz_position_callback(const VehicleOdometry::SharedPtr msg);
        // void rviz_publish_path();

    private:
        rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<VehicleLocalPosition>::SharedPtr current_position_publisher_;
        rclcpp::Subscription<VehicleLocalPosition>::SharedPtr current_position_subscriber;

        std::vector<rclcpp::Subscription<VehicleLocalPosition>::SharedPtr> other_drone_position_subscribers;
        std::map<int, XYZ> other_drone_positions;
        std::map<int, double> other_drone_directions;

        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr collision_publisher;

        // rclcpp::Publisher<PoseStamped>::SharedPtr rviz_path_publisher_;
        // rclcpp::Subscription<VehicleOdometry>::SharedPtr rviz_odometry_subscriber_;
        // PoseStamped pose_msg;

        TrajectorySetpoint waypoint_message;
        VehicleLocalPosition current_position_message;

        std::string drone_naming{};
        int drone_number;
        XYZ position_initial{};
        XYZ position_destination{};
        XYZ position_local{};
        XYZ position_global{};
        XYZ waypoint;
        double angle_to_destination;
        bool takeoff_complete;

        // Constants
        double takeoff_accuracy = 0.75;
        double takeoff_altitude = 10;

        double collision_distance = 0.5;
        double target_slowdown_distance = 15;
        double target_precision = 1;
        double repulsion_max_distance = 10;
        double repulsion_min_distance = 1;
        double attraction_drone_max_distance = 15;
        double attraction_drone_min_distance = 1;

        double force_multiplier = 4;
        double attraction_destination_force_multipier = 1;
        double attraction_drone_force_multiplier = 2;
        double repulsion_ground_force_multiplier = 1;
        double repulsion_force_multiplier = 1.5;

        double waypoint_max_distance = 10;

        double angle_allignment_max = 30;
};
