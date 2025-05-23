cmake_minimum_required(VERSION 3.8)
project(px4_swarm_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# I didn't build the message in this package because it seems to create issues with ament python
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${YAML_CPP_INCLUDE_DIRS})
add_executable(arming include src/Arming.cpp)
ament_target_dependencies(arming rclcpp px4_msgs)

add_executable(swarm_drone include src/SwarmDrone.cpp)
ament_target_dependencies(swarm_drone rclcpp px4_msgs nav_msgs geometry_msgs)
target_link_libraries(swarm_drone ${YAML_CPP_LIBRARIES})

install(TARGETS
        arming
        swarm_drone
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

# Install Py Packages
ament_python_install_package(${PROJECT_NAME})

# Install Py executables
install(PROGRAMS ${PROJECT_NAME}/simulation_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/px4_position_logger.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})
ament_package()
