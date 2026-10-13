# SBEG 361 Lab 3, Part B -- optional C++ (rclcpp) version of nav_tf_broadcaster.py (ROS 2 Jazzy, ament_cmake).
# Not auto-tested. Build it in a colcon workspace: see the comment at the top of src/nav_tf_broadcaster.cpp.
cmake_minimum_required(VERSION 3.16)
project(sbeg361_nav_tf CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json 3 REQUIRED)

# Default data folder: the Lab code folder two levels above this package (nav_tree.json, tool_track.csv).
get_filename_component(SBEG_LAB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." REALPATH)

add_executable(nav_tf_broadcaster src/nav_tf_broadcaster.cpp)
target_compile_definitions(nav_tf_broadcaster PRIVATE "SBEG_LAB_DIR=\"${SBEG_LAB_DIR}\"")
target_link_libraries(nav_tf_broadcaster nlohmann_json::nlohmann_json)
ament_target_dependencies(nav_tf_broadcaster rclcpp geometry_msgs tf2_ros)

install(TARGETS nav_tf_broadcaster DESTINATION lib/${PROJECT_NAME})
ament_package()
