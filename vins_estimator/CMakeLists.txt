cmake_minimum_required(VERSION 3.5)
project(vins)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)

set(CMAKE_BUILD_TYPE Release)

# use gdb to debug
# find_package(backward_ros REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_models REQUIRED)
find_package(image_transport REQUIRED)

# set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})

# include camera_models
include_directories("../camera_models/include")

#template sophus
#set(Sophus_DIR "/home/td/slam/install/sophus/share/sophus/cmake/")
find_package(Sophus REQUIRED)
# include_directories(/home/td/slam/install/sophus/include)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)


add_library(vins_lib
        src/estimator/parameters.cpp
        src/estimator/estimator.cpp
        src/estimator/feature_manager.cpp
        src/factor/pose_local_parameterization.cpp
        src/factor/pose_subset_parameterization.cpp
        src/factor/orientation_local_parameterization.cpp
        src/factor/orientation_subset_parameterization.cpp
        src/factor/projectionTwoFrameOneCamFactor.cpp
        src/factor/projectionTwoFrameTwoCamFactor.cpp
        src/factor/projectionOneFrameTwoCamFactor.cpp
        src/factor/marginalization_factor.cpp
        src/utility/utility.cpp
        src/utility/visualization.cpp
        src/utility/CameraPoseVisualization.cpp
        src/initial/solve_5pts.cpp
        src/initial/initial_aligment.cpp
        src/initial/initial_sfm.cpp
        src/initial/initial_ex_rotation.cpp
        src/featureTracker/feature_tracker.cpp)
target_link_libraries(vins_lib ${OpenCV_LIBS} ${CERES_LIBRARIES})

ament_target_dependencies(vins_lib rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_models image_transport)

add_executable(vins_node src/rosNodeTest.cpp)
ament_target_dependencies(vins_node rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_models image_transport)
target_link_libraries(vins_node vins_lib)

add_executable(viwo_node src/rosNodeTestWheel.cpp)
ament_target_dependencies(viwo_node rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_models image_transport)
target_link_libraries(viwo_node vins_lib)

#add_executable(viwo_gazebo_node src/rosNodeTestWheelGazebo.cpp)
#target_link_libraries(viwo_gazebo_node vins_lib)

#add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
#target_link_libraries(kitti_odom_test vins_lib)

add_executable(simulation_node src/simulationTest.cpp)
ament_target_dependencies(simulation_node rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_models image_transport)
target_link_libraries(simulation_node vins_lib)

#add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
#target_link_libraries(kitti_gps_test vins_lib)

#TODO: fix bagwriter in ros2
#add_executable(bag_writer src/tool/bagwriter.cpp)
#target_link_libraries(bag_writer ${catkin_LIBRARIES})

# added from DEBUG

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS vins_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS viwo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS simulation_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
