cmake_minimum_required(VERSION 3.8)
project(livox_mapping_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs)
find_package(nav_msgs)
find_package(sensor_msgs)
find_package(rclcpp)
find_package(rclpy)
find_package(std_msgs)
find_package(pcl_ros)
find_package(tf2)
find_package(tf2_ros)
find_package(livox_ros_driver2)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP QUIET)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(DEPENDENCIES
  "rclcpp"
  "rclpy"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "std_msgs"
  "pcl_ros"
  "tf2"
  "tf2_ros"
  "livox_ros_driver2"
)

set(EXECUTABLE_NAME "loam_scanRegistration")

add_executable(${EXECUTABLE_NAME} src/scanRegistration.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

set(EXECUTABLE_NAME2 "loam_laserMapping")

add_executable(${EXECUTABLE_NAME2} src/laserMapping.cpp)
target_link_libraries(${EXECUTABLE_NAME2} ${PCL_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(${EXECUTABLE_NAME2} ${DEPENDENCIES})

set(EXECUTABLE_NAME3 "livox_repub")

add_executable(${EXECUTABLE_NAME3} src/livox_repub.cpp)
target_link_libraries(${EXECUTABLE_NAME3} ${PCL_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(${EXECUTABLE_NAME3} ${DEPENDENCIES})


set(EXECUTABLE_NAME4 "loam_scanRegistration_horizon")

add_executable(${EXECUTABLE_NAME4} src/scanRegistration_horizon.cpp)
target_link_libraries(${EXECUTABLE_NAME4} ${PCL_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(${EXECUTABLE_NAME4} ${DEPENDENCIES})


################################################################################
# Install
################################################################################

install(TARGETS ${EXECUTABLE_NAME} ${EXECUTABLE_NAME2} ${EXECUTABLE_NAME3} ${EXECUTABLE_NAME4}
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
