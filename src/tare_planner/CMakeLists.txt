cmake_minimum_required(VERSION 3.1)
project(tare_planner)

# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  tf
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coverage_planner
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs pcl_ros pcl_conversions tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  or-tools/include
  ${EIGEN3_INCLUDE_DIR}
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
  or-tools/lib
)

## ---- 统一的声明宏: 消除每个目标 3 行的重复样板 ---------------------------
## add_tare_library(<目标名> <源文件> [额外链接库...])
macro(add_tare_library name source)
  add_library(${name} ${source})
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${name} ${catkin_LIBRARIES} ${ARGN})
endmacro()

## add_tare_node(<目标名> <源文件> [额外链接库...])
macro(add_tare_node name source)
  add_executable(${name} ${source})
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${name} ${catkin_LIBRARIES} ${ARGN})
endmacro()

add_tare_library(lidar_model src/lidar_model/lidar_model.cpp ${PCL_LIBRARIES})
add_tare_library(graph src/graph/graph.cpp)
add_tare_library(misc_utils src/utils/misc_utils.cpp ${PCL_LIBRARIES})
add_tare_library(pointcloud_utils src/utils/pointcloud_utils.cpp ${PCL_LIBRARIES})
add_tare_library(tsp_solver src/tsp_solver/tsp_solver.cpp ${CMAKE_CURRENT_SOURCE_DIR}/or-tools/lib/libortools.so)
add_tare_library(viewpoint src/viewpoint/viewpoint.cpp lidar_model)
add_tare_library(rolling_grid src/rolling_grid/rolling_grid.cpp misc_utils pointcloud_utils)
add_tare_library(viewpoint_manager src/viewpoint_manager/viewpoint_manager.cpp rolling_grid viewpoint grid_world)
add_tare_library(local_coverage_planner src/local_coverage_planner/local_coverage_planner.cpp viewpoint_manager)
add_tare_library(grid_world src/grid_world/grid_world.cpp viewpoint_manager tsp_solver keypose_graph exploration_path)

add_tare_library(pointcloud_manager src/pointcloud_manager/pointcloud_manager.cpp)
add_tare_library(keypose_graph src/keypose_graph/keypose_graph.cpp ${PCL_LIBRARIES} viewpoint_manager)
add_tare_library(rolling_occupancy_grid src/rolling_occupancy_grid/rolling_occupancy_grid.cpp)
add_tare_library(planning_env src/planning_env/planning_env.cpp ${PCL_LIBRARIES} rolling_occupancy_grid)
add_tare_library(exploration_path src/exploration_path/exploration_path.cpp ${PCL_LIBRARIES})
add_tare_library(tare_visualizer src/tare_visualizer/tare_visualizer.cpp ${PCL_LIBRARIES})
add_tare_library(sensor_coverage_planner_ground src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp graph planning_env keypose_graph viewpoint_manager pointcloud_manager grid_world local_coverage_planner tare_visualizer)

## ---- 可执行节点 ----------------------------------------------------------
add_tare_node(navigationBoundary src/navigation_boundary_publisher/navigationBoundary.cpp ${PCL_LIBRARIES})
add_tare_node(tare_planner_node src/tare_planner_node/tare_planner_node.cpp sensor_coverage_planner_ground)
add_tare_node(uav_local_planner_node src/uav_local_planner/uav_local_planner_node.cpp ${PCL_LIBRARIES})
add_tare_node(uav_saved_map_planner_node src/uav_saved_map_planner/uav_saved_map_planner_node.cpp ${PCL_LIBRARIES})

## ---- Python 脚本 (通配收集, 新增脚本后需重新运行 cmake) -------------------
file(GLOB TARE_PY_SCRIPTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
catkin_install_python(PROGRAMS ${TARE_PY_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Install ##
#############

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )
install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        )
install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
        )

install(TARGETS uav_saved_map_planner_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  ## 纯 C++ 单元测试
  add_executable(test_uav_local_planner_core test/test_uav_local_planner_core.cpp)
  target_link_libraries(test_uav_local_planner_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  add_test(NAME test_uav_local_planner_core COMMAND test_uav_local_planner_core)

  ## launch / 脚本 静态一致性检查 (纯文本+XML 解析, 不需要 roscore)
  set(TARE_PY_TESTS
    test_uav_handoff_profile
    test_uav_manual_navigation_profile
    test_uav_waypoint_bridge_profile
    test_uav_offboard_manager
  )
  foreach(t ${TARE_PY_TESTS})
    add_test(NAME ${t}
             COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/${t}.py)
  endforeach()
endif()
