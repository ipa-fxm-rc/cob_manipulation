cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_add_boost_directories()

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(cob_pick_place_action ros/src/cob_pick_place_action.cpp common/src/GraspTable.cpp)
rosbuild_add_executable(cob_pick_action_client ros/src/cob_pick_action_client.cpp)
rosbuild_add_executable(modify_start_state ros/src/modify_start_state.cpp)
#rosbuild_add_executable(get_chain ros/src/get_chain.cpp)
rosbuild_add_executable(ik_solver_kdl ros/src/ik_solver_kdl.cpp)
rosbuild_add_executable(cob_base_pick_planning ros/src/cob_base_pick_planning.cpp)
rosbuild_add_executable(cob_grasp_base_position_server ros/src/cob_grasp_base_position_server.cpp)
rosbuild_add_executable(get_joint_states ros/src/get_joint_states.cpp)
rosbuild_add_executable(cob_place_action_client ros/src/cob_place_action_client.cpp)
rosbuild_link_boost(cob_pick_place_action signals)
rosbuild_link_boost(cob_pick_action_client signals)
rosbuild_link_boost(cob_place_action_client signals)
#target_link_libraries(example ${PROJECT_NAME})
