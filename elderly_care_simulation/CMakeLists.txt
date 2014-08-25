cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

include_directories(include)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# Libraries
rosbuild_add_library(Robot src/Robot.cpp)
rosbuild_add_library(DiceRoller src/DiceRoller.cpp)
rosbuild_add_library(Poi src/Poi.cpp)
rosbuild_add_library(StaticPoi src/StaticPoi.cpp)

# Invisible Nodes
rosbuild_add_executable(MoralSupportDiceRoller src/MoralSupportDiceRoller.cpp)
target_link_libraries(MoralSupportDiceRoller DiceRoller)

rosbuild_add_executable(IllDiceRoller src/IllDiceRoller.cpp)
target_link_libraries(IllDiceRoller DiceRoller)

rosbuild_add_executable(VeryIllDiceRoller src/VeryIllDiceRoller.cpp)
target_link_libraries(VeryIllDiceRoller DiceRoller)

rosbuild_add_executable(Scheduler src/Scheduler.cpp)

# Testing
rosbuild_add_gtest(test/TestDiceRoller test/TestDiceRoller.cpp)
target_link_libraries(test/TestDiceRoller DiceRoller)

# Robot Nodes
rosbuild_add_executable(Resident src/Resident.cpp) 		#R0
target_link_libraries(Resident Robot Poi StaticPoi)

rosbuild_add_executable(Visitor src/Visitor.cpp)		#R1
target_link_libraries(Visitor Robot Poi StaticPoi)

rosbuild_add_executable(Assistant src/Assistant.cpp)	#R2
target_link_libraries(Assistant Robot Poi StaticPoi)

rosbuild_add_executable(Entertainment src/Entertainment.cpp)	        #R3
target_link_libraries(Entertainment Robot Poi StaticPoi)