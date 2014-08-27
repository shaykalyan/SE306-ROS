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
rosbuild_add_library(EscortRobot src/EscortRobot.cpp)
target_link_libraries(StaticPoi Poi)

# Invisible Nodes
rosbuild_add_executable(MoralSupportDiceRoller src/MoralSupportDiceRoller.cpp)
target_link_libraries(MoralSupportDiceRoller DiceRoller)

rosbuild_add_executable(IllDiceRoller src/IllDiceRoller.cpp)
target_link_libraries(IllDiceRoller DiceRoller)

rosbuild_add_executable(VeryIllDiceRoller src/VeryIllDiceRoller.cpp)
target_link_libraries(VeryIllDiceRoller DiceRoller)

rosbuild_add_executable(Scheduler src/Scheduler.cpp)
target_link_libraries(Scheduler)

# Testing
rosbuild_add_gtest(test/TestDiceRoller test/TestDiceRoller.cpp)
target_link_libraries(test/TestDiceRoller DiceRoller)

# rostest
rosbuild_add_executable(TestResident EXCLUDE_FROM_ALL src/test/TestResident.cpp)
rosbuild_add_gtest_build_flags(TestResident)
rosbuild_add_rostest(test/TestResident.test)

rosbuild_add_executable(TestRelativeRobot EXCLUDE_FROM_ALL src/test/TestRelativeRobot.cpp)
rosbuild_add_library(RelativeRobotLib src/RelativeRobot.cpp)
rosbuild_add_gtest_build_flags(TestRelativeRobot)
rosbuild_add_rostest(test/TestRelativeRobot.test)
target_link_libraries(TestRelativeRobot RelativeRobotLib Robot Poi StaticPoi)

# Robot Nodes
rosbuild_add_executable(Resident src/Resident.cpp)
target_link_libraries(Resident Robot Poi StaticPoi)

rosbuild_add_executable(Visitor src/Visitor.cpp)								#R1
target_link_libraries(Visitor Robot Poi StaticPoi)

rosbuild_add_executable(Assistant src/Assistant.cpp)							#R2
target_link_libraries(Assistant Robot Poi StaticPoi)

rosbuild_add_executable(RelativeRobot src/RelativeRobot.cpp)	   				#R3
target_link_libraries(RelativeRobot Robot Poi StaticPoi)

rosbuild_add_executable(FriendRobot src/FriendRobot.cpp)	        			#R4
target_link_libraries(FriendRobot Robot Poi StaticPoi)

rosbuild_add_executable(CompanionshipRobot src/CompanionshipRobot.cpp)	        #R5
target_link_libraries(CompanionshipRobot Robot Poi StaticPoi)

rosbuild_add_executable(ChefRobot src/ChefRobot.cpp)         					#R6
target_link_libraries(ChefRobot Robot Poi StaticPoi)

rosbuild_add_executable(MedicationRobot src/MedicationRobot.cpp)    			#R7
target_link_libraries(MedicationRobot Robot Poi StaticPoi)

rosbuild_add_executable(NurseRobot src/NurseRobot.cpp)  						#R8
target_link_libraries(NurseRobot Robot Poi StaticPoi)

rosbuild_add_executable(Caregiver src/Caregiver.cpp)							#R9
target_link_libraries(Caregiver Robot Poi StaticPoi)

rosbuild_add_executable(EntertainmentRobot src/EntertainmentRobot.cpp)          #R10
target_link_libraries(EntertainmentRobot Robot Poi StaticPoi)

rosbuild_add_executable(FeedingRobot src/FeedingRobot.cpp)  					#R11
target_link_libraries(FeedingRobot Robot EscortRobot Poi StaticPoi)

rosbuild_add_executable(DoctorRobot src/DoctorRobot.cpp)                        #R12
target_link_libraries(DoctorRobot Robot EscortRobot Poi StaticPoi)