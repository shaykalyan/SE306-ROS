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


rosbuild_add_executable(TestCaregiverRobot EXCLUDE_FROM_ALL src/test/TestCaregiverRobot.cpp)
rosbuild_add_library(CaregiverLib src/Caregiver.cpp)
rosbuild_add_gtest_build_flags(TestCaregiverRobot)
rosbuild_add_rostest(test/TestCaregiverRobot.test)
target_link_libraries(TestCaregiverRobot CaregiverLib Robot Poi StaticPoi)

rosbuild_add_executable(TestFriendRobot EXCLUDE_FROM_ALL src/test/TestFriendRobot.cpp)
rosbuild_add_library(FriendRobotLib src/FriendRobot.cpp)
rosbuild_add_gtest_build_flags(TestFriendRobot)
rosbuild_add_rostest(test/TestFriendRobot.test)
target_link_libraries(TestFriendRobot FriendRobotLib Robot Poi StaticPoi)

rosbuild_add_executable(TestRelativeRobot EXCLUDE_FROM_ALL src/test/TestRelativeRobot.cpp)
rosbuild_add_library(RelativeRobotLib src/RelativeRobot.cpp)
rosbuild_add_gtest_build_flags(TestRelativeRobot)
rosbuild_add_rostest(test/TestRelativeRobot.test)
target_link_libraries(TestRelativeRobot RelativeRobotLib Robot Poi StaticPoi)

rosbuild_add_executable(TestScheduler EXCLUDE_FROM_ALL src/test/TestScheduler.cpp)
rosbuild_add_library(SchedulerLib src/Scheduler.cpp)
rosbuild_add_gtest_build_flags(TestScheduler)
rosbuild_add_rostest(test/TestScheduler.test)
target_link_libraries(TestScheduler SchedulerLib)

rosbuild_add_executable(TestPathServer EXCLUDE_FROM_ALL src/test/TestPathServer.cpp)
rosbuild_add_gtest_build_flags(TestPathServer)
rosbuild_add_rostest(test/TestPathServer.test)

rosbuild_add_executable(TestCompanionshipRobot EXCLUDE_FROM_ALL src/test/TestCompanionshipRobot.cpp)
rosbuild_add_library(CompanionshipRobotLib src/CompanionshipRobot.cpp)
rosbuild_add_gtest_build_flags(TestCompanionshipRobot)
rosbuild_add_rostest(test/TestCompanionshipRobot.test)
target_link_libraries(TestCompanionshipRobot CompanionshipRobotLib Robot Poi StaticPoi)

rosbuild_add_executable(TestMedicationRobot EXCLUDE_FROM_ALL src/test/TestMedicationRobot.cpp)
rosbuild_add_library(MedicationRobotLib src/MedicationRobot.cpp)
rosbuild_add_gtest_build_flags(TestMedicationRobot)
rosbuild_add_rostest(test/TestMedicationRobot.test)
target_link_libraries(TestMedicationRobot MedicationRobotLib Robot Poi StaticPoi)

rosbuild_add_executable(TestEntertainmentRobot EXCLUDE_FROM_ALL src/test/TestEntertainmentRobot.cpp)
rosbuild_add_library(EntertainmentRobotLib src/EntertainmentRobot.cpp)
rosbuild_add_gtest_build_flags(TestEntertainmentRobot)
rosbuild_add_rostest(test/TestEntertainmentRobot.test)
target_link_libraries(TestEntertainmentRobot EntertainmentRobotLib Robot Poi StaticPoi)

rosbuild_add_executable(TestChefRobot EXCLUDE_FROM_ALL src/test/TestChefRobot.cpp)
rosbuild_add_library(ChefRobotLib src/ChefRobot.cpp)
rosbuild_add_gtest_build_flags(TestChefRobot)
rosbuild_add_rostest(test/TestChefRobot.test)
target_link_libraries(TestChefRobot ChefRobotLib Robot Poi StaticPoi)

rosbuild_add_executable(TestEscortRobot EXCLUDE_FROM_ALL src/test/TestEscortRobot.cpp)
rosbuild_add_library(EscortRobotLib src/EscortRobot.cpp)
rosbuild_add_gtest_build_flags(TestEscortRobot)
rosbuild_add_rostest(test/TestEscortRobot.test)
target_link_libraries(TestEscortRobot EscortRobotLib Robot Poi StaticPoi)


# Robot Nodes
rosbuild_add_executable(Resident src/Resident.cpp)								#R0
target_link_libraries(Resident Robot Poi StaticPoi)

rosbuild_add_executable(RelativeRobot src/RelativeRobot.cpp)	   				#R1
target_link_libraries(RelativeRobot Robot Poi StaticPoi)

rosbuild_add_executable(FriendRobot src/FriendRobot.cpp)	        			#R2
target_link_libraries(FriendRobot Robot Poi StaticPoi)

rosbuild_add_executable(CompanionshipRobot src/CompanionshipRobot.cpp)	        #R3
target_link_libraries(CompanionshipRobot Robot Poi StaticPoi)

rosbuild_add_executable(ChefRobot src/ChefRobot.cpp)         					#R4
target_link_libraries(ChefRobot Robot Poi StaticPoi)

rosbuild_add_executable(MedicationRobot src/MedicationRobot.cpp)    			#R5
target_link_libraries(MedicationRobot Robot Poi StaticPoi)

rosbuild_add_executable(NurseRobot src/NurseRobot.cpp)  						#R6
target_link_libraries(NurseRobot Robot Poi StaticPoi)

rosbuild_add_executable(Caregiver src/Caregiver.cpp)							#R7
target_link_libraries(Caregiver Robot Poi StaticPoi)

rosbuild_add_executable(EntertainmentRobot src/EntertainmentRobot.cpp)          #R8
target_link_libraries(EntertainmentRobot Robot Poi StaticPoi)

rosbuild_add_executable(FeedingRobot src/FeedingRobot.cpp)  					#R9
target_link_libraries(FeedingRobot Robot EscortRobot Poi StaticPoi)

rosbuild_add_executable(DoctorRobot src/DoctorRobot.cpp)                        #R10
target_link_libraries(DoctorRobot Robot EscortRobot Poi StaticPoi)
