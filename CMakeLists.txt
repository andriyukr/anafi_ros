cmake_minimum_required(VERSION 3.0.2)
project(olympe_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  dynamic_reconfigure
  geometry_msgs
  roslib
  sensor_msgs
  tf
  message_generation
)

find_package(cmake_modules REQUIRED)

catkin_python_setup()

add_message_files(
  FILES
  PilotingCommand.msg
  MoveByCommand.msg 
  MoveToCommand.msg 
  SkycontrollerCommand.msg
  CameraCommand.msg
  GimbalCommand.msg
  TargetTrajectory.msg
)

add_service_files(
  FILES
  PilotedPOI.srv
  FlightPlan.srv
  FollowMe.srv
  Location.srv
  String.srv
)

generate_messages(
  DEPENDENCIES 
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/setAnafi.cfg
  cfg/setAnafiThermal.cfg 
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime 
  rospy 
  std_msgs
)
