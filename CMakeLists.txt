cmake_minimum_required(VERSION 2.8.3)
project(autopilot_bridge)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
)

add_message_files(
  FILES
  GeoPoint.msg
  GeoPose.msg
  GeoPoseWithCovariance.msg
  Geodometry.msg
  Heartbeat.msg
  LLA.msg
  ParamPair.msg
  Status.msg
  Waypoint.msg
)

add_service_files(
  FILES
  ParamGet.srv
  ParamGetList.srv
  ParamSet.srv
  SlaveSetup.srv
  WPGetAll.srv
  WPSetAll.srv
)

generate_messages(
  DEPENDENCIES 
  std_msgs 
  sensor_msgs 
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
