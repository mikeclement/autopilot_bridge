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
  AutoPilotMsg.msg
  Fencepoint.msg
  GeoPoint.msg
  GeoPose.msg
  GeoPoseWithCovariance.msg
  Geodometry.msg
  Heartbeat.msg
  LLA.msg
  ParamPair.msg
  Rallypoint.msg
  Status.msg
  Waypoint.msg
  WeatherData.msg
)

add_service_files(
  FILES
  FenceGetAll.srv
  FenceSetAll.srv
  FileLoad.srv
  ParamGet.srv
  ParamGetList.srv
  ParamSet.srv
  ParamSetList.srv
  RallyGetAll.srv
  RallySetAll.srv
  ReqPrevNMsgs.srv
  SlaveSetup.srv
  TimedAction.srv
  Version.srv
  WPGetAll.srv
  WPGetRange.srv
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
