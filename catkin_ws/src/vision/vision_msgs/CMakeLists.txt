cmake_minimum_required(VERSION 2.8.3)
project(vision_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  VisionFlattenedObject.msg
  VisionFlattenedObjectList.msg
  MSG_VisionPlasticTray.msg
)

add_service_files(
  FILES
  RecognizeFlattenedObjects.srv
  SRV_DetectPlasticTrayZones.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs   sensor_msgs   std_msgs   vision_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
