cmake_minimum_required(VERSION 2.8.3)
project(imitation_learning_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
)

add_action_files(
  FILES
    Learn.action
    Play.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
)

catkin_package(
 CATKIN_DEPENDS actionlib_msgs message_runtime
)
