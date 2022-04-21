cmake_minimum_required(VERSION 3.0.2)
project(lifecycle_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
  DIRECTORY msg
  FILES
  State.msg
  TransitionDescription.msg
  TransitionEvent.msg
  Transition.msg
)

add_service_files(
  DIRECTORY srv  
  FILES
  ChangeState.srv
  GetAvailableTransitions.srv
  GetAvailableStates.srv
  GetState.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime)
