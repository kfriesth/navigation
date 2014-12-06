cmake_minimum_required(VERSION 2.8.3)
project(move_base_msgs)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        message_generation
        actionlib_msgs
        geometry_msgs
        std_msgs
        )


# actions
add_action_files(
    DIRECTORY action
    FILES
        MoveBase.action
)

# services
add_service_files(
  DIRECTORY srv
  FILES
  ChangePlanner.srv
)

generate_messages(
    DEPENDENCIES
        actionlib_msgs
        std_msgs
        geometry_msgs
)

catkin_package(
)


