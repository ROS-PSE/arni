cmake_minimum_required(VERSION 2.8.3)
project(arni_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    rosgraph_msgs
    std_msgs
)

add_message_files(
    FILES
    HostStatistics.msg
    MasterApi.msg
    MasterApiEntity.msg
    NodeStatistics.msg
    RatedStatistics.msg
    RatedStatisticsEntity.msg
)

add_service_files(
    FILES
    NodeReaction.srv
    StatisticHistory.srv
)

generate_messages(
  DEPENDENCIES
  # Or other packages containing msgs
  rosgraph_msgs  std_msgs

)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arni_msg
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
    CATKIN_DEPENDS message_runtime rosgraph_msgs
)

# add_library(arni_msg
#   src/${PROJECT_NAME}/arni_msg.cpp

# add_executable(arni_msg_node src/arni_msg_node.cpp)

# add_dependencies(arni_msg_node arni_msg_generate_messages_cpp)

# target_link_libraries(arni_msg_node

# install(TARGETS arni_msg arni_msg_node

# catkin_add_gtest(${PROJECT_NAME}-test test/test_arni_msg.cpp)

