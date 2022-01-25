cmake_minimum_required(VERSION 2.8.3)
project(arni_nodeinterface)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
#  LIBRARIES arni_countermeasure
)
install(FILES test/test_service.test test/test_statistical_calc.test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)

# add_library(arni_countermeasure
#   src/${PROJECT_NAME}/arni_countermeasure.cpp

# add_executable(arni_countermeasure_node src/arni_countermeasure_node.cpp)

# add_dependencies(arni_countermeasure_node arni_countermeasure_generate_messages_cpp)

# target_link_libraries(arni_countermeasure_node

# install(TARGETS arni_countermeasure arni_countermeasure_node

catkin_install_python(PROGRAMS scripts/arni_nodeinterface test/test_service.py test/test_statistical_calc.py test/writer.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
 

# catkin_add_gtest(${PROJECT_NAME}-test test/test_arni_countermeasure.cpp)

