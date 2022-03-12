cmake_minimum_required(VERSION 2.8.3)
project(arni_processing)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
#  LIBRARIES arni_countermeasure
)
install(FILES resources/testconfig.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
)
install(FILES test/test_loading_specifications.test test/test_rating_data.test test/test_seuid.test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
install(FILES test/mockdata/keksconf.yaml test/mockdata/mock_host.yaml test/mockdata/mock_host_spec.yaml test/mockdata/mock_host_spec2.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test/mockdata
)

# add_library(arni_countermeasure
#   src/${PROJECT_NAME}/arni_countermeasure.cpp

# add_executable(arni_countermeasure_node src/arni_countermeasure_node.cpp)

# add_dependencies(arni_countermeasure_node arni_countermeasure_generate_messages_cpp)

# target_link_libraries(arni_countermeasure_node

# install(TARGETS arni_countermeasure arni_countermeasure_node

catkin_install_python(PROGRAMS scripts/arni_processing test/test_loading_specifications.py test/test_rating_data.py test/test_seuid.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# catkin_add_gtest(${PROJECT_NAME}-test test/test_arni_countermeasure.cpp)

