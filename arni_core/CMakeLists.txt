cmake_minimum_required(VERSION 2.8.3)
project(arni_core)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

#catkin_install_python(PROGRAMS scripts/   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES test/constraint_node_stop.yaml test/iceland_constraint.yaml test/iceland_specification.yaml test/test_1_3x3_specification.yaml test/test_1_constraint.yaml test/test_1_specification.yaml test/test_2_constraint.yaml test/test_2_specification.yaml test/test_3_constraint.yaml test/test_3_specification.yaml test/test_4_constraint.yaml test/test_4_specification.yaml test/test_5_constraint.yaml test/test_5_specification.yaml test/test_node_shutdown.test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
catkin_install_python(PROGRAMS tutorial/tutorial_listening_mole tutorial/tutorial_publishing_owl tutorial/tutorial_publishing_snake test/predefined_publisher.py test/test_node_shutdown.py test/predefined_subscriber.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
