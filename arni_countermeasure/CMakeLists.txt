cmake_minimum_required(VERSION 2.8.3)
project(arni_countermeasure)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()
install(FILES src/arni_countermeasure/changes.md
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src/arni_countermeasure
)
install(FILES tests/test_parsing_constraint.test tests/test_reaction.test tests/test_scenario.test tests/test_storage.test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)
install(FILES tests/config/default.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/config
)
install(FILES tests/constraints/john_high_publish.yaml tests/constraints/test_scenario.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/constraints
)

catkin_install_python(PROGRAMS scripts/arni_countermeasure tests/increased_clock.py tests/test_parsing_constraint.py tests/test_reaction.py tests/test_scenario.py tests/test_storage.py 
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 

