cmake_minimum_required(VERSION 2.8.3)
project(arni_rqt_overview_plugin)

find_package(catkin REQUIRED COMPONENTS qt_gui)
catkin_python_setup()
#	COMPONENTS
#	rospy
#  	rqt_gui
#  	rqt_gui_py
#)

catkin_package()

install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS scripts/arni_rqt_overview_plugin
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES resources/OverviewWidget.ui
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
)
install(FILES resources/graphics/Ampel Zeichen grey.png resources/graphics/Ampel Zeichen.png resources/graphics/block_green.png resources/graphics/block_red.png resources/graphics/light_green.png resources/graphics/light_orange.png resources/graphics/light_red.png resources/graphics/minus.png resources/graphics/plus.png
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources/graphics
)
install(FILES src/arni_rqt_overview_plugin/test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src/arni_rqt_overview_plugin
)

#catkin_package(
#  LIBRARIES rqt_overview
#  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#)

catkin_install_python(PROGRAMS scripts/arni_rqt_overview_plugin src/arni_rqt_overview_plugin/arni_gui_overview.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#include_directories(
 #)

