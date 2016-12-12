cmake_minimum_required(VERSION 2.8.3)
project(arni_rqt_overview_plugin)


find_package(catkin REQUIRED COMPONENTS qt_gui)
#	COMPONENTS
#	rospy
#  	rqt_gui
#  	rqt_gui_py
#)

catkin_package()
catkin_python_setup()

install(FILES plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/arni_rqt_overview_plugin
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rqt_overview
#  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
#)

catkin_install_python(PROGRAMS scripts/arni_rqt_overview_plugin DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#include_directories(
 # ${catkin_INCLUDE_DIRS}
#)


