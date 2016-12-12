cmake_minimum_required(VERSION 2.8.3)
project(rqt_arni_gui_overview)


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
install(PROGRAMS scripts/rqt_arni_gui_overview
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rqt_overview
#  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
#)

catkin_install_python(PROGRAMS scripts/rqt_arni_gui_overview DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#include_directories(
 # ${catkin_INCLUDE_DIRS}
#)


