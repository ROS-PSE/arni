cmake_minimum_required(VERSION 2.8.3)
project(rqt_overview)


find_package(catkin REQUIRED)
#COMPONENTS
# rospy
#  rqt_gui
#  rqt_gui_py
#)

catkin_package()
catkin_python_setup()

install(FILES plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resource
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS script/rqt_overview
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rqt_overview
#  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
#)


#include_directories(
 # ${catkin_INCLUDE_DIRS}
#)


