cmake_minimum_required(VERSION 2.8.3)
project(arni_rqt_detail_plugin)

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
install(PROGRAMS scripts/arni_rqt_detail_plugin
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES resources/SelectionWidget.ui resources/TreeWidget.ui
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
)
install(FILES resources/graphics/Ampel Zeichen.png resources/graphics/block_green.png resources/graphics/block_green_old.png resources/graphics/block_grey.png resources/graphics/block_orange.png resources/graphics/block_red.png resources/graphics/block_red_old.png resources/graphics/collaps.png resources/graphics/expand.png resources/graphics/light_green.png resources/graphics/light_grey.png resources/graphics/light_orange.png resources/graphics/light_red.png resources/graphics/minus.png resources/graphics/plus.png
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources/graphics
)

#catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES rqt_tree
  #CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
 # DEPENDS system_lib
#)

catkin_install_python(PROGRAMS scripts/arni_rqt_detail_plugin
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#include_directories(
 #)

