import os
import rospy
import rospkg


from python_qt_binding import loadUi

#imoprt only when slots will be connected in this class: from python_qt_binding.QtCore import *

from python_qt_binding.QtGui import QWidget

class SelectionWidget(QWidget):

    def __init__(self):
        super(SelectionWidget, self).__init__()
        self.setObjectName('SelectionWidget')       
      
        # Get path to UI file which is a sibling of this file
	rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_tree'), 'resources', 'SelectionWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.setObjectName('SelectionWidgetUi')

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

