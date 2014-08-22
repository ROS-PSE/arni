import os
import rospy
import rospkg

from tree_widget import TreeWidget
from selection_widget import SelectionWidget

from python_qt_binding import loadUi
from python_qt_binding.QtCore import *  # Slot
from python_qt_binding.QtGui import *  # QWidget

from rqt_gui_py.plugin import Plugin


class ArniGuiDetail(Plugin):
    def __init__(self, context):
        super(ArniGuiDetail, self).__init__(context)
        self.setObjectName('arni_gui_detail')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser

        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self.__tree_widget = TreeWidget()
        context.add_widget(self.__tree_widget)

        self.__selection_widget = SelectionWidget()
        context.add_widget(self.__selection_widget)

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

        # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

