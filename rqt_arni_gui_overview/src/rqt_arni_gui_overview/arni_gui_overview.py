import os
import rospy
import rospkg

from overview_widget import OverviewWidget

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, qApp

from rqt_gui_py.plugin import Plugin


class ArniGuiOverview(Plugin):
    """
    The ArniGuiOverview-Plugin.
    """
    
    def __init__(self, context):
        """
        Initializes the Plugin.
        
        :param context: the context for the plugin
        :type context:
        """
        super(ArniGuiOverview, self).__init__(context)
        self.setObjectName('arni_gui_overview')

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

        self.__overview_widget = OverviewWidget()
        context.add_widget(self.__overview_widget)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass


    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        instance_settings.set_value("tab_widget", self.__overview_widget.get_current_tab())
        instance_settings.set_value("range_combo_box", self.__overview_widget.get_range_combo_box_index())


    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        tab_value = instance_settings.value("tab_widget")
        combo_box_value = instance_settings.value("range_combo_box")
        self.__overview_widget.set_current_tab(0 if tab_value is None else int(tab_value))
        self.__overview_widget.set_range_combo_box_index(0 if combo_box_value is None else int(combo_box_value))