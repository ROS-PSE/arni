import os
import rospy
import rospkg

from tree_widget import TreeWidget
from selection_widget import SelectionWidget

from python_qt_binding import loadUi
from python_qt_binding.QtCore import *  # Slot
from python_qt_binding.QtGui import *  # QWidget

from rqt_gui_py.plugin import Plugin

from arni_gui.ros_model import ROSModel


class ArniGuiDetail(Plugin):
    """The ArniGuiDetail-Plugin"""
    
    def __init__(self, context):
        """Initializes the Plugin"""
        
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

        self.__model = ROSModel()


        #TODO init the selection widget with an given entry from the tree_widget or find another solution for the start of the plugin
        self.__selection_widget = SelectionWidget(self.__model)
        #self.__selection_widget.set_selected_item(None)
        context.add_widget(self.__selection_widget)

        self.__tree_widget = TreeWidget(self.__model, self.__selection_widget)
        context.add_widget(self.__tree_widget)
        self.__tree_widget.connect_slots()
        self.__tree_widget.show_erroneous_check_box.setCheckState(0)

        self.__selection_widget.connect_slots()
        #todo: does this work as expected?
        self.__selection_widget.destroyed.connect(self.__tree_widget.close)
        self.__tree_widget.destroyed.connect(self.__selection_widget.close)
        
        #: is handeld here for the widget communication
        #todo: changed to clicked!
        self.__tree_widget.item_tree_view.clicked.connect(self.__on_item_in_item_tree_view_double_clicked)


    def __on_item_in_item_tree_view_double_clicked(self, index):
        """
        Handels the double-click action and opens the clicked item in the SelectionWidget

        :param item: the double-clicked item
        :type item: QModelIndex
        """
        self.__selection_widget.set_selected_item(index)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass


    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        instance_settings.set_value("tab_widget", self.__selection_widget.get_current_tab())
        instance_settings.set_value("range_combo_box", self.__selection_widget.get_range_combo_box_index())
        instance_settings.set_value("show_nodes_check_box", self.__tree_widget.show_nodes_check_box.checkState())
        instance_settings.set_value("show_hosts_check_box", self.__tree_widget.show_hosts_check_box.checkState())
        instance_settings.set_value("show_topics_check_box", self.__tree_widget.show_topics_check_box.checkState())
        instance_settings.set_value("show_connections_check_box", self.__tree_widget.show_connections_check_box.checkState())
        instance_settings.set_value("show_erroneous_check_box", self.__tree_widget.show_erroneous_check_box.checkState())
        instance_settings.set_value("relative_font_size", self.__tree_widget.get_relative_font_size())


    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        tab_value = instance_settings.value("tab_widget")
        combo_box_value = instance_settings.value("range_combo_box")
        show_nodes_check_box = instance_settings.value("show_nodes_check_box")
        show_hosts_check_box = instance_settings.value("show_hosts_check_box")
        show_topics_check_box = instance_settings.value("show_topics_check_box")
        show_erroneous_check_box = instance_settings.value("show_erroneous_check_box")
        show_connections_check_box = instance_settings.value("show_connections_check_box")
        relative_font_size = instance_settings.value("relative_font_size")

        self.__selection_widget.set_current_tab(0 if tab_value is None else int(tab_value))
        self.__selection_widget.set_range_combo_box_index(0 if combo_box_value is None else int(combo_box_value))
        self.__tree_widget.show_nodes_check_box.setCheckState(2 if show_nodes_check_box is None else int(show_nodes_check_box))
        self.__tree_widget.show_hosts_check_box.setCheckState(2 if show_hosts_check_box is None else int(show_hosts_check_box))
        self.__tree_widget.show_topics_check_box.setCheckState(2 if show_topics_check_box is None else int(show_topics_check_box))
        self.__tree_widget.show_connections_check_box.setCheckState(2 if show_connections_check_box is None else int(show_connections_check_box))
        self.__tree_widget.show_erroneous_check_box.setCheckState(2 if show_erroneous_check_box is None else int(show_erroneous_check_box))
        self.__tree_widget.set_relative_font_size(0 if relative_font_size is None else int(relative_font_size))

        # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

