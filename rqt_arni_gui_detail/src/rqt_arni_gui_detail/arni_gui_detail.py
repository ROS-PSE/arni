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
    """
    The ArniGuiDetail-Plugin
    """
    
    def __init__(self, context):
        """
        Initializes the Plugin
        """
        
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

        self.__selection_widget = SelectionWidget(self.__model)
        context.add_widget(self.__selection_widget)

        self.__tree_widget = TreeWidget(self.__model, self.__selection_widget, self)
        context.add_widget(self.__tree_widget)
        self.__tree_widget.connect_slots()
        self.__tree_widget.show_erroneous_check_box.setCheckState(0)

        self.__selection_widget.connect_slots()
        
        #: is handeld here for the widget communication
        self.__tree_widget.item_tree_view.clicked.connect(self.__on_item_in_item_tree_view_clicked)




    def __on_item_in_item_tree_view_clicked(self, index):
        """
        Handels the click action and opens the clicked item in the SelectionWidget

        :param item: the clicked item
        :type item: QModelIndex
        """
        # the index comes from the proxy_model so it has to be converted but not here
        self.__selection_widget.set_selected_item(index)

    def shutdown_plugin(self):
        pass


    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("tab_widget", self.__selection_widget.get_current_tab())
        instance_settings.set_value("range_combo_box", self.__selection_widget.get_range_combo_box_index())
        instance_settings.set_value("show_nodes_check_box", self.__tree_widget.show_nodes_check_box.checkState())
        instance_settings.set_value("hide_debug_check_box", self.__tree_widget.hide_debug_check_box.checkState())
        instance_settings.set_value("show_topics_check_box", self.__tree_widget.show_topics_check_box.checkState())
        instance_settings.set_value("show_connections_check_box", self.__tree_widget.show_connections_check_box.checkState())
        instance_settings.set_value("also_show_subscribers_check_box", self.__tree_widget.also_show_subscribers_check_box.checkState())
        instance_settings.set_value("show_erroneous_check_box", self.__tree_widget.show_erroneous_check_box.checkState())
        instance_settings.set_value("relative_font_size", self.__tree_widget.get_relative_font_size())


    def restore_settings(self, plugin_settings, instance_settings):
        tab_value = instance_settings.value("tab_widget")
        combo_box_value = instance_settings.value("range_combo_box")
        show_nodes_check_box = instance_settings.value("show_nodes_check_box")
        hide_debug_check_box = instance_settings.value("hide_debug_check_box")
        show_topics_check_box = instance_settings.value("show_topics_check_box")
        show_erroneous_check_box = instance_settings.value("show_erroneous_check_box")
        show_connections_check_box = instance_settings.value("show_connections_check_box")
        also_show_subscribers_check_box = instance_settings.value("also_show_subscribers_check_box")
        relative_font_size = instance_settings.value("relative_font_size")

        self.__selection_widget.set_current_tab(0 if tab_value is None else int(tab_value))

        self.__selection_widget.set_range_combo_box_index(0 if combo_box_value is None else int(combo_box_value))

        state = 2 if show_nodes_check_box is None else int(show_nodes_check_box)
        self.__tree_widget.show_nodes_check_box.setCheckState(state)
        self.__tree_widget.show_nodes_check_box.stateChanged.emit(state)

        state = 2 if hide_debug_check_box is None else int(hide_debug_check_box)
        self.__tree_widget.hide_debug_check_box.setCheckState(state)
        self.__tree_widget.hide_debug_check_box.stateChanged.emit(state)

        state = 2 if show_topics_check_box is None else int(show_topics_check_box)
        self.__tree_widget.show_topics_check_box.setCheckState(state)
        self.__tree_widget.show_topics_check_box.stateChanged.emit(state)

        state = 2 if show_connections_check_box is None else int(show_connections_check_box)
        self.__tree_widget.show_connections_check_box.setCheckState(state)
        self.__tree_widget.show_connections_check_box.stateChanged.emit(state)

        state = 2 if also_show_subscribers_check_box is None else int(also_show_subscribers_check_box)
        self.__tree_widget.also_show_subscribers_check_box.setCheckState(state)
        self.__tree_widget.also_show_subscribers_check_box.stateChanged.emit(state)

        state = 0 if show_erroneous_check_box is None else int(show_erroneous_check_box)
        self.__tree_widget.show_erroneous_check_box.setCheckState(state)
        self.__tree_widget.show_erroneous_check_box.stateChanged.emit(state)

        self.__tree_widget.set_relative_font_size(0 if relative_font_size is None else int(relative_font_size))