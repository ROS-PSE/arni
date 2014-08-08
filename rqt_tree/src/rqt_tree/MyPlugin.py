import os
import rospy
import rospkg

from TreeWidget import TreeWidget
from SelectionWidget import SelectionWidget


from python_qt_binding import loadUi
from python_qt_binding.QtCore import *#Slot
from python_qt_binding.QtGui import *#QWidget

from rqt_gui_py.plugin import Plugin

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        self.setObjectName('MyPlugin')

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
	
	
	self._tree_widget = TreeWidget()
	context.add_widget(self._tree_widget)

	self._selection_widget = SelectionWidget()
	context.add_widget(self._selection_widget)

	self._connect_slots()

#TODO: connect slots from the selectionWidget, this are only treeWidget slots
    def _connect_slots(self):
	"""Initializes the slots from the TreeWidget and the SelectionWidget."""

	#: show_nodes_check_box
	self._tree_widget.show_nodes_check_box.stateChanged.connect(self._tree_widget._on_show_nodes_check_box_state_changed)
	#: show_hosts_check_box
	self._tree_widget.show_hosts_check_box.stateChanged.connect(self._tree_widget._on_show_hosts_check_box_state_changed)
	#: show_topics_check_box
	self._tree_widget.show_topics_check_box.stateChanged.connect(self._tree_widget._on_show_topics_check_box_state_changed)
	#: show_connections_check_box
	self._tree_widget.show_connections_check_box.stateChanged.connect(self._tree_widget._on_show_connections_check_box_state_changed)
	#: show_erroneous_check_box
	self._tree_widget.show_erroneous_check_box.stateChanged.connect(self._tree_widget._on_show_erroneous_check_box_state_changed)
	#: apply_push_button
	self._tree_widget.apply_push_button.clicked.connect(self._tree_widget._on_apply_push_button_clicked)
	#: minus_push_button
	self._tree_widget.minus_push_button.clicked.connect(self._tree_widget._on_minus_push_button_clicked)
	#: plus_push_button
	self._tree_widget.plus_push_button.clicked.connect(self._tree_widget._on_plus_push_button_clicked)
	#: item_tree_view
	self._tree_widget.item_tree_view.doubleClicked.connect(self._tree_widget._on_item_in_item_tree_view_double_clicked)

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

