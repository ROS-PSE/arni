import os
import rospy
import rospkg


from python_qt_binding import loadUi
from python_qt_binding.QtCore import *#Slot
from python_qt_binding.QtGui import *#QWidget

from rqt_gui_py.plugin import Plugin

class TreePlugin(Plugin):

    def __init__(self, context):
        super(TreePlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TreePlugin')

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

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
	rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_tree'), 'resources', 'TreeWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('TreePluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

	
	##only for testing
	model = QStandardItemModel()
	for k in range(0, 4):
	    parentItem = model.invisibleRootItem()
	    for i in range(0, 4):
		item = QStandardItem(("row %d, column %d") % (k, i))
		parentItem.appendRow(item)
		parentItem = item
	self._widget.item_tree_view.setModel(model)
	##
	
	self._connect_slots()

#TODO: Comments !!!!
    def _connect_slots(self):
	self._widget.show_nodes_check_box.stateChanged.connect(self._on_show_nodes_check_box_state_changed)
	self._widget.show_hosts_check_box.stateChanged.connect(self._on_show_hosts_check_box_state_changed)
	self._widget.show_topics_check_box.stateChanged.connect(self._on_show_topics_check_box_state_changed)
	self._widget.show_connections_check_box.stateChanged.connect(self._on_show_connections_check_box_state_changed)
	self._widget.show_erroneous_check_box.stateChanged.connect(self._on_show_erroneous_check_box_state_changed)
	self._widget.apply_push_button.clicked.connect(self._on_apply_push_button_clicked)
	self._widget.minus_push_button.clicked.connect(self._on_minus_push_button_clicked)
	self._widget.plus_push_button.clicked.connect(self._on_plus_push_button_clicked)
	self._widget.item_tree_view.doubleClicked.connect(self._on_item_in_item_tree_view_double_clicked)

    def _on_show_nodes_check_box_state_changed(self):
	self._widget.filter_line_Edit.setText("Nodes")

    def _on_show_hosts_check_box_state_changed(self):
	self._widget.filter_line_Edit.setText("Hosts")

    def _on_show_topics_check_box_state_changed(self):
	self._widget.filter_line_Edit.setText("Topics")

    def _on_show_connections_check_box_state_changed(self):
	self._widget.filter_line_Edit.setText("Connections")

    def _on_show_erroneous_check_box_state_changed(self):
	self._widget.filter_line_Edit.setText("Erroneous")

    def _on_apply_push_button_clicked(self):
	self._widget.filter_line_Edit.setText("Applay-Push-Button")

    def _on_minus_push_button_clicked(self):
	self._widget.filter_line_Edit.setText("Minus-Push-Button")

    def _on_plus_push_button_clicked(self):
	self._widget.filter_line_Edit.setText("Plus-Push-Button")

    def _on_item_in_item_tree_view_double_clicked(self):
	self._widget.filter_line_Edit.setText("Double-Click")

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

