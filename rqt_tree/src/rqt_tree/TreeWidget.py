import os
import rospy
import rospkg


from python_qt_binding import loadUi
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *

class TreeWidget(QWidget):

    def __init__(self):
        super(TreeWidget, self).__init__()
        self.setObjectName('TreeWidget')

        # Get path to UI file which is a sibling of this file
	rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_tree'), 'resources', 'TreeWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.setObjectName('TreeWidgetUi')

	##only for testing
	model = QStandardItemModel()
	for k in range(0, 4):
	    parentItem = model.invisibleRootItem()
	    for i in range(0, 4):
		item = QStandardItem(("row %d, column %d") % (k, i))
		parentItem.appendRow(item)
		parentItem = item
	self.item_tree_view.setModel(model)
	##

   
    def on_show_nodes_check_box_state_changed(self, activated):
	"""Displays or delete the nodes in the box wether the check box is set or unset.

	:param activated: 2 if checkBox is set, 0 if checkBox is unset
	:type activated: Integer
	"""
	pass

    def on_show_hosts_check_box_state_changed(self, activated):
	"""Displays or delete the hosts in the box wether the checkBox is set or unset.

	:param activated: 2 if checkBox is set, 0 if check is unset
	:type activated: Integer
	"""
	pass

    def on_show_topics_check_box_state_changed(self, activated):
	"""Displays or delete the topics in the box wether the checkBox is set or unset.

	:param activated: 2 if checkBox is set, 0 if check is unset
	:type activated: Integer
	"""
	pass

    def on_show_connections_check_box_state_changed(self, activated):
	"""Displays or delete the connections in the box wether the checkBox is set or unset.

	:param activated: 2 if checkBox is set, 0 if check is unset
	:type activated: Integer
	"""
	pass

    def on_show_erroneous_check_box_state_changed(self, activated):
	"""If this checkBox is set, only erroneous hosts and nodes will be displayed.

	:param activated: 2 if checkBox is set, 0 if check is unset
	:type activated: Integer
	"""
	pass

    def on_apply_push_button_clicked(self):
	"""Filters the content in the box according to the content of the filter_line_edit"""
	pass

    def on_minus_push_button_clicked(self):
	"""Checks if the minus_push_button is clicked and zoomes out (decrease the size of the font)"""
	pass

    def on_plus_push_button_clicked(self):
	"""Checks if the plus_push_button is clicked and zoomes in (increase the size of the font)"""
	pass

    def on_item_in_item_tree_view_double_clicked(self, item):
	"""Handels the double-click action and opens the clicked item in the SelectionWidget

	:param item: the double-clicked item
	:type item: QModelIndex
	"""
	pass

