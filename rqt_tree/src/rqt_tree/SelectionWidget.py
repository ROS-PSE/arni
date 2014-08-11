import os
import rospy
import rospkg


from python_qt_binding import loadUi
from python_qt_binding.QtCore import *#Slot
from python_qt_binding.QtGui import *#QWidget
#imoprt only when slots will be connected in this class: from python_qt_binding.QtCore import *

#from python_qt_binding.QtGui import QWidget

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

    def set_selected_item(self, selected_item):
	"""Set the selected item.
	
	:param selected_item: the selected item
	:type selected_item: item
	"""
	pass

    def on_current_tab_changed(self, tab):
	"""Will be called when you switch betwee tabs.

	:param tab: index of the current tab
	:type tab: int
	"""
	pass
		
    def on_restart_push_button_clicked(self):
	"""Handels the restart button and restarts a host or node.
	"""
	pass

    def on_stop_push_button_clicked(self):
	"""Handels the stop button and stops a host or node.
	"""
	pass

    def on_start_push_button_clicked(self):
	"""Handels the start button and starts a host or node.
	"""
	pass

    def on_range_combo_box_index_changed(self, index):
	"""Handels the change of the graph range.
	
	:param index: the index of the selected range
	:type index: int
	"""
	pass	

    def on_changed_selected_item(self, index):
	"""Handels the change of the selected item.

	:param index: the index of the selected item
	:type index: QModelIndex
	"""	
	pass

    def update_graphs(self):
	"""Updates the graph plot.
	"""
	pass

 

