import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class selection_widget(QWidget):

    def __init__(self):
        super(selection_widget, self).__init__()
        self.setObjectName('selection_widget')       
      
        # Get path to UI file which is a sibling of this file
	rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_arni_gui'), 'resources', 'SelectionWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.setObjectName('SelectionWidgetUi')

	self.__connect_slots()

    def __connect_slots(self):
	#: tab_widget
	self.tab_widget.currentChanged.connect(self.__on_current_tab_changed)
	#: restart_push_button
	self.restart_push_button.clicked.connect(self.__on_restart_push_button_clicked)
	#: stop_push_button
	self.stop_push_button.clicked.connect(self.__on_stop_push_button_clicked)
	#: start_push_button
	self.start_push_button.clicked.connect(self.__on_start_push_button_clicked)
	#: range_combo_box
	self.range_combo_box.currentIndexChanged.connect(self.__on_range_combo_box_index_changed)

    def __set_selected_item(self, selected_item):
	"""Set the selected item.
	
	:param selected_item: the selected item
	:type selected_item: item
	"""
	pass

    def __on_current_tab_changed(self, tab):
	"""Will be called when you switch betwee tabs.

	:param tab: index of the current tab
	:type tab: int
	"""
	pass
		
    def __on_restart_push_button_clicked(self):
	"""Handels the restart button and restarts a host or node.
	"""
	pass

    def __on_stop_push_button_clicked(self):
	"""Handels the stop button and stops a host or node.
	"""
	pass

    def __on_start_push_button_clicked(self):
	"""Handels the start button and starts a host or node.
	"""
	pass

    def __on_range_combo_box_index_changed(self, index):
	"""Handels the change of the graph range.
	
	:param index: the index of the selected range
	:type index: int
	"""
	pass	

    def __on_changed_selected_item(self, index):
	"""Handels the change of the selected item.

	:param index: the index of the selected item
	:type index: QModelIndex
	"""	
	pass

    def update_graphs(self):
	"""Updates the graph plot.
	"""
	pass

 

