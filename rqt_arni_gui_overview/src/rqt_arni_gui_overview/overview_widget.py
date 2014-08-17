import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class overview_widget(QWidget):

    def __init__(self):
        super(overview_widget, self).__init__()
        self.setObjectName('overview_widget')       
      
        # Get path to UI file which is a sibling of this file
	rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_arni_gui_overview'), 'resources', 'OverviewWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.setObjectName('SelectionWidgetUi')

	self.__connect_slots()

    def __connect_slots(self):
        """Initializes the slots of the OverviewPlugin."""
        self.tab_widget.currentChanged.connect(self.__on_current_tab_changed)
        self.range_combo_box.currentIndexChanged.connect(self.__on_range_combo_box_index_changed)

    def __on_current_tab_changed(self, tab):
        """The Plugin wants to get notified when the tab changed so it can e.g. draw the graphs.

	:param tab: the index of the selected tab
	:type tab: int
	"""
        pass

    def __on_range_combo_box_index_changed(self, index):
        """Handels the change of the graph range.
	
	:param index: the index of the selected range
	:type index: int
	"""
        pass

    def update(self):
        """Updates the Plugin and draws the graphs if draw_graphs is true."""
        pass

    def update_graphs(sef):
        """Updates and redraws the graphs"""
        pass

