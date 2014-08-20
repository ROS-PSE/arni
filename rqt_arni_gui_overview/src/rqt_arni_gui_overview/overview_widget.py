import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from rospy.rostime import Time
from arni_gui.ros_model import ROSModel
from arni_gui.log_filter_proxy import LogFilterProxy

class OverviewWidget(QWidget):

    def __init__(self):
        super(OverviewWidget, self).__init__()
        self.setObjectName('overview_widget')       
      
        # Get path to UI file which is a sibling of this file
	rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_arni_gui_overview'), 'resources', 'OverviewWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.setObjectName('SelectionWidgetUi')
        
        self.__draw_graphs = True
        
        self.__last_update = rospy.Time.now()
        
        #TODO self.__graph_layout =
        
        #TODO self.__graph_dict =
        
        #TODO self.__values_dict =
        
        self.__model = ROSModel(self)
        
        self.__log_filter_proxy = LogFilterProxy(self.log_tab_tree_widget)

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
        if tab is 1:
            self.__draw_graphs = True
        else:
            self.__draw_graphs = False

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

