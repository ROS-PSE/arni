import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QTabWidget, QWidget
#from python_qt_binding.QtCore import QObject
from python_qt_binding import QtCore
from python_qt_binding.QtCore import QRegExp

from rospy.rostime import Time

from arni_gui.ros_model import ROSModel
from arni_gui.log_filter_proxy import LogFilterProxy
from arni_gui.log_delegate import LogDelegate

import pyqtgraph as pg

class OverviewWidget(QWidget):
    def __init__(self):
        super(OverviewWidget, self).__init__()
        #self.setObjectName('overview_widget')

        # Get path to UI file which is a sibling of this file
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_arni_gui_overview'), 'resources', 'OverviewWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        #self.setObjectName('SelectionWidgetUi')

        self.__draw_graphs = True

        #self.__log_delegate = LogDelegate()
        #self.log_tab_tree_view.setItemDelegate(self.__log_delegate)

        self.__last_update = rospy.Time.now()

        # TODO self.__graph_layout =

        # TODO self.__graph_dict =

        # TODO self.__values_dict =

        self.__model = ROSModel()

        self.__log_filter_proxy = LogFilterProxy()

        self.__log_model = self.__model.get_log_model()

        self.__connect_slots()

        self.__log_filter_proxy.filter_by_item(None)
        self.__log_filter_proxy.setDynamicSortFilter(True)

        #set proxy model
        self.__log_filter_proxy.setSourceModel(self.__log_model)

        self.log_tab_tree_view.setModel(self.__log_filter_proxy)
        #todo: should this be false?
        self.log_tab_tree_view.setRootIsDecorated(True)
        # todo: test: eventually remove this
        self.log_tab_tree_view.setAlternatingRowColors(True)
        self.log_tab_tree_view.setSortingEnabled(True)




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

    def get_current_tab(self):
        return self.tab_widget.currentIndex()

    def set_current_tab(self, index=0):
        if index is None:
            index = 0
        self.tab_widget.setCurrentIndex(index)

    def get_range_combo_box_index(self):
        return self.range_combo_box.currentIndex()

    def set_range_combo_box_index(self, index=0):
        if index is None:
            index = 0
        self.range_combo_box.setCurrentIndex(index)

