import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QTabWidget, QWidget
from python_qt_binding.QtCore import QObject, Qt
#from python_qt_binding.QtCore import *
from python_qt_binding import QtCore
from python_qt_binding.QtCore import QRegExp
from python_qt_binding.QtGui import QPixmap, QLabel, QVBoxLayout, QSizePolicy

from rospy.rostime import Time, Duration
from rospy.timer import Timer

from arni_gui.ros_model import ROSModel
from arni_gui.log_filter_proxy import LogFilterProxy
from arni_gui.log_delegate import LogDelegate
from threading import Lock

from date_axis import DateAxis

import time

import numpy as np

try:
    import pyqtgraph as pg
except ImportError as e:
    print("An error occured trying to import pyqtgraph. Please install pyqtgraph via \"pip install pyqtgraph\".")
    raise


class OverviewWidget(QWidget):
    """The overviewWidget of the ArniGuiOverview-Plugin."""
    
    def __init__(self):
        """Initializes the widget."""
        super(OverviewWidget, self).__init__()
        #self.setObjectName('overview_widget')

        # Get path to UI file which is a sibling of this file
        self.rp = rospkg.RosPack()
        ui_file = os.path.join(self.rp.get_path('rqt_arni_gui_overview'), 'resources', 'OverviewWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        #self.setObjectName('SelectionWidgetUi')

        self.__draw_graphs = True

        self.__log_delegate = LogDelegate()
        self.log_tab_tree_view.setItemDelegate(self.__log_delegate)

        self.__last_update = rospy.Time.now()

        # self.__overview_dict = {
        #     "total_traffic": 0,
        #     "connected_hosts": 0,
        #     "connected_nodes": 0,
        #     "topic_counter": 0,
        #     "connection_counter": 0,
        #     "cpu_usage_max": 0,
        #     "cpu_temp_mean": 0,
        #     "average_ram_load": 0,
        #     "cpu_usage_mean": 0,
        #     "cpu_temp_max": 0,
        #     "ram_usage_max": 0
        # }

        self.__model = ROSModel()

        self.__log_filter_proxy = LogFilterProxy()

        self.__logger = self.__model.get_logger()
        #self.__log_model = self.__model.get_log_model()

        self.information_tab_text_browser.setStyleSheet("font-size: %dpt;" % 12)

        self.__connect_slots()

        self.__log_filter_proxy.filter_by_item(None)
        self.__log_filter_proxy.setDynamicSortFilter(True)

        #set proxy model
        self.__log_filter_proxy.setSourceModel(self.__logger.get_representation())

        self.log_tab_tree_view.setModel(self.__log_filter_proxy)
        #todo: should this be false?
        self.log_tab_tree_view.setRootIsDecorated(True)
        # todo: test: eventually remove this
        self.log_tab_tree_view.setAlternatingRowColors(True)
        self.log_tab_tree_view.setSortingEnabled(True)
        self.log_tab_tree_view.sortByColumn(1, Qt.AscendingOrder)

        self.__model.layoutChanged.connect(self.update)

        self.__state = "ok"
        self.__previous_state = "ok"

        self.__current_combo_box_index = 0
        self.__last_update = rospy.Time.now()

        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        self.__graph_layout = pg.GraphicsLayoutWidget()
        #self.__multiplotitem = pg.MultiPlotItem()
        #self.__multiplotitem.setCentralWidget(self.__graph_layout)
        self.graph_scroll_area.setWidget(self.__graph_layout)

        self.__update_graphs_lock = Lock()


        self.__graph_dict = {}
        #
        # self.__maximum_values = {
        #     "total_traffic": 10,
        #     "connected_hosts": 10,
        #     "connected_nodes": 10,
        #     "topic_counter": 10,
        #     "connection_counter": 10,
        #     "cpu_usage_max": 100,
        #     "cpu_temp_mean": 50,
        #     "average_ram_load": 100,
        #     "cpu_usage_mean": 100,
        #     "cpu_temp_max": 90,
        #     "ram_usage_max": 100
        # }
        #
        # self.__values_dict = {
        #     "total_traffic": None,
        #     "connected_hosts": None,
        #     "connected_nodes": None,
        #     "topic_counter": None,
        #     "connection_counter": None,
        #     "cpu_usage_max": None,
        #     "cpu_temp_mean": None,
        #     "average_ram_load": None,
        #     "cpu_usage_mean": None,
        #     "cpu_temp_max": None,
        #     "ram_usage_max": None
        # }

        #self.__graph_layout = QVBoxLayout()
        #self.__graph_widget = QWidget()
        #self.__graph_widget.setLayout(self.__graph_layout)
        #self.graph_scroll_area.setWidget(self.__graph_widget)
        self.__graph_layout.setMinimumSize(self.__graph_layout.maximumWidth(), 3 * 200)
        #self.graph_scroll_area.resize(self.graph_scroll_area.maximumWidth(), len(self.__graph_dict) * 200)
        #self.__graph_widget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum))

        self.__plotted_curves = {}
        self.create_graphs()

        #todo: make a first, special update at the beginning (might be that there is fresh data)
        #todo: separate from the layoutChanged signal --> own timer!
        #self.update_graphs(None)
        self.__timer = Timer(Duration(secs=1.5), self.update_graphs)


    def __del__(self):
        """Destructor of the widget."""
        self.__timer.stop()
        del self.__timer

    def create_graphs(self):
        """Creates the graphs for the plot."""

        first_iteration = True
        first_view = None
        i = 0

        for key in self.__model.get_root_item().get_plotable_items():
            #y=np.arrange(0, self.__maximum_values[key], 2)
            plot_widget = None
            if first_iteration:
                first_iteration = False
                date_axis = DateAxis(orientation="bottom")
                #values_axis = pg.AxisItem(orientation="left")
                #values_axis.setHeight(h=400)
                #vb = CustomViewBox()
                #plot_widget = pg.PlotWidget()
                first_view = pg.ViewBox()
                #fist_view.setBackgroundColor(color=(0, 0, 0, 100))

                plot_widget = self.__graph_layout.addPlot(title=key, axisItems={'bottom': date_axis}, )#, viewBox=first_view)
            else:

                date_axis = DateAxis(orientation="bottom")
                #values_axis = pg.AxisItem(orientation="left")
                #values_axis.setHeight(h=400)
                #vb = CustomViewBox()
                #plot_widget = pg.PlotWidget()
                view_box = pg.ViewBox()
                #view_box.setBackgroundColor(color=(0, 0, 0, 100))
                # , 'left': values_axis
                plot_widget = self.__graph_layout.addPlot(title=key, viewBox=view_box, axisItems={'bottom': date_axis})
                view_box.setXLink(first_view)
                #print(view_box.viewRect())

            #localUpdatePlots = lambda: self.updatePlots(plot_widget)
            #plot_widget.sigXRangeChanged.connect(localUpdatePlots)
            # has no effect:
            #plot_widget.resize(plot_widget.maximumWidth(), 250)

            #performance enhancements when only a short range of the plot is shown
            #plot_widget.setClipToView(clip=True)
            plot_widget.setYRange(-1, 1)
            self.__graph_dict[key] = plot_widget
            self.__graph_layout.nextRow()
            plot_widget = self.__graph_dict[key]
            plot_widget.showGrid(x=True, y=True)
            plot_widget.setMenuEnabled(enableMenu=True)
            plot_widget.enableAutoRange('xy', True)
            x = np.array([1])
            y = np.array([int(str(Time.now()))/1000000000])
            self.__plotted_curves[key] = plot_widget.plot(x=x, y=y, fillLevel=0, brush=(50, 50, 200, 100),
                                                          pen=(255, 0, 0))

    # def updatePlots(self, changed_item):
    #     """
    #     Updates the range of the plots.
    #
    #     :param changed_item:
    #     :return:
    #     """
    #     new_x_range = changed_item.viewRange()[0]
    #     for entry in self.__graph_dict.values():
    #         if entry is not changed_item:
    #             entry.setXRange(min=new_x_range[0], max=new_x_range[1])


    def __connect_slots(self):
        """Connects the slots."""
        self.tab_widget.currentChanged.connect(self.__on_current_tab_changed)
        self.range_combo_box.currentIndexChanged.connect(self.__on_range_combo_box_index_changed)


    def __on_current_tab_changed(self, tab):
        """
        The Plugin wants to get notified when the tab changed so it can e.g. draw the graphs.

        :param tab: the index of the selected tab
        :type tab: int
        """
        if tab is 1:
            self.__draw_graphs = True
        else:
            self.__draw_graphs = False


    def __on_range_combo_box_index_changed(self, index):
        """
        Handels the change of the graph range.

        :param index: the index of the selected range
        :type index: int
        """
        #print("current_combo_box index changed\n")
        self.__current_combo_box_index = index


    def update(self):
        """Updates the Plugin and draws the graphs if draw_graphs is true."""
        data_dict = self.__model.get_root_item().get_latest_data("state")
        #rated_data_dict = self.__model.get_root_item().get_

        #print(data_dict)
        self.__state = data_dict["state"]
        #print("view: " + self.__state)

        # for testing only:
        #self.__state = "warning"

        if self.__previous_state is not self.__state:
            self.__previous_state = self.__state
            if self.__state == "ok":
                self.status_text_line_edit.setText("Current status: ok")
                pixmap = QPixmap(os.path.join(self.rp.get_path('rqt_arni_gui_overview'), 'resources/graphics',
                                              'light_green.png'))
            elif self.__state == "warning":
                self.status_text_line_edit.setText("Current status: warning")
                pixmap = QPixmap(os.path.join(self.rp.get_path('rqt_arni_gui_overview'), 'resources/graphics',
                                              'light_orange.png'))
            else:
                self.status_text_line_edit.setText("Current status: error")
                pixmap = QPixmap(os.path.join(self.rp.get_path('rqt_arni_gui_overview'), 'resources/graphics',
                                              'light_red.png'))
            self.status_light_label.setPixmap(pixmap)
            #self.status_light_label.setMask(pixmap.mask())
            #self.status_light_label.resize(50, 50)

        if self.information_tab_text_browser:
	    scroll_value = self.information_tab_text_browser.verticalScrollBar().value()
            self.information_tab_text_browser.setHtml(self.__model.get_overview_text())
            self.information_tab_text_browser.verticalScrollBar().setSliderPosition(scroll_value)

        #self.update_graphs()
        #todo: currently not needed
        self.__last_update = rospy.Time.now()


    def update_graphs(self, event):
        """Updates and redraws the graphs."""
        self.__update_graphs_lock.acquire()
        if self.__draw_graphs is True:
            now = rospy.Time.now()
            #plot_data = self.__model.get_overview_data_since(Time.now() - Duration(secs=self.__combo_box_index_to_seconds(self.__current_combo_box_index)))
            #now plotting
            plotable_items = self.__model.get_root_item().get_plotable_items()
            plotable_data = self.__model.get_root_item().get_items_younger_than(Time.now() - Duration(secs=self.__combo_box_index_to_seconds(self.__current_combo_box_index)), "window_stop", *plotable_items)

            #print("length time: " + str(len(plotable_data["window_end"])) + " length data: " + str(len(plotable_data[key])))
            temp_time = []
            temp_content = []

            x = None
            modulo = (len(plotable_data["window_stop"]) / 200) + 1

            length = len(plotable_data["window_stop"])
            for i in range(0, length, modulo):
                # now having maximally 100 items to plot :)
                temp_time.append(int(str(plotable_data["window_stop"][i]))/1000000000)
                    #print("time" + time.strftime("%d.%m-%H:%M:%S", time.localtime(int(str(item))/1000000000)) + "ms actual time: " + time.strftime("%d.%m-%H:%M:%S", time.localtime(int(str(Time.now()))/1000000000))+ "ms")
            x = np.array(temp_time)
                #del temp_time[:]


            for key in plotable_items:
                #print("length time: " + str(length) + " length data: " + str(len(plotable_data[key])))
                for i in range(0, length, modulo):
                    temp_content.append(plotable_data[key][i])
                #todo: does this also work, when ints are inputed (or None values^^). is f8 needed here?
                #print("length content before:" + str(len(temp_content)))
                y = np.array(temp_content)
                #print("length content after:" + str(len(temp_content)))
                # if len(x) is not len(y):
                #     print("another time: length time: " + str(len(plotable_data["window_stop"])) + " length data: " + str(len(plotable_data[key])))
                #     print(temp_time)
                #     print(temp_content)
                #     print("computed values\n")
                #     print(x)
                #     print(y)
                #print(len(temp_time))
                #print(len(temp_content))
                del temp_content[:]
                now2 = rospy.Time.now()

                #print("shortened length time: " + str(len(x)) + " shortened length data: " + str(len(y)))
                self.__plotted_curves[key].setData(x=x, y=y)

                string = "update_graphs - plot_data took: " + str(int(str(rospy.Time.now() - now2)) / 1000000) + "ms"
                self.__logger.log("info",  rospy.Time.now(), "OverviewWidget", string)





                #because of autoarrange y should not be set again and again
            string = "update_graphs took: " + str(int(str(rospy.Time.now() - now)) / 1000000) + "ms"
            self.__logger.log("info",  rospy.Time.now(), "OverviewWidget", string)

        self.__update_graphs_lock.release()


    def __combo_box_index_to_seconds(self, index):
        """
        Calculates the range from the combo-box index.
        
        :param index: the index of teh combo-box
        :type index: int
        
        :returns: int
        """
        if self.__current_combo_box_index == 0:
            return 10
        elif self.__current_combo_box_index == 1:
            return 30
        else:
            return 300


    def get_current_tab(self):
        """
        Returns the current tab.
        
        :returns: int
        """
        return self.tab_widget.currentIndex()


    def set_current_tab(self, index=0):
        """Sets the default tab.
        
        :param index: the index of the tab
        :type index: int
        """
        if index is None:
            index = 0
        self.tab_widget.setCurrentIndex(index)


    def get_range_combo_box_index(self):
        """
        Returns the index of the combo-box.
        
        :returns: int
        """
        return self.range_combo_box.currentIndex()


    def set_range_combo_box_index(self, index=0):
        """
        Sets the default value of the combo-box.
        
        :param index: the index of the combo-box
        :type index: int
        """
        if index is None:
            index = 0
        self.range_combo_box.setCurrentIndex(index)

