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

import pyqtgraph as pg

class OverviewWidget(QWidget):
    def __init__(self):
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
        self.log_tab_tree_view.sortByColumn(1, Qt.AscendingOrder)

        self.__model.layoutChanged.connect(self.update)

        self.__state = "ok"
        self.__previous_state = "ok"

        self.__current_combo_box_index = 0
        self.__last_update = rospy.Time.now()

        self.__graph_layout = pg.GraphicsLayoutWidget()
        #self.__multiplotitem = pg.MultiPlotItem()
        #self.__multiplotitem.setCentralWidget(self.__graph_layout)
        self.graph_scroll_area.setWidget(self.__graph_layout)

        self.__update_graphs_lock = Lock()


        self.__graph_dict = {
            "total_traffic": None,
            "connected_hosts": None,
            "connected_nodes": None,
            "topic_counter": None,
            "connection_counter": None,
            "cpu_usage_max": None,
            "cpu_temp_mean": None,
            "average_ram_load": None,
            "cpu_usage_mean": None,
            "cpu_temp_max": None,
            "ram_usage_max": None
        }
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
        self.__graph_layout.resize(self.__graph_layout.maximumWidth(), len(self.__graph_dict) * 200)
        self.graph_scroll_area.resize(self.graph_scroll_area.maximumWidth(), len(self.__graph_dict) * 200)
        #self.__graph_widget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum))

        for key in self.__model.get_root_item().get_plotable_items():
            #y=np.arrange(0, self.__maximum_values[key], 2)
            date_axis = DateAxis(orientation="bottom")
            values_axis = pg.AxisItem(orientation="left")
            #values_axis.setHeight(h=400)
            #vb = CustomViewBox()
            #plot_widget = pg.PlotWidget()
            plot_widget = self.__graph_layout.addPlot(title=key, axisItems={'bottom': date_axis, "left": values_axis})
            plot_widget.resize(plot_widget.maximumWidth(), 250)
            self.__graph_dict[key] = plot_widget
            self.__graph_layout.nextRow()
            plot_widget = self.__graph_dict[key]
            plot_widget.showGrid(x=True, y=True)
            plot_widget.setMenuEnabled(enableMenu=True)
            plot_widget.enableAutoRange('xy', True)

        #todo: make a first, special update at the beginning (might be that there is fresh data)
        #todo: separate from the layoutChanged signal --> own timer!
        #self.update_graphs(None)
        self.__timer = Timer(Duration(secs=2), self.update_graphs)






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
        #print("current_combo_box index changed\n")
        self.__current_combo_box_index = index

    def update(self):
        """Updates the Plugin and draws the graphs if draw_graphs is true."""
        data_dict = self.__model.get_overview_data_since()

        self.__state = data_dict["state"]
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



        self.information_tab_text_browser.setHtml(self.__model.get_overview_text())

        #self.update_graphs()
        #todo: currently not needed
        self.__last_update = rospy.Time.now()

    def update_graphs(self, event):
        """Updates and redraws the graphs"""
        self.__update_graphs_lock.acquire()
        if self.__draw_graphs is True:
            now = rospy.Time.now()
            #plot_data = self.__model.get_overview_data_since(Time.now() - Duration(secs=self.__combo_box_index_to_seconds(self.__current_combo_box_index)))
            #now plotting
            plotable_items = self.__model.get_root_item().get_plotable_items()
            plotable_data = self.__model.get_root_item().get_items_younger_than(Time.now() - Duration(secs=self.__combo_box_index_to_seconds(self.__current_combo_box_index)), "window_end", *plotable_items)

            #print("length time: " + str(len(plotable_data["window_end"])) + " length data: " + str(len(plotable_data[key])))
            temp_time = []
            temp_content = []

            x = None
            modulo = (len(plotable_data["window_end"]) / 100) + 1

            for i in range(0, len(plotable_data["window_end"]), modulo):
                #now having maximally 100 items to plot :)
                temp_time.append(int(str(plotable_data["window_end"][i]))/1000000000)
                    #print("time" + time.strftime("%d.%m-%H:%M:%S", time.localtime(int(str(item))/1000000000)) + "ms actual time: " + time.strftime("%d.%m-%H:%M:%S", time.localtime(int(str(Time.now()))/1000000000))+ "ms")
                x = np.array(temp_time)
                #del temp_time[:]

            for key in plotable_items:
                #print("length time: " + str(len(plotable_data["window_end"])) + " length data: " + str(len(plotable_data[key])))
                #print("secs = " + str(self.__combo_box_index_to_seconds(self.__current_combo_box_index)))
                for i in range(0, len(plotable_data["window_end"]), modulo):
                    temp_content.append(plotable_data[key][i])
                        #print(x)
                        #print("\n")
                        #todo: does this also work, when ints are inputed (or None values^^). is f8 needed here?
                y = np.array(temp_content, np.dtype('f4'))
                #print(len(temp_time))
                #print(len(temp_content))
                del temp_content[:]
                now2 = rospy.Time.now()
                self.__graph_dict[key].plot(x=x, y=y, fillLevel=0)


                #todo: will this plot every time a new line?



                string = "update_graphs - plot_data took: " + str(int(str(rospy.Time.now() - now2)) / 1000000) + "ms"
                self.__model.add_log_entry("info",  rospy.Time.now(), "OverviewWidget", string)





                #because of autoarrange y should not be set again and again
            string = "update_graphs took: " + str(int(str(rospy.Time.now() - now)) / 1000000) + "ms"
            self.__model.add_log_entry("info",  rospy.Time.now(), "OverviewWidget", string)

        self.__update_graphs_lock.release()


    def __combo_box_index_to_seconds(self, index):
        if self.__current_combo_box_index == 0:
            return 10
        elif self.__current_combo_box_index == 1:
            return 30
        else:
            return 300

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

