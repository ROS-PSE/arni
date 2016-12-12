import os
from threading import Lock
import math

import rospy
import rospkg

from rospy.rostime import Time, Duration
from rospy.timer import Timer

from python_qt_binding import loadUi
try:  # Qt4 vs Qt5
  from python_qt_binding.QtGui import QTabWidget, QWidget, QLabel, QVBoxLayout, QSizePolicy
except ImportError:
  from python_qt_binding.QtWidgets import QTabWidget, QWidget, QLabel, QVBoxLayout, QSizePolicy
from python_qt_binding.QtCore import QObject, Qt
from python_qt_binding import QtCore
from python_qt_binding.QtCore import QRegExp
from python_qt_binding.QtGui import QPixmap

from arni_gui.ros_model import ROSModel
from arni_gui.log_filter_proxy import LogFilterProxy
from arni_gui.log_delegate import LogDelegate
from arni_gui.helper_functions import ResizeableGraphicsLayoutWidget

from arni_gui.helper_functions import DateAxis

import numpy as np

try:
    import pyqtgraph as pg
except ImportError as e:
    print("An error occured trying to import pyqtgraph. Please install pyqtgraph via \"pip install pyqtgraph\".")
    raise


class OverviewWidget(QWidget):
    """
    The overviewWidget of the ArniGuiOverview-Plugin.
    """
    
    def __init__(self):
        """
        Initializes the widget.
        """
        super(OverviewWidget, self).__init__()

        # Get path to UI file which is a sibling of this file
        self.rp = rospkg.RosPack()
        ui_file = os.path.join(self.rp.get_path('arni_rqt_overview_plugin'), 'resources', 'OverviewWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)

        self.__draw_graphs = False

        self.__log_delegate = LogDelegate()
        self.log_tab_tree_view.setItemDelegate(self.__log_delegate)

        self.__last_update = rospy.Time.now()
        
        self.__model = ROSModel()

        self.__log_filter_proxy = LogFilterProxy()

        self.__logger = self.__model.get_logger()
        self.__style_string = ".detailed_data_overview {\n" \
                               "    font-size: 13\n;" \
                               "}\n"

        self.information_tab_text_browser.setStyleSheet(self.__style_string)

        self.range_combo_box.clear()
        #todo: are these in the right order?
        self.range_combo_box.addItem("10 " + self.tr("Seconds"))
        self.range_combo_box.addItem("30 " + self.tr("Seconds"))
        #todo: adapt time!!!
        self.range_combo_box.addItem("60 " + self.tr("Seconds"))
        self.range_combo_box.setCurrentIndex(0)

        self.tab_widget.setTabText(0, self.tr("Information"))
        self.tab_widget.setTabText(1, self.tr("Graphs"))
        self.tab_widget.setTabText(2, self.tr("Log"))

        self.selected_label.setText(self.tr("Selected") + ":")
        self.range_label.setText(self.tr("Range") + ":")


        self.__log_filter_proxy.filter_by_item(None)
        self.__log_filter_proxy.setDynamicSortFilter(True)

        #set proxy model
        self.__log_filter_proxy.setSourceModel(self.__logger.get_representation())

        self.log_tab_tree_view.setModel(self.__log_filter_proxy)
        self.log_tab_tree_view.setRootIsDecorated(True)
        self.log_tab_tree_view.setAlternatingRowColors(True)
        self.log_tab_tree_view.setSortingEnabled(True)
        self.log_tab_tree_view.sortByColumn(1, Qt.AscendingOrder)

        self.__connect_slots()

        self.__state = "ok"
        self.__previous_state = "ok"

        self.__current_range_combo_box_index = 0
        self.__current_selected_combo_box_index = 0
        self.__last_update = rospy.Time.now()

        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        self.__graph_layout = ResizeableGraphicsLayoutWidget(self.__on_graph_window_size_changed)
        self.graph_scroll_area.setWidget(self.__graph_layout)
        self.__plotable_items = self.__model.get_root_item().get_plotable_items()
        self.__items_per_group = 1
        self.__expected_items_per_group = 1
        self.__number_of_groups = 1

        self.__update_graphs_lock = Lock()
        self.__first_update_pending = True

        self.__graph_dict = {}

        self.__first_resize = True

        self.__plotted_curves = {}
        self.create_graphs()

        self.__timer = Timer(Duration(secs=1.0), self.update_graphs)


    def __del__(self):
        """
        Destructor of the widget.
        """
        self.__draw_graphs = False
        self.__timer.stop()
        del self.__timer

    def create_graphs(self):
        """
        Creates the graphs for the plot.
        """
        self.__update_graphs_lock.acquire()
        first_iteration = True
        first_view = None
        i = 0
        self.__expected_items_per_group = 0
        self.__graph_layout.clear()

        for key in self.__plotable_items[min(self.__current_selected_combo_box_index *
                                            self.__items_per_group, len(self.__plotable_items)):
                                            min((self.__current_selected_combo_box_index + 1)
                                            * self.__items_per_group, len(self.__plotable_items))]:
            plot_widget = None
            if first_iteration:
                first_iteration = False
                date_axis = DateAxis(orientation="bottom")
                first_view = pg.ViewBox()

                plot_widget = self.__graph_layout.addPlot(title=self.tr(key), axisItems={'bottom': date_axis}, viewBox=first_view)
            else:

                date_axis = DateAxis(orientation="bottom")
                view_box = pg.ViewBox()
                plot_widget = self.__graph_layout.addPlot(title=self.tr(key), viewBox=view_box, axisItems={'bottom': date_axis})
                view_box.setXLink(first_view)

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
            self.__expected_items_per_group += 1
        self.__first_update_pending = True
        self.__update_graphs_lock.release()


    def __connect_slots(self):
        """
        Connects the slots.
        """
        self.tab_widget.currentChanged.connect(self.__on_current_tab_changed)
        self.range_combo_box.currentIndexChanged.connect(self.__on_range_combo_box_index_changed)
        self.__model.layoutChanged.connect(self.update)
        self.pause_button.clicked.connect(self.__on_pause_button_clicked)
        self.selected_combo_box.currentIndexChanged.connect(self.__on_selected_combo_box_index_changed)


    def __on_graph_window_size_changed(self):
        # getting the size
        size = self.__graph_layout.size()
        items_per_group = max(int(math.ceil((size.height() - 100) / 200 + 1)), 1)
        if items_per_group is not self.__items_per_group or self.__first_resize:
            self.__first_resize = False
            self.__graph_layout.set_blocked(True)
            self.__items_per_group = 1 if items_per_group < 1 else items_per_group
            self.__number_of_groups = int(math.ceil(len(self.__plotable_items) / float(self.__items_per_group)))
            # change the groups in the widget
            self.selected_combo_box.clear()
            for group in range(0, self.__number_of_groups):
                list = self.__plotable_items[min(group *
                                    self.__items_per_group, len(self.__plotable_items)):min((group + 1)
                                                            * self.__items_per_group, len(self.__plotable_items))]
                content = ""
                for i in range(0, len(list) - 1):
                    content += self.tr(list[i])
                    content += ", "
                content += list[len(list) - 1]
                self.selected_combo_box.addItem(content)
            # redraw
            self.create_graphs()
            self.update_graphs(None)
            self.__graph_layout.set_blocked(False)


    def __on_selected_combo_box_index_changed(self, index):
        """
        Updates what is shown in the graphs

        :param index: the index of the selected range
        :type index: int
        """
        if index is not -1:
            self.__current_selected_combo_box_index = index
            self.create_graphs()
            self.update_graphs(None)


    def __on_pause_button_clicked(self):
        """
        To be called whenever the pause button is clicked. Stops the graphs from updating until the pause button
        is clicked again and the other way.
        """
        if self.__draw_graphs:
            self.__draw_graphs = False
            self.pause_button.setText(self.tr("Continue"))
        else:
            self.__draw_graphs = True
            self.pause_button.setText(self.tr("Pause"))


    def __on_current_tab_changed(self, tab):
        """
        The Plugin wants to get notified when the tab changed so it can e.g. draw the graphs.

        :param tab: the index of the selected tab
        :type tab: int
        """
        if tab is 1:
            if self.pause_button.text() is not "Continue":
                self.__draw_graphs = True
            else:
                self.__draw_graphs = False
        else:
            self.__draw_graphs = False


    def __on_range_combo_box_index_changed(self, index):
        """
        Handels the change of the graph range.

        :param index: the index of the selected range
        :type index: int
        """
        self.__current_range_combo_box_index = index


    def update(self):
        """
        Updates the Plugin and draws the graphs if draw_graphs is true.
        """
        data_dict = self.__model.get_root_item().get_latest_data("state")
        
        self.__state = data_dict["state"]

        if self.__previous_state is not self.__state:
            self.__previous_state = self.__state
            if self.__state == "ok":
                self.status_text_line_edit.setText(self.tr("Current status: Ok"))
                pixmap = QPixmap(os.path.join(self.rp.get_path('arni_rqt_overview_plugin'), 'resources/graphics',
                                              'light_green.png'))
            elif self.__state == "warning":
                self.status_text_line_edit.setText(self.tr("Current status: Warning"))
                pixmap = QPixmap(os.path.join(self.rp.get_path('arni_rqt_overview_plugin'), 'resources/graphics',
                                              'light_orange.png'))
            else:
                self.status_text_line_edit.setText(self.tr("Current status: Error"))
                pixmap = QPixmap(os.path.join(self.rp.get_path('arni_rqt_overview_plugin'), 'resources/graphics',
                                              'light_red.png'))
            self.status_light_label.setPixmap(pixmap)

        if self.information_tab_text_browser:
            scroll_value = self.information_tab_text_browser.verticalScrollBar().value()
            self.information_tab_text_browser.setHtml(self.__model.get_overview_text())
            self.information_tab_text_browser.verticalScrollBar().setSliderPosition(scroll_value)

    def update_graphs(self, event):
        """
        Updates and redraws the graphs.
        """
        self.__update_graphs_lock.acquire()
        if self.__draw_graphs or self.__first_update_pending:

            plotable_items = self.__plotable_items[min(self.__current_selected_combo_box_index *
                                self.__items_per_group, len(self.__plotable_items)):min((self.__current_selected_combo_box_index + 1)
                                                        * self.__items_per_group, len(self.__plotable_items))]
            plotable_data = self.__model.get_root_item().get_items_younger_than(
                Time.now() - (Duration(secs=self.__combo_box_index_to_seconds(self.__current_range_combo_box_index)) if int(Duration(secs=self.__combo_box_index_to_seconds(self.__current_range_combo_box_index)).to_sec()) <= int(Time.now().to_sec()) else Time(0) ),
                "window_stop", *plotable_items)
            temp_time = []
            temp_content = []

            if plotable_data["window_stop"]:
                modulo = (len(plotable_data["window_stop"]) / 200) + 1

                length = len(plotable_data["window_stop"])
            else:
                length = 0
                modulo = 1
            for i in range(0, length, modulo):
                # now having maximally 100 items to plot :)
                temp_time.append(int(str(plotable_data["window_stop"][i]))/1000000000)
            x = np.array(temp_time)


            for key in plotable_items:
                for i in range(0, length, modulo):
                    temp_content.append(plotable_data[key][i])
                y = np.array(temp_content)
                del temp_content[:]

                self.__plotted_curves[key].setData(x=x, y=y)
                
        self.__first_update_pending = False
        self.__update_graphs_lock.release()


    def __combo_box_index_to_seconds(self, index):
        """
        Calculates the range from the combo-box index.
        
        :param index: the index of teh combo-box
        :type index: int
        
        :returns: the seconds of the selected index 
        :rtype: int
        """
        if self.__current_range_combo_box_index == 0:
            return 10
        elif self.__current_range_combo_box_index == 1:
            return 30
        else:
            return 60


    def get_current_tab(self):
        """
        Returns the current tab.
        
        :returns: the current tab 
        :rtype: int
        """
        return self.tab_widget.currentIndex()


    def set_current_tab(self, index=0):
        """
        Sets the default tab.
        
        :param index: the index of the tab
        :type index: int
        """
        if index is None:
            index = 0
        self.tab_widget.setCurrentIndex(index)
        # WARNING: PROBABLY DOUBLE CALL OF __ON_CURRENT_TAB_CHANGED HERE BUT CANNOT BE FIXED SO EASILY
        self.__on_current_tab_changed(index)



    def get_range_combo_box_index(self):
        """
        Returns the index of the combo-box.
        
        :returns: the index 
        :rtype: int
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
