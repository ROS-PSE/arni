import os
from threading import Lock
import math
import numpy as np

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QPixmap
from python_qt_binding.QtCore import QObject, Qt

from rospy.rostime import Time, Duration
from rospy.timer import Timer

from arni_gui.ros_model import ROSModel
from arni_gui.log_filter_proxy import LogFilterProxy
from arni_gui.log_delegate import LogDelegate
from arni_gui.helper_functions import ResizeableGraphicsLayoutWidget, DateAxis


try:
    import pyqtgraph as pg
except ImportError as e:
    print("An error occured trying to import pyqtgraph. Please install pyqtgraph via \"pip install pyqtgraph\".")
    raise


class SelectionWidget(QWidget):
    """
    The SelectionWidget of the ArniGuiDetail-Plugin.
    """
    
    def __init__(self, model):
        """
        Initializes the Widget.
        
        :param model: the model of the widget
        :type model: ROSModel
        """
        super(SelectionWidget, self).__init__()
        self.setObjectName('selection_widget')
        self.__model = model

        # Get path to UI file which is a sibling of this file
        self.rp = rospkg.RosPack()
        ui_file = os.path.join(self.rp.get_path('arni_rqt_detail_plugin'), 'resources', 'SelectionWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.setObjectName('SelectionWidgetUi')

        self.__selected_item = None


        self.__draw_graphs = True
        self.__current_combo_box_index = 0

        self.__last_update = rospy.Time.now()

        try:
            if rospy.get_param("/enable_statistics") == False:
                raise KeyError('/enable_statistics')
        except KeyError:
            self.__overview_widget = None
            raise EnvironmentError("/enable_statistics is either not set or set to false - arni gui would not work correctly. Please make sure to start "
                             "roscore with the neccesary parameters or to load these while running (see init_params.launch)")

        self.__values_dict = {
            "bandwidth_mean": 0,
            "bandwidth_stddev": 0,
            "bandwidth_max": 0,
        }

        self.__logger = self.__model.get_logger()
        self.__log_filter_proxy = LogFilterProxy()       
        self.__log_filter_proxy.filter_by_item(self.__selected_item)
        self.__log_filter_proxy.setDynamicSortFilter(True)        
        self.__log_filter_proxy.setSourceModel(self.__logger.get_representation())
        self.log_tab_tree_view.setModel(self.__log_filter_proxy)
        self.__log_delegate = LogDelegate()
        self.log_tab_tree_view.setItemDelegate(self.__log_delegate)

        self.__style_string = ".detailed_data {\n" \
                               "    font-size: 12\n;" \
                               "}\n"
        self.__style_string = ".erroneous_entry {\n" \
                               "    color: red\n;" \
                               "}\n"

        self.information_tab_text_browser.setStyleSheet(self.__style_string)

        self.range_combo_box.clear()
        self.range_combo_box.addItem("10 " + self.tr("Seconds"))
        self.range_combo_box.addItem("30 " + self.tr("Seconds"))
        self.range_combo_box.addItem("60 " + self.tr("Seconds"))
        self.range_combo_box.setCurrentIndex(0)

        #self.scrollAreaWidgetContents_2.setWidget(self.host_node_label)

        self.tab_widget.setTabText(0, self.tr("Information"))
        self.tab_widget.setTabText(1, self.tr("Graphs"))
        self.tab_widget.setTabText(2, self.tr("Log"))
        self.tab_widget.setTabText(3, self.tr("Actions"))

        self.stop_push_button.setText(self.tr("Stop Node"))
        self.restart_push_button.setText(self.tr("Restart Node"))
        self.stop_push_button.setEnabled(False)
        self.restart_push_button.setEnabled(False)

        self.selected_label.setText(self.tr("Selected") + ":")
        self.range_label.setText(self.tr("Range") + ":")
        
        self.log_tab_tree_view.setRootIsDecorated(False)
        self.log_tab_tree_view.setAlternatingRowColors(True)
        self.log_tab_tree_view.setSortingEnabled(True)
        self.log_tab_tree_view.sortByColumn(1, Qt.AscendingOrder)

        self.__current_range_combo_box_index = 0
        self.__current_selected_combo_box_index = 0

        self.set_selected_item(self.__selected_item)
        self.__model.layoutChanged.connect(self.update)

        self.__state = "ok"
        self.__previous_state = "ok"

        self.__selected_item_changed = True

        self.__deleted = False
        
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        self.__graph_layout = ResizeableGraphicsLayoutWidget(self.__on_graph_window_size_changed)
        self.graph_scroll_area.setWidget(self.__graph_layout)
        self.__plotable_items = None#self.__selected_item.get_plotable_items()
        self.__items_per_group = 1
        self.__expected_items_per_group = 1
        self.__number_of_groups = 1

        self.__update_graphs_lock = Lock()
        self.__first_update_pending = True

        self.__graph_dict = {}

        self.__plotted_curves = {}
        #self.create_graphs()

        self.__timer = Timer(Duration(secs=1.0), self.update_graphs)

    def __del__(self):
        self.__deleted = True

    def connect_slots(self):
        """
        Connects the slots.
        """
        # : tab_widget
        self.tab_widget.currentChanged.connect(self.__on_current_tab_changed)
        #: restart_push_button
        self.restart_push_button.clicked.connect(self.__on_restart_push_button_clicked)
        #: stop_push_button
        self.stop_push_button.clicked.connect(self.__on_stop_push_button_clicked)
        #: range_combo_box
        self.range_combo_box.currentIndexChanged.connect(self.__on_range_combo_box_index_changed)
        #pause button
        self.pause_button.clicked.connect(self.__on_pause_button_clicked)
        # selected combo box
        self.selected_combo_box.currentIndexChanged.connect(self.__on_selected_combo_box_index_changed)

    def create_graphs(self):
        """
        Creates the graphs for the plot.
        """
        self.__update_graphs_lock.acquire()
        if self.__selected_item is not None:
            first_iteration = True
            first_view = None
            i = 0
            self.__expected_items_per_group = 0
            self.__graph_layout.clear()

            if len(self.__plotable_items) is 0:
                raise UserWarning()
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


    def __on_graph_window_size_changed(self):
        # getting the size
        if self.__selected_item is not None:
            size = self.__graph_layout.size()
            items_per_group = max(int(math.ceil((size.height() - 100) / 200 + 1)), 1)
            if items_per_group is not self.__items_per_group or self.__selected_item_changed:
                self.__graph_layout.set_blocked(True)
                self.__items_per_group = items_per_group
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


    def set_selected_item(self, index):
        """
        Sets the selected item.

        :param index: the index of the selected item
        :type selected_item: QModelIndex
        """
        if index is not None:
            self.__update_graphs_lock.acquire()
            src_index = index.model().mapToSource(index)
            self.__selected_item = src_index.internalPointer()
            self.__log_filter_proxy.filter_by_item(self.__selected_item)
            if self.__selected_item is not None:
                self.__plotable_items = self.__selected_item.get_plotable_items()
                for key in self.__selected_item.get_list_items():
                    self.__plotable_items.remove(key)
                if self.__selected_item.can_execute_actions():
                    self.stop_push_button.setEnabled(True)
                    self.restart_push_button.setEnabled(True)
                else:
                    self.stop_push_button.setEnabled(False)
                    self.restart_push_button.setEnabled(False)
                self.__selected_item_changed = True
            self.__update_graphs_lock.release()
            self.update()
            self.__on_graph_window_size_changed()


    def __on_current_tab_changed(self, tab):
        """
        Will be called when switching between tabs.

        :param tab: index of the current tab
        :type tab: int
        """
        if tab is 1:
            self.__draw_graphs = True
        else:
            self.__draw_graphs = False


    def __on_restart_push_button_clicked(self):
        """
        Handels the restart button and restarts a host or node.
        """
        if self.__selected_item is not None:
            self.__selected_item.execute_action("restart")


    def __on_stop_push_button_clicked(self):
        """Handels the stop button and stops a host or node."""
        if self.__selected_item is not None:
            self.__selected_item.execute_action("stop")


    def __on_range_combo_box_index_changed(self, index):
        """
        Handels the change of the graph range.

        :param index: the index of the selected range
        :type index: int
        """
        self.__current_combo_box_index = index


    def update_graphs(self, event):
        """
        Updates and redraws the graphs.
        """
        self.__update_graphs_lock.acquire()
        if self.__selected_item is not None:
            if self.__draw_graphs or self.__first_update_pending:
                #print("items per group: " + str(self.__items_per_group))
                #print("combo index: " + str(self.__current_selected_combo_box_index))
                #print("len plot " + str(len(self.__plotable_items)))
                plotable_items = self.__plotable_items[min(self.__current_selected_combo_box_index *
                                    self.__items_per_group, len(self.__plotable_items)):min((self.__current_selected_combo_box_index + 1)
                                                            * self.__items_per_group, len(self.__plotable_items))]
                plotable_data = self.__selected_item.get_items_younger_than(
                    #Time.now() - Duration(secs=self.__combo_box_index_to_seconds(self.__current_range_combo_box_index)),
                    Time.now() - (Duration(secs=(self.__combo_box_index_to_seconds(self.__current_range_combo_box_index) + 10 )) if int(Duration(secs=self.__combo_box_index_to_seconds(self.__current_range_combo_box_index)).to_sec()) <= int(Time.now().to_sec()) else Time(0) ),
                    "window_stop", *plotable_items)
                if "window_stop" in plotable_data:
                    if plotable_data["window_stop"]:
                        temp_time = []
                        temp_content = []

                        length = len(plotable_data["window_stop"])
                        modulo = (length / 200) + 1

                        for i in range(0, length, modulo):
                            # now having maximally 100 items to plot :)
                            temp_time.append(plotable_data["window_stop"][i].to_sec())
                        x = np.array(temp_time)

                        list_entries = self.__selected_item.get_list_items()
                        time_entries = self.__selected_item.get_time_items()

                        for key in plotable_items:
                            if key in list_entries:
                                pass
                            else:
                                if key in time_entries:
                                    for i in range(0, length, modulo):
                                        temp_content.append(float(str(plotable_data[key][i]))/1000000000)
                                else:
                                    for i in range(0, length, modulo):
                                        temp_content.append(plotable_data[key][i])
                                y = np.array(temp_content)
                                del temp_content[:]
                                self.__plotted_curves[key].setData(x=x, y=y)
                    else:
                        pass

            self.__first_update_pending = False
        self.__update_graphs_lock.release()
      
    
    def update(self):
        """
        Updates the widget.
        """
        if not self.__deleted:
            if self.__selected_item is not None:
                data_dict = self.__selected_item.get_latest_data()
                self.__state = data_dict["state"]
                self.host_node_label.setText(self.tr(self.__selected_item.get_seuid()))

                if self.__previous_state is not self.__state:
                    self.__previous_state = self.__state
                    if self.__state == "ok":
                        self.current_status_label.setText(self.tr("ok"))
                        #self.host_node_label.setText(self.tr("Current Status: Ok"))
                        pixmap = QPixmap(os.path.join(self.rp.get_path('arni_rqt_detail_plugin'), 'resources/graphics',
                                                      'block_green.png'))
                    elif self.__state == "warning":
                        self.current_status_label.setText(self.tr("warning"))
                        #self.host_node_label.setText(self.tr("Current Status: Warning"))
                        pixmap = QPixmap(os.path.join(self.rp.get_path('arni_rqt_detail_plugin'), 'resources/graphics',
                                                      'block_orange.png'))
                    elif self.__state == "unknown":
                        self.current_status_label.setText(self.tr("unkown"))
                        #self.host_node_label.setText(self.tr("Current Status: Unkown"))
                        pixmap = QPixmap(os.path.join(self.rp.get_path('arni_rqt_detail_plugin'), 'resources/graphics',
                                                      'block_grey.png'))
                    else: # error or offline
                        self.current_status_label.setText(self.tr(self.__state))
                        pixmap = QPixmap(os.path.join(self.rp.get_path('arni_rqt_detail_plugin'), 'resources/graphics',
                                                      'block_red.png'))
                    self.status_light_label.setPixmap(pixmap)
                content = self.__selected_item.get_detailed_data()

                scroll_value = self.information_tab_text_browser.verticalScrollBar().value()
                self.information_tab_text_browser.setHtml(content)
                self.information_tab_text_browser.verticalScrollBar().setSliderPosition(scroll_value)
            else:
                self.host_node_label.setText(self.tr("No item selected"))
                self.current_status_label.setText(self.tr("Offline"))
                self.information_tab_text_browser.setText(self.tr("Please select an item in the TreeView to get more information"
                                                          " about it"))


    def __combo_box_index_to_seconds(self, index):
        """
        Calculates the range from the combo-box index.
        
        :param index: the index of teh combo-box
        :type index: int
        
        :returns: the seconds of the selected index 
        :rtype: int
        """
        if self.__current_combo_box_index == 0:
            return 10
        elif self.__current_combo_box_index == 1:
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
