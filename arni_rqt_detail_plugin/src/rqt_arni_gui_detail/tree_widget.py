import os
import sys
import rospy
import rospkg

from rospy.rostime import Time, Duration

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QObject, Qt, QRegExp

from arni_gui.ros_model import ROSModel
from arni_gui.size_delegate import SizeDelegate
from arni_gui.item_filter_proxy import ItemFilterProxy

try:  # Qt4 vs Qt5
  from python_qt_binding.QtGui import QSortFilterProxyModel
  from python_qt_binding.QtGui import *
except ImportError:
  from python_qt_binding.QtCore import QSortFilterProxyModel
  from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QAction
from python_qt_binding.QtCore import QPoint

import yaml


class TreeWidget(QWidget):
    """
    The TreeWidget of the ArniGuiDetail-Plugin.
    """

    def __init__(self, model, selection_widget, parent):
        """
        Initializes the widget.
        
        :param model: the model of the widget
        :type model: ROSModel
        :param selection_widget: the selection_widget
        :type selection_widget: QWidget
        :param parent:
        :type parent:
        """
        super(TreeWidget, self).__init__()
        self.setObjectName('treewidget')
        self.__selection_widget = selection_widget
        self.__model = model
        self.parent = parent

        # Get path to UI file which is a sibling of this file
        self.rp = rospkg.RosPack()
        ui_file = os.path.join(self.rp.get_path('arni_rqt_detail_plugin'), 'resources', 'TreeWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.setObjectName('TreeWidgetUi')

        self.__filter_proxy = ItemFilterProxy()
        self.__filter_proxy.setSourceModel(self.__model)
        self.__filter_proxy.setDynamicSortFilter(True)
        self.__filter_proxy.setFilterCaseSensitivity(Qt.CaseInsensitive)
        self.item_tree_view.setModel(self.__filter_proxy)
        self.item_tree_view.setContextMenuPolicy(Qt.CustomContextMenu)

        self.item_tree_view.setRootIsDecorated(True)
        self.item_tree_view.setAlternatingRowColors(True)
        self.item_tree_view.setSortingEnabled(True)
        self.item_tree_view.sortByColumn(1, Qt.AscendingOrder)

        self.__model.layoutChanged.connect(self.update)

        self.__size_delegate = SizeDelegate()
        self.item_tree_view.setItemDelegate(self.__size_delegate)

        self.__font_size = 10
        self.item_tree_view.setStyleSheet("font-size: %dpt;" % self.__font_size)
        self.item_tree_view.expandAll()
        self.__is_expanded = True
        self.__resize_columns()

        self.__relative_font_size = 0

        self.show_nodes_check_box.setText(self.tr("Show Nodes"))
        self.hide_debug_check_box.setText(self.tr("Hide Debug"))
        self.show_topics_check_box.setText(self.tr("Show Topics"))
        self.show_connections_check_box.setText(self.tr("Show Connections"))
        self.show_erroneous_check_box.setText(self.tr("Only Erroneous"))
        self.apply_push_button.setText(self.tr("Apply"))
        self.also_show_subscribers_check_box.setText(self.tr("Also show Subscribers"))

        self.selected_item = self.__model.get_root_item()

        self.__marked_items_map = dict()

        self.__recording_running = False
        self.loaded_specs = 0

    def connect_slots(self):
        """Connects the slots."""
        self.show_nodes_check_box.stateChanged.connect(self.__on_show_nodes_check_box_state_changed)
        self.hide_debug_check_box.stateChanged.connect(self.__on_hide_debug_check_box_state_changed)
        self.show_topics_check_box.stateChanged.connect(self.__on_show_topics_check_box_state_changed)
        self.show_connections_check_box.stateChanged.connect(self.__on_show_connections_check_box_state_changed)
        self.show_erroneous_check_box.stateChanged.connect(self.__on_show_erroneous_check_box_state_changed)
        self.also_show_subscribers_check_box.stateChanged.connect(
            self.__on_also_show_subscribers_check_box_state_changed)
        self.apply_push_button.clicked.connect(self.__on_apply_push_button_clicked)
        self.minus_push_button.clicked.connect(self.__on_minus_push_button_clicked)
        self.plus_push_button.clicked.connect(self.__on_plus_push_button_clicked)

        self.filter_line_Edit.returnPressed.connect(self.__on_apply_push_button_clicked)

        self.expand_push_button.clicked.connect(self.__on_expand_push_button_clicked)

        self.item_tree_view.customContextMenuRequested.connect(self.__contextual_menu)

        self.load_config_push_button.clicked.connect(self.__on_load_config_push_button_clicked)
        self.recording_push_button.clicked.connect(self.__on_recording_push_button_clicked)

    def __on_load_config_push_button_clicked(self):
        filename = QFileDialog.getOpenFileName(self)

        output = os.system("rosparam load " + filename[0] + " /arni/specifications/rqt_arni_loaded" + str(self.loaded_specs))
        os.system("rosservice call /monitoring_node/reload_specifications")
        print("If there just popped up an error message, please make sure the processing node is running / "
              "running correctly.")
        self.loaded_specs += 1

    def __on_recording_push_button_clicked(self):
        storage = []


        if self.__recording_running:  # stop now
            self.recording_push_button.setText("Start recording")
            print("Stopping the recording and saving the data")
            self.__recording_running = False
            abort = False

            for seuid, item in self.__marked_items_map.iteritems():
                storage.append({})
                storage[-1][seuid] = {}

                plotable_items = item.get_plotable_items()
                plotable_data = item.get_items_younger_than(self.start_time, "window_stop", *plotable_items)
                list_entries = item.get_list_items()
                time_entries = item.get_time_items()

                if plotable_data["window_stop"]:
                    length = len(plotable_data["window_stop"])

                    for key in plotable_items:

                        if key not in list_entries:
                            min = sys.maxint
                            max = 0

                            if key in list_entries:
                                # todo: currently not possible to track these!
                                pass
                            else:
                                if key in time_entries:
                                    for i in range(0, length):
                                        value = float(str(plotable_data[key][i])) / 1000000000.0
                                        if value < min:
                                            min = value
                                        if value > max:
                                            max = value
                                else:
                                    for i in range(0, length):
                                        value = plotable_data[key][i]
                                        if value < min:
                                            min = value
                                        if value > max:
                                            max = value

                            # add new min/max pair to the storage
                            storage[-1][seuid][key] = [min, max]

                    filename = QFileDialog.getSaveFileName(self)

                else:
                    abort = True
                    QMessageBox.warning(self, "Warning", "Not enough data for elements provided. "
                                                         "Please try recording for a longer period of time"
                                                     " or start further components. ")

            if not abort and filename[0] is not u"":
                with open(filename[0], u"w") as outfile:
                    outfile.write(yaml.dump(storage, default_flow_style=False))



        else:  # start now
            if len(self.__marked_items_map) == 0:
                QMessageBox.warning(self, "Warning", "You did not mark any items for recording. Please do so "
                                                     "before pushing the button. Aborting the recording.")
            else:
                print("Started recording")
                self.recording_push_button.setText("Stop recording")
                self.start_time = Time.now()

                self.__recording_running = True

    def __contextual_menu(self, point):
        index = self.item_tree_view.indexAt(point)
        src_index = index.model().mapToSource(index)
        self.selected_item = src_index.internalPointer()

        global_pos = self.item_tree_view.header().mapToGlobal(point)

        action = QAction(self)
        action.setText("Mark for Recording")
        action.setCheckable(True)
        if self.selected_item.marked:
            action.setChecked(True)
        else:
            action.setChecked(False)
        action.triggered.connect(self.__item_marked)

        menu = QMenu()
        menu.addAction(action)
        menu.exec_(global_pos)

    def __item_marked(self, marked):
        if marked:
            self.selected_item.marked = True
            self.__marked_items_map[self.selected_item.seuid] = self.selected_item
            # print("marked")
            pass
        else:
            self.selected_item.marked = False
            self.__marked_items_map.pop(self.selected_item.seuid)
            # print("no longer marked")
            pass

    def __on_show_nodes_check_box_state_changed(self, activated):
        """
        Displays or delete the nodes in the box wether the check box is set or unset.

        :param activated: 2 if checkBox is set, 0 if checkBox is unset
        :type activated: Integer
        """
        if activated is 2:
            self.__filter_proxy.show_nodes(True)
        else:
            self.__filter_proxy.show_nodes(False)
            if self.show_topics_check_box.checkState():
                self.show_topics_check_box.click()
            if self.show_connections_check_box.checkState():
                self.show_connections_check_box.click()
            if self.also_show_subscribers_check_box.checkState():
                self.also_show_subscriber_check_box.click()

    def __on_hide_debug_check_box_state_changed(self, activated):
        """
        Displays or delete the hosts in the box wether the checkBox is set or unset.

        :param activated: 2 if checkBox is set, 0 if check is unset
        :type activated: Integer
        """
        if activated is 2:
            self.__filter_proxy.hide_debug(True)
        else:
            self.__filter_proxy.hide_debug(False)


        # legacy code from show_hosts_check_box. Check box was deleted and replaced by hide debug check box.
        # if activated is 2:
        #     self.__filter_proxy.show_hosts(True)
        # else:
        #     self.__filter_proxy.show_hosts(False)
        #     if self.show_nodes_check_box.checkState():
        #         self.show_nodes_check_box.click()
        #     if self.show_topics_check_box.checkState():
        #         self.show_topics_check_box.click()
        #     if self.show_connections_check_box.checkState():
        #         self.show_connections_check_box.click()
        #     if self.also_show_subscribers_check_box.checkState():
        #         self.also_show_subscribers_check_box.click()

    def __on_show_topics_check_box_state_changed(self, activated):
        """
        Displays or delete the topics in the box wether the checkBox is set or unset.

        :param activated: 2 if checkBox is set, 0 if check is unset
        :type activated: Integer
        """
        if activated is 2:
            self.__filter_proxy.show_topics(True)
            if not self.show_nodes_check_box.checkState():
                self.show_nodes_check_box.click()
        else:
            self.__filter_proxy.show_topics(False)
            if self.show_connections_check_box.checkState():
                self.show_connections_check_box.click()
            if self.also_show_subscribers_check_box.checkState():
                self.also_show_check_box.click()

    def __on_show_connections_check_box_state_changed(self, activated):
        """
        Displays or delete the connections in the box wether the checkBox is set or unset.

        :param activated: 2 if checkBox is set, 0 if check is unset
        :type activated: Integer
        """
        if activated is 2:
            self.__filter_proxy.show_connections(True)
            if not self.show_nodes_check_box.checkState():
                self.show_nodes_check_box.click()
            if not self.show_topics_check_box.checkState():
                self.show_topics_check_box.click()
        else:
            self.__filter_proxy.show_connections(False)
            if self.also_show_subscribers_check_box.checkState():
                self.also_show_subscribers_check_box.click()

    def __on_show_erroneous_check_box_state_changed(self, activated):
        """
        If this checkBox is set, only erroneous hosts and nodes will be displayed.

        :param activated: 2 if checkBox is set, 0 if check is unset
        :type activated: Integer
        """
        if activated is 2:
            self.__filter_proxy.setFilterRegExp(QRegExp("error"))
            self.__filter_proxy.setFilterKeyColumn(2)
        else:
            self.__filter_proxy.setFilterRegExp(QRegExp(""))

    def __on_also_show_subscribers_check_box_state_changed(self, activated):
        """
        If this checkBox is set, also subscribers will be displayed.

        :param activated: 2 if checkBox is set, 0 if check is unset
        :type activated: Integer
        """
        if activated is 2:
            self.__filter_proxy.show_subscribers(True)
            if not self.show_nodes_check_box.checkState():
                self.show_nodes_check_box.click()
            if not self.show_topics_check_box.checkState():
                self.show_topics_check_box.click()
            if not self.show_connections_check_box.checkState():
                self.show_connections_check_box.click()
        else:
            self.__filter_proxy.show_subscribers(False)

    def __on_apply_push_button_clicked(self):
        """
        Filters the content in the box according to the content of the filter_line_edit.
        """
        self.__filter_proxy.set_filter_string(self.filter_line_Edit.text())

    def __on_minus_push_button_clicked(self):
        """
        Checks if the minus_push_button is clicked and zoomes out (decrease the size of the font).
        """
        if self.__font_size > 1:
            self.__font_size -= 1
            self.item_tree_view.setStyleSheet("font-size: %dpt;" % self.__font_size)
            self.__resize_columns()

    def __on_plus_push_button_clicked(self):
        """
        Checks if the plus_push_button is clicked and zoomes in (increase the size of the font).
        """
        self.__font_size += 1
        self.item_tree_view.setStyleSheet("font-size: %dpt;" % self.__font_size)
        self.__resize_columns()

    def __on_expand_push_button_clicked(self):
        """
        Lets the Treeview collaps/expand on click.
        """
        pixmap = None
        if self.__is_expanded:
            self.__is_expanded = False
            self.item_tree_view.collapseAll()
        else:
            self.__is_expanded = True
            self.item_tree_view.expandAll()

    def get_relative_font_size(self):
        """
        Returns the ralitve font size.

        :returns: the relative font size
        :rtype: int
        """
        return self.__relative_font_size

    def set_relative_font_size(self, relative_font_size):
        """
        Sets the relative font size.

        :param relative_font_size: the actual relative font size
        :type relative_font_size: int
        """
        if relative_font_size >= 0:
            for i in range(0, relative_font_size):
                self.__on_plus_push_button_clicked()
        else:
            for i in range(relative_font_size, 0):
                self.__on_minus_push_button_clicked()

    def __resize_columns(self):
        """
        Resizes the columns according to their content
        """
        for i in range(0, self.__filter_proxy.columnCount() - 1):
            self.item_tree_view.resizeColumnToContents(i)
        size = self.item_tree_view.columnWidth(1)
        self.item_tree_view.setColumnWidth(1, size / 2 if size > 120 else 160)
