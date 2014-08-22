import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from rospy.rostime import Time
from arni_gui.ros_model import ROSModel
from arni_gui.log_filter_proxy import LogFilterProxy


class SelectionWidget(QWidget):
    def __init__(self):
        super(SelectionWidget, self).__init__()
        self.setObjectName('selection_widget')

        # Get path to UI file which is a sibling of this file
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_arni_gui_detail'), 'resources', 'SelectionWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.setObjectName('SelectionWidgetUi')

        self.__draw_graphs = True

        self.__last_update = rospy.Time.now()

        # TODO self.__graph_layout =

        #TODO self.__graph_dict =

        #TODO self.__values_dict =

        self.__model = ROSModel(self)

        self.__log_filter_proxy = LogFilterProxy(self.log_tab_list_widget)

        self.__connect_slots()

    def __connect_slots(self):
        # : tab_widget
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
        """Will be called when you switch between tabs.

        :param tab: index of the current tab
        :type tab: int
        """
        if tab is 1:
            self.__draw_graphs = True
        else:
            self.__draw_graphs = False

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

 

