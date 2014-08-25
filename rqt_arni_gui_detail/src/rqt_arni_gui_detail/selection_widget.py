import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QPixmap
from python_qt_binding.QtCore import QObject

from rospy.rostime import Time
from arni_gui.ros_model import ROSModel
from arni_gui.log_filter_proxy import LogFilterProxy


class SelectionWidget(QWidget):
    def __init__(self):
        super(SelectionWidget, self).__init__()
        self.setObjectName('selection_widget')

        # Get path to UI file which is a sibling of this file
        self.rp = rospkg.RosPack()
        ui_file = os.path.join(self.rp.get_path('rqt_arni_gui_detail'), 'resources', 'SelectionWidget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.setObjectName('SelectionWidgetUi')

        self.__selected_item = None

        self.__draw_graphs = True

        self.__last_update = rospy.Time.now()

        # TODO self.__graph_layout =

        #TODO self.__graph_dict =

        #TODO fill the dict
        self.__values_dict = {
            "bandwith_mean": 0,
            "bandwith_stddev": 0,
            "bandwith_max": 0,
        }	

        self.__model = ROSModel()

        self.__log_model = self.__model.get_log_model()
        self.__log_filter_proxy = LogFilterProxy()       
        self.__log_filter_proxy.filter_by_item(self.__selected_item)
        self.__log_filter_proxy.setDynamicSortFilter(True)        
        self.__log_filter_proxy.setSourceModel(self.__log_model)        
        self.log_tab_tree_view.setModel(self.__log_filter_proxy)
        
        #todo: should this be false?
        self.log_tab_tree_view.setRootIsDecorated(True)
        # todo: test: eventually remove this
        self.log_tab_tree_view.setAlternatingRowColors(True)
        self.log_tab_tree_view.setSortingEnabled(True)

        self.__model.layoutChanged.connect(self.update)

        self.__state = "ok"
        self.__previous_state = "ok"
        pixmap = QPixmap(os.path.join(self.rp.get_path('rqt_arni_gui_detail'), 'resources/graphics',
                                              'light_red.png'))
	self.status_light_label.setPixmap(pixmap)
        

    def connect_slots(self):
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

    def set_selected_item(self, selected_item):
        """Set the selected item.

        :param selected_item: the selected item
        :type selected_item: item
        """
        self.__selected_item = selected_item
        self.__on_changed_selected_item(self.__selected_item)

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
        self.__log_filter_proxy.filter_by_item(self.__selected_item)
        self.update()

    def update_graphs(self):
        """Updates the graph plot.
        """
        pass
      
    
    def update(self):
        data_dict = self.__model.data(self.__selected_item, 0)
        
        self.__state = data_dict["state"]
        
        if self.__previous_state is not self.__state:
            self.__previous_state = self.__state
            if self.__state == "ok":
                self.status_text_line_edit.setText("Current status: ok")
                pixmap = QPixmap(os.path.join(self.rp.get_path('rqt_arni_gui_detail'), 'resources/graphics',
                                              'light_green.png'))
            elif self.__state == "warning":
                self.status_text_line_edit.setText("Current status: warning")
                pixmap = QPixmap(os.path.join(self.rp.get_path('rqt_arni_gui_detail'), 'resources/graphics',
                                              'light_orange.png'))
            else:
                self.status_text_line_edit.setText("Current status: error")
                pixmap = QPixmap(os.path.join(self.rp.get_path('rqt_arni_gui_detail'), 'resources/graphics',
                                              'light_red.png'))
            self.status_light_label.setPixmap(pixmap)
        
        content = ""
        
        #TODO fil content
        content += "bandwith_mean: " + str(data_dict["bandwith_mean"]) + "<br>"
        content += "bandwith_stddev: " + str(data_dict["bandwith_stddev"]) + "<br>"
        content += "bandwith_meanmax: " + str(data_dict["bandwith_max"]) + "<br>"
   
        self.information_tab_text_browser.setHtml(content)