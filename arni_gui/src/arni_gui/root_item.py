from abstract_item import AbstractItem

from python_qt_binding.QtCore import QObject

class RootItem(AbstractItem):
    def __init__(self, seuid, parent=QObject(), *args):
        super(RootItem, self).__init__("root", parent)
        #AbstractItem.__init__(self, "root", parent)
        self._attributes.extend(["cpu_usage_mean", "cpu_temp_mean", "cpu_usage_max", "cpu_temp_max",
                                        "average_ram_load", "ram_usage_max", "total_traffic", "connected_hosts",
                                        "connected_nodes", "topic_counter", "connection_counter"])

        self.__new_attributes = []
        self.__new_attributes.extend(self._attributes)
        del self._attributes

        for item in self.__new_attributes:
            self._add_data_list(item)

    def get_detailed_data(self):

        data_dict = self.get_latest_data()

        content = "<p style=\"font-size:15px\">"

        content += "total_traffic: " + str(data_dict["total_traffic"]) + "<br>"
        content += "connected_hosts: " + str(data_dict["connected_hosts"]) + "<br>"
        content += "connected_nodes:" + str(data_dict["connected_nodes"]) + "<br>"
        content += "topic_counter" + str(data_dict["topic_counter"]) + "<br>"
        content += "connection_counter: " + str(data_dict["connection_counter"]) + "<br>"
        content += "cpu_usage_max: " + str(data_dict["cpu_usage_max"]) + "<br>"
        content += "cpu_temp_mean: " + str(data_dict["cpu_temp_mean"]) + "<br>"
        content += "average_ram_load: " + str(data_dict["average_ram_load"]) + "<br>"
        content += "cpu_usage_mean:" + str(data_dict["cpu_usage_mean"]) + "<br>"
        content += "cpu_temp_max: " + str(data_dict["cpu_temp_max"]) + "<br>"
        content += "ram_usage_max: " + str(data_dict["ram_usage_max"]) + "<br>"

        content += "</p>"

        return content


    def get_plotable_items(self):
        return ["average_ram_load", "cpu_usage_mean", "total_traffic"]

#todo: what do i need this for?
    def units_of_plotable_items(self):
        return {"average_ram_load": "%",
                "cpu_usage_mean": "%",
                "total_traffic": "%"
                }