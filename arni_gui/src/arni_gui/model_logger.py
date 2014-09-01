from python_qt_binding.QtGui import QStandardItemModel
from rospy.rostime import Time
import time


class ModelLogger:
    """
    Convenience class for logging data.
    """
    
    def __init__(self):
        """
        Initializes the ModelLogger.
        """
        self.__log_model = QStandardItemModel(0, 4, None)
        self.__log_model.setHorizontalHeaderLabels(["type", "date", "location", "message"])


    def log(self, type, date, location, message):
        """
        Adds a log entry to the log_model.

        :param type: the type of the log_entry
        :type type: str
        :param date: the time of the log_entry
        :type date: Time
        :param location: the location, were the fault/info/... occured
        :type location: str
        :param message: the message of the log_entry
        :type message: str
        """
        self.__log_model.insertRow(0)
        self.__log_model.setData(self.__log_model.index(0, 0), str(type))
        self.__log_model.setData(self.__log_model.index(0, 1), time.strftime("%d.%m-%H:%M:%S", time.localtime(int(str(date)) / 1000000000)))
        self.__log_model.setData(self.__log_model.index(0, 2), str(location))
        self.__log_model.setData(self.__log_model.index(0, 3), str(message))


    def get_representation(self):
        """
        Returns the log as a QStandartItemModel

        :returns: the log-model
        :rtype: QStandartItemModel
        """
        return self.__log_model