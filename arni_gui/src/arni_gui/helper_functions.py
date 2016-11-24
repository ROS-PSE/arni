from python_qt_binding.QtGui import QBrush, QColor
from python_qt_binding import QtCore

from arni_msgs.msg import RatedStatistics

import pyqtgraph as pg

import genpy
import time

# define constants

import struct

"""
update frequency of the model in nsecs
"""
UPDATE_FREQUENCY = 1000000000

"""
The maximal amount of entries allowed in the model (for not crashing it).
Entries are the data_entries in the childs of the model
"""

"""
Note: This value was found by simply guessing. Changing it might result in lower speed / higher memory footprint.
It affects the number of entries in the model.
"""
MAXIMUM_AMOUNT_OF_ENTRIES = 10000

"""
The minimum time that should be recorded in the model.
"""
MINIMUM_RECORDING_TIME = 100

"""
The time intervall after which the topics should be aggregated in nsecs.
"""
TOPIC_AGGREGATION_FREQUENCY = 1000000000

"""
Specifies in seconds the rate at which the alive timer of the items will be called.
"""
ALIVE_TIMER_CALLBACK = 3

"""
Defines how long an item is allowed to be offline before the GUI shows "offline" as the state.
"""
MAXIMUM_OFFLINE_TIME = 20

"""
Defines how long warning will be displayed after an error occured.
"""
WARNING_TIMEOUT = 5

"""
The amount of digits to which the GUI typically rounds. Might not be used everywhere yet.
"""
ROUND_DIGITS = 2

try:
    import pyqtgraph as pg
except ImportError as e:
    print("An error occured trying to import pyqtgraph. Please install pyqtgraph via \"pip install pyqtgraph\".")
    raise


class ResizeableGraphicsLayoutWidget(pg.GraphicsLayoutWidget):
    def __init__(self, function_to_call, parent=None, **kwargs):
        self.function_to_call = function_to_call
        self.__is_blocked = False
        pg.GraphicsLayoutWidget.__init__(self, parent, **kwargs)

    def set_blocked(self, is_blocked):
        self.__is_blocked = is_blocked

    def resizeEvent(self, ev):
        if not self.__is_blocked:
            try:
                self.function_to_call()
            except AttributeError:
                # only occurs when widget is resized before the set_function method is called
                pass
        pg.GraphicsLayoutWidget.resizeEvent(self, ev)


class DateAxis(pg.AxisItem):
    def tickStrings(self, values, scale, spacing):
        strns = []
        if values is None or len(values) is 0:
            pass
        else:
            rng = max(values) - min(values)
            string = '%Y'
            if rng < 3600 * 24:
                string = '%H:%M:%S'
            elif 3600 * 24 <= rng < 3600 * 24 * 30:
                string = '%d'
            elif 3600 * 24 * 30 <= rng < 3600 * 24 * 30 * 24:
                string = '%b'
            elif rng >= 3600 * 24 * 30 * 24:
                string = '%Y'
            for x in values:
                try:
                    strns.append(time.strftime(string, time.localtime(x)))
                except ValueError:  # Windows can't handle dates before 1970
                    strns.append('')
        return strns


def choose_brush(index):
    """
    Chooses the brush according o the content of a cell.
    
    :param index: the index of the item 
    :type index: QModelIndex
    """
    if index.data() == "ok":
        return QColor(127, 255, 0)
    elif index.data() == "warning":
        return QColor(255, 165, 0)
    elif index.data() == "error":
        return QColor(255, 0, 0)

    return QColor(255, 255, 255)


def prepare_number_for_representation(number):
    """
    Prepares the incoming statistics for the GUI.
    Rounds the number to two decimals.

    :param number: the number to be rounded

    :return: the rounded number
    :rtype: str
    """
    if number is None:
        return "unknown"
    if type(number) is int or type(number) is float:
        return str(round(number, ROUND_DIGITS))
    if type(number) is genpy.rostime.Duration or type(number) is genpy.rostime.Time:
        return str(round(number.to_sec(), ROUND_DIGITS))
    if type(number) is str or type(number) is unicode:
        return number
    if isinstance(number, (list, tuple)):
        ret = "["
        for item in number:
            ret += prepare_number_for_representation(item) + ","
        ret = ret[:-1]
        ret += "]"
        return ret
    return str(round(number, ROUND_DIGITS))


def find_qm_files(translation_directory):
    """

    :param translation_directory:
    :return:
    """
    trans_dir = QtCore.QDir(translation_directory)
    fileNames = trans_dir.entryList(["*.qm"], QtCore.QDir.Files, QtCore.QDir.Name)

    fileNames = [trans_dir.filePath(p) for p in fileNames]

    return fileNames


def topic_statistics_state_to_string(element, state):
    """
    Converts the state type from int to string.
    :returns: the string
    :rtype: str
    """
    if state is not None:
        number = struct.unpack('B', state)[0]
        if number is element.OK:
            return "ok"
        elif number is element.HIGH:
            return "high"
        elif number is element.LOW:
            return "low"
        elif number is element.UNKNOWN:
            return "unknown"
    raise TypeError("the state of the element is None or not known")
