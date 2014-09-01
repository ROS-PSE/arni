from python_qt_binding.QtGui import QBrush, QColor
from python_qt_binding import QtCore

from arni_msgs.msg import RatedStatistics

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

#todo: apapt to a good number by trial and failure :)
MAXIMUM_AMOUNT_OF_ENTRIES = 10000

"""
The minimum time that should be recorded in the model.
"""
MINIMUM_RECORDING_TIME = 100


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

    return None


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
    if type(number) is str or type(number) is unicode:
        return number
    return str(round(number, 2))


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
