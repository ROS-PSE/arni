__author__ = 'Matthias Hadlich'

from python_qt_binding.QtGui import QStyledItemDelegate, QFont, QBrush, QColor
from helper_functions import choose_brush


class LogDelegate(QStyledItemDelegate):
    """
    Makes it possible to change the background-color of the log-model.
    """

    def __init__(self, parent=None):
        """
        Initializes the LogDelegate
        """
        super(LogDelegate, self).__init__(parent)

    def initStyleOption(self, option, index):
        """
        Manupulates the background-color of a cell in the model.

        :param option: The options parameter
        :type option: QStyleOptionViewItem
        :param index: The QModelIndex that will be painted
        :type index: QModelIndex
        """
        super(LogDelegate, self).initStyleOption(option, index)
        option.backgroundBrush = QBrush(choose_brush(index))
