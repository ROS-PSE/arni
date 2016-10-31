try:  # Qt4 vs Qt5
  from python_qt_binding.QtGui import QStyledItemDelegate
except ImportError:
  from python_qt_binding.QtWidgets import QStyledItemDelegate
from python_qt_binding.QtGui import QFont, QBrush, QColor
from helper_functions import choose_brush
from python_qt_binding.QtCore import QObject, QSize

class SizeDelegate(QStyledItemDelegate):
    """
    Makes it possible to change the background-color of a cell in the Model.
    """
    
    def __init__(self, parent=None):
        super(SizeDelegate, self).__init__(parent)
        self.__current_font_size = 10
   
     
    def initStyleOption(self, option, index):
        """
        Manupulates the background-color of a cell in the model.

        :param option: The options parameter
        :type option: QStyleOptionViewItem
        :param index: The QModelIndex that will be painted
        :type index: QModelIndex
        """
        super(SizeDelegate,self).initStyleOption(option, index)
        temp = index.model().mapToSource(index)
        if temp.internalPointer().marked:
            option.backgroundBrush = QBrush(QColor(255, 165, 0, 70))
        else:
            option.backgroundBrush = QBrush(choose_brush(index))
