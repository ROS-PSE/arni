__author__ = 'Matthias Hadlich'

from python_qt_binding.QtGui import QStyledItemDelegate, QFont, QBrush, QColor
from helper_functions import choose_brush

class LogDelegate(QStyledItemDelegate):
    """Makes it possible to change the font size of the Gui-Plugin content."""

    def __init__(self, parent=None):
        super(LogDelegate, self).__init__(parent)

    # def paint(self, painter, option, index):
    #     """Defines how the items of the model will be painted in the view.
    #
    #     :param painter: The painter which will be used to paint
    #     :type painter: QPainter
    #     :param option: The options parameter
    #     :type option: QStyleOptionViewItem
    #     :param index: The QModelIndex that will be painted
    #     :type index: QModelIndex
    #     """
    #     #todo:#Iignoretheoptionsparameter
    #     painter.save()
    #
    #     brush = choose_brush(index)
    #     painter.setBrush(brush)
    #
    #     #todo:set painter background for error cases?
    #     super(LogDelegate, self).paint(painter, option, index)
    #
    #     painter.restore()


    def initStyleOption(self, option, index):
        super(LogDelegate,self).initStyleOption(option, index)
        option.backgroundBrush = QBrush(choose_brush(index))