"""
Would probably already work.
The todos should be fixed earlier or later.
"""
from python_qt_binding.QtGui import QStyledItemDelegate, QFont, QBrush, QColor
from helper_functions import choose_brush
from python_qt_binding.QtCore import QObject, QSize

class SizeDelegate(QStyledItemDelegate):
    """Makes it possible to change the font size of the Gui-Plugin content."""
    #todo: is QObject here a dirty hack or does it work like this?
    def __init__(self, parent=QObject()):
        super(SizeDelegate, self).__init__(parent)
        #QStyledItemDelegate.__init__(parent)
        self.__current_font_size = 10
        

    def paint(self, painter, option, index):
        """Defines how the items of the model will be painted in the view. Can be used  to draw e.g. bigger or smaller fonts.

        :param painter: The painter which will be used to paint
        :type painter: QPainter
        :param option: The options parameter
        :type option: QStyleOptionViewItem
        :param index: The QModelIndex that will be painted
        :type index: QModelIndex
        """
        #todo:#Iignoretheoptionsparameter
        painter.save()
        option.font.setPointSize(self.__current_font_size)
        QStyledItemDelegate.paint(self, painter, option, index)
        painter.restore()


    # TODO: trigger a repaint without focus the tree_view
    def set_bigger_font_size(self):
        """Increases the displayed font-size."""
        self.__current_font_size += 2
        

    def set_smaller_font_size(self):
        """Decreases the displayed font-size."""
        self.__current_font_size -= 2
        
    # will only be calles at initialization
    # only the height of a cell can be changed, changing the width has no effect
    def sizeHint(self, option, index):
        default = QStyledItemDelegate.sizeHint(self, option, index)
        return QSize(default.width(), default.height())
        
     
    # TODO: DOKU
    # handels the backgroundcolour from a cell
    def initStyleOption(self, option, index):
        super(SizeDelegate,self).initStyleOption(option, index)
        option.backgroundBrush = QBrush(choose_brush(index))