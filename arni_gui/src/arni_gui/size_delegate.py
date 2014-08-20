"""
Would probably already work.
The todos should be fixed earlier or later.
"""
from python_qt_binding.QtGui import QStyledItemDelegate, QFont, QBrush, QColor


class SizeDelegate(QStyledItemDelegate):
    """Makes it possible to change the font size of the Gui-Plugin content."""


    def __init__(self, parent=None):
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
        font = QFont()
        #setting the size of the font
        #todo:does this have to be set in the option param?
        font.setPixelSize(self.__current_font_size)
 
        brush = self.__choose_brush(index)
        painter.setFont(font)
        painter.setBrush(brush)

        #todo:set painter background for error cases?
        QStyledItemDelegate.paint(self, painter, option, index)

        painter.restore()


    def set_bigger_font_size(self):
        """Increases the displayed font-size."""
        self.__current_font_size += 2


    def set_smaller_font_size(self):
        """Decreases the displayed font-size."""
        self.__current_font_size -= 2


    def __choose_brush(self, index):
        """

        :type index: QModelIndex
        """
        brush = QBrush()
        # todo:can we assume this is always a string?
        #todo: add further keywords etc and check for errors. Is this the best possible way?
        if index.internalPointer().find("ok"):
            brush.setColor(QColor(127, 255, 0))
        elif index.internalPointer().find("warning"):
            brush.setColor(QColor(255, 165, 0))
        elif index.internalPointer().find("error"):
            brush.setColor(QColor(255, 0, 0))

        return brush