from python_qt_binding.QtGui import QBrush, QColor


# define constants
# update frequency of the model in nsecs
UPDATE_FREQUENCY = 100000000



def choose_brush(index):
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

