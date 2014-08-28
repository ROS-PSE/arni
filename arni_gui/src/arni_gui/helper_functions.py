from python_qt_binding.QtGui import QBrush, QColor

# define constants
# update frequency of the model in nsecs
UPDATE_FREQUENCY = 1000000000


def choose_brush(index):
    """
    Chooses the brush according o the content of a cell.
    
    :param index: the index of the item 
    :type index: QModelIndex
    """
   
    # todo:can we assume this is always a string?
    #todo: add further keywords etc and check for errors. Is this the best possible way?
    if index.data() == "ok":
        return QColor(127, 255, 0)
    elif index.data() == "warning":
        return QColor(255, 165, 0)
    elif index.data() == "error":
        return QColor(255, 0, 0)

    return None