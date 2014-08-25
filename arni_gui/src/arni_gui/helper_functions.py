from python_qt_binding.QtGui import QBrush, QColor


# define constants
# update frequency of the model in nsecs
UPDATE_FREQUENCY = 100000000



def choose_brush(index):
    """
    :type index: QModelIndex
    """
   
    # todo:can we assume this is always a string?
    #todo: add further keywords etc and check for errors. Is this the best possible way?
    if index.data() == "ok":
        return QColor(127, 255, 0)
    elif index.data() == "warning":
        return QColor(255, 165, 0)
    elif index.data() == "error":
        print("\n\n\n Testint\n\n\n")
        return QColor(255, 0, 0)
    else:
        return QColor(255, 255, 255)

