from threading import Thread, Lock

"""TODO: ROSModel richtig verlinken"""
from ros_model import ROSModel
from rospy.timer import Timer
from rospy.impl.tcpros_service import ServiceProxy
from rospy.rostime import Duration

class BufferThread(Thread):
    """
    This thread should buffer the incoming data and regulary update the model and hence also the model.
    """


def __init__(self, model):
    """
    Initializes the BufferThread

    :param model: the object of the ROSModel
    :type model: ROSModel
    """
    Thread.__init__(self)
    self.__buffer_lock = Lock()
    self.__model = model
    self.__timer = Timer(Duration(nsecs=100000000), start)
    self.__buffer = list()
    self.__rated_buffer = list()
    self.__running = bool
    self.__monitoring_proxy = ServiceProxy

    #setting up the timer



def start(self):
    """
    Starts the thread and also the timer for regulary updates of the model. It is ensured via the running attribute that this function cannot be called multiple times.
    """
    pass


# TODO: shouldn't this be private?
def update_model(self):
    """
    Starts the update of the model. Will be called regulary by the timer. Will first read the data from the *buffer* and add the according data items to the items of the model and afterwards use the *rated_buffer* to add a rating to these entries.
    """
    pass


#TODO: shouldn't this be private?
def __add_buffer_item(self, item):
    """
    Adds the item to the buffer list. Will be called whenever data from the topics is available.

    :param item: the item which will be added to the buffer
    :type item: object
      """

    self.__buffer.add(item)


#TODO: shouldn't this be private?
def add_buffer_item(self, item):
    """
    Adds the RatedStatistics item to the *rated_buffer*

    :param RatedStatistics: the item which will be added to the rated_buffer
    :type item: RatedStatistics
    """
    self.__rated_buffer.add(item)
