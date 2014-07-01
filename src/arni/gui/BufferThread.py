import threading.Thread
"""TODO: ROSModel richtig verlinken"""
import ROSModel
import rospy.Timer
import rospy.ServiceProxy


class BufferThread:threading.Thread:
"""
This thread should buffer the incoming data and regulary update the model and hence also the model.
"""
	def __init__(model):
		"""
		Starts the thread and also the timer for regulary updates of the model. It is ensured via the running attribute that this function cannot be called multiple times.
		"""
		threading.Thread.__init__(self)
		self.__buffer_lock = threading.Lock()
		self.__model = model
		self.__timer = rospy.Timer
		self.__buffer = list()
		self.__rated_buffer:list()
		self.__running: bool
		self.__monitoring_proxy: rospy.ServiceProxy


	def start():
		"""
		Starts the thread and also the timer for regulary updates of the model. It is ensured via the running attribute that this function cannot be called multiple times.
		"""
		pass

#TODO: shouldn't this be private?
	def update_model():
		"""
		Starts the update of the model. Will be called regulary by the timer. Will first read the data from the *buffer* and add the according data items to the items of the model and afterwards use the *rated_buffer* to add a rating to these entries.
		"""
		pass

#TODO: shouldn't this be private?
	def __add_buffer_item(object):
		"""
		Adds the item to the buffer list. Will be called whenever data from the topics is available.
  		"""
		pass

#TODO: shouldn't this be private?
	def add_buffer_item(RatedStatistics):
		"""
		Adds the RatedStatistics item to the *rated_buffer*
		"""
		pass
