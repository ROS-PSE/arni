from arni_core.helper import older_than
import rospy
import threading
from threading import Timer
import thread
from storage_container import StorageContainer


class MetadataStorage:
    """
    The MetadataStorage holds StorageContainer objects with raw topic data and the rated values
    and can provide them on request.
    """
    storage = {}

    def __cleanup_timer(self):
        last_cleanup = rospy.Time.now()
        sleeptime = rospy.Duration(0.5)
        while not rospy.is_shutdown():
            if rospy.get_param('~storage/auto_cleanup', True) and \
                    older_than(last_cleanup, rospy.Duration(rospy.get_param('~storage/cleanup_timer', 30))):
                    # not rospy.Duration(
                    #         rospy.get_param('/arni/storage/cleanup_timer', 30)) > rospy.Time.now() - last_cleanup:
                last_cleanup = rospy.Time.now()
                self.__clean_up()
            rospy.sleep(sleeptime)


    def __clean_up(self):
        """
        Automatically removes all objects in the storage older than *duration*.
        """
        counter = 0
        for ident in self.storage.keys():
            for stamp in self.storage[ident].keys():
                if older_than(stamp, rospy.Duration(self.duration)):
                # if rospy.Time.now() - stamp > rospy.Duration(self.duration):
                    del self.storage[ident][stamp]
                    counter += 1
        rospy.logdebug("[MetadataStorage] Cleared storage, removed %s packages." % counter)

    def store(self, container):
        """
        Stores a given StorageContainer object.

        :param container: The data to store.
        :type container: StorageContainer
        """
        if not container.identifier in self.storage:
            self.storage[container.identifier] = {}
        self.storage[container.identifier][container.timestamp] = container

    def get(self, identifier="*", timestamp=rospy.Time(0)):
        """
        Returns all StorageContainers for a given identifier since a given timestamp.

        :param identifier: The identifier to return StorageContainer objects for. Use ``*`` to get all.
        :type identifier: str.
        :param timestamp: A timestamp marking the point of the oldest data you want. 0 returns all.
        :type timestamp: rospy.Time.
        """
        results = []
        for ident in self.storage.keys():
            if ident == identifier or identifier == "*":
                for stamp in self.storage[ident].keys():
                    if stamp >= timestamp:
                        results.append(self.storage[ident][stamp])
        return results

    def clear(self):
        """
        Clears the whole storage.
        """
        self.storage.clear()

    def __init__(self, duration=300):
        """
        Saves received metadata packages for a given period of time and can provide them on request.

        :param duration: Optional the duration to keep objects in storage. Set to 5 minutes by default.
        :type duration: int.
        """
        self.storage = {}
        self.duration = rospy.get_param('~/storage/timeout', duration)
        self.timer_running = True
        thr = threading.Thread(target=self.__cleanup_timer)
        thr.start()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.timer_running = False
