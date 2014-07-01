from StorageContainer import StorageContainer

class MetadataStorage:

    storage = {}

    duration = 0

    def __clean_up(self):
        """
        Automatically removes all objects in the storage older than *duration*.
        """
        pass

    def store(self, container):
        """
        Stores a given StorageContainer object.

        :param container: The data to store.
        :type container StorageContainer
        """
        if not container.identifier in self.storage:
            self.storage[container.identifier] = {}
        self.storage[container.identifier][container.timestamp] = container

    def get(self, identifier = "*", timestamp = rospy.Time(0)):
        """
        Returns all StorageContainers for a given identifier since a given timestamp.

        :param identifier: The identifier to return StorageContainer objects for. Use ``*`` to get all.
        :type identifier: str.
        :param timestamp: A timestamp marking the point of the oldest data you want. 0 returns all.
        :type timestamp: rospy.Time.
        """
        pass

    def clear(self):
        """
        Clears the whole storage.
        """
        self.storage.clear()

    def __init__(self, duration = 300000):
        """
        Saves received metadata packages for a given period of time and can provide them on request.

        :param duration: Optional the duration to keep objects in storage. Set to 5 minutes by default.
        :type duration: int.
        """
        self.duration = duration