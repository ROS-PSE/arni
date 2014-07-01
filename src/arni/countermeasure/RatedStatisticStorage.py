class RatedStatisticStorage(object):

    """A database which contains the current state
    of all rated statistics.
    """

    def __init__(self):
        super(RatedStatisticStorage, self).__init__()

        #: A dictionary containing all rated statistic
        #: information with their outcome and an timestamp
        #: when they got added / updated to the dictionary.
        self.__statistic_storage

        #: The timeout after which an item in ratedstatistic
        #: is declared too old and should be removed
        #: from the dict.
        self.__timeout

    def clean_old_statistic(self):
        """Check the complete dictionary for statistics
        older than timeout seconds and remove them.
        """
        pass

    def callback_rated_statistic(self, msg):
        """Callback for incoming rated statistics.
        Add them to the dictionary or remove items from
        the dictionary if the rated statistic
        says that its within bounds again.
        """
        pass

    def get_outcome(self, seuid, statistic_type):
        """Returns the outcome of the specific seuid
        and statistic_type.
        """
        pass
