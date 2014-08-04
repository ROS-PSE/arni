import rospy
from outcome import *
from arni_msgs.msg import RatedStatistics, RatedStatisticsEntity


class RatedStatisticStorage(object):

    """A database which contains the current state
    of all rated statistics.
    """

    def __init__(self):
        super(RatedStatisticStorage, self).__init__()

        #: A dictionary containing all rated statis tic
        #: information with their outcome and an timestamp
        #: when they got added / updated to the dictionary.
        self.__statistic_storage = dict()

        #: The timeout after which an item in ratedstatistic
        #: is declared too old and should be removed
        #: from the dict.
        self.__timeout = 10

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

        :param msg: The rated statistic to be added to the storage.
        :type msg:  RatedStatistics
        """
        rospy.loginfo("countermeasure: got a rated statistic for " + msg.seuid)

        seuid = msg.seuid

        # todo: removeme, im just here for autocompletion
        msg = RatedStatistics()

        for entity in msg.rated_statistics_entity:
            stat_type = entity.statistic_type

            # its not an array, so treat it differently
            if len(entity.actual_value) == 1:
                self.__add_single_outcome(
                    seuid, stat_type, entity.state[0], msg.window_stop)
            else:
                # split the array in a lot of entries
                for i in range(len(entity.actual_value)):
                    self.__add_single_outcome(
                        seuid, stat_type + "_" + i, entity.state[i],
                        msg.window_stop)

        # try:

        #     # init second dict if needed
        #     if msg.seuid not in store:
        #         store[msg.seuid] = dict()

        #     else:
        #         type_dict = store[msg.seuid]

        #         # special handling if this is just a single value wrapped
        #         # in an array
                
        # except Exception:
        #     pass

    def __add_single_outcome(
            self, seuid, statistic_type, outcome, timestamp):
        # thats just too long..
        store = self.__statistic_storage
        if seuid not in store:
            store[msg.seuid] = dict()


        pass

    def get_outcome(self, seuid, statistic_type):
        """Return the outcome of the specific seuid
        and statistic_type.

        :param seuid:   The seuid of the entity the statistic belongs to.
        :type seuid:    string

        :param statistic_type:  Type of statistic.
                                Arrays get a suffix: type_0 type_1 and so on.
        :type statistic_type:   string

        :return:    The outcome the type currently has.
                    Returns Outcome.UNKNOWN if there is no saved outcome.
        :rtype: Outcome (int)

        :raises AttributeError: If the outcome in the storage 
                                is not a valid outcome. Should not occur
                                because there is a validation check upon 
                                entering the outcome into the storage.

        """
        try:
            type_dict = self.__statistic_storage[seuid]
            outTuple = type_dict[statistic_type]

            outcome = outTuple[0]
            timestamp = outTuple[1]

            if rospy.get_rostime() - timestamp > self.__timeout:
                self.__remove_item(seuid, statistic_type)
                return Outcome.UNKNOWN

            elif Outcome.is_valid(outcome):
                return outcome
            else:
                raise AttributeError

        except KeyError:
            return Outcome.UNKNOWN

    def __remove_item(self, seuid, statistic_type):
        pass



if __name__ == '__main__':
    rss = RatedStatisticStorage()
    rss.get_outcome("blabla", "cpu")