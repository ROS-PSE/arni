import rospy
from outcome import *
from arni_msgs.msg import RatedStatistics, RatedStatisticsEntity
import helper
import time


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
        #: type: Duration
        self.__timeout = helper.get_param_duration(
            helper.ARNI_CTM_CFG_NS + "storage_timeout")

    def clean_old_statistic(self):
        """Check the complete dictionary for statistics
        older than timeout seconds and remove them.
        """
        store = self.__statistic_storage

        # get the time once
        curtime = rospy.get_rostime()

        todelete = list()

        # grab all old statistics
        for seuid in store:
            for statistic_type in store[seuid]:
                timestamp = store[seuid][statistic_type][1]
                if curtime - timestamp >= self.__timeout:
                    todelete.append((seuid, statistic_type))

        # remove them
        for seuid, statistic_type in todelete:
            self.__remove_item(seuid, statistic_type)

    def callback_rated_statistic(self, msg):
        """Callback for incoming rated statistics.
        Add them to the dictionary or remove items from
        the dictionary if the rated statistic
        says that its within bounds again.

        :param msg: The rated statistic to be added to the storage.
        :type msg:  RatedStatistics
        """
        seuid = msg.seuid

        for entity in msg.rated_statistics_entity:
            stat_type = entity.statistic_type

            # check if actual value, state, expected value have the same size
            if (len(entity.actual_value) == len(entity.state)
                    and
                    len(entity.actual_value) == len(entity.expected_value)):

                # its not an array, so treat it differently
                if len(entity.actual_value) == 1:
                    self.__add_single_outcome(
                        seuid, stat_type,
                        ord(entity.state[0]), msg.window_stop)
                else:
                    # split the array in a lot of entries
                    for i in range(len(entity.actual_value)):
                        self.__add_single_outcome(
                            seuid, "%s_%d" % (stat_type, i),
                            ord(entity.state[i]),
                            msg.window_stop)
            else:
                rospy.logwarn(
                    "Inconsistency in received data packet: actual_value, "
                    + "expected_value, state have to have the same size."
                    + " Happened in a rated msg  of type %s from %s"
                    % (stat_type, seuid))

    def __add_single_outcome(
            self, seuid, statistic_type, outcome, timestamp):
        """Add a single outcome to the storage.

        :param seuid:   The seuid from the entity.
        :type seuid:    string

        :param statistic_type:  The type of statistic to add.
        :type statistic_type:   string

        :param outcome: The outcome the type had.
        :type outcome:  Outcome

        :param timestamp:   The time when this outcome was send.
        :type timestamp:    rospy.Time

        """
        # thats just too long..
        store = self.__statistic_storage
        if seuid not in store:
            store[seuid] = dict()

        # the dictionary for a specific entity having the specified entity
        entity_dict = store[seuid]

        # check if there is an entry thats newer:
        if (
            ((not statistic_type in entity_dict) or (
                entity_dict[statistic_type][1] < timestamp)) and (
                rospy.Time.now() - timestamp < self.__timeout)):

            entity_dict[statistic_type] = outcome, timestamp

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

            # check if the item is too old
            if rospy.get_rostime() - timestamp >= self.__timeout:
                self.__remove_item(seuid, statistic_type)
                return Outcome.UNKNOWN

            elif Outcome.is_valid(outcome):
                return outcome
            else:
                raise AttributeError
        except KeyError:
            return Outcome.UNKNOWN

    def __remove_item(self, seuid, statistic_type):
        """Remove an statistic_type from an entity from the storage.

        :param seuid:   The seuid of the entity.
        :type seuid:    string

        :param statistic_type:  The type to be removed.
        :type statistic_type:   string

        :raises KeyError:   If the seuid/statistic_type does not exist
                            in the storage.
        """
        try:
            del self.__statistic_storage[seuid][statistic_type]
        except KeyError:
            rospy.logdebug(
                "Tried to delete"
                + " an unexisting entry in the storage.")
