class HostLookup(object):

    """Contains a dictionary of all nodes and the hosts they run on.
    Singleton.
    Works only for nodes which are on an host who
    has an HostStatisticNode running.
    """

    def __init__(self):
        super(HostLookup, self).__init__()
