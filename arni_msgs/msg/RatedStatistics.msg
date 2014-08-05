# name of node/host/connection 
string seuid

# only used if seuid is a node. is the host ip the node runs on.
string host  

# the rated statistics apply to statistics from this time window
time window_start
time window_stop

# an array of rated entities
RatedStatisticsEntity[] rated_statistics_entity