# name of node/host/connection 
string seuid  

# the rated statistics apply to statistics from this time window
time window_start
time window_stop

# an array of rated entities
RatedStatisticsEntity[] rated_statistics_entity

# state of the metadata from the node/host/connection : 
# state: { 0 = high ; 1 = low ; 2 = unknown}  
uint8[] state