#ip of the host this node belongs to
string host

#identifier of this node
string node

# the statistics apply to this time window
time window_start
time window_stop

#CPU

float32 node_cpu_usage_mean
float32 node_cpu_usage_stddev
float32 node_cpu_usage_max

float32[] node_cpu_usage_core_mean
float32[] node_cpu_usage_core_stddev
float32[] node_cpu_usage_core_max


#GPU 

float32[] node_gpu_usage_mean
float32[] node_gpu_usage_stddev
float32[] node_gpu_usage_max

# ram  
float32 node_ramusage_mean
float32 node_ramusage_stddev
float32 node_ramusage_max
  
# network load of the node
 
float32 node_message_frequency_mean
float32 node_message_frequency_stddev
float32 node_message_frequency_max

float32 node_bandwidth_mean
float32 node_bandwidth_stddev
float32 node_bandwidth_max

# Drive I/O statistics of the node 

float32 node_write_mean
float32 node_write_stddev
float32 node_write_max

float32 node_read_mean
float32 node_read_stddev
float32 node_read_max
