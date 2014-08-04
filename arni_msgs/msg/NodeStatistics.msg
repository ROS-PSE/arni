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
 
int32 node_message_frequency_mean
int32 node_message_frequency_stddev
int32 node_message_frequency_max

int32 node_bandwith_mean
int32 node_bandwith_stddev
int32 node_bandwith_max

# Drive I/O statistics of the node 

int32 node_write_mean
int32 node_write_stddev
int32 node_write_max

int32 node_read_mean
int32 node_read_stddev
int32 node_read_max