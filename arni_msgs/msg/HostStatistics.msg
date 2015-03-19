# ip of the host  
string host

# the statistics apply to this time window
time window_start
time window_stop

# cpu 
# temp for whole cpu in celsius 
float32 cpu_temp_mean
float32 cpu_temp_stddev
float32 cpu_temp_max

# usage for whole cpu in percent
float32 cpu_usage_mean
float32 cpu_usage_stddev
float32 cpu_usage_max

#cpu usage per core in percent
float32[] cpu_usage_core_mean
float32[] cpu_usage_core_stddev
float32[] cpu_usage_core_max

# temp per core, in celsius
float32[] cpu_temp_core_mean
float32[] cpu_temp_core_stddev
float32[] cpu_temp_core_max


# gpu, placeholder not implemented
# temp in celsius
float32[] gpu_temp_mean
float32[] gpu_temp_stddev
float32[] gpu_temp_max
#usage in percent
float32[] gpu_usage_mean
float32[] gpu_usage_stddev
float32[] gpu_usage_max
 

# ram usage in percent
float32 ram_usage_mean
float32 ram_usage_stddev
float32 ram_usage_max
  
# network
# message_frequency measured in packtes/s
string[] interface_name 
int32[] message_frequency_mean
int32[] message_frequency_stddev
int32[] message_frequency_max

# bandwidth of each network interface
# bytes/s
float32[] bandwidth_mean
float32[] bandwidth_stddev
float32[] bandwidth_max

# drive, free_space in Megabytes
string[] drive_name  
int32[] drive_free_space  
  
# input output operations on a specific drive
# in bytes/s
float32[] drive_read  
float32[] drive_write  
