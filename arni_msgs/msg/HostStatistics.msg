# ip of the host  
string host

# the statistics apply to this time window
time window_start
time window_stop

# cpu  
float32 cpu_temp_mean
float32 cpu_temp_stddev
float32 cpu_temp_max

float32 cpu_usage_mean
float32 cpu_usage_stddev
float32 cpu_usage_max

float32[] cpu_usage_core_mean
float32[] cpu_usage_core_stddev
float32[] cpu_usage_core_max

float32[] cpu_temp_core  
float32[] cpu_temp_core_mean
float32[] cpu_temp_core_stddev
float32[] cpu_temp_core_max


# gpu
float32[] gpu_temp_mean
float32[] gpu_temp_stddev
float32[] gpu_temp_max

float32[] gpu_usage_mean
float32[] gpu_usage_stddev
float32[] gpu_usage_max
 

# ram  
float32 ram_usage_mean
float32 ram_usage_stddev
float32 ram_usage_max
  
# network

string[] interface_name 
int32[] message_frequency_mean
int32[] message_frequency_stddev
int32[] message_frequency_max

# bandwith of each network interface

int32[] bandwith_mean
int32[] bandwith_stddev
int32[] bandwith_max

# drive  
string[] drive_name  
int32[] drive_free_space  
  
# input output operations on a specific drive
int32[] drive_read  
int32[] drive_write  
