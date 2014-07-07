# type of statistic like cpu_usage_core or cpu_usage
string statistic_type

# the value of the type
string[] actual_value

# the expected value like "40 - 70"
string[] expected_value

# state of the metadata from the node/host/connection : 
# state: { 0 = high ; 1 = low ; 2 = unknown}  
uint8[] state