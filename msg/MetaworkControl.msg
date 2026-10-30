# for recording the control packets in Metawork Protocol on MQTT

# a timestamp of the time at which a node received a MQTT packet
std_msgs/Header header

builtin_interfaces/Time payload_stamp
float64[] data
