# One packet from the capacitive cup's board. Used inside this repo only.
std_msgs/Header header    # stamp: when the packet arrived here
float64 device_time_s     # the board's own clock
int32[8] values           # one raw count per electrode
