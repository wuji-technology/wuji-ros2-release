# SDK timestamp and frame ID converted to a ROS header.
std_msgs/Header header
# Number of rows in the tactile grid.
uint32 rows
# Number of columns in the tactile grid.
uint32 columns
# Row-major SDK values with exactly rows * columns elements. Their meaning depends on the topic.
float32[] data
