std_msgs/Header header

# Row-major height map distance data.
# data[row * width + col] corresponds to y row, x column.
uint32 width
uint32 height
float32 resolution
float32 x_min
float32 x_max
float32 y_min
float32 y_max

# Values are distances from the target frame along the vertical axis:
#   observed   = -point_z_in_target_frame
#   unobserved = base_height
# height_scan_offset is kept for message compatibility and is published as 0.
float32 height_scan_offset
float32 base_height
float32 fill_value

float32[] data
uint8[] valid_mask
