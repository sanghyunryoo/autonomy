std_msgs/Header header

# Row-major height scan data matching IsaacLab masked_height_scan().
# data[row * width + col] corresponds to y row, x column.
uint32 width
uint32 height
float32 resolution
float32 x_min
float32 x_max
float32 y_min
float32 y_max

# Values follow IsaacLab:
#   observed   = sensor_height - hit_point_z - height_scan_offset
#              = -point_z_in_base_frame - height_scan_offset
#   unobserved = base_height - height_scan_offset
float32 height_scan_offset
float32 base_height
float32 fill_value

float32[] data
uint8[] valid_mask
