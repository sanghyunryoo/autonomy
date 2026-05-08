uint8 IDLE=0
uint8 DRIVE=1
uint8 ADAS=2
uint8 FSD=3
uint8 MAPPING=4
uint8 ERROR=5
uint8 ESTOP=6

std_msgs/Header header
uint8 mode
string mode_name
bool autonomy_enabled
bool estop_active
bool error_active
string status
string[] active_nodes
string[] inactive_nodes
string[] degraded_nodes
