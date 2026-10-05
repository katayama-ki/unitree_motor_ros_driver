uint8 BRAKE=0
uint8 FOC=1
uint8 CALIBRATE=2
uint8 UNKNOWN=255

std_msgs/Header header  # time of successful receive, not time of publication
uint8 id
uint8 mode
float32 position       # output-side rad
float32 velocity       # output-side rad/s
float32 torque         # output-side Nm
int8 temperature       # degrees Celsius
uint8 error            # unmodified SDK merror
