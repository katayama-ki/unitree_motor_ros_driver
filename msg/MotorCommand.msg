# Output-side FOC command. The driver manages the motor mode.
uint8 id
float32 position  # rad
float32 velocity  # rad/s
float32 torque    # Nm
float32 kp        # Nm/rad
float32 kd        # Nm/(rad/s)
