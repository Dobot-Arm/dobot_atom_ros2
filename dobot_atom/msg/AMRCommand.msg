# AMR Command Types
uint8 CANCEL_TASK = 0
uint8 PAUSE_TASK = 1
uint8 RESUME_TASK = 2
uint8 MOVE_TO_TAG = 3
uint8 REMOTE_CONTROL = 4

uint8 command_type
uint32 target_id
float32 linear_vel
float32 angular_vel
uint32 command_id
uint64 timestamp