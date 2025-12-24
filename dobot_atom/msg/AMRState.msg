uint8 UNKNOWN=0
uint8 QUEUING=1
uint8 RUNNING=2
uint8 COMPLETED=3
uint8 FAILED=4
uint8 PAUSED=5
uint8 CANCELED=6
uint8 WAITING_CONFIRM=7
uint8 IDLE=8
uint8 STOPPED=9

uint8 DEVUNKNOWN=0
uint8 DEVIDLE=1
uint8 TASKING=2
uint8 ERROR=3
uint8 OFFLINE=4
uint8 INIT=5
uint8 CHARGING=6
uint8 UPGRADE=7

uint8 device_status
uint8 navigation_status
AMRBasicStatus basic_status
float32[3] position
AMREventStatus amr_event
uint32[32] error_code
uint32 task_id
uint32 work_mode
