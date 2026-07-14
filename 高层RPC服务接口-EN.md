# High-level RPC service interface

## 3. High-level motion service interface

The main functions of the high-level motion service interface are robot complete machine control and upper limb control.

The whole machine control can control the mode switching and movement of the robot, and provides an RPC interface;

Upper limb control can independently control the upper limb motors to complete operating tasks while running the robot's built-in mobile controller, and provides a DDS interface.

### 3.1 High-level machine control rpc interface

| **Interface Name** | **GetFsmId** |
| --- | --- |
| Interface definition | RpcErrorCode GetFsmId(int32\_t &fsm\_id); |
| Function Overview | Get the current state machine id.|
| Parameters | fsm\_id: The obtained current state machine id.|
| Return value | Returns 0 if the call is successful, otherwise returns the relevant error code, see the error code chapter for details.|
| Remarks | For details about the state machine ID, please refer to the state machine ID chapter.|

| **Interface Name** | **SetFsmId** |
| --- | --- |
| Interface definition | RpcErrorCode SetFsmId(int32\_t fsm\_id); |
| Function overview | Set state machine id.|
| Parameters | fsm\_id: target state machine id.|
| Return value | Returns 0 if the call is successful, otherwise returns the relevant error code, see the error code chapter for details.|
| Remarks | For details about the state machine ID, please refer to the state machine ID chapter.|

| **Interface Name** | **SwitchUpperLimbControl** |
| --- | --- |
| Interface definition | RpcErrorCode SwitchUpperLimbControl(bool is\_on); |
| Function Overview | Switch upper body control.|
| Parameters | is\_on: true means switching upper limb control to interface control, false means turning off the switch and returning to main operation control and overall machine control.|
| Return value | Returns 0 if the call is successful, otherwise returns the relevant error code, see the error code chapter for details.|
| Remarks | 1. Switching is only allowed in position control states such as "preparation" and walking state; 2. After entering a state where switching is not allowed, switching will be automatically turned off.|

| **Interface Name** | **SetVel** |
| --- | --- |
| Interface definition | RpcErrorCode SetVel(float vx, float vy, float vyaw, float duration = 1.0); |
| Function overview | Set speed command.|
| Parameters | vx: forward and backward movement speed, forward is positive, unit m/s;<br>vy: left and right movement speed, left is positive, unit m/s;<br>vyaw: rotation speed, counterclockwise is positive, unit rad/s;<br>duration: speed command duration, stops after timeout, unit s.|
| Return value | Returns 0 if the call is successful, otherwise returns the relevant error code, see the error code chapter for details.|
| Remarks | 1. The bottom layer will automatically limit the setting parameters to a reasonable range; 2. The remote control command has a higher priority; 3. The whole machine stops moving: Set the speed command to zero SetVel( 0, 0, 0).|

### 3.2 Error code (RpcErrorCode)

| **ID** | **Description** |
| --- | --- |
| 0 | No errors |
| 1 | Socket creation failed |
| 2 | Communication connection failed |
| 3 | Failed to read response |
| 4 | Response data parsing failed |
| 5 | Response data format error |
| 6 | Server internal error |
| 100 | Illegal state jump request, please call in allowed state |
| 101 | Illegal upper limb control request, please call in allowed state |
| 102 | Illegal setting speed command request, please call in allowed state |
