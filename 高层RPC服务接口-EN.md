# High-level RPC service interface

## 3. High-level motion service interface

The main functions of the high-level motion service interface are robot complete machine control and upper limb control.

The whole machine control can control the mode switching and movement of the robot, and provides an RPC interface;

Upper limb control can independently control the upper limb motors to complete operating tasks while running the robot's built-in mobile controller, and provides a DDS interface.

### 3.1 High-level machine control rpc interface

| **Interface Name** | **GetFsmId** |
| --- | --- |
| Interface definition | RpcErrorCode GetFsmId(int32\_t &fsm\_id); |
| Function Overview | Get the current state machine ID.|
| Parameters | fsm\_id: The obtained current state machine ID.|
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
| Function Overview | Switch upper limb control authority.|
| Parameters | is_on: If true, switches upper limb control to API-based control; if false, disables the switch and reverts to main motion controller’s full-body control.
| Return value | Returns 0 if the call is successful, otherwise returns the relevant error code, see the error code chapter for details.|
| Remarks | 1. Switching is only permitted in position-controlled states such as “preparation” and walking states. 2. After entering a state where switching is not allowed, switching will be automatically turned off.|

| **Interface Name** | **SetVel** |
| --- | --- |
| Interface definition | RpcErrorCode SetVel(float vx, float vy, float vyaw, float duration = 1.0); |
| Function overview | Issue a velocity command.|
| Parameters | vx: forward and backward movement velocity, forward is positive, unit m/s;<br>vy: left and right movement velocity, left is positive, unit m/s;<br>vyaw: rotation velocity, counterclockwise is positive, unit rad/s;<br>duration: velocity command duration, the robot​ stops after timeout, unit s.|
| Return value | Returns 0 if the call is successful, otherwise returns the relevant error code, see the error code chapter for details.|
| Remarks | 1. The underlying controller​ automatically clamps the setting parameters to a reasonable range; 2. The remote control command has a higher priority; 3. The whole machine stops moving: Issue a zero-velocity​ command：SetVel(0, 0, 0).|

### 3.2 Error code (RpcErrorCode)

| **ID** | **Description** |
| --- | --- |
| 0 | No errors |
| 1 | Socket creation failed |
| 2 | Communication connection failed |
| 3 | Response read failed |
| 4 | Response data parsing failed |
| 5 | Response data format error |
| 6 | Internal server error |
| 100 | Invalid state transition request; invoke only in an allowed state.|
| 101 | Invalid upper limb control switch request; invoke only in an allowed state. |
| 102 |	Invalid velocity command request; invoke only in an allowed state. |
