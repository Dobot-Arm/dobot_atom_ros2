# Atom RPC Bridge

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)](#)

🤖 **Atom Robot RPC to DDS Middleware Bridge Package**

## 📖 Overview

`atom_rpc_bridge` is a ROS2 package specifically designed for Atom robots, providing seamless bridging from traditional RPC interfaces to ROS2 DDS communication. This package implements an independent RPC client that does not depend on external `atom_sdk` libraries and can be independently compiled and run in any environment that supports ROS2.

## ✨ Key Features

- 🔗 **Independent RPC Client**: No dependency on external SDK, completely self-contained
- 🚀 **Native ROS2 Support**: Fully compatible with ROS2 DDS communication mechanisms
- 🎯 **API Compatibility**: Maintains compatibility with original RPC interfaces
- 🛡️ **Thread Safety**: Supports multi-threaded concurrent access
- 📊 **Real-time Status Monitoring**: Provides connection status and error code feedback
- 🔧 **Easy Integration**: Simple C++ API interface

## 🏗️ Architecture Design

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   ROS2 Apps     │◄──►│  AtomDdsClient   │◄──►│ StandaloneRpc   │
│                 │    │                  │    │     Client      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                ▲                        ▲
                                │                        │
                                ▼                        ▼
                       ┌─────────────────┐    ┌─────────────────┐
                       │   ROS2 Topics   │    │   RPC Server    │
                       │   & Services    │    │ (192.168.8.234) │
                       └─────────────────┘    └─────────────────┘
```

## 🚀 Quick Start

### Dependencies

- ROS2 Humble version

### Build and Install

```bash
# Build
colcon build

# Setup environment
source install/setup.bash
```

### Basic Usage

```cpp
#include "atom_rpc_bridge/atom_dds_client.hpp"

using namespace atom_rpc_bridge;

int main() {
    // Create client instance
    AtomDdsClient robot("my_robot_node");
  
    // Initialize
    if (!robot.initialize()) {
        return -1;
    }
  
    // Set FSM state
    robot.SetFsmId(1);  // Damping mode
  
    // Control robot movement
    robot.SetVel(0.1f, 0.0f, 0.0f);  // Move forward
  
    // Enable upper limb control
    robot.SwitchUpperLimbControl(true);
  
    return 0;
}
```

## 📚 API Reference

### AtomDdsClient Class

#### Constructor

```cpp
AtomDdsClient(const std::string& node_name, 
              const std::string& rpc_ip = "192.168.8.234", 
              int rpc_port = 51234);
```

#### Main Methods

| Method                                     | Description           | Return Value         |
| ------------------------------------------ | --------------------- | -------------------- |
| `initialize()`                           | Initialize client     | `bool`             |
| `IsConnected()`                          | Check connection status | `bool`             |
| `SetFsmId(uint32_t fsm_id)`              | Set FSM state         | `AtomRpcErrorCode` |
| `GetFsmId(uint32_t& fsm_id)`             | Get FSM state         | `AtomRpcErrorCode` |
| `SetVel(float vx, float vy, float vyaw)` | Set velocity command  | `AtomRpcErrorCode` |
| `SwitchUpperLimbControl(bool enable)`    | Upper limb control switch | `AtomRpcErrorCode` |

### Error Code Definitions

```cpp
enum class AtomRpcErrorCode {
    SUCCESS = 0,           // Success
    CONNECTION_FAILED,     // Connection failed
    TIMEOUT,              // Timeout
    INVALID_PARAMS,       // Invalid parameters
    RPC_ERROR,            // RPC error
    UNKNOWN_ERROR         // Unknown error
};
```

## 🔌 ROS2 Interface

### Published Topics

| Topic Name               | Message Type            | Description        |
| ------------------------ | ----------------------- | ------------------ |
| `/atom/fsm_cmd`        | `std_msgs/UInt32`     | FSM state command  |
| `/atom/velocity_cmd`   | `geometry_msgs/Twist` | Velocity control command |
| `/atom/upper_limb_cmd` | `std_msgs/Bool`       | Upper limb control command |

### Subscribed Topics

| Topic Name                  | Message Type            | Description       |
| --------------------------- | ------------------- | ----------------- |
| `/atom/fsm_state`         | `std_msgs/UInt32` | Current FSM state |
| `/atom/connection_status` | `std_msgs/Bool`   | Connection status |

### Provided Services

| Service Name            | Service Type         | Description      |
| ----------------------- | -------------------- | ---------------- |
| `/atom/get_fsm_state` | `std_srvs/Trigger` | Get FSM state    |
| `/atom/reconnect`     | `std_srvs/Trigger` | Reconnect RPC    |

## 🎯 Usage Examples

### Run Test Program

```bash
# Launch test node
ros2 launch atom_rpc_bridge atom_dds_test.launch.py

# Or run directly
ros2 run atom_rpc_bridge atom_dds_test
```

### Custom RPC Server Address

```bash
ros2 launch atom_rpc_bridge atom_dds_test.launch.py rpc_ip:=192.168.1.100 rpc_port:=8080
```

### Usage in Code

See `examples/simple_usage_example.cpp` for complete examples.

## 🔧 FSM State Reference

| FSM ID | State Name     | Description      |
| ------ | -------------- | ---------------- |
| 0      | PASSIVE        | Passive mode     |
| 1      | DAMPING        | Damping mode     |
| 2      | RECOVERY_STAND | Recovery stand   |
| 3      | LOCOMOTION     | Locomotion mode  |
| 4      | BALANCE_STAND  | Balance stand    |

## 🐛 Debugging and Troubleshooting

### Common Issues

1. **Connection Failed**

   ```bash
   # Check network connection
   ping 192.168.8.234

   # Check if port is open
   telnet 192.168.8.234 51234
   ```
2. **Build Errors**

   ```bash
   # Ensure dependencies are installed
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. **Runtime Errors**

   ```bash
   # Enable debug logging
   ros2 launch atom_rpc_bridge atom_dds_test.launch.py log_level:=debug
   ```

### Log Levels

Set environment variable to enable verbose logging:

```bash
export RCUTILS_LOGGING_SEVERITY=DEBUG
```

## 📈 Performance Optimization

- **Connection Pool**: Reuse RPC connections to reduce overhead
- **Asynchronous Processing**: Non-blocking RPC calls
- **Caching Mechanism**: Reduce redundant status queries
- **Thread Optimization**: Reasonable thread pool configuration

## 📝 Changelog

### v1.0.0 (2025-09-02)

- ✨ Initial release
- 🔧 Independent RPC client implementation
- 📚 Complete API documentation
- 🧪 Test cases and examples