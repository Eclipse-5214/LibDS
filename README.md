# LibDS

A C library that abstracts the network communication layer between a Driver Station application and an FRC robot. Vendored and extended for use in [NovaDS](../../README.md).

Originally by [Alex Spataru](https://github.com/FRC-Utilities/LibDS).

## Protocols

| Protocol | Notes |
|---|---|
| FRC 2014 | Legacy CRIO protocol |
| FRC 2015 | Base modern protocol |
| FRC 2016 | 2015 + updated robot address resolution |
| FRC 2020 | Current roboRIO protocol |
| FRC 2026 | **Default in NovaDS** — extends 2020 with TCP channel |

### FRC 2026 Extensions

Built on top of FRC 2020 with the following additions:

- **TCP channel (port 1740)** — joystick descriptors, match info, game data sent on connect; robot stdout and error messages received
- **Brownout detection** — `DS_GetRobotBrownout()` tracks the brownout bit from robot status packets
- **FMS port switching** — switches robot UDP out-port 1110 → 1115 when FMS connects
- **Simulation auto-detect** — suppresses `cDSConnected` flag and TCP when target is `127.0.0.1`/`localhost`

## Architecture

### Event loop

LibDS runs its own event loop on a background thread (`Protocols_Init`). Applications poll the event queue periodically:

```c
DS_Init();
DS_ConfigureProtocol(DS_GetProtocolFRC_2026());

// in your main loop:
DS_Event event;
while (DS_PollEvent(&event)) {
    switch (event.type) {
    case DS_ROBOT_CONNECTED:
        // ...
    case DS_ROBOT_VOLTAGE_CHANGED:
        printf("voltage: %f\n", event.robot.voltage);
        break;
    }
}
```

### Public API (`DS_Client.h`)

All functions a host application needs — getters for comms, code, voltage, CPU/RAM/disk usage, and setters for team number, alliance, control mode, enable state, custom addresses, etc.

### Protocol state (`DS_Config.h`)

Internal setters called by protocol implementations to update LibDS state. Each setter fires a corresponding event into the queue.

### Sockets (`socket.c`)

Each socket (FMS, radio, robot, netconsole) is represented as a `DS_Socket` struct defining ports, protocol type (UDP/TCP), and remote address. The socket module manages a background `select()`-based receive thread per socket.

### Watchdogs (`protocols.c`)

Three independent watchdog timers (FMS, radio, robot). If no valid packet is received within the timeout window, the corresponding watchdog fires:
- `CFG_FMSWatchdogExpired()` / `CFG_RadioWatchdogExpired()` / `CFG_RobotWatchdogExpired()`

`CFG_RobotWatchdogExpired` also calls `protocol.reset_robot()` to tear down protocol-level state (e.g. the TCP connection in FRC 2026) so it can reconnect cleanly.

### Packet format — Robot → DS (FRC 2020/2026)

| Byte | Field |
|---|---|
| 0–1 | Sequence number (big-endian) |
| 2 | Comm version (`0x01`) |
| 3 | Control byte (mirrored from DS) |
| 4 | Status (`0x20` = has code, `0x10` = brownout) |
| 5 | Voltage integer |
| 6 | Voltage decimal |
| 7 | Request (`0x01` = wants time sync) |
| 8+ | Extended tags (CPU, RAM, disk, CAN) |

Minimum valid packet: **7 bytes**. Byte 7 is safely defaulted to `0` when absent.

## Usage in NovaDS

LibDS is compiled directly into the Tauri backend via the `cc` crate in `build.rs`. Rust bindings are generated at build time by `bindgen` from `include/LibDS.h`. No separate build step is needed.
