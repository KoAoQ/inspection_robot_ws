# V3 architecture

## Command path

- `/cmd_vel/manual`, `/cmd_vel/nav`, `/cmd_vel/follow` are source-specific commands.
- `inspection_robot_core` selects exactly one source and publishes `/control/cmd_vel`.
- `inspection_robot_safety` validates environment safety and publishes `/base/safe_cmd_vel`.
- `inspection_robot_base` is the only node allowed to command the STM32.

## Base concurrency model

ROS callbacks only mutate `SharedControlState`. `BaseIoWorker` is the sole serial reader/writer at 50 Hz. `SerialTransport`, `SerialFrameRouter`, `BaseFaultManager`, and protocol parsers are never concurrently mutated by ROS callbacks.

## Fail-safe rules

- invalid NaN/Inf command -> latched fault + zero output
- command timeout -> zero output, no fault latch
- feedback timeout -> zero best effort, latched fault, close serial, reconnect
- serial write/read failure -> latched fault, close serial, reconnect
- e-stop -> latched fault + zero output
- reconnect clears old command/feedback freshness; a post-reconnect motion frame is required
- resetting a fault invalidates the old command; a new command is required

## Serial frames

Router recognizes the legacy S200-compatible receive frames:
- motion feedback: `0x7B`, 24 bytes, `0x7D`
- ultrasonic: `0xFA`, 19 bytes, `0xFC`
- charging: `0x7C`, 8 bytes, `0x7F`

Every accepted frame is tail-checked and XOR-BCC-checked before construction of `ValidatedFrame`.
