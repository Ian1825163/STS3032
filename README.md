# STS3032

Arduino/PlatformIO library for Feetech STS3032 serial servos.

## Layers

- Packet layer: `sendPacket()`, `readStatusPacket()`, `checksum()`, `writeByte()`, `writeWord()`, `readBytes()`
- Servo API layer: `setPosition()`, `setPositionSpeed()`, `readPosition()`, `readVelocity()`, `readAngularVelocity()`, `readCurrent()`, `readTemperature()`, `readFeedback()`, `torqueOn()`, `torqueOff()`, `setMode()`
- Feedback decode: `position_tick`, `position_deg`, `speed_raw`, `speed_rpm`, `speed_deg_s`, `load_raw`, `voltage_v`, `temperature_c`, `status`, `moving`, `current_a`


## Packet Test Example

`examples/PacketFunctionTest/PacketFunctionTest.ino` is a bring-up sketch for checking the packet layer and direct motor commands on hardware. It defaults to `Serial4` for the STS3032 bus, matching the Teensy wiring style used in `doggy_ws`.

Open the USB serial monitor at `115200` baud and set line ending to `Newline`. Useful commands:

```text
id 3          select servo ID 3
p             ping selected servo
f             read full feedback block
fp            read position only
fv            read angular velocity only
fc            read current only
ft            read temperature only
t 2048 1000   set absolute position tick and speed
a 90 800      set +90 degree offset from center at speed 800
c 800         return to center
v 360         velocity mode at +360 deg/s
v -360        velocity mode at -360 deg/s
vr 1000       velocity mode raw speed command
w 800         180 degree sweep around center
x             stop velocity/sweep
```

Write commands use write-ack disabled by default, so `sent` means the Teensy wrote the packet to the servo bus. Use `p` or `f` to confirm the receive path works.

## Motion Limits From Docs

These values are from `docs/磁编码SMS_STS-内存表解析_220328(2).xlsx`.

| Register | Name | Range | Unit / Meaning | Note |
| --- | --- | --- | --- | --- |
| `0x2E` | Run speed | `-32766` to `32766` | steps/s | `50 steps/s = 0.732 RPM`, so `1 raw ~= 0.01464 RPM ~= 0.08784 deg/s`. Sign controls direction in velocity mode. |
| `0x29` | Acceleration | `0` to `254` | `100 steps/s^2` | Example: `10` means `1000 steps/s^2`. |
| `0x10` | Max torque | `0` to `1000` | `0.001` of stall torque | EEPROM setting. `1000 = 100%`; copied to torque limit on power-up. |
| `0x30` | Torque limit | `0` to `1000` | `0.001` of max torque | SRAM runtime limit. `1000 = 100%`; can be changed by program. |

## Register Notes

Mode `2` is exposed as `MODE_OPEN_LOOP` because STS3032 documents it as open-loop motor mode, not direct PWM control.

`ServoStatus` bits follow the STS3032 status byte: voltage, sensor, temperature, current, angle, and overload.

`REG_PHASE` is a special function setting register, so avoid changing it during normal motion control.

`REG_LOCK` is related to saving EEPROM settings such as ID, baud rate, and angle limits. Normal gait commands and feedback reads should not touch it.
