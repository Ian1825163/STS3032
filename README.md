# STS3032

Arduino/PlatformIO library for Feetech STS3032 serial servos.

## Layers

- Packet layer: `sendPacket()`, `readStatusPacket()`, `checksum()`, `writeByte()`, `writeWord()`, `readBytes()`
- Servo API layer: `setPosition()`, `setPositionSpeed()`, `readPosition()`, `readFeedback()`, `torqueOn()`, `torqueOff()`, `setMode()`
- Feedback decode: `position_tick`, `position_deg`, `speed_raw`, `load_raw`, `voltage_v`, `temperature_c`, `status`, `moving`, `current_a`

## Quick Start

```cpp
#include <STS3032.h>

STS3032 servo;

void setup() {
  Serial.begin(115200);
  servo.begin(Serial1, 1000000);

  servo.torqueOn(1);
  servo.setMode(1, STS3032::MODE_POSITION);
  servo.setPositionSpeed(1, 2048, 1500);
}

void loop() {
  STS3032::Feedback feedback;
  if (servo.readFeedback(1, feedback)) {
    Serial.print("pos=");
    Serial.print(feedback.position_tick);
    Serial.print(" deg=");
    Serial.print(feedback.position_deg);
    Serial.print(" voltage=");
    Serial.println(feedback.voltage_v);
  }
  delay(100);
}
```

For half-duplex RS485 adapters with a direction pin:

```cpp
servo.begin(Serial1, 1000000, 2); // TX enable pin 2, HIGH while sending
```

By default, write helpers do not wait for write status packets. Enable this if the servo response level is configured to return status packets for write instructions:

```cpp
servo.setWriteAck(true);
```

## Packet Test Example

`examples/PacketFunctionTest/PacketFunctionTest.ino` is a bring-up sketch for checking the packet layer on hardware. It defaults to `Serial4` for the STS3032 bus, matching the Teensy wiring style used in `doggy_ws`.

Open the USB serial monitor at `115200` baud. The sketch runs checksum, ping, identity read, and feedback read tests on boot. Press `h` in the serial monitor to show commands for re-running ping/read tests and sending a no-motion SYNC_WRITE using the current position as the goal.

## Register Notes

Mode `2` is exposed as `MODE_OPEN_LOOP` because STS3032 documents it as open-loop motor mode, not direct PWM control.

`ServoStatus` bits follow the STS3032 status byte: voltage, sensor, temperature, current, angle, and overload.

`REG_PHASE` is a special function setting register, so avoid changing it during normal motion control.

`REG_LOCK` is related to saving EEPROM settings such as ID, baud rate, and angle limits. Normal gait commands and feedback reads should not touch it.
