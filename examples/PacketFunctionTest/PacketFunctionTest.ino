#include <Arduino.h>
#include <STS3032.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// This example defaults to Serial4 to match doggy_ws on Teensy.
// Change this to Serial1/Serial2/etc. if your STS3032 bus is wired elsewhere.
#ifndef STS3032_SERVO_SERIAL
#define STS3032_SERVO_SERIAL Serial4
#endif

static const uint8_t DEFAULT_SERVO_ID = 1;
static const uint32_t USB_BAUD = 115200;
static const uint32_t SERVO_BAUD = STS3032::DEFAULT_BAUDRATE;
static const int8_t DIRECTION_PIN = -1;
static const uint16_t PACKET_TIMEOUT_MS = 30;

static const uint16_t CENTER_TICK = 2048;
static const uint16_t DEFAULT_POSITION_SPEED = 1000;
static const uint8_t DEFAULT_ACCELERATION = 0;
static const float SPEED_UNIT_RPM = 0.732f / 50.0f;
static const float SPEED_UNIT_DEG_PER_SEC = SPEED_UNIT_RPM * 6.0f;
static const float CURRENT_UNIT_A = 0.0065f;
static const float SWEEP_AMPLITUDE_DEG = 90.0f; // -90 to +90 = 180 deg total
static const uint32_t SWEEP_INTERVAL_MS = 1200;
static const uint32_t COMMAND_IDLE_PROCESS_MS = 250;

static const bool RUN_CHECKSUM_ON_BOOT = true;
static const bool RUN_READ_TESTS_ON_BOOT = false;

STS3032 servo;

uint8_t selectedServoId = DEFAULT_SERVO_ID;
bool sweepEnabled = false;
bool sweepHighTarget = false;
uint16_t sweepSpeed = DEFAULT_POSITION_SPEED;
uint8_t sweepAcceleration = DEFAULT_ACCELERATION;
unsigned long lastSweepMs = 0;

char inputLine[96];
uint8_t inputLineLength = 0;
unsigned long lastInputByteMs = 0;
unsigned long lastCommandEndMs = 0;

long clampLong(long value, long low, long high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

uint16_t wrap4096(long value) {
  value %= 4096;
  if (value < 0) {
    value += 4096;
  }
  return static_cast<uint16_t>(value);
}

uint16_t degOffsetToTick(float degrees) {
  const long offset =
      static_cast<long>(lroundf(degrees * (4096.0f / 360.0f)));
  return wrap4096(static_cast<long>(CENTER_TICK) + offset);
}

uint16_t clampPositionTick(long tick) {
  return static_cast<uint16_t>(clampLong(tick, 0, 4095));
}

uint16_t clampPositionSpeed(long speed) {
  return static_cast<uint16_t>(clampLong(speed, 0, 32766));
}

int16_t clampSignedSpeed(long speed) {
  return static_cast<int16_t>(clampLong(speed, -32766, 32766));
}

uint8_t clampAcceleration(long acceleration) {
  return static_cast<uint8_t>(clampLong(acceleration, 0, 254));
}

uint16_t makeWord(uint8_t low, uint8_t high) {
  return static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
}

float speedRawToRpm(int16_t speedRaw) {
  return speedRaw * SPEED_UNIT_RPM;
}

float speedRawToDegPerSecond(int16_t speedRaw) {
  return speedRaw * SPEED_UNIT_DEG_PER_SEC;
}

int16_t degPerSecondToSpeedRaw(float speedDegPerSecond) {
  const long raw = static_cast<long>(lroundf(speedDegPerSecond /
                                            SPEED_UNIT_DEG_PER_SEC));
  return clampSignedSpeed(raw);
}

float currentRawToAmp(int16_t currentRaw) {
  return currentRaw * CURRENT_UNIT_A;
}

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void printPacketPreview(uint8_t id, uint8_t instruction,
                        const uint8_t *parameters, uint8_t parameterLength) {
  const uint8_t length = parameterLength + 2;
  const uint8_t packetChecksum =
      servo.checksum(id, length, instruction, parameters, parameterLength);

  Serial.print("TX preview: FF FF ");
  printHexByte(id);
  Serial.print(' ');
  printHexByte(length);
  Serial.print(' ');
  printHexByte(instruction);

  for (uint8_t i = 0; i < parameterLength; ++i) {
    Serial.print(' ');
    printHexByte(parameters[i]);
  }

  Serial.print(' ');
  printHexByte(packetChecksum);
  Serial.println();
}

void printStatusBits(uint8_t status) {
  Serial.print(" status_bits=[");
  bool printed = false;

  struct StatusName {
    uint8_t bit;
    const char *name;
  };

  const StatusName names[] = {
      {STS3032::STATUS_VOLTAGE, "voltage"},
      {STS3032::STATUS_SENSOR, "sensor"},
      {STS3032::STATUS_TEMPERATURE, "temperature"},
      {STS3032::STATUS_CURRENT, "current"},
      {STS3032::STATUS_ANGLE, "angle"},
      {STS3032::STATUS_OVERLOAD, "overload"},
  };

  for (uint8_t i = 0; i < sizeof(names) / sizeof(names[0]); ++i) {
    if ((status & names[i].bit) == 0) {
      continue;
    }
    if (printed) {
      Serial.print(',');
    }
    Serial.print(names[i].name);
    printed = true;
  }

  if (!printed) {
    Serial.print("none");
  }
  Serial.print(']');
}

void printStatusPacket(const STS3032::StatusPacket &packet) {
  Serial.print("RX status: id=");
  Serial.print(packet.id);
  Serial.print(" error=0x");
  printHexByte(packet.error);
  Serial.print(" params=");
  Serial.print(packet.parameter_length);
  Serial.print(" checksum=0x");
  printHexByte(packet.checksum);
  Serial.println();
}

void checksumSelfTest() {
  const uint8_t readParams[2] = {STS3032::REG_PRESENT_POSITION_L, 15};
  const uint8_t pingChecksum =
      servo.checksum(1, 2, STS3032::INST_PING);
  const uint8_t readChecksum =
      servo.checksum(1, 4, STS3032::INST_READ, readParams,
                     sizeof(readParams));

  Serial.println();
  Serial.println("Checksum self-test:");
  Serial.print("  PING id=1 checksum expected FB, got ");
  printHexByte(pingChecksum);
  Serial.println(pingChecksum == 0xFB ? " OK" : " FAIL");
  Serial.print("  READ present block checksum expected B1, got ");
  printHexByte(readChecksum);
  Serial.println(readChecksum == 0xB1 ? " OK" : " FAIL");
}

bool pingServo() {
  Serial.println();
  Serial.println("PING test");
  printPacketPreview(selectedServoId, STS3032::INST_PING, nullptr, 0);

  if (!servo.sendPacket(selectedServoId, STS3032::INST_PING)) {
    Serial.println("PING send failed");
    return false;
  }

  STS3032::StatusPacket packet;
  if (!servo.readStatusPacket(packet)) {
    Serial.println("PING timeout or bad checksum");
    return false;
  }

  printStatusPacket(packet);
  return packet.id == selectedServoId && packet.error == 0;
}

bool readIdentityBlock() {
  Serial.println();
  Serial.println("READ identity block");
  const uint8_t readParams[2] = {STS3032::REG_ID, 4};
  printPacketPreview(selectedServoId, STS3032::INST_READ, readParams,
                     sizeof(readParams));

  uint8_t data[4] = {0};
  if (!servo.readBytes(selectedServoId, STS3032::REG_ID, data, sizeof(data))) {
    Serial.println("READ identity block failed");
    return false;
  }

  Serial.print("id=");
  Serial.print(data[0]);
  Serial.print(" baud_raw=");
  Serial.print(data[1]);
  Serial.print(" return_delay=");
  Serial.print(data[2]);
  Serial.print(" response_level=");
  Serial.println(data[3]);
  return true;
}

bool readFeedbackOnce() {
  Serial.println();
  Serial.println("READ feedback block");
  const uint8_t readParams[2] = {STS3032::REG_PRESENT_POSITION_L, 15};
  printPacketPreview(selectedServoId, STS3032::INST_READ, readParams,
                     sizeof(readParams));

  STS3032::Feedback feedback;
  if (!servo.readFeedback(selectedServoId, feedback)) {
    Serial.println("READ feedback failed");
    return false;
  }

  Serial.print("position_tick=");
  Serial.print(feedback.position_tick);
  Serial.print(" position_deg=");
  Serial.print(feedback.position_tick * (360.0f / 4096.0f), 2);
  Serial.print(" speed_raw=");
  Serial.print(feedback.speed_raw);
  Serial.print(" speed_rpm=");
  Serial.print(speedRawToRpm(feedback.speed_raw), 2);
  Serial.print(" speed_deg_s=");
  Serial.print(speedRawToDegPerSecond(feedback.speed_raw), 1);
  Serial.print(" load_raw=");
  Serial.print(feedback.load_raw);
  Serial.print(" voltage_v=");
  Serial.print(feedback.voltage_v, 1);
  Serial.print(" temperature_c=");
  Serial.print(feedback.temperature_c);
  Serial.print(" moving=");
  Serial.print(feedback.moving);
  Serial.print(" current_a=");
  Serial.print(feedback.current_a, 3);
  Serial.print(" status=0x");
  printHexByte(feedback.status);
  printStatusBits(feedback.status);
  Serial.println();
  return true;
}

bool readPositionTest() {
  Serial.println();
  Serial.println("READ position");
  const uint8_t readParams[2] = {STS3032::REG_PRESENT_POSITION_L, 2};
  printPacketPreview(selectedServoId, STS3032::INST_READ, readParams,
                     sizeof(readParams));

  uint8_t data[2] = {0, 0};
  if (!servo.readBytes(selectedServoId, STS3032::REG_PRESENT_POSITION_L, data,
                       sizeof(data))) {
    Serial.println("READ position failed");
    return false;
  }

  const uint16_t positionTick = makeWord(data[0], data[1]);
  Serial.print("position_tick=");
  Serial.print(positionTick);
  Serial.print(" position_deg=");
  Serial.println(positionTick * (360.0f / 4096.0f), 2);
  return true;
}

bool readVelocityTest() {
  Serial.println();
  Serial.println("READ angular velocity");
  const uint8_t readParams[2] = {STS3032::REG_PRESENT_SPEED_L, 2};
  printPacketPreview(selectedServoId, STS3032::INST_READ, readParams,
                     sizeof(readParams));

  int16_t speedRaw = 0;
  uint8_t data[2] = {0, 0};
  if (!servo.readBytes(selectedServoId, STS3032::REG_PRESENT_SPEED_L, data,
                       sizeof(data))) {
    Serial.println("READ angular velocity failed");
    return false;
  }
  speedRaw = STS3032::decodeSigned15(makeWord(data[0], data[1]));

  Serial.print("speed_raw=");
  Serial.print(speedRaw);
  Serial.print(" speed_rpm=");
  Serial.print(speedRawToRpm(speedRaw), 2);
  Serial.print(" speed_deg_s=");
  Serial.println(speedRawToDegPerSecond(speedRaw), 1);
  return true;
}

bool readCurrentTest() {
  Serial.println();
  Serial.println("READ current");
  const uint8_t readParams[2] = {STS3032::REG_PRESENT_CURRENT_L, 2};
  printPacketPreview(selectedServoId, STS3032::INST_READ, readParams,
                     sizeof(readParams));

  uint8_t data[2] = {0, 0};
  if (!servo.readBytes(selectedServoId, STS3032::REG_PRESENT_CURRENT_L, data,
                       sizeof(data))) {
    Serial.println("READ current failed");
    return false;
  }

  const int16_t currentRaw =
      STS3032::decodeSigned15(makeWord(data[0], data[1]));
  Serial.print("current_raw=");
  Serial.print(currentRaw);
  Serial.print(" current_a=");
  Serial.println(currentRawToAmp(currentRaw), 3);
  return true;
}

bool readTemperatureTest() {
  Serial.println();
  Serial.println("READ temperature");
  const uint8_t readParams[2] = {STS3032::REG_PRESENT_TEMPERATURE, 1};
  printPacketPreview(selectedServoId, STS3032::INST_READ, readParams,
                     sizeof(readParams));

  uint8_t temperatureC = 0;
  if (!servo.readBytes(selectedServoId, STS3032::REG_PRESENT_TEMPERATURE,
                       &temperatureC, 1)) {
    Serial.println("READ temperature failed");
    return false;
  }

  Serial.print("temperature_c=");
  Serial.println(temperatureC);
  return true;
}

bool setPositionMode() {
  Serial.println();
  Serial.println("WRITE mode=position");
  const uint8_t params[2] = {STS3032::REG_MODE, STS3032::MODE_POSITION};
  printPacketPreview(selectedServoId, STS3032::INST_WRITE, params,
                     sizeof(params));

  if (!servo.setMode(selectedServoId, STS3032::MODE_POSITION)) {
    Serial.println("WRITE mode failed");
    return false;
  }

  Serial.println("WRITE mode sent (write ack disabled)");
  return true;
}

bool setVelocityMode() {
  Serial.println();
  Serial.println("WRITE mode=velocity");
  const uint8_t params[2] = {STS3032::REG_MODE, STS3032::MODE_VELOCITY};
  printPacketPreview(selectedServoId, STS3032::INST_WRITE, params,
                     sizeof(params));

  if (!servo.setMode(selectedServoId, STS3032::MODE_VELOCITY)) {
    Serial.println("WRITE velocity mode failed");
    return false;
  }

  Serial.println("WRITE velocity mode sent (write ack disabled)");
  return true;
}

bool torqueOnSelected() {
  Serial.println();
  Serial.println("WRITE torque=on");
  const uint8_t params[2] = {STS3032::REG_TORQUE_ENABLE, 1};
  printPacketPreview(selectedServoId, STS3032::INST_WRITE, params,
                     sizeof(params));
  return servo.torqueOn(selectedServoId);
}

bool torqueOffSelected() {
  Serial.println();
  Serial.println("WRITE torque=off");
  const uint8_t params[2] = {STS3032::REG_TORQUE_ENABLE, 0};
  printPacketPreview(selectedServoId, STS3032::INST_WRITE, params,
                     sizeof(params));
  return servo.torqueOff(selectedServoId);
}

bool writeVelocitySpeedRaw(int16_t signedSpeedRaw) {
  const uint16_t speedRaw = STS3032::encodeSigned15(signedSpeedRaw);
  const uint8_t params[3] = {
      STS3032::REG_GOAL_SPEED_L,
      static_cast<uint8_t>(speedRaw & 0xFF),
      static_cast<uint8_t>((speedRaw >> 8) & 0xFF),
  };

  Serial.println();
  Serial.print("WRITE velocity speed_raw=");
  Serial.print(signedSpeedRaw);
  Serial.print(" speed_deg_s=");
  Serial.println(speedRawToDegPerSecond(signedSpeedRaw), 1);
  printPacketPreview(selectedServoId, STS3032::INST_WRITE, params,
                     sizeof(params));

  if (!servo.writeWord(selectedServoId, STS3032::REG_GOAL_SPEED_L, speedRaw)) {
    Serial.println("WRITE velocity speed failed");
    return false;
  }

  Serial.println("WRITE velocity speed sent (write ack disabled)");
  return true;
}

bool writePositionSpeed(uint16_t tick, uint16_t speed, uint8_t acceleration) {
  const uint16_t positionRaw = STS3032::encodeSigned15(tick);
  const uint16_t speedRaw = STS3032::encodeSigned15(speed);

  Serial.println();
  Serial.print("WRITE position tick=");
  Serial.print(tick);
  Serial.print(" speed=");
  Serial.print(speed);
  Serial.print(" acceleration=");
  Serial.println(acceleration);

  if (acceleration == 0) {
    const uint8_t params[7] = {
        STS3032::REG_GOAL_POSITION_L,
        static_cast<uint8_t>(positionRaw & 0xFF),
        static_cast<uint8_t>((positionRaw >> 8) & 0xFF),
        0x00,
        0x00,
        static_cast<uint8_t>(speedRaw & 0xFF),
        static_cast<uint8_t>((speedRaw >> 8) & 0xFF),
    };
    printPacketPreview(selectedServoId, STS3032::INST_WRITE, params,
                       sizeof(params));
    return servo.setPositionSpeed(selectedServoId, tick, speed);
  }

  const uint8_t params[8] = {
      STS3032::REG_ACCELERATION,
      acceleration,
      static_cast<uint8_t>(positionRaw & 0xFF),
      static_cast<uint8_t>((positionRaw >> 8) & 0xFF),
      0x00,
      0x00,
      static_cast<uint8_t>(speedRaw & 0xFF),
      static_cast<uint8_t>((speedRaw >> 8) & 0xFF),
  };
  printPacketPreview(selectedServoId, STS3032::INST_WRITE, params,
                     sizeof(params));
  return servo.setPositionSpeed(selectedServoId, tick, speed, acceleration);
}

bool commandPositionTick(uint16_t tick, uint16_t speed, uint8_t acceleration) {
  sweepEnabled = false;
  setPositionMode();
  torqueOnSelected();
  if (!writePositionSpeed(tick, speed, acceleration)) {
    Serial.println("Position command send failed");
    return false;
  }
  Serial.println("Position command sent");
  return true;
}

bool commandVelocityRaw(int16_t signedSpeedRaw) {
  sweepEnabled = false;
  setVelocityMode();
  torqueOnSelected();
  return writeVelocitySpeedRaw(signedSpeedRaw);
}

bool commandVelocityDegPerSecond(float speedDegPerSecond) {
  const int16_t speedRaw =
      degPerSecondToSpeedRaw(speedDegPerSecond);
  Serial.print("angular velocity command deg_s=");
  Serial.print(speedDegPerSecond, 1);
  Serial.print(" -> raw=");
  Serial.println(speedRaw);
  return commandVelocityRaw(speedRaw);
}

bool syncWriteHoldCurrentPosition() {
  STS3032::Feedback feedback;
  if (!servo.readFeedback(selectedServoId, feedback)) {
    Serial.println("Cannot SYNC_WRITE: feedback read failed");
    return false;
  }

  const uint16_t goal = feedback.position_tick;
  uint8_t params[9] = {
      STS3032::REG_GOAL_POSITION_L,
      6,
      selectedServoId,
      static_cast<uint8_t>(goal & 0xFF),
      static_cast<uint8_t>((goal >> 8) & 0xFF),
      0x00,
      0x01,
      0xE8,
      0x03,
  };

  Serial.println();
  Serial.println("SYNC_WRITE hold current position");
  printPacketPreview(STS3032::BROADCAST_ID, STS3032::INST_SYNC_WRITE, params,
                     sizeof(params));
  Serial.println("Broadcast SYNC_WRITE has no status packet by design");

  return servo.sendPacket(STS3032::BROADCAST_ID, STS3032::INST_SYNC_WRITE,
                          params, sizeof(params));
}

void sendSweepTarget() {
  const float targetDeg = sweepHighTarget ? SWEEP_AMPLITUDE_DEG
                                          : -SWEEP_AMPLITUDE_DEG;
  const uint16_t targetTick = degOffsetToTick(targetDeg);

  Serial.println();
  Serial.print("SWEEP target deg_offset=");
  Serial.print(targetDeg, 1);
  Serial.print(" tick=");
  Serial.println(targetTick);

  writePositionSpeed(targetTick, sweepSpeed, sweepAcceleration);
  sweepHighTarget = !sweepHighTarget;
  lastSweepMs = millis();
}

void startSweep(uint16_t speed, uint8_t acceleration) {
  sweepSpeed = speed;
  sweepAcceleration = acceleration;
  sweepEnabled = true;
  sweepHighTarget = false;

  setPositionMode();
  torqueOnSelected();
  sendSweepTarget();
}

void stopMotion() {
  sweepEnabled = false;
  writeVelocitySpeedRaw(0);
  setPositionMode();
  Serial.println("Stop command sent");
}

bool parseLongToken(char *token, long &value) {
  if (token == nullptr) {
    return false;
  }
  char *end = nullptr;
  value = strtol(token, &end, 10);
  return end != token && *end == '\0';
}

bool parseFloatToken(char *token, float &value) {
  if (token == nullptr) {
    return false;
  }
  char *end = nullptr;
  value = strtof(token, &end);
  return end != token && *end == '\0';
}

void printHelp() {
  Serial.println();
  Serial.println("STS3032 packet/control test commands:");
  Serial.println("  h                 help");
  Serial.println("  id <n>            select servo ID, example: id 3");
  Serial.println("  k                 checksum self-test");
  Serial.println("  p                 ping selected servo");
  Serial.println("  i                 read ID/baud/response registers");
  Serial.println("  f                 read full feedback block");
  Serial.println("  fp                read position only");
  Serial.println("  fv                read angular velocity only");
  Serial.println("  fc                read current only");
  Serial.println("  ft                read temperature only");
  Serial.println("  e                 torque on");
  Serial.println("  q                 torque off");
  Serial.println("  m                 write position mode");
  Serial.println("  t <tick> <speed> [accel]");
  Serial.println("                    position command, example: t 2048 1000");
  Serial.println("  a <deg> <speed> [accel]");
  Serial.println("                    degree offset from center, example: a 90 800");
  Serial.println("  c [speed]         center position, example: c 800");
  Serial.println("  v <deg_s>         velocity mode angular speed, example: v 360");
  Serial.println("  vr <raw>          velocity mode raw speed, example: vr 1000");
  Serial.println("  w <speed> [accel] 180 deg sweep around center");
  Serial.println("  x                 stop velocity/sweep");
  Serial.println("  y                 SYNC_WRITE current position as goal");
  Serial.println();
  Serial.println("Newline is best, but No line ending also works after a short idle.");
  Serial.println("Note: write commands print 'sent' even if readback wiring is not working.");
  Serial.println();
}

void processCommandLine(char *line) {
  char *command = strtok(line, " ,\t");
  if (command == nullptr) {
    return;
  }

  for (char *p = command; *p != '\0'; ++p) {
    *p = static_cast<char>(tolower(*p));
  }

  if (strcmp(command, "h") == 0 || strcmp(command, "?") == 0) {
    printHelp();
  } else if (strcmp(command, "id") == 0) {
    long id = 0;
    if (!parseLongToken(strtok(nullptr, " ,\t"), id) || id < 1 || id > 253) {
      Serial.println("Usage: id <1-253>");
      return;
    }
    selectedServoId = static_cast<uint8_t>(id);
    Serial.print("selected servo id=");
    Serial.println(selectedServoId);
  } else if (strcmp(command, "k") == 0) {
    checksumSelfTest();
  } else if (strcmp(command, "p") == 0) {
    pingServo();
  } else if (strcmp(command, "i") == 0) {
    readIdentityBlock();
  } else if (strcmp(command, "f") == 0) {
    readFeedbackOnce();
  } else if (strcmp(command, "fp") == 0) {
    readPositionTest();
  } else if (strcmp(command, "fv") == 0) {
    readVelocityTest();
  } else if (strcmp(command, "fc") == 0) {
    readCurrentTest();
  } else if (strcmp(command, "ft") == 0) {
    readTemperatureTest();
  } else if (strcmp(command, "e") == 0) {
    torqueOnSelected();
  } else if (strcmp(command, "q") == 0) {
    torqueOffSelected();
  } else if (strcmp(command, "m") == 0) {
    setPositionMode();
  } else if (strcmp(command, "t") == 0) {
    long tick = 0;
    long speed = 0;
    long acceleration = DEFAULT_ACCELERATION;
    if (!parseLongToken(strtok(nullptr, " ,\t"), tick) ||
        !parseLongToken(strtok(nullptr, " ,\t"), speed)) {
      Serial.println("Usage: t <tick 0-4095> <speed> [accel]");
      return;
    }
    parseLongToken(strtok(nullptr, " ,\t"), acceleration);
    commandPositionTick(clampPositionTick(tick), clampPositionSpeed(speed),
                        clampAcceleration(acceleration));
  } else if (strcmp(command, "a") == 0) {
    float degrees = 0.0f;
    long speed = 0;
    long acceleration = DEFAULT_ACCELERATION;
    if (!parseFloatToken(strtok(nullptr, " ,\t"), degrees) ||
        !parseLongToken(strtok(nullptr, " ,\t"), speed)) {
      Serial.println("Usage: a <deg offset from center> <speed> [accel]");
      return;
    }
    parseLongToken(strtok(nullptr, " ,\t"), acceleration);
    commandPositionTick(degOffsetToTick(degrees), clampPositionSpeed(speed),
                        clampAcceleration(acceleration));
  } else if (strcmp(command, "c") == 0) {
    long speed = DEFAULT_POSITION_SPEED;
    long acceleration = DEFAULT_ACCELERATION;
    parseLongToken(strtok(nullptr, " ,\t"), speed);
    parseLongToken(strtok(nullptr, " ,\t"), acceleration);
    commandPositionTick(CENTER_TICK, clampPositionSpeed(speed),
                        clampAcceleration(acceleration));
  } else if (strcmp(command, "v") == 0) {
    float speedDegPerSecond = 0.0f;
    if (!parseFloatToken(strtok(nullptr, " ,\t"), speedDegPerSecond)) {
      Serial.println("Usage: v <deg_s>, example: v 360");
      return;
    }
    commandVelocityDegPerSecond(speedDegPerSecond);
  } else if (strcmp(command, "vr") == 0) {
    long signedSpeed = 0;
    if (!parseLongToken(strtok(nullptr, " ,\t"), signedSpeed)) {
      Serial.println("Usage: vr <signed_speed_raw>, example: vr 1000");
      return;
    }
    commandVelocityRaw(clampSignedSpeed(signedSpeed));
  } else if (strcmp(command, "w") == 0) {
    long speed = DEFAULT_POSITION_SPEED;
    long acceleration = DEFAULT_ACCELERATION;
    parseLongToken(strtok(nullptr, " ,\t"), speed);
    parseLongToken(strtok(nullptr, " ,\t"), acceleration);
    startSweep(clampPositionSpeed(speed), clampAcceleration(acceleration));
  } else if (strcmp(command, "x") == 0) {
    stopMotion();
  } else if (strcmp(command, "y") == 0) {
    syncWriteHoldCurrentPosition();
  } else {
    Serial.println("Unknown command. Type h for help.");
  }
}

void processBufferedInput() {
  if (inputLineLength == 0) {
    return;
  }

  inputLine[inputLineLength] = '\0';
  Serial.print("RX command: ");
  Serial.println(inputLine);
  processCommandLine(inputLine);
  inputLineLength = 0;
  lastCommandEndMs = millis();
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r' || c == '\n') {
      processBufferedInput();
      return;
    }

    if (inputLineLength < sizeof(inputLine) - 1) {
      inputLine[inputLineLength++] = c;
      lastInputByteMs = millis();
    } else {
      inputLineLength = 0;
      Serial.println("Input line too long");
    }
  }

  if (inputLineLength > 0 &&
      millis() - lastInputByteMs >= COMMAND_IDLE_PROCESS_MS &&
      millis() - lastCommandEndMs >= COMMAND_IDLE_PROCESS_MS) {
    processBufferedInput();
  }
}

void runBootTests() {
  if (RUN_CHECKSUM_ON_BOOT) {
    checksumSelfTest();
  }

  if (RUN_READ_TESTS_ON_BOOT) {
    pingServo();
    readIdentityBlock();
    readFeedbackOnce();
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  STS3032_SERVO_SERIAL.begin(SERVO_BAUD);

  while (!Serial && millis() < 3000) {
    delay(10);
  }

  servo.begin(static_cast<Stream &>(STS3032_SERVO_SERIAL), DIRECTION_PIN);
  servo.setTimeout(PACKET_TIMEOUT_MS);
  servo.setWriteAck(false);

  Serial.println();
  Serial.println("STS3032 PacketFunctionTest");
  Serial.print("selected_servo_id=");
  Serial.print(selectedServoId);
  Serial.print(" servo_baud=");
  Serial.println(SERVO_BAUD);

  runBootTests();
  printHelp();
}

void loop() {
  handleSerialInput();

  if (sweepEnabled && millis() - lastSweepMs >= SWEEP_INTERVAL_MS) {
    sendSweepTarget();
  }
}
