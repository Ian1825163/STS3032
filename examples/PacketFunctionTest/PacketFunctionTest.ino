#include <Arduino.h>
#include <STS3032.h>

// This example defaults to Serial4 to match doggy_ws on Teensy.
// Change this to Serial1/Serial2/etc. if your STS3032 bus is wired elsewhere.
#ifndef STS3032_SERVO_SERIAL
#define STS3032_SERVO_SERIAL Serial4
#endif

static const uint8_t SERVO_ID = 1;
static const uint32_t USB_BAUD = 115200;
static const uint32_t SERVO_BAUD = STS3032::DEFAULT_BAUDRATE;
static const int8_t DIRECTION_PIN = -1;
static const uint16_t PACKET_TIMEOUT_MS = 30;
static const bool RUN_TESTS_ON_BOOT = true;

STS3032 servo;

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
      servo.checksum(SERVO_ID, 2, STS3032::INST_PING);
  const uint8_t readChecksum =
      servo.checksum(SERVO_ID, 4, STS3032::INST_READ, readParams,
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
  printPacketPreview(SERVO_ID, STS3032::INST_PING, nullptr, 0);

  if (!servo.sendPacket(SERVO_ID, STS3032::INST_PING)) {
    Serial.println("PING send failed");
    return false;
  }

  STS3032::StatusPacket packet;
  if (!servo.readStatusPacket(packet)) {
    Serial.println("PING timeout or bad checksum");
    return false;
  }

  printStatusPacket(packet);
  return packet.id == SERVO_ID && packet.error == 0;
}

bool readIdentityBlock() {
  Serial.println();
  Serial.println("READ identity block");
  const uint8_t readParams[2] = {STS3032::REG_ID, 4};
  printPacketPreview(SERVO_ID, STS3032::INST_READ, readParams,
                     sizeof(readParams));

  uint8_t data[4] = {0};
  if (!servo.readBytes(SERVO_ID, STS3032::REG_ID, data, sizeof(data))) {
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
  printPacketPreview(SERVO_ID, STS3032::INST_READ, readParams,
                     sizeof(readParams));

  STS3032::Feedback feedback;
  if (!servo.readFeedback(SERVO_ID, feedback)) {
    Serial.println("READ feedback failed");
    return false;
  }

  Serial.print("position_tick=");
  Serial.print(feedback.position_tick);
  Serial.print(" position_deg=");
  Serial.print(feedback.position_deg, 2);
  Serial.print(" speed_raw=");
  Serial.print(feedback.speed_raw);
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

bool setPositionMode() {
  Serial.println();
  Serial.println("WRITE mode=position");
  const uint8_t params[2] = {STS3032::REG_MODE, STS3032::MODE_POSITION};
  printPacketPreview(SERVO_ID, STS3032::INST_WRITE, params, sizeof(params));

  if (!servo.setMode(SERVO_ID, STS3032::MODE_POSITION)) {
    Serial.println("WRITE mode failed");
    return false;
  }

  Serial.println("WRITE mode sent");
  return true;
}

bool syncWriteHoldCurrentPosition() {
  STS3032::Feedback feedback;
  if (!servo.readFeedback(SERVO_ID, feedback)) {
    Serial.println("Cannot SYNC_WRITE: feedback read failed");
    return false;
  }

  const uint16_t goal = feedback.position_tick;
  uint8_t params[9] = {
      STS3032::REG_GOAL_POSITION_L,
      6,
      SERVO_ID,
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

void printHelp() {
  Serial.println();
  Serial.println("STS3032 packet test commands:");
  Serial.println("  h - help");
  Serial.println("  k - checksum self-test");
  Serial.println("  p - ping servo");
  Serial.println("  i - read ID/baud/response registers");
  Serial.println("  f - read feedback block");
  Serial.println("  m - write position mode");
  Serial.println("  y - SYNC_WRITE current position as goal");
  Serial.println();
}

void runBootTests() {
  checksumSelfTest();
  pingServo();
  readIdentityBlock();
  readFeedbackOnce();
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
  Serial.print("servo_id=");
  Serial.print(SERVO_ID);
  Serial.print(" servo_baud=");
  Serial.println(SERVO_BAUD);

  if (RUN_TESTS_ON_BOOT) {
    runBootTests();
  }

  printHelp();
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  const char command = static_cast<char>(Serial.read());
  switch (command) {
  case 'h':
  case '?':
    printHelp();
    break;
  case 'k':
    checksumSelfTest();
    break;
  case 'p':
    pingServo();
    break;
  case 'i':
    readIdentityBlock();
    break;
  case 'f':
    readFeedbackOnce();
    break;
  case 'm':
    setPositionMode();
    break;
  case 'y':
    syncWriteHoldCurrentPosition();
    break;
  case '\r':
  case '\n':
    break;
  default:
    Serial.println("Unknown command. Press h for help.");
    break;
  }
}
