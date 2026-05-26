#pragma once

#include <Arduino.h>

class STS3032 {
public:
  static const uint8_t BROADCAST_ID = 0xFE;
  static const uint32_t DEFAULT_BAUDRATE = 1000000UL;
  static const uint16_t DEFAULT_TIMEOUT_MS = 20;
  static const uint8_t MAX_STATUS_PARAMS = 32;
  static const int16_t READ_FAILED = -32768;
  static constexpr float SPEED_UNIT_RPM = 0.732f / 50.0f;
  static constexpr float SPEED_UNIT_DEG_PER_SEC = SPEED_UNIT_RPM * 6.0f;
  static constexpr float CURRENT_UNIT_A = 0.0065f;

  enum Instruction : uint8_t {
    INST_PING = 0x01,
    INST_READ = 0x02,
    INST_WRITE = 0x03,
    INST_REG_WRITE = 0x04,
    INST_ACTION = 0x05,
    INST_RESET = 0x06,
    INST_SYNC_WRITE = 0x83
  };

  enum Register : uint8_t {
    REG_ID = 0x05,
    REG_BAUD_RATE = 0x06,
    REG_RESPONSE_LEVEL = 0x08,
    REG_MIN_ANGLE_LIMIT_L = 0x09,
    REG_MAX_ANGLE_LIMIT_L = 0x0B,
    REG_MAX_TORQUE_L = 0x10,
    REG_PHASE = 0x12,
    REG_UNLOAD_CONDITION = 0x13,
    REG_LED_ALARM_CONDITION = 0x14,
    REG_MODE = 0x21,
    REG_TORQUE_ENABLE = 0x28,
    REG_ACCELERATION = 0x29,
    REG_GOAL_POSITION_L = 0x2A,
    REG_GOAL_TIME_L = 0x2C,
    REG_GOAL_SPEED_L = 0x2E,
    REG_TORQUE_LIMIT_L = 0x30,
    REG_LOCK = 0x37,
    REG_PRESENT_POSITION_L = 0x38,
    REG_PRESENT_SPEED_L = 0x3A,
    REG_PRESENT_LOAD_L = 0x3C,
    REG_PRESENT_VOLTAGE = 0x3E,
    REG_PRESENT_TEMPERATURE = 0x3F,
    REG_ASYNC_WRITE_FLAG = 0x40,
    REG_STATUS = 0x41,
    REG_MOVING = 0x42,
    REG_PRESENT_CURRENT_L = 0x45
  };

  enum Mode : uint8_t {
    MODE_POSITION = 0,  // Position servo mode
    MODE_VELOCITY = 1,  // Closed-loop velocity mode
    MODE_OPEN_LOOP = 2, // Open-loop motor mode
    MODE_STEP = 3       // Relative step mode
  };

  enum ServoStatus : uint8_t {
    STATUS_VOLTAGE = 1 << 0,
    STATUS_SENSOR = 1 << 1,
    STATUS_TEMPERATURE = 1 << 2,
    STATUS_CURRENT = 1 << 3,
    STATUS_ANGLE = 1 << 4,
    STATUS_OVERLOAD = 1 << 5
  };

  struct StatusPacket {
    uint8_t id;
    uint8_t length;
    uint8_t error;
    uint8_t parameters[MAX_STATUS_PARAMS];
    uint8_t parameter_length;
    uint8_t checksum;

    StatusPacket();
  };

  struct Feedback {
    uint16_t position_tick; // 0-4095 in single-turn position feedback
    float position_deg;    // position_tick * 360.0 / 4096.0
    int16_t speed_raw;     // Signed steps/s after decoding the direction bit
    float speed_rpm;       // speed_raw * (0.732 / 50.0) RPM
    float speed_deg_s;     // speed_rpm * 6.0
    int16_t load_raw;      // Signed after decoding the load direction bit
    float voltage_v;       // Raw * 0.1 V
    uint8_t temperature_c; // Celsius
    uint8_t status;        // ServoStatus bit flags
    bool moving;           // true when the servo reports motion
    float current_a;       // Raw * 0.0065 A

    Feedback();
  };

  STS3032();
  explicit STS3032(Stream &serial, int8_t directionPin = -1);

  void begin(Stream &serial, int8_t directionPin = -1);
  void begin(HardwareSerial &serial, uint32_t baudrate = DEFAULT_BAUDRATE,
             int8_t directionPin = -1);
  void setDirectionPin(int8_t directionPin);
  void setTimeout(uint16_t timeoutMs);
  void setWriteAck(bool enabled);

  uint8_t checksum(uint8_t id, uint8_t length, uint8_t instructionOrError,
                   const uint8_t *parameters = nullptr,
                   uint8_t parameterLength = 0) const;
  bool sendPacket(uint8_t id, uint8_t instruction,
                  const uint8_t *parameters = nullptr,
                  uint8_t parameterLength = 0);
  bool readStatusPacket(StatusPacket &packet, uint16_t timeoutMs = 0);
  bool writeByte(uint8_t id, uint8_t address, uint8_t value);
  bool writeWord(uint8_t id, uint8_t address, uint16_t value);
  bool readBytes(uint8_t id, uint8_t address, uint8_t *data, uint8_t length);

  bool setPosition(uint8_t id, int16_t positionTick);
  bool setPositionSpeed(uint8_t id, int16_t positionTick, uint16_t speed);
  bool setPositionSpeed(uint8_t id, int16_t positionTick, uint16_t speed,
                        uint8_t acceleration);
  bool readPosition(uint8_t id, uint16_t &positionTick);
  bool readPosition(uint8_t id, int16_t &positionTick);
  int16_t readPosition(uint8_t id);
  bool readVelocity(uint8_t id, int16_t &speedRaw);
  bool readAngularVelocity(uint8_t id, float &speedDegPerSecond);
  bool readCurrent(uint8_t id, float &currentA);
  bool readTemperature(uint8_t id, uint8_t &temperatureC);
  bool readFeedback(uint8_t id, Feedback &feedback);
  bool torqueOn(uint8_t id);
  bool torqueOff(uint8_t id);
  bool setMode(uint8_t id, Mode mode);
  bool setMode(uint8_t id, uint8_t mode);

  static uint16_t encodeSigned15(int16_t value);
  static int16_t decodeSigned15(uint16_t raw);
  static int16_t decodeSigned10(uint16_t raw);
  static float speedRawToRpm(int16_t speedRaw);
  static float speedRawToDegPerSecond(int16_t speedRaw);
  static int16_t degPerSecondToSpeedRaw(float speedDegPerSecond);
  static float currentRawToAmp(int16_t currentRaw);

private:
  static const uint8_t MAX_WRITE_DATA = 32;
  static const uint16_t MAX_PACKET_BYTES = 259;

  Stream *_serial;
  int8_t _directionPin;
  uint16_t _timeoutMs;
  bool _writeAck;
  uint8_t _lastTxPacket[MAX_PACKET_BYTES];
  uint16_t _lastTxPacketLength;

  void setTxMode();
  void setRxMode();
  void clearRx();
  bool readByteTimed(uint8_t &value, unsigned long deadline);
  bool isLastTxEcho(const StatusPacket &packet) const;
  bool writeData(uint8_t id, uint8_t address, const uint8_t *data,
                 uint8_t length);
  static uint16_t makeWord(uint8_t low, uint8_t high);
  static uint16_t clampMagnitude15(int32_t value);
};
