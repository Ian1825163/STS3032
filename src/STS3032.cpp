#include "STS3032.h"

#include <math.h>

STS3032::StatusPacket::StatusPacket()
    : id(0), length(0), error(0), parameter_length(0), checksum(0) {
  for (uint8_t i = 0; i < MAX_STATUS_PARAMS; ++i) {
    parameters[i] = 0;
  }
}

STS3032::Feedback::Feedback()
    : position_tick(0), position_deg(0.0f), speed_raw(0), speed_rpm(0.0f),
      speed_deg_s(0.0f), load_raw(0), voltage_v(0.0f), temperature_c(0),
      status(0), moving(false), current_a(0.0f) {}

STS3032::STS3032()
    : _serial(nullptr), _directionPin(-1),
      _timeoutMs(DEFAULT_TIMEOUT_MS), _writeAck(false) {}

STS3032::STS3032(Stream &serial, int8_t directionPin) : STS3032() {
  begin(serial, directionPin);
}

void STS3032::begin(Stream &serial, int8_t directionPin) {
  _serial = &serial;
  setDirectionPin(directionPin);
}

void STS3032::begin(HardwareSerial &serial, uint32_t baudrate,
                    int8_t directionPin) {
  serial.begin(baudrate);
  begin(static_cast<Stream &>(serial), directionPin);
}

void STS3032::setDirectionPin(int8_t directionPin) {
  _directionPin = directionPin;
  if (_directionPin >= 0) {
    pinMode(_directionPin, OUTPUT);
    setRxMode();
  }
}

void STS3032::setTimeout(uint16_t timeoutMs) { _timeoutMs = timeoutMs; }

void STS3032::setWriteAck(bool enabled) { _writeAck = enabled; }

uint8_t STS3032::checksum(uint8_t id, uint8_t length,
                          uint8_t instructionOrError,
                          const uint8_t *parameters,
                          uint8_t parameterLength) const {
  uint16_t sum = id + length + instructionOrError;
  for (uint8_t i = 0; i < parameterLength; ++i) {
    sum += parameters[i];
  }
  return static_cast<uint8_t>(~(sum & 0xFF));
}

bool STS3032::sendPacket(uint8_t id, uint8_t instruction,
                         const uint8_t *parameters,
                         uint8_t parameterLength) {
  if (_serial == nullptr || parameterLength > 253) {
    return false;
  }

  const uint8_t length = parameterLength + 2;
  const uint8_t packetChecksum =
      checksum(id, length, instruction, parameters, parameterLength);

  clearRx();
  setTxMode();

  size_t written = 0;
  written += _serial->write(0xFF);
  written += _serial->write(0xFF);
  written += _serial->write(id);
  written += _serial->write(length);
  written += _serial->write(instruction);
  for (uint8_t i = 0; i < parameterLength; ++i) {
    written += _serial->write(parameters[i]);
  }
  written += _serial->write(packetChecksum);

  _serial->flush();
  setRxMode();

  return written == static_cast<size_t>(parameterLength + 6);
}

bool STS3032::readStatusPacket(StatusPacket &packet, uint16_t timeoutMs) {
  if (_serial == nullptr) {
    return false;
  }

  if (timeoutMs == 0) {
    timeoutMs = _timeoutMs;
  }

  const unsigned long deadline = millis() + timeoutMs;
  uint8_t previous = 0;
  uint8_t current = 0;
  bool foundHeader = false;

  while (readByteTimed(current, deadline)) {
    if (previous == 0xFF && current == 0xFF) {
      foundHeader = true;
      break;
    }
    previous = current;
  }

  if (!foundHeader) {
    return false;
  }

  if (!readByteTimed(packet.id, deadline) ||
      !readByteTimed(packet.length, deadline) ||
      !readByteTimed(packet.error, deadline)) {
    return false;
  }

  if (packet.length < 2) {
    return false;
  }

  packet.parameter_length = packet.length - 2;
  if (packet.parameter_length > MAX_STATUS_PARAMS) {
    return false;
  }

  for (uint8_t i = 0; i < packet.parameter_length; ++i) {
    if (!readByteTimed(packet.parameters[i], deadline)) {
      return false;
    }
  }

  if (!readByteTimed(packet.checksum, deadline)) {
    return false;
  }

  const uint8_t expected =
      checksum(packet.id, packet.length, packet.error, packet.parameters,
               packet.parameter_length);
  return packet.checksum == expected;
}

bool STS3032::writeByte(uint8_t id, uint8_t address, uint8_t value) {
  return writeData(id, address, &value, 1);
}

bool STS3032::writeWord(uint8_t id, uint8_t address, uint16_t value) {
  const uint8_t data[2] = {
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF),
  };
  return writeData(id, address, data, 2);
}

bool STS3032::readBytes(uint8_t id, uint8_t address, uint8_t *data,
                        uint8_t length) {
  if (data == nullptr || length == 0 || length > MAX_STATUS_PARAMS) {
    return false;
  }

  const uint8_t params[2] = {address, length};
  if (!sendPacket(id, INST_READ, params, sizeof(params))) {
    return false;
  }

  StatusPacket packet;
  if (!readStatusPacket(packet)) {
    return false;
  }

  if (packet.id != id || packet.error != 0 ||
      packet.parameter_length != length) {
    return false;
  }

  for (uint8_t i = 0; i < length; ++i) {
    data[i] = packet.parameters[i];
  }
  return true;
}

bool STS3032::setPosition(uint8_t id, int16_t positionTick) {
  return writeWord(id, REG_GOAL_POSITION_L, encodeSigned15(positionTick));
}

bool STS3032::setPositionSpeed(uint8_t id, int16_t positionTick,
                               uint16_t speed) {
  const uint16_t positionRaw = encodeSigned15(positionTick);
  const uint16_t speedRaw = clampMagnitude15(speed);
  const uint8_t data[6] = {
      static_cast<uint8_t>(positionRaw & 0xFF),
      static_cast<uint8_t>((positionRaw >> 8) & 0xFF),
      0x00,
      0x00,
      static_cast<uint8_t>(speedRaw & 0xFF),
      static_cast<uint8_t>((speedRaw >> 8) & 0xFF),
  };
  return writeData(id, REG_GOAL_POSITION_L, data, sizeof(data));
}

bool STS3032::setPositionSpeed(uint8_t id, int16_t positionTick,
                               uint16_t speed, uint8_t acceleration) {
  if (acceleration == 0) {
    return setPositionSpeed(id, positionTick, speed);
  }

  const uint16_t positionRaw = encodeSigned15(positionTick);
  const uint16_t speedRaw = clampMagnitude15(speed);
  const uint8_t data[7] = {
      acceleration,
      static_cast<uint8_t>(positionRaw & 0xFF),
      static_cast<uint8_t>((positionRaw >> 8) & 0xFF),
      0x00,
      0x00,
      static_cast<uint8_t>(speedRaw & 0xFF),
      static_cast<uint8_t>((speedRaw >> 8) & 0xFF),
  };
  return writeData(id, REG_ACCELERATION, data, sizeof(data));
}

bool STS3032::readPosition(uint8_t id, uint16_t &positionTick) {
  uint8_t data[2] = {0, 0};
  if (!readBytes(id, REG_PRESENT_POSITION_L, data, sizeof(data))) {
    return false;
  }
  positionTick = makeWord(data[0], data[1]);
  return true;
}

bool STS3032::readPosition(uint8_t id, int16_t &positionTick) {
  uint16_t unsignedPositionTick = 0;
  if (!readPosition(id, unsignedPositionTick)) {
    return false;
  }
  positionTick = static_cast<int16_t>(unsignedPositionTick);
  return true;
}

int16_t STS3032::readPosition(uint8_t id) {
  int16_t positionTick = 0;
  if (!readPosition(id, positionTick)) {
    return READ_FAILED;
  }
  return positionTick;
}

bool STS3032::readVelocity(uint8_t id, int16_t &speedRaw) {
  uint8_t data[2] = {0, 0};
  if (!readBytes(id, REG_PRESENT_SPEED_L, data, sizeof(data))) {
    return false;
  }
  speedRaw = decodeSigned15(makeWord(data[0], data[1]));
  return true;
}

bool STS3032::readAngularVelocity(uint8_t id, float &speedDegPerSecond) {
  int16_t speedRaw = 0;
  if (!readVelocity(id, speedRaw)) {
    return false;
  }
  speedDegPerSecond = speedRawToDegPerSecond(speedRaw);
  return true;
}

bool STS3032::readCurrent(uint8_t id, float &currentA) {
  uint8_t data[2] = {0, 0};
  if (!readBytes(id, REG_PRESENT_CURRENT_L, data, sizeof(data))) {
    return false;
  }
  currentA = currentRawToAmp(decodeSigned15(makeWord(data[0], data[1])));
  return true;
}

bool STS3032::readTemperature(uint8_t id, uint8_t &temperatureC) {
  uint8_t data = 0;
  if (!readBytes(id, REG_PRESENT_TEMPERATURE, &data, 1)) {
    return false;
  }
  temperatureC = data;
  return true;
}

bool STS3032::readFeedback(uint8_t id, Feedback &feedback) {
  uint8_t data[15] = {0};
  if (!readBytes(id, REG_PRESENT_POSITION_L, data, sizeof(data))) {
    return false;
  }

  feedback.position_tick = makeWord(data[0], data[1]);
  feedback.position_deg = feedback.position_tick * (360.0f / 4096.0f);
  feedback.speed_raw = decodeSigned15(makeWord(data[2], data[3]));
  feedback.speed_rpm = speedRawToRpm(feedback.speed_raw);
  feedback.speed_deg_s = speedRawToDegPerSecond(feedback.speed_raw);
  feedback.load_raw = decodeSigned10(makeWord(data[4], data[5]));
  feedback.voltage_v = data[6] * 0.1f;
  feedback.temperature_c = data[7];
  feedback.status = data[9];
  feedback.moving = data[10] != 0;
  feedback.current_a =
      currentRawToAmp(decodeSigned15(makeWord(data[13], data[14])));
  return true;
}

bool STS3032::torqueOn(uint8_t id) {
  return writeByte(id, REG_TORQUE_ENABLE, 1);
}

bool STS3032::torqueOff(uint8_t id) {
  return writeByte(id, REG_TORQUE_ENABLE, 0);
}

bool STS3032::setMode(uint8_t id, Mode mode) {
  return setMode(id, static_cast<uint8_t>(mode));
}

bool STS3032::setMode(uint8_t id, uint8_t mode) {
  if (mode > MODE_STEP) {
    return false;
  }
  return writeByte(id, REG_MODE, mode);
}

uint16_t STS3032::encodeSigned15(int16_t value) {
  if (value < 0) {
    return clampMagnitude15(-static_cast<int32_t>(value)) | 0x8000;
  }
  return clampMagnitude15(value);
}

int16_t STS3032::decodeSigned15(uint16_t raw) {
  const int16_t magnitude = static_cast<int16_t>(raw & 0x7FFF);
  return (raw & 0x8000) ? -magnitude : magnitude;
}

int16_t STS3032::decodeSigned10(uint16_t raw) {
  const int16_t magnitude = static_cast<int16_t>(raw & 0x03FF);
  return (raw & 0x0400) ? -magnitude : magnitude;
}

float STS3032::speedRawToRpm(int16_t speedRaw) {
  return speedRaw * SPEED_UNIT_RPM;
}

float STS3032::speedRawToDegPerSecond(int16_t speedRaw) {
  return speedRaw * SPEED_UNIT_DEG_PER_SEC;
}

int16_t STS3032::degPerSecondToSpeedRaw(float speedDegPerSecond) {
  const long raw = lroundf(speedDegPerSecond / SPEED_UNIT_DEG_PER_SEC);
  if (raw > 32766) {
    return 32766;
  }
  if (raw < -32766) {
    return -32766;
  }
  return static_cast<int16_t>(raw);
}

float STS3032::currentRawToAmp(int16_t currentRaw) {
  return currentRaw * CURRENT_UNIT_A;
}

void STS3032::setTxMode() {
  if (_directionPin >= 0) {
    digitalWrite(_directionPin, HIGH);
  }
}

void STS3032::setRxMode() {
  if (_directionPin >= 0) {
    digitalWrite(_directionPin, LOW);
  }
}

void STS3032::clearRx() {
  if (_serial == nullptr) {
    return;
  }

  while (_serial->available() > 0) {
    _serial->read();
  }
}

bool STS3032::readByteTimed(uint8_t &value, unsigned long deadline) {
  while (static_cast<long>(millis() - deadline) < 0) {
    if (_serial->available() > 0) {
      const int readValue = _serial->read();
      if (readValue >= 0) {
        value = static_cast<uint8_t>(readValue);
        return true;
      }
    }
    yield();
  }
  return false;
}

bool STS3032::writeData(uint8_t id, uint8_t address, const uint8_t *data,
                        uint8_t length) {
  if (data == nullptr || length == 0 || length > MAX_WRITE_DATA) {
    return false;
  }

  uint8_t params[MAX_WRITE_DATA + 1];
  params[0] = address;
  for (uint8_t i = 0; i < length; ++i) {
    params[i + 1] = data[i];
  }

  if (!sendPacket(id, INST_WRITE, params, length + 1)) {
    return false;
  }

  if (!_writeAck || id == BROADCAST_ID) {
    return true;
  }

  StatusPacket packet;
  return readStatusPacket(packet) && packet.id == id && packet.error == 0;
}

uint16_t STS3032::makeWord(uint8_t low, uint8_t high) {
  return static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
}

uint16_t STS3032::clampMagnitude15(int32_t value) {
  if (value < 0) {
    value = -value;
  }
  if (value > 32766) {
    value = 32766;
  }
  return static_cast<uint16_t>(value);
}
