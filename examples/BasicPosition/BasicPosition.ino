#include <STS3032.h>

static const uint8_t SERVO_ID = 1;

STS3032 servo;

void setup() {
  Serial.begin(115200);
  servo.begin(Serial1, STS3032::DEFAULT_BAUDRATE);
  servo.setTimeout(25);

  servo.torqueOn(SERVO_ID);
  servo.setMode(SERVO_ID, STS3032::MODE_POSITION);
  servo.setPositionSpeed(SERVO_ID, 2048, 1500);
}

void loop() {
  STS3032::Feedback feedback;

  if (servo.readFeedback(SERVO_ID, feedback)) {
    Serial.print("position_tick=");
    Serial.print(feedback.position_tick);
    Serial.print(" position_deg=");
    Serial.print(feedback.position_deg, 1);
    Serial.print(" speed_raw=");
    Serial.print(feedback.speed_raw);
    Serial.print(" speed_deg_s=");
    Serial.print(feedback.speed_deg_s, 1);
    Serial.print(" load_raw=");
    Serial.print(feedback.load_raw);
    Serial.print(" voltage_v=");
    Serial.print(feedback.voltage_v, 1);
    Serial.print(" temperature_c=");
    Serial.print(feedback.temperature_c);
    Serial.print(" status=0x");
    Serial.print(feedback.status, HEX);
    Serial.print(" moving=");
    Serial.print(feedback.moving);
    Serial.print(" current_a=");
    Serial.println(feedback.current_a, 3);
  }

  delay(100);
}
