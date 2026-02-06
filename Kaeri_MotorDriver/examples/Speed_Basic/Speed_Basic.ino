#include "KaeriMotor.h"

KaeriMotor motor1(5, 6, 7, 8, 20); // 모터 핀 매핑 (모터A, 모터B, 엔코더A, 엔코더B, 전류측정핀)
KaeriMotor motor2(10, 2, 3, 4, 1);

void setup() {
  Serial.begin(115200);

  motor1.encoder(16000, 1); // 모터 PPR 설정 (엔코더 PPR, 기어비)
  motor1.PIDset(5, 0, 0); // PID 게인 설정 (모터별 설정)
  motor1.Begin(2); // PID Loop 시작 (주기 2ms)

  motor2.encoder(7, 100);
  motor2.PIDset(5, 0, 0);
  motor2.Begin(2);
}

void loop() {
  motor1.SetSpeed(50);
  motor2.SetSpeed(0);
  delay(1000);
  motor1.SetSpeed(0);
  motor2.SetSpeed(50);
  delay(1000);
}
