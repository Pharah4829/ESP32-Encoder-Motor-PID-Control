#include "KaeriMotor.h"

KaeriMotor motor1(5, 6, 7, 8);

bool running = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== Encoder Calibration Mode ===");
  Serial.println("Type a PWM value (0~255) and press Enter to start rotating");
  Serial.println("When ONE full revolution is completed, type 's' and press Enter");
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // 정지 + 캘리브레이션 종료
    if (input == "s" || input == "S") {
      if (running) {
        float ticks = motor1.CalibrationFinish();

        Serial.println();
        Serial.print("Measured ticksPerRev = ");
        Serial.println(ticks);
        Serial.println("Calibration finished.");
        Serial.println();

        running = false;
      } else {
        Serial.println("Motor is not running.");
      }
      return;
    }

    int pwm = input.toInt();
    if (pwm > 0 && pwm <= 255) {
      Serial.print("Start rotating with PWM = ");
      Serial.println(pwm);

      motor1.CalibrationStart(pwm);
      running = true;
    } else {
      Serial.println("Invalid input. Enter PWM (1~255) or 's' to stop.");
    }
  }
}
