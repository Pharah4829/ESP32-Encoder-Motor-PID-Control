#include "KaeriMotor.h"

KaeriMotor motor1(5, 6, 7, 8, 20);

int flag=0;

void setup() {
  Serial.begin(115200);
  motor1.encoder(16000, 1);
  motor1.PIDset(0.15, 0.002, 0.2);
  motor1.Begin(2);
}

void loop() {
  if (flag >= 100) {
    motor1.SetPositionInc(0.5);  
    flag = 0;
  }

  flag++;
  Serial.println(motor1.GetPosition(), 2);
  delay(10);   
}
