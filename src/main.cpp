#include <Arduino.h>

#include "EDULITE_05.hpp"

QDD_Motor motor;

void setup() {
  Serial.begin(115200);

  if (!motor.begin(25, 26)) {
    Serial.println("CAN initialization failed");
    return;
  }

  // 速度制御モード

  motor.setControlMode(motor.Velocity);
  // トルク制限 [N·m]
  motor.setTorqueLimitNm(0.1f);
  // 目標速度 [rad/s]
  motor.setVelocityRadPerSec(0.0f);


  //位置制御
/*
  motor.setControlMode(motor.Position);
  //motor.setPositionRad(1.5708f); // 目標位置 約90°
  motor.setPositionDeg(-180);
  
  //電流制御
  motor.setControlMode(motor.Current);
  motor.setCurrentAmp(0.2f); // 指令電流 [A]
  */

  motor.enable();

  //motor.disable();
}

float ax=0;
void loop() {
  delay(500);
  ax-=0.1f;
  motor.setVelocityRadPerSec(ax);
  if(ax<-8.0f) while(1);
}