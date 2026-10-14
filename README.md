# QDD-EDULITE-05-Arduino-Library

このライブラリは、Robstride EDULITE 05 をESP32でCANで制御するためのライブラリです。

## 機能
現在搭載されている機能として、速度・トルク制御、角度制御、電流制御を用意しています。
電流センサについては動作を確認していません。
また、その他のセンサーの読み取りなどはまだ行っていません。いずれ追加します。

## 使用しているライブラリ
esp_canを使用しています。

## 使い方

例
```cpp

#include <Arudino.h>

//ライブラリのファイルの読み込み
#include "EDULITE_05.hpp"

//インスタンスの初期化
QDD_Motor motor;

void setup(){

  //CAN通信に使用するピンの設定。bool begin(TxPin, RxPin)となっています。
  if(!motor.begin(25, 26)){
    Serial.println("CAN通信の初期化に失敗しました。");
  }
  Serial.println("CAN通信の初期化に成功しました。");


  // 速度制御モード
  motor.setControlMode(motor.Velocity);
  // トルク制限 [N·m]
  motor.setTorqueLimitNm(0.1f);
  // 目標速度 [rad/s]
  motor.setVelocityRadPerSec(0.5f);


  //位置制御
  motor.setControlMode(motor.Position);
  motor.setPositionRad(1.5708f); // 目標位置 約90°
  motor.setPositionDeg(-180);
  
  //電流制御
  motor.setControlMode(motor.Current);
  motor.setCurrentAmp(0.2f); // 指令電流 [A]
  
  //モーターの有効化、無効化
  motor.enable();
  motor.disable();
}

void loop(){
  
}

```