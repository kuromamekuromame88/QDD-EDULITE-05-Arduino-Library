
/*
 Robstride EDULITE 05をESP32で使用するためのライブラリ


 残りの作業
 複数モーターの制御を前提とした形式に再構築
*/

#pragma once

#include <Arduino.h>
#include <esp_can.hpp>
#include <cstring>
#include <cmath>

using namespace std;

class QDD_Motor {
  public:
    enum ControlMode{
      Motion,
      Position,
      Velocity,
      Current,
      CSP,
    };

    bool begin(uint8_t CanTxPin, uint8_t CanRxPin);

    // 制御モード
    void setControlMode(ControlMode mode);

    // モーター有効化・停止
    void enable();
    void disable();

    //角度指定
    void setPositionRad(float positionRad);
    void setPositionDeg(float positionDeg);

    //速度指定
    void setVelocityRadPerSec(float velocityRadPerSec);
    void setCurrentAmp(float currentAmp);

    //電流制限
    void setTorqueLimitNm(float torqueLimitNm);
    void setCurrentLimitAmp(float currentLimitAmp);

  private:
    CanDriver EDULITE_05;

    // パラメータアドレス
    static constexpr uint16_t PARAM_RUN_MODE     = 0x7005;
    static constexpr uint16_t PARAM_IQ_REF       = 0x7006;
    static constexpr uint16_t PARAM_SPD_REF      = 0x700A;
    static constexpr uint16_t PARAM_LIMIT_TORQUE = 0x700B;
    static constexpr uint16_t PARAM_LOC_REF      = 0x7016;
    static constexpr uint16_t PARAM_LIMIT_CUR    = 0x7018;

    // CAN ID
    static constexpr uint32_t CAN_ID_WRITE_PARAM = 0x1200FD7F;
    static constexpr uint32_t CAN_ID_ENABLE      = 0x0300FD7F;
    static constexpr uint32_t CAN_ID_STOP        = 0x0400FD7F;

    void writeParameterU8(uint16_t address, uint8_t value);
    void writeParameterFloat(uint16_t address, float value);

    static void onReceive(twai_message_t msg);
};


// 初期化
inline bool QDD_Motor::begin(uint8_t CanTxPin,uint8_t CanRxPin){
  if(!EDULITE_05.begin(1000000, CanTxPin, CanRxPin)){
    return false;
  }

  //一旦無効化
  //EDULITE_05.onReceive(onReceive);

  return true;
}


// CAN受信ログ
inline void QDD_Motor::onReceive(twai_message_t msg){
  Serial.printf(
    "RX <- ID:0x%08lX DLC:%d DATA:",
    static_cast<unsigned long>(msg.identifier),
    msg.data_length_code
  );

  for(uint8_t i = 0; i < msg.data_length_code; i++){
    Serial.printf(" %02X", msg.data[i]);
  }

  Serial.println();
}


// パラメータ書き込み

inline void QDD_Motor::writeParameterU8(uint16_t address, uint8_t value){
  uint8_t data[8] = {};

  data[0] = address & 0xFF;
  data[1] = (address >> 8) & 0xFF;
  data[4] = value;

  EDULITE_05.sendExtended(CAN_ID_WRITE_PARAM, data, sizeof(data));
}

inline void QDD_Motor::writeParameterFloat(uint16_t address, float value){
  static_assert(sizeof(float) == 4, "32-bit float required");

  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));

  uint8_t data[8] = {};

  data[0] = address & 0xFF;
  data[1] = (address >> 8) & 0xFF;
  // IEEE 754 floatをリトルエンディアンで送信
  data[4] = bits & 0xFF;
  data[5] = (bits >> 8) & 0xFF;
  data[6] = (bits >> 16) & 0xFF;
  data[7] = (bits >> 24) & 0xFF;

  EDULITE_05.sendExtended(CAN_ID_WRITE_PARAM, data, sizeof(data));
}


// 制御モード
inline void QDD_Motor::setControlMode(ControlMode mode){
  writeParameterU8(PARAM_RUN_MODE, static_cast<uint8_t>(mode));
  // モード設定が反映される時間を確保
  delay(50);
}


// モーター有効化・停止
inline void QDD_Motor::enable(){
  uint8_t data[8] = {};
  EDULITE_05.sendExtended(CAN_ID_ENABLE, data, sizeof(data));
}

inline void QDD_Motor::disable(){
  // 先に速度指令をゼロにする
  writeParameterFloat(PARAM_SPD_REF, 0.0f);
  delay(50);
  uint8_t data[8] = {};
  EDULITE_05.sendExtended(CAN_ID_STOP, data, sizeof(data));
}


// 位置制御

// positionRad : 目標位置 [rad]
inline void QDD_Motor::setPositionRad(float positionRad){
  if(!isfinite(positionRad)) return;
  writeParameterFloat(PARAM_LOC_REF, positionRad);
}

// positionDeg : 目標位置 [degree]
inline void QDD_Motor::setPositionDeg(float positionDeg){
  if(!isfinite(positionDeg)) return;
  //constexpr float DEG_TO_RAD = 0.017453292519943295f;
  setPositionRad(positionDeg * DEG_TO_RAD);
}


// 速度制御

// velocityRadPerSec : 目標角速度 [rad/s]
inline void QDD_Motor::setVelocityRadPerSec(float velocityRadPerSec){
  if(!isfinite(velocityRadPerSec)) return;
  writeParameterFloat(PARAM_SPD_REF, velocityRadPerSec);
}


// 電流制御

// currentAmp : 電流指令 [A]
// ※ iq_refがA単位であることを機種仕様で確認して使用する
inline void QDD_Motor::setCurrentAmp(float currentAmp){
  if(!isfinite(currentAmp)) return;
  writeParameterFloat( PARAM_IQ_REF, currentAmp);
}


// 制限値

// torqueLimitNm : トルク制限 [N·m]
inline void QDD_Motor::setTorqueLimitNm(float torqueLimitNm){
  if(!isfinite(torqueLimitNm) || torqueLimitNm < 0.0f) return;
  writeParameterFloat(PARAM_LIMIT_TORQUE, torqueLimitNm);
}

// currentLimitAmp : 電流制限 [A]
inline void QDD_Motor::setCurrentLimitAmp(float currentLimitAmp){
  if(!isfinite(currentLimitAmp) || currentLimitAmp < 0.0f) return;
  writeParameterFloat(PARAM_LIMIT_CUR, currentLimitAmp);
}