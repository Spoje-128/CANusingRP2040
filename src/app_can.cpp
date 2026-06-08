#include "app_can.hpp"
#include "can_schema.h"

#include <Arduino.h>

namespace AppCan {

static CanBus* can = nullptr;

// heartbeatに入れる簡易状態。
// 例:
//   0 = boot
//   1 = normal
//   2 = error
static uint8_t node_state = 1;

// モータ指令の通し番号などを見たい場合に使う。
static uint8_t last_motor_cmd_seq[16] = {0};

// ------------------------------------------------------------
// ダミーセンサ
// ------------------------------------------------------------
//
// 最小構成で動作確認しやすいように、実センサの代わりに仮値を作ります。
// あとでIMU読み取り関数やエンコーダ読み取り関数に置き換えます。
// ------------------------------------------------------------

static int16_t fakeWave() {
  static int16_t v = 0;
  v += 100;
  return v;
}

// ------------------------------------------------------------
// publish系
// ------------------------------------------------------------
//
// 送信するデータ種類ごとに publishXxx() を作ります。
// 新しいセンサが増えたら、can_schema.hにPayloadを定義して、
// ここにpublish関数を足すだけで済む形にしています。
// ------------------------------------------------------------

static bool publishImuAccelGyro(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_z) {
  CanSchema::ImuAccelGyroFrame frame = {};

  // unionのvalue側に意味のある名前で値を入れる。
  // bytes側をsendStd()へ渡せば、そのまま8バイトのCANペイロードになる。
  frame.value.acc_x = acc_x;
  frame.value.acc_y = acc_y;
  frame.value.acc_z = acc_z;
  frame.value.gyro_z = gyro_z;

  return can->sendStd(CanSchema::ID_IMU_ACCEL_GYRO, frame.bytes, sizeof(frame.bytes));
}

static bool publishImuMagStatus(int16_t mag_x, int16_t mag_y, int16_t mag_z, uint8_t status) {
  CanSchema::ImuMagStatusFrame frame = {};

  frame.value.mag_x = mag_x;
  frame.value.mag_y = mag_y;
  frame.value.mag_z = mag_z;
  frame.value.status = status;
  frame.value.node_id = CanSchema::NODE_ID;

  return can->sendStd(CanSchema::ID_IMU_MAG_STATUS, frame.bytes, sizeof(frame.bytes));
}

static bool publishRpm(uint8_t motor_id, int16_t rpm, int16_t current_mA) {
  CanSchema::RpmFrame frame = {};

  frame.value.rpm = rpm;
  frame.value.current_mA = current_mA;
  frame.value.node_id = CanSchema::NODE_ID;
  frame.value.motor_id = motor_id;
  frame.value.reserved0 = 0;
  frame.value.reserved1 = 0;

  return can->sendStd(CanSchema::ID_RPM_BASE + motor_id, frame.bytes, sizeof(frame.bytes));
}

static bool publishHeartbeat() {
  CanSchema::HeartbeatFrame frame = {};

  frame.value.node_id = CanSchema::NODE_ID;
  frame.value.state = node_state;
  frame.value.rx_drop_count_lsb = can->rxDropCount() & 0xFF;
  frame.value.can_error_count_lsb = can->errorCount() & 0xFF;

  return can->sendStd(
    CanSchema::ID_HEARTBEAT_BASE + CanSchema::NODE_ID,
    frame.bytes,
    sizeof(frame.bytes)
  );
}

// ------------------------------------------------------------
// handle系
// ------------------------------------------------------------
//
// 受信フレームはIDで振り分けます。
// 必要ないIDは何もしません。
// CANの基本は「みんな流す、必要なノードだけ拾う」です。
// ------------------------------------------------------------

static void handleEstop(const CanBus::Frame& frame) {
  (void)frame;

  // ここでモータPWM停止、出力遮断、状態遷移などを行う。
  node_state = 2;

  Serial.println("ESTOP received");
}

static void handleMotorCommand(uint8_t motor_id, const CanBus::Frame& frame) {
  if (frame.dlc < 8) {
    return;
  }

  CanSchema::MotorCommandFrame cmd = {};

  // 受信した8バイトをunionのbytes側に入れる。
  // その後、value側を読むとフィールド名付きで扱える。
  memcpy(cmd.bytes, frame.data, sizeof(cmd.bytes));

  last_motor_cmd_seq[motor_id] = cmd.value.seq;

  Serial.print("CMD motor=");
  Serial.print(motor_id);
  Serial.print(" target_rpm=");
  Serial.print(cmd.value.target_rpm);
  Serial.print(" limit_mA=");
  Serial.print(cmd.value.limit_mA);
  Serial.print(" mode=");
  Serial.print(cmd.value.mode);
  Serial.print(" seq=");
  Serial.println(cmd.value.seq);

  // 実機ならここで制御目標値を書き換える。
  //
  // 例:
  //   motor[motor_id].setTargetRpm(cmd.value.target_rpm);
  //   motor[motor_id].setCurrentLimit(cmd.value.limit_mA);
  //   motor[motor_id].setMode(cmd.value.mode);
}

static void handleFrame(const CanBus::Frame& frame) {
  const uint32_t id = frame.id;

  // 拡張IDは今回使わない。
  // can2040では拡張IDの場合 CAN2040_ID_EFF フラグが立つ。
  if (id & CAN2040_ID_EFF) {
    return;
  }

  if (id == CanSchema::ID_ESTOP) {
    handleEstop(frame);
    return;
  }

  // 0x300〜0x30Fをモータ指令として扱う例。
  if (id >= CanSchema::ID_CMD_MOTOR_BASE && id < CanSchema::ID_CMD_MOTOR_BASE + 16) {
    const uint8_t motor_id = id - CanSchema::ID_CMD_MOTOR_BASE;
    handleMotorCommand(motor_id, frame);
    return;
  }

  // 他のIDはこのノードでは不要なので無視。
}

// ------------------------------------------------------------
// public API
// ------------------------------------------------------------

void begin(CanBus* bus) {
  can = bus;
}

void poll() {
  if (can == nullptr) {
    return;
  }

  CanBus::Frame frame;

  // 溜まっている受信フレームを全部処理する。
  // ただし処理が重くなりすぎる場合は、1ループあたりの処理数に上限を設けてもよい。
  while (can->receive(frame)) {
    handleFrame(frame);
  }
}

void tick() {
  if (can == nullptr) {
    return;
  }

  static uint32_t next_imu_ms = 0;
  static uint32_t next_rpm_ms = 0;
  static uint32_t next_heartbeat_ms = 0;

  const uint32_t now = millis();

  // ----------------------------------------------------------
  // IMU送信: 100Hz
  // ----------------------------------------------------------
  //
  // Classic CANでは1フレーム8バイトなので、9軸IMUは複数IDに分割します。
  // ここでは
  //   0x120: acc_x, acc_y, acc_z, gyro_z
  //   0x121: mag_x, mag_y, mag_z, status
  // の2フレーム構成。
  //
  // 実際にはgyro_x/gyro_yも欲しいはずなので、
  //   0x122: gyro_x, gyro_y, temp, timestamp
  // のように追加していくとよいです。
  // ----------------------------------------------------------
  if ((int32_t)(now - next_imu_ms) >= 0) {
    const int16_t v = fakeWave();

    publishImuAccelGyro(
      v + 1,
      v + 2,
      v + 3,
      v + 4
    );

    publishImuMagStatus(
      v + 5,
      v + 6,
      v + 7,
      0
    );

    next_imu_ms = now + 10;
  }

  // ----------------------------------------------------------
  // RPM送信: 50Hz
  // ----------------------------------------------------------
  if ((int32_t)(now - next_rpm_ms) >= 0) {
    publishRpm(0, 1234, 250);
    publishRpm(1, 2345, 300);

    next_rpm_ms = now + 20;
  }

  // ----------------------------------------------------------
  // Heartbeat送信: 1Hz
  // ----------------------------------------------------------
  if ((int32_t)(now - next_heartbeat_ms) >= 0) {
    publishHeartbeat();

    next_heartbeat_ms = now + 1000;
  }
}

}  // namespace AppCan
