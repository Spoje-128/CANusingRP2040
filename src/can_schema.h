#pragma once

#include <stdint.h>

// ============================================================
// CANスキーマ定義
// ============================================================
//
// このファイルは「CANで流れるデータの仕様書」です。
// ノードが増えても、まずここを見るだけでIDと中身が分かるようにします。
//
// Classic CAN標準IDは11bit、つまり 0x000〜0x7FF。
// CANではIDが小さいほど優先度が高いです。
// 緊急停止などは低いID、周期センサは中くらい、デバッグは高いIDにします。
// ============================================================

namespace CanSchema {

// ------------------------------------------------------------
// ノードID
// ------------------------------------------------------------
//
// 全ノードに同じファームを書く場合は、DIPスイッチ、EEPROM、Flash設定、
// コンパイル時defineなどでNODE_IDを変える設計にすると便利です。
//
static constexpr uint8_t NODE_ID = 1;

// ------------------------------------------------------------
// CAN ID割り当て
// ------------------------------------------------------------

static constexpr uint16_t ID_ESTOP          = 0x010;

static constexpr uint16_t ID_IMU_ACCEL_GYRO = 0x120;
static constexpr uint16_t ID_IMU_MAG_STATUS = 0x121;

static constexpr uint16_t ID_RPM_BASE       = 0x200;  // motor_idを足す
static constexpr uint16_t ID_CMD_MOTOR_BASE = 0x300;  // motor_idを足す

static constexpr uint16_t ID_HEARTBEAT_BASE = 0x700;  // node_idを足す

// ------------------------------------------------------------
// 共通
// ------------------------------------------------------------
//
// Classic CANのデータ長は最大8バイト。
// そのため、各Payloadは必ず8バイト以内にします。
// static_assertでサイズを強制しておくと、あとでフィールドを足しすぎた時に
// コンパイルエラーで気づけます。
// ------------------------------------------------------------

// GCC/Clang用のpacked指定。
// Arduino-PicoはGCCなのでこれでOK。
#define CAN_PACKED __attribute__((packed))

// ------------------------------------------------------------
// IMU: 加速度 + ジャイロの一部
// ------------------------------------------------------------
//
// 9軸IMUをClassic CANで1フレームに全部入れるのは無理です。
// ここでは例として、
//   acc_x, acc_y, acc_z, gyro_z
// を1フレームに入れています。
//
// 単位はアプリ側で決めます。
// 例:
//   acc_*  = mg単位
//   gyro_z = 0.01 dps単位
//
// CANはただのバイト列なので、単位をコメントで固定しておくのが重要です。
// ------------------------------------------------------------

struct CAN_PACKED ImuAccelGyro {
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t gyro_z;
};

union ImuAccelGyroFrame {
  ImuAccelGyro value;
  uint8_t bytes[8];
};

static_assert(sizeof(ImuAccelGyro) == 8, "ImuAccelGyro must be 8 bytes");

// ------------------------------------------------------------
// IMU: 磁気 + 状態
// ------------------------------------------------------------
//
// mag_*:
//   磁気センサ値。単位は任意。
// status:
//   キャリブレーション状態、センサ異常、データ有効フラグなどに使う。
// node_id:
//   どのノードが出したデータか。
// ------------------------------------------------------------

struct CAN_PACKED ImuMagStatus {
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
  uint8_t status;
  uint8_t node_id;
};

union ImuMagStatusFrame {
  ImuMagStatus value;
  uint8_t bytes[8];
};

static_assert(sizeof(ImuMagStatus) == 8, "ImuMagStatus must be 8 bytes");

// ------------------------------------------------------------
// RPM
// ------------------------------------------------------------
//
// motor_idごとに CAN ID を分けます。
//   0x200: motor 0
//   0x201: motor 1
//   0x202: motor 2
//
// データ部にもmotor_idを入れておくと、ログを見た時に分かりやすいです。
// ------------------------------------------------------------

struct CAN_PACKED Rpm {
  int16_t rpm;
  int16_t current_mA;
  uint8_t node_id;
  uint8_t motor_id;
  uint8_t reserved0;
  uint8_t reserved1;
};

union RpmFrame {
  Rpm value;
  uint8_t bytes[8];
};

static_assert(sizeof(Rpm) == 8, "Rpm must be 8 bytes");

// ------------------------------------------------------------
// モータ指令
// ------------------------------------------------------------
//
// 受信側が拾う想定のフレーム。
//   0x300: motor 0 command
//   0x301: motor 1 command
//
// target_rpm:
//   目標回転数
// limit_mA:
//   電流制限
// mode:
//   0=停止, 1=速度制御, 2=開ループ など、好きに定義
// seq:
//   コマンドの通し番号。通信抜け検出に使える。
// ------------------------------------------------------------

struct CAN_PACKED MotorCommand {
  int16_t target_rpm;
  int16_t limit_mA;
  uint8_t mode;
  uint8_t seq;
  uint8_t reserved0;
  uint8_t reserved1;
};

union MotorCommandFrame {
  MotorCommand value;
  uint8_t bytes[8];
};

static_assert(sizeof(MotorCommand) == 8, "MotorCommand must be 8 bytes");

// ------------------------------------------------------------
// Heartbeat
// ------------------------------------------------------------
//
// 各ノードが1Hz程度で流す生存通知。
// メイン制御基板やロガーはこれを見て、ノードが生きているか確認します。
// ------------------------------------------------------------

struct CAN_PACKED Heartbeat {
  uint8_t node_id;
  uint8_t state;
  uint8_t rx_drop_count_lsb;
  uint8_t can_error_count_lsb;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
};

union HeartbeatFrame {
  Heartbeat value;
  uint8_t bytes[8];
};

static_assert(sizeof(Heartbeat) == 8, "Heartbeat must be 8 bytes");

#undef CAN_PACKED

}  // namespace CanSchema
