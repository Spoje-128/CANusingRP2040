#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include "can2040.h"

// ============================================================
// ユーザー設定
// ============================================================

// can2040で使うPIO番号。
// RP2040なら 0 または 1。
// Pico 2 / RP2350では 0, 1, 2 が使える場合があります。
#define CAN_PIO_NUM        0

// SN65HVD230の R ピン、つまりCAN受信出力をつなぐPico側GPIO。
// トランシーバからPicoへ入ってくる信号なので RX。
#define CAN_RX_GPIO        4

// SN65HVD230の D ピン、つまりCAN送信入力をつなぐPico側GPIO。
// Picoからトランシーバへ出す信号なので TX。
#define CAN_TX_GPIO        5

// CANバス速度。
// 最初は500kbpsがおすすめ。配線や終端が多少雑でも1Mbpsより安定しやすいです。
// 全ノードで必ず同じビットレートにしてください。
#define CAN_BITRATE        500000

// 受信メッセージを一時保存するリングバッファのサイズ。
// 2の累乗にしておくと処理が軽いです。
// IMUなどがバンバン流れるなら 32, 64, 128 あたりに増やします。
#define CAN_RX_QUEUE_SIZE  32

// ============================================================
// CAN ID設計
// ============================================================
//
// Classic CANの標準IDは11bitなので 0x000〜0x7FF。
// CANではIDが小さいほど優先度が高いです。
// 緊急系は小さいID、周期センサは中くらい、デバッグは大きいIDにすると扱いやすいです。
//
// 例:
//   0x010〜0x01F: 緊急停止、重大エラー
//   0x100〜0x1FF: IMU、姿勢、加速度、ジャイロ
//   0x200〜0x2FF: モータ、回転数
//   0x300〜0x3FF: 制御指令
//   0x700〜0x7FF: heartbeat、デバッグ
//

#define CAN_ID_ESTOP            0x010

#define CAN_ID_IMU_ACCEL_GYRO   0x120
#define CAN_ID_IMU_MAG_STATUS   0x121

#define CAN_ID_RPM_BASE         0x200
#define CAN_ID_CMD_MOTOR_BASE   0x300

#define CAN_ID_HEARTBEAT_BASE   0x700

// このノード自身の番号。
// 複数ノードに同じファームを書いて、DIPスイッチや設定値で変えてもいいです。
#define NODE_ID                 1

// ============================================================
// グローバル変数
// ============================================================

// can2040本体。
// can2040は内部でmallocしないので、この構造体をユーザー側で持ちます。
static struct can2040 cbus;

// IRQで受け取ったCANメッセージをメインループへ渡すためのリングバッファ。
// can2040のコールバックはIRQ内で呼ばれるため、そこでprintfや複雑な処理をしない方が安全です。
static struct can2040_msg rx_queue[CAN_RX_QUEUE_SIZE];

static volatile uint32_t rx_head = 0;
static volatile uint32_t rx_tail = 0;
static volatile uint32_t rx_drop_count = 0;
static volatile uint32_t can_error_count = 0;

// ============================================================
// 小物ユーティリティ
// ============================================================

// リングバッファに1件pushします。
// IRQ内から呼ばれるので、処理は極力短くします。
static void rx_queue_push_from_irq(const struct can2040_msg *msg)
{
    uint32_t next_head = (rx_head + 1) & (CAN_RX_QUEUE_SIZE - 1);

    // headを1つ進めるとtailに追いつくなら、キュー満杯。
    // ここでは古いデータを守って、新しいデータを捨てます。
    // センサ用途なら「古いのを捨てて最新を残す」設計もありです。
    if (next_head == rx_tail) {
        rx_drop_count++;
        return;
    }

    // can2040のmsgポインタはコールバック中だけ有効なので、必ずコピーします。
    rx_queue[rx_head] = *msg;
    rx_head = next_head;
}

// メインループ側でリングバッファから1件popします。
// 取れたらtrue、空ならfalse。
static bool rx_queue_pop(struct can2040_msg *out)
{
    if (rx_tail == rx_head) {
        return false;
    }

    // ここで短時間だけIRQを止めると、head/tailの競合を避けやすいです。
    // この程度ならCANの処理に大きな悪影響は出にくいですが、
    // 重い処理をIRQ禁止区間に入れないでください。
    uint32_t irq_state = save_and_disable_interrupts();

    if (rx_tail == rx_head) {
        restore_interrupts(irq_state);
        return false;
    }

    *out = rx_queue[rx_tail];
    rx_tail = (rx_tail + 1) & (CAN_RX_QUEUE_SIZE - 1);

    restore_interrupts(irq_state);
    return true;
}

// int16_tをリトルエンディアンで2バイトに詰めます。
// CANのデータ部はただのバイト列なので、エンディアンを自分たちで決めます。
// Pico同士ならリトルエンディアンで統一するとラクです。
static void put_i16_le(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
}

// リトルエンディアン2バイトからint16_tを取り出します。
static int16_t get_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// uint16_tをリトルエンディアンで2バイトに詰めます。
static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
}

// uint16_tをリトルエンディアンで取り出します。
static uint16_t get_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// CAN送信の薄いラッパ。
// dlcはClassic CANでは最大8バイト。
// idは標準11bit ID想定です。拡張29bit IDは今回は使いません。
static bool can_send_std(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    if (dlc > 8) {
        return false;
    }

    struct can2040_msg msg;
    memset(&msg, 0, sizeof(msg));

    msg.id = id;
    msg.dlc = dlc;

    if (data != NULL && dlc > 0) {
        memcpy(msg.data, data, dlc);
    }

    // can2040_transmitは送信キューに積めたら0を返します。
    // キューが詰まっている場合は負の値になります。
    int ret = can2040_transmit(&cbus, &msg);
    return ret == 0;
}

// ============================================================
// アプリ用 publish 関数
// ============================================================
//
// ここから上のコードは「CAN土台」。
// ここから下が「自分のロボット/装置の通信仕様」です。
// データ種類が増えても publish_xxx() を増やせばよい形にします。
// ============================================================

// 加速度とジャイロを送ります。
// 9軸IMUをClassic CANで送る場合、1フレーム8バイトに全部は入りません。
// ここでは acc_x, acc_y, acc_z, gyro_z の4個だけ例として詰めています。
// 実運用では 0x120: acc xyz + status、0x121: gyro xyz + temp のように分割するのがおすすめ。
static bool publish_imu_accel_gyro(
    int16_t acc_x,
    int16_t acc_y,
    int16_t acc_z,
    int16_t gyro_z
) {
    uint8_t data[8];

    put_i16_le(&data[0], acc_x);
    put_i16_le(&data[2], acc_y);
    put_i16_le(&data[4], acc_z);
    put_i16_le(&data[6], gyro_z);

    return can_send_std(CAN_ID_IMU_ACCEL_GYRO, data, 8);
}

// 磁気センサとステータスを送ります。
// mag_x, mag_y, mag_z が各int16_tで6バイト。
// statusが1バイト、node_idが1バイトで合計8バイト。
static bool publish_imu_mag_status(
    int16_t mag_x,
    int16_t mag_y,
    int16_t mag_z,
    uint8_t status
) {
    uint8_t data[8];

    put_i16_le(&data[0], mag_x);
    put_i16_le(&data[2], mag_y);
    put_i16_le(&data[4], mag_z);
    data[6] = status;
    data[7] = NODE_ID;

    return can_send_std(CAN_ID_IMU_MAG_STATUS, data, 8);
}

// モータ回転数を送ります。
// motor_idをCAN IDに足すことで、モータごとにIDを分けます。
// 例:
//   motor_id=0 -> 0x200
//   motor_id=1 -> 0x201
//   motor_id=2 -> 0x202
//
// データ部:
//   byte0-1: rpm int16_t
//   byte2-3: 電流など。今回は仮にcurrent_mA
//   byte4:   node_id
//   byte5:   motor_id
//   byte6-7: 予約
static bool publish_rpm(uint8_t motor_id, int16_t rpm, int16_t current_mA)
{
    uint8_t data[8];

    put_i16_le(&data[0], rpm);
    put_i16_le(&data[2], current_mA);
    data[4] = NODE_ID;
    data[5] = motor_id;
    data[6] = 0;
    data[7] = 0;

    return can_send_std(CAN_ID_RPM_BASE + motor_id, data, 8);
}

// heartbeatを送ります。
// 各ノードが「生きてるよ」と周期的に流す用途。
// ロガーやメイン制御基板がこれを見て、ノードの生存確認をします。
static bool publish_heartbeat(uint8_t state)
{
    uint8_t data[8];

    data[0] = NODE_ID;
    data[1] = state;
    data[2] = (uint8_t)(rx_drop_count & 0xff);
    data[3] = (uint8_t)(can_error_count & 0xff);
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    return can_send_std(CAN_ID_HEARTBEAT_BASE + NODE_ID, data, 8);
}

// ============================================================
// 受信ハンドラ
// ============================================================

// モータ指令を処理する例。
// 実際にはここでPWM指令や制御目標値を書き換える感じになります。
static void handle_motor_command(uint8_t motor_id, const struct can2040_msg *msg)
{
    if (msg->dlc < 4) {
        return;
    }

    int16_t target_rpm = get_i16_le(&msg->data[0]);
    int16_t limit_mA   = get_i16_le(&msg->data[2]);

    printf("CMD motor=%u target_rpm=%d limit_mA=%d\n",
           motor_id, target_rpm, limit_mA);
}

// 緊急停止を処理する例。
// CANではIDが低いほど優先されるので、緊急系は低IDにしておくとよいです。
static void handle_estop(const struct can2040_msg *msg)
{
    (void)msg;

    // ここでモータ停止、出力遮断、状態遷移などを行います。
    printf("ESTOP received\n");
}

// 受信したCANメッセージをIDで振り分けます。
// 新しいデータ種類が増えたら、ここにcaseや範囲判定を足します。
static void handle_can_message(const struct can2040_msg *msg)
{
    uint32_t id = msg->id;

    // 今回は標準IDのみ扱う想定。
    // 拡張IDフラグが立っていたら無視します。
    if (id & CAN2040_ID_EFF) {
        return;
    }

    if (id == CAN_ID_ESTOP) {
        handle_estop(msg);
        return;
    }

    // モータ指令IDの範囲。
    // 0x300〜0x30F を motor_id 0〜15 として使う例。
    if (id >= CAN_ID_CMD_MOTOR_BASE && id < CAN_ID_CMD_MOTOR_BASE + 16) {
        uint8_t motor_id = (uint8_t)(id - CAN_ID_CMD_MOTOR_BASE);
        handle_motor_command(motor_id, msg);
        return;
    }

    // 自分が必要ないIDは何もしない。
    // CANの基本は「みんな流す、必要なやつだけ拾う」です。
}

// ============================================================
// can2040 コールバックとIRQ
// ============================================================

// can2040から呼ばれるコールバック。
// 注意: これはIRQコンテキストで呼ばれます。
// ここでprintf、I2C読み書き、重い計算、mallocなどはしない方が安全です。
static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    (void)cd;

    if (notify == CAN2040_NOTIFY_RX) {
        // 受信成功。
        // msgはこの関数を抜けると無効になるので、リングバッファへコピーします。
        rx_queue_push_from_irq(msg);
        return;
    }

    if (notify == CAN2040_NOTIFY_TX) {
        // 送信成功。
        // 必要なら送信カウンタを増やす程度に留めます。
        return;
    }

    if (notify == CAN2040_NOTIFY_ERROR) {
        // can2040内部の受信バッファあふれなど。
        // エラー詳細をここで解析せず、カウンタだけ増やします。
        can_error_count++;
        return;
    }
}

// PIO0のIRQハンドラ。
// CAN_PIO_NUMを1にしたら PIO1_IRQ_0 に変える必要があります。
static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

// CANを初期化します。
static void canbus_setup(void)
{
    uint32_t sys_clock = clock_get_hz(clk_sys);

    // can2040の内部構造体を初期化します。
    can2040_setup(&cbus, CAN_PIO_NUM);

    // 受信/送信/エラー通知のコールバックを登録します。
    can2040_callback_config(&cbus, can2040_cb);

    // can2040はPIOのIRQを使います。
    // CAN_PIO_NUM=0なら PIO0_IRQ_0。
    // CAN_PIO_NUM=1なら PIO1_IRQ_0 にしてください。
    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);

    // 優先度は小さい数値ほど高いです。
    // can2040は割り込み遅延に弱いので、そこそこ高めにします。
    irq_set_priority(PIO0_IRQ_0, 1);
    irq_set_enabled(PIO0_IRQ_0, true);

    // CAN開始。
    // gpio_rx: SN65HVD230のRから来る線
    // gpio_tx: SN65HVD230のDへ行く線
    can2040_start(&cbus, sys_clock, CAN_BITRATE, CAN_RX_GPIO, CAN_TX_GPIO);
}

// ============================================================
// ダミーセンサ
// ============================================================
//
// 最小構成で動作確認しやすいように、センサ値は仮データにしています。
// 実機ではここをIMU読み取りやエンコーダ読み取りに置き換えます。
// ============================================================

static int16_t fake_wave_i16(void)
{
    static int16_t v = 0;
    v += 100;
    return v;
}

// ============================================================
// main
// ============================================================

int main(void)
{
    stdio_init_all();
    sleep_ms(1000);

    printf("Pico PIO CAN node start\n");

    canbus_setup();

    // 周期管理用の時刻。
    absolute_time_t next_imu_time = get_absolute_time();
    absolute_time_t next_rpm_time = get_absolute_time();
    absolute_time_t next_heartbeat_time = get_absolute_time();

    while (true) {
        // ----------------------------------------------------
        // 1. 受信処理
        // ----------------------------------------------------
        //
        // IRQで受け取ったCANメッセージを、メインループ側で処理します。
        // これにより、IRQ内の処理を軽く保てます。
        struct can2040_msg msg;
        while (rx_queue_pop(&msg)) {
            handle_can_message(&msg);
        }

        // ----------------------------------------------------
        // 2. 周期送信: IMU
        // ----------------------------------------------------
        //
        // 例として100HzでIMUデータを送ります。
        // 9軸全部をClassic CANで送るなら、複数IDに分けるのが基本です。
        if (absolute_time_diff_us(get_absolute_time(), next_imu_time) <= 0) {
            int16_t a = fake_wave_i16();

            publish_imu_accel_gyro(
                a + 1,   // acc_x 仮値
                a + 2,   // acc_y 仮値
                a + 3,   // acc_z 仮値
                a + 4    // gyro_z 仮値
            );

            publish_imu_mag_status(
                a + 5,   // mag_x 仮値
                a + 6,   // mag_y 仮値
                a + 7,   // mag_z 仮値
                0        // status
            );

            next_imu_time = delayed_by_us(next_imu_time, 10000); // 100Hz
        }

        // ----------------------------------------------------
        // 3. 周期送信: RPM
        // ----------------------------------------------------
        //
        // 例として50Hzでモータ回転数を送ります。
        if (absolute_time_diff_us(get_absolute_time(), next_rpm_time) <= 0) {
            publish_rpm(0, 1234, 250);
            publish_rpm(1, 2345, 300);

            next_rpm_time = delayed_by_us(next_rpm_time, 20000); // 50Hz
        }

        // ----------------------------------------------------
        // 4. 周期送信: heartbeat
        // ----------------------------------------------------
        //
        // 例として1Hzで生存通知。
        if (absolute_time_diff_us(get_absolute_time(), next_heartbeat_time) <= 0) {
            publish_heartbeat(1);

            next_heartbeat_time = delayed_by_us(next_heartbeat_time, 1000000); // 1Hz
        }

        // ----------------------------------------------------
        // 5. 少しだけ休む
        // ----------------------------------------------------
        //
        // 完全にbusy loopでも動きますが、ここではUSB printfなどの都合で少し休ませます。
        // can2040のCAN処理自体はPIO IRQで進みます。
        tight_loop_contents();
    }
}
