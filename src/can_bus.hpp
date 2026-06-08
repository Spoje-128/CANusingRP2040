#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

extern "C" {
#include "can2040.h"
}

// ============================================================
// CanBus
// ============================================================
//
// can2040をC++/Arduinoから使いやすくするための薄いクラスです。
//
// 役割:
//   - can2040初期化
//   - PIO IRQ設定
//   - IRQ内受信をリングバッファへ退避
//   - メインループからpopできるようにする
//   - 標準ID 11bit の送信を簡単にする
//
// 注意:
//   can2040はClassic CAN、つまりCAN 2.0Bです。
//   CAN FDではありません。
// ============================================================

class CanBus {
public:
  struct Config {
    uint32_t pio_num;   // 0 or 1。Pico 2/RP2350なら2も使える場合あり
    int32_t rx_gpio;    // SN65HVD230のRから来るGPIO
    int32_t tx_gpio;    // SN65HVD230のDへ行くGPIO
    uint32_t bitrate;   // 例: 500000
  };

  struct Frame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
  };

  bool begin(const Config& config);

  // 標準ID 11bitのCANフレームを送る。
  // dataは最大8バイト。
  bool sendStd(uint16_t id, const uint8_t* data, uint8_t dlc);

  // 受信キューから1件取り出す。
  // 取れたらtrue、空ならfalse。
  bool receive(Frame& out);

  uint32_t rxDropCount() const {
    return rx_drop_count_;
  }

  uint32_t errorCount() const {
    return error_count_;
  }

private:
  static constexpr uint32_t RX_QUEUE_SIZE = 32;

  // can2040のCコールバックから、このインスタンスへ戻るためのポインタ。
  // 最小構成ではCANバス1本だけを想定。
  static CanBus* instance_;

  static void onCan2040Callback(struct can2040* cd, uint32_t notify, struct can2040_msg* msg);
  static void pio0IrqHandler();
  static void pio1IrqHandler();

#if defined(PIO2_IRQ_0)
  static void pio2IrqHandler();
#endif

  void handleCallback(uint32_t notify, struct can2040_msg* msg);
  void pushFromIrq(const struct can2040_msg* msg);

private:
  struct can2040 cbus_;

  uint32_t pio_num_ = 0;

  volatile uint32_t rx_head_ = 0;
  volatile uint32_t rx_tail_ = 0;
  volatile uint32_t rx_drop_count_ = 0;
  volatile uint32_t error_count_ = 0;

  Frame rx_queue_[RX_QUEUE_SIZE];
};
