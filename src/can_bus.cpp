#include "can_bus.hpp"

#include <string.h>

#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/platform.h"

CanBus* CanBus::instance_ = nullptr;

bool CanBus::begin(const Config& config) {
  if (instance_ != nullptr) {
    // 最小実装ではCANインスタンス1個だけ対応。
    // 複数CANバスを使いたい場合は、PIOごとにインスタンス管理を分ける。
    return false;
  }

  instance_ = this;
  pio_num_ = config.pio_num;

  const uint32_t sys_clock = clock_get_hz(clk_sys);

  can2040_setup(&cbus_, config.pio_num);
  can2040_callback_config(&cbus_, CanBus::onCan2040Callback);

  // can2040はPIOブロックごとのIRQを使います。
  // PIO0ならPIO0_IRQ_0、PIO1ならPIO1_IRQ_0。
  //
  // IRQ優先度は小さい数値ほど高いです。
  // can2040は割り込み遅延に弱いので、やや高めにしておきます。
  if (config.pio_num == 0) {
    irq_set_exclusive_handler(PIO0_IRQ_0, CanBus::pio0IrqHandler);
    irq_set_priority(PIO0_IRQ_0, 1);
    irq_set_enabled(PIO0_IRQ_0, true);
  } else if (config.pio_num == 1) {
    irq_set_exclusive_handler(PIO1_IRQ_0, CanBus::pio1IrqHandler);
    irq_set_priority(PIO1_IRQ_0, 1);
    irq_set_enabled(PIO1_IRQ_0, true);
  }
#if defined(PIO2_IRQ_0)
  else if (config.pio_num == 2) {
    irq_set_exclusive_handler(PIO2_IRQ_0, CanBus::pio2IrqHandler);
    irq_set_priority(PIO2_IRQ_0, 1);
    irq_set_enabled(PIO2_IRQ_0, true);
  }
#endif
  else {
    instance_ = nullptr;
    return false;
  }

  // CAN開始。
  // rx_gpio: トランシーバのR
  // tx_gpio: トランシーバのD
  can2040_start(&cbus_, sys_clock, config.bitrate, config.rx_gpio, config.tx_gpio);

  return true;
}

bool CanBus::sendStd(uint16_t id, const uint8_t* data, uint8_t dlc) {
  if (id > 0x7FF) {
    return false;
  }

  if (dlc > 8) {
    return false;
  }

  struct can2040_msg msg;
  memset(&msg, 0, sizeof(msg));

  msg.id = id;
  msg.dlc = dlc;

  if (data != nullptr && dlc > 0) {
    memcpy(msg.data, data, dlc);
  }

  // can2040_transmit()は、内部送信キューに積めたら0を返す。
  // 内部キューは最大4件なので、詰まる可能性はあります。
  return can2040_transmit(&cbus_, &msg) == 0;
}

bool CanBus::receive(Frame& out) {
  if (rx_tail_ == rx_head_) {
    return false;
  }

  // tail/headはIRQ側からも触られるので、短時間だけ割り込み禁止にする。
  // ここに重い処理を入れてはいけません。
  const uint32_t irq_state = save_and_disable_interrupts();

  if (rx_tail_ == rx_head_) {
    restore_interrupts(irq_state);
    return false;
  }

  out = rx_queue_[rx_tail_];
  rx_tail_ = (rx_tail_ + 1) & (RX_QUEUE_SIZE - 1);

  restore_interrupts(irq_state);
  return true;
}

void CanBus::onCan2040Callback(struct can2040* cd, uint32_t notify, struct can2040_msg* msg) {
  (void)cd;

  if (instance_ != nullptr) {
    instance_->handleCallback(notify, msg);
  }
}

void __not_in_flash_func(CanBus::pio0IrqHandler)() {
  if (instance_ != nullptr) {
    can2040_pio_irq_handler(&instance_->cbus_);
  }
}

void __not_in_flash_func(CanBus::pio1IrqHandler)() {
  if (instance_ != nullptr) {
    can2040_pio_irq_handler(&instance_->cbus_);
  }
}

#if defined(PIO2_IRQ_0)
void __not_in_flash_func(CanBus::pio2IrqHandler)() {
  if (instance_ != nullptr) {
    can2040_pio_irq_handler(&instance_->cbus_);
  }
}
#endif

void __not_in_flash_func(CanBus::handleCallback)(uint32_t notify, struct can2040_msg* msg) {
  if (notify == CAN2040_NOTIFY_RX) {
    // 受信成功。
    // msgポインタはこのコールバック中だけ有効なので、必ずコピーする。
    pushFromIrq(msg);
    return;
  }

  if (notify == CAN2040_NOTIFY_TX) {
    // 送信完了通知。
    // 最小構成では何もしない。
    return;
  }

  if (notify == CAN2040_NOTIFY_ERROR) {
    // can2040内部の受信バッファあふれなど。
    // IRQ内ではカウンタだけ増やす。
    error_count_++;
    return;
  }
}

void __not_in_flash_func(CanBus::pushFromIrq)(const struct can2040_msg* msg) {
  const uint32_t next_head = (rx_head_ + 1) & (RX_QUEUE_SIZE - 1);

  if (next_head == rx_tail_) {
    // キュー満杯。
    // ここでは新しいフレームを捨てる。
    // センサ用途で「最新値だけ欲しい」なら古いものを捨てる方針もあり。
    rx_drop_count_++;
    return;
  }

  Frame f;
  f.id = msg->id;
  f.dlc = msg->dlc > 8 ? 8 : msg->dlc;
  memcpy(f.data, msg->data, 8);

  rx_queue_[rx_head_] = f;
  rx_head_ = next_head;
}
