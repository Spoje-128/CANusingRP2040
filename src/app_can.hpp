#pragma once

#include "can_bus.hpp"

namespace AppCan {

void begin(CanBus* bus);

// 受信したCANフレームを処理する。
// loop()から高頻度で呼ぶ。
void poll();

// 周期送信を処理する。
// loop()から高頻度で呼ぶ。
void tick();

}  // namespace AppCan
