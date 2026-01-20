# ESP32CanFD

ESP32のTWAI（Two-Wire Automotive Interface / CAN）ドライバをArduino環境で使いやすくするためのライブラリです。ESP-IDFの最新のHandleベースAPI（v5.x以降のスタイル）をラップしており、CAN-FDにも対応しています。

## 特徴
- **CAN-FD対応**: ESP32-C5チップで高速通信（BRS）を利用可能。
- **メソッドチェーン**: `CANFD.beginPacket(0x123).extended().fd().write(data);` のような直感的な記述が可能。
- **Stream継承**: `Print`や`read()`など、Arduino標準のI/O操作に対応。
- **フィルタ設定**: マスクフィルタ、デュアルフィルタ、範囲フィルタ（チップ依存）をサポート。

## 使い方

```cpp
#include <ESP32CanFD.h>

ESP32CanFD CANFD;

void setup() {
  Serial.begin(115200);
  
  // ピン設定とボーレート設定 (仲裁500k, データ2M)
  CANFD.setPins(4, 5); 
  CANFD.setBaudRate(500000, 2000000);
  
  if (!CANFD.begin()) {
    Serial.println("CAN初期化失敗");
    while(1);
  }
}

void loop() {
  // 送信例
  CANFD.beginPacket(0x123456)
       .fd()      // CAN-FD強制
       .write("HELLO!!");
  CANFD.endPacket();

  // 受信例
  int packetSize = CANFD.parsePacket();
  if (packetSize) {
    Serial.printf("ID: 0x%08X, Len: %d\n", CANFD.packetId(), packetSize);
    while (CANFD.available()) {
      Serial.write(CANFD.read());
    }
    Serial.println();
  }
  delay(1000);
}
```
