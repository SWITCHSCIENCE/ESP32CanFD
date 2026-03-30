# ESP32CanFD

ESP32CanFDは、Arduino環境でESP32のTWAI/CANおよびCAN-FD機能を使いやすくするためのラッパーライブラリです。内蔵のESP-IDF TWAI Handle APIをベースに、メソッドチェーンと`Stream`互換インターフェースを提供します。

## 主要機能

- TWAI / CAN / CAN-FD送受信
- `Print`/`Stream`互換: `write`, `read`, `available`, `peek`
- 送信メソッドチェーン: `beginPacket().standard()/extended().fd()/classic().write().endPacket()`
- フィルタ: マスク、デュアル、レンジ（ハードウェア依存）
- セルフテスト/リッスンオンリー/ループバックモード
- バス状態とエラーカウント取得: Bus-Off判定、TEC/REC、送信/受信ドロップ

## API概要

### 初期化/終了
- `void setPins(int rxPin, int txPin)`
- `void setBaudRate(uint32_t nomBaud, uint32_t dataBaud = 0, uint16_t nomSP = 0, uint16_t dataSP = 0, uint16_t nomSSP = 0, uint16_t dataSSP = 0)`
  - `dataBaud`が0ならClassic CAN、非0ならCAN-FD（`_is_fd_enabled`有効）
- `void setModeFlags(bool selfTest, bool listenOnly = false, bool loopback = false)`
- `bool begin()` : TWAIノード生成・コールバック登録・有効化
- `void end()` : ノード削除、リソース解放
- `bool recover()` : Bus-Off状態からの復帰試行

### 送信（メソッドチェーン）
- `ESP32CanFD& beginPacket(uint32_t id)`
  - 既定では、`id > 0x7FF`で拡張ID、ID<=0x7FFで標準ID
  - `dataBaud`設定時はFD、BRS有効
- `ESP32CanFD& standard()`
- `ESP32CanFD& extended()`
- `ESP32CanFD& fd(bool brs = true)`
- `ESP32CanFD& classic()`
- `size_t write(uint8_t byte)` / `size_t write(const uint8_t* buffer, size_t size)`
- `int endPacket()` : 1=送信要求成功, 0=失敗

### 受信（Stream準拠）
- `bool parsePacket()` : 受信済みフレームを取り出し、`packet*`/`available`参照に準備
- `int available()`
- `int read()`
- `int peek()`

### 受信パケット情報
- `uint32_t packetId()`
- `bool packetExtended()`
- `bool packetRtr()`
- `bool packetFdf()`
- `bool packetBrs()`
- `uint8_t packetDlc()`

### フィルタ管理
- `bool setFilterMask(uint8_t filterId, uint32_t id, uint32_t mask, bool isExtended = false)`
- `bool setFilterDual(uint8_t filterId, uint32_t id1, uint32_t mask1, uint32_t id2, uint32_t mask2, bool isExtended = false)`
  - ESP32-C5は現状未対応（Dualモード非対応）
- `bool setFilterRange(uint8_t filterId, uint32_t lowId, uint32_t highId, bool isExtended = false)`

### 状態確認
- `bool isBusOff()`
- `twai_error_state_t state()`
- `uint32_t getBusErrorCount()`
- `uint32_t getRxDropCount()`
- `uint16_t getTEC()`
- `uint16_t getREC()`

## チップ依存のフィルタ上限

- ESP32C5 / ESP32H4: `SOC_TWAI_MASK_FILTER_NUM=3`, `SOC_TWAI_RANGE_FILTER_NUM=1`
- その他: `SOC_TWAI_MASK_FILTER_NUM=1`, `SOC_TWAI_RANGE_FILTER_NUM=0`

## 使用例

```cpp
#include <ESP32CanFD.h>

ESP32CanFD CANFD;

void setup() {
  Serial.begin(115200);
  CANFD.setPins(4, 5);
  CANFD.setBaudRate(500000, 2000000); // CAN-FD

  if (!CANFD.begin()) {
    Serial.println("CAN初期化失敗");
    while (1) {
      delay(1000);
    }
  }

  // 例: マスクフィルタ設定（filterId=0）
  CANFD.setFilterMask(0, 0x100, 0x700, false);
}

void loop() {
  // 送信
  CANFD.beginPacket(0x123)
       .fd(true)
       .write((const uint8_t*)"Hello", 5);
  if (!CANFD.endPacket()) {
    Serial.println("送信失敗");
  }

  // 受信
  if (CANFD.parsePacket()) {
    Serial.printf("ID=0x%X ext=%d fd=%d brs=%d dlc=%d\n",
                  CANFD.packetId(), CANFD.packetExtended(), CANFD.packetFdf(), CANFD.packetBrs(), CANFD.packetDlc());
    while (CANFD.available()) {
      Serial.write(CANFD.read());
    }
    Serial.println();
  }

  if (CANFD.isBusOff()) {
    Serial.println("Bus-Off状態検出。recover()を試行します。");
    CANFD.recover();
  }
  delay(500);
}
```

## 注意事項

- 受信IRQからリングバッファへ書き込みし、アプリで`parsePacket()`呼び出しが間に合わないと受信ドロップが発生します。
- TX/RX内蔵リングバッファサイズはそれぞれ1024バイトです。
- CAN-FDを使う場合、`setBaudRate()`の`dataBaud`を必ず指定してください。

