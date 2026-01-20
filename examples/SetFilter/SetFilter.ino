#include <ESP32CanFD.h>

ESP32CanFD CANFD;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  CANFD.setPins(4, 5);
  // Classic CANのみの場合: 仲裁フェーズ(500k)
  CANFD.setBaudRate(500000);
  // CAN-FDの場合: 仲裁フェーズ(500k), データフェーズ(2M)
  // CANFD.setBaudRate(500000, 2000000);

  if (!CANFD.begin()) {
    Serial.println("CAN初期化失敗");
    while (1)
      ;
  }

  // --- フィルタ設定の例 (beginの後に設定) ---

  /**
   * 1. マスクフィルタ (setFilter)
   * 特定のID、または特定のビットパターンが一致するものだけを通します。
   * 
   * 例: ID 0x123 のみを完全に一致させて受信したい場合
   * ID: 0x123
   * Mask: 0x7FF (全11ビットをチェックする = 1の意味)
   */
  CANFD.setFilter(0, 0x123, 0x7FF);  // フィルタID 0番を使用
  // CANFD.setFilter(0, 0x123, 0x7FF, false);// Standard ID(11bit)のみ受信したい場合、isExtended=falseに（デフォルトtrue）

  /**
   * 例: 0x100 〜 0x10F の範囲（0x10X）を受信したい場合
   * ID: 0x100
   * Mask: 0x7F0 (下位4ビットを無視する = 0の意味)
   */
  // CANFD.setFilter(1, 0x100, 0x7F0);

  /**
   * 2. デュアルフィルタ (setDualFilter)
   * 1つの設定枠で「2つの特定のID」を個別に指定して受信します。
   * 
   * 例: 0x123 と 0x456 の2つだけを受信したい場合
   */
  // CANFD.setDualFilter(0, 0x123, 0x7FF, 0x456, 0x7FF);

  /**
   * 3. 範囲フィルタ (setRangeFilter) ※CAN FD対応チップ向け
   * 指定したIDの範囲（最小〜最大）をまるごと受信します。
   * 
   * 例: 0x200 から 0x300 までのすべてのIDを受信したい場合
   */
  // CANFD.setRangeFilter(0, 0x200, 0x300);

  // --- フィルタ設定終了 ---

  Serial.println("CAN Filter Sample Started");
}

void loop() {
  // フィルタを通過したパケットのみがここに届く
  int packetSize = CANFD.parsePacket();
  if (packetSize) {
    Serial.print("Received ");
    if (CANFD.packetFdf()) Serial.print("CAN-FD ");
    else Serial.print("Classic ");

    Serial.printf("Packet: ID=0x%X, DLC=%d, Size=%d\n",
                  CANFD.packetId(), CANFD.packetDlc(), packetSize);

    Serial.print("Data: ");
    while (CANFD.available()) {
      uint8_t b = CANFD.read();
      Serial.printf("%02X ", b);
    }
    Serial.println();
  }
}