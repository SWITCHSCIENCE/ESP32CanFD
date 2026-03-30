#include <ESP32CanFD.h>

ESP32CanFD CANFD;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  // ピン設定(RX 4 TX 5)
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
   * 1. マスクフィルタ (setFilterMask)
   * 特定のID、または特定のビットパターンが一致するものだけを通します。
   * 
   * 例: ID 0x100 のみを完全に一致させて受信したい場合
   * ID: 0x100
   * Mask: 0x7FF (全11ビットをチェックする = 1の意味)
   */
  CANFD.setFilterMask(0, 0x100, 0x7FF);
  CANFD.setFilterMask(1, 0x1000, 0x7FFF, true); // 拡張IDの場合

  /**
   * 2. 範囲フィルタ (setFilterRange)
   * 指定したIDの範囲（最小〜最大）をまるごと受信します。
   * 
   * 例: 0x3000 から 0x4000 までのすべてのIDを受信したい場合
   */
  CANFD.setFilterRange(0, 0x2000, 0x2FFF, true);

  /**
   * 3. デュアルフィルタ (setFilterDual)
   * 1つの設定枠で「2つの特定のID」を個別に指定して受信します。
   * 
   * 例: 0x2000 と 0x3000 の2つだけを受信したい場合
   */
  // CANFD.setFilterDual(0, 0x3000, 0x7FFF, 0x4000, 0x7FFF, true);

  // --- フィルタ設定終了 ---

  Serial.println("CAN Filter Sample Started");
}

void loop() {
  static unsigned long lastSend = 0;
  static int testIndex = 0;

  // フィルタを通過したパケットのみがここに届く
  if (CANFD.parsePacket()) {
    Serial.print("Received ");
    if (CANFD.packetFdf()) Serial.print("CAN-FD ");
    else Serial.print("Classic ");

    Serial.printf("Packet: ID=0x%X, DLC=%d, Size=%d\n",
                  CANFD.packetId(), CANFD.packetDlc(), CANFD.available());

    Serial.print("Data: ");
    while (CANFD.available()) {
      Serial.printf("%02X ", CANFD.read());
    }
    Serial.println();
  }

  // フィルター設定テスト用の送信 (1秒ごとに異なるIDで送信)
  if (millis() - lastSend > 500) {
    uint32_t testIds[] = { 0x100, 0x250, 0x300, 0x400, 0x500, 0x1000, 0x2500, 0x3000, 0x4000, 0x5000 };  // フィルター対象 + 非対象
    int num_ids = sizeof(testIds) / sizeof(testIds[0]);
    uint32_t sendId = testIds[testIndex % num_ids];

    CANFD.beginPacket(sendId);
    CANFD.write(0xAA);
    CANFD.write(0xBB);
    CANFD.write(0xCC);
    CANFD.write(testIndex % 256);  // テスト番号
    CANFD.endPacket();

    Serial.print("送信 ID: 0x");
    Serial.print(sendId, HEX);
    Serial.print(" (テスト ");
    Serial.print(testIndex % num_ids + 1);
    Serial.print("/");
    Serial.print(num_ids);
    Serial.println(")");

    lastSend = millis();
    testIndex++;
  }

  delay(200);
}