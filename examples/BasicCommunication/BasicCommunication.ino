#include <ESP32CanFD.h>

// インスタンス作成
ESP32CanFD CANFD;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  // 1. GPIOピンの設定 (RX, TX)
  // チップに合わせて適切なピン番号に変更してください
  CANFD.setPins(4, 5);

  // 2. ボーレートの設定
  // Classic CANのみの場合: 仲裁フェーズ(500k)
  CANFD.setBaudRate(500000);
  // CAN-FDの場合: 仲裁フェーズ(500k), データフェーズ(2M)
  // CANFD.setBaudRate(500000, 2000000);

  // 3. ドライバ開始
  if (!CANFD.begin()) {
    Serial.println("Failed to initialize CAN FD!");
    while (1);
  }

  Serial.println("CAN FD Initialized.");
}

void loop() {
  // --- 送信処理 ---
  static uint32_t lastSend = 0;
  if (millis() - lastSend > 1000) {
    lastSend = millis();

    // ID 0x123 でパケット開始 (デフォルトで29bit Extended ID)
    // .standard() を呼べば 11bit ID になります
    CANFD.beginPacket(0x12345)
         .fd(true)  // CAN-FD形式、BRS(速度切替)有効
         .write(0xAA);
    
    const char* msg = "ESP32-FD";
    CANFD.write((uint8_t*)msg, strlen(msg));

    if (CANFD.endPacket()) {
      Serial.println("Packet sent!");
    } else {
      Serial.println("Send failed.");
    }
  }

  // --- 受信処理 ---
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
    Serial.println("\n---");
  }

  // バスオフ時のリカバリ処理例
  if (CANFD.isBusOff()) {
    Serial.println("Bus-Off detected! Recovering...");
    CANFD.recover();
  }
}
