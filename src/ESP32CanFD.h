#ifndef ESP32_CAN_FD_H
#define ESP32_CAN_FD_H

#include <Arduino.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"

/* 送信フォーマット指定用内部定数 */
#define CAN_FMT_AUTO -1    // データ長で自動切替
#define CAN_FMT_CLASSIC 0  // Classic CAN強制
#define CAN_FMT_FD 1       // CAN FD強制

class ESP32CanFD : public Stream {
public:
  ESP32CanFD();
  ~ESP32CanFD();

  // 通信ピンの設定、begin()の前に呼び出すこと
  void setPins(int rxPin, int txPin);

  /**
   * ボーレートとサンプルポイントの設定、begin()の前に呼び出すこと
   * @param nomBaud   仲裁フェーズの速度 (Hz)
   * @param dataBaud  データフェーズの速度 (Hz) 0指定でClassic CANモード
   * @param nomSP     仲裁フェーズのサンプルポイント (1/1000単位。0でドライバデフォルト)
   * @param dataSP    データフェーズのサンプルポイント (1/1000単位。0でドライバデフォルト)
   * @param nomSSP    仲裁フェーズのセカンダリサンプルポイント (1/1000単位。0でドライバデフォルト)
   * @param dataSSP   データフェーズのセカンダリサンプルポイント (1/1000単位。0でドライバデフォルト)
   */
  void setBaudRate(uint32_t nomBaud, uint32_t dataBaud = 0, uint16_t nomSP = 0, uint16_t dataSP = 0, uint16_t nomSSP = 0, uint16_t dataSSP = 0);

  /**
   * ハードウェア動作フラグの設定 (begin前に設定)
   * @param selfTest   ACKを待たずに送信完了とする (単体テスト用)
   * @param listenOnly 送信(ACK含む)を行わない
   * @param loopback   自分が送信した内容を受信する
   */
  void setModeFlags(bool selfTest, bool listenOnly = false, bool loopback = false);

  // ドライバの初期化と開始
  bool begin();
  // ドライバの停止とリソース解放
  void end();
  // バスオフ(Bus-Off)状態からの復帰試行
  bool recover();

  /* ------------------- 送信関連 (メソッドチェーン対応) ------------------- */

  /**
   * 送信パケットの構築開始
   * デフォルト設定: Extended(29bit), FD有効ならデータ長に応じAUTO切替, BRS有効
   * @param id CAN ID
   * @return ESP32CanFD& 自身を返すことでメソッドチェーンを可能にする
   */
  ESP32CanFD& beginPacket(uint32_t id);

  // 送信パケットの属性変更メソッド (beginPacketの後に繋げて使用)
  ESP32CanFD& standard();           // 11bit IDを強制 (IDが0x7FF以下の場合のみ有効)
  ESP32CanFD& extended();           // 29bit IDを強制 (デフォルト)
  ESP32CanFD& fd(bool brs = true);  // FDフォーマットを強制。BRSの有無を指定可能
  ESP32CanFD& classic();            // Classicフォーマットを強制

  virtual size_t write(uint8_t byte) override;
  virtual size_t write(const uint8_t* buffer, size_t size) override;
  using Stream::write;
  int endPacket();

  /**
   * リモートフレーム(RTR)の送信
   * CAN FD仕様により、RTRは常にClassic CANフォーマットで送信されます。
   * @param id       要求するメッセージのID
   * @param dlc      要求するデータの長さ (0〜8)
   * @param extended true(デフォルト): 29bit ID, false: 11bit ID (ID > 0x7FFの場合は強制的にtrue)
   * @return 1(成功), 0(失敗)
   */
  int request(uint32_t id, uint8_t dlc = 0, bool extended = true);

  /* ------------------- 受信関連 (Stream実装) ------------------- */

  int parsePacket();  // 受信パケットの確認。あればデータ長を返す
  virtual int read() override;
  virtual int available() override;
  virtual int peek() override;

  /* パケット情報取得 (parsePacket成功後に有効) */
  uint32_t packetId();
  bool packetExtended();
  bool packetRtr();
  bool packetFdf();
  bool packetBrs();
  uint8_t packetDlc();

  /* ------------------- フィルタ・ステータス関連 ------------------- */

  /**
   * マスクフィルタの設定
   * @param filterId 0〜(SOC_TWAI_MASK_FILTER_NUM-1)
   */
  bool setFilter(uint8_t filterId, uint32_t id, uint32_t mask, bool isExtended = true);

  /**
   * 2つのIDを個別に通すフィルタの設定 (Dual Filterモード)
   */
  bool setDualFilter(uint8_t filterId, uint32_t id1, uint32_t mask1, uint32_t id2, uint32_t mask2, bool isExtended = true);

  /**
   * 指定したID範囲を通すフィルタの設定 (Range Filter)
   * ※ハードウェアが対応している場合のみ。SOC_TWAI_SUPPORT_FDチップ等。
   */
  bool setRangeFilter(uint8_t filterId, uint32_t lowId, uint32_t highId, bool isExtended = true);

  /* ノード状態の取得 */
  bool isBusOff() {
    return _last_state == TWAI_ERROR_BUS_OFF;
  }
  twai_error_state_t state() {
    return _last_state;
  }
  uint32_t getBusErrorCount() {
    return _bus_error_count;
  }
  uint32_t getRxDropCount() {
    return _rx_drop_count;
  }

  // エラーカウンタの取得 (TEC:送信エラー, REC:受信エラー)
  uint16_t getTEC();
  uint16_t getREC();

private:
  twai_node_handle_t _node_handle;
  twai_onchip_node_config_t _config;
  bool _is_fd_enabled;
  volatile twai_error_state_t _last_state = TWAI_ERROR_ACTIVE;
  volatile uint32_t _bus_error_count = 0;
  volatile uint32_t _rx_drop_count = 0;

  // 排他制御用スピンロック (ISRとTask間の変数保護)
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

  // 受信バッファと一時情報
  static const size_t RX_RING_SIZE = 1024;
  uint8_t _rxRingBuffer[RX_RING_SIZE];
  size_t _rx_head = 0;
  uint8_t *_rxDataPtr;

  // 送信バッファと一時情報
  static const size_t TX_RING_SIZE = 1024;
  uint8_t _txRingBuffer[TX_RING_SIZE];
  size_t _txBufferIdx;
  size_t _head = 0;
  volatile size_t _used_count = 0;

  QueueHandle_t _tx_len_queue;
  SemaphoreHandle_t _tx_buf_sem;

  struct {
    uint32_t id;
    int format;
    bool brs;
    bool extended;
  } _txPacketInfo;

  // 受信バッファと情報
  QueueHandle_t _rx_queue;
  uint8_t _rxBuffer[64];
  size_t _rxBufferIdx;
  size_t _rxLength;
  twai_frame_header_t _rxHeader;

  // ISRから呼び出されるフレンド関数
  friend bool IRAM_ATTR app_twai_tx_done_callback(twai_node_handle_t handle, const twai_tx_done_event_data_t* edata, void* user_ctx);
  friend bool IRAM_ATTR app_twai_rx_done_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t* edata, void* user_ctx);
  friend bool IRAM_ATTR app_twai_state_change_callback(twai_node_handle_t handle, const twai_state_change_event_data_t* edata, void* user_ctx);
  friend bool IRAM_ATTR app_twai_error_callback(twai_node_handle_t handle, const twai_error_event_data_t* edata, void* user_ctx);
};

#endif