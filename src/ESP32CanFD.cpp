#include "ESP32CanFD.h"
#include <esp_log.h>

static const char* TAG = "ESP32CanFD";
#define RX_QUEUE_SIZE 32
#define TX_QUEUE_SIZE 32
#define MAX_DATA_SIZE 64

// ISRからメインメモリへデータを渡すための構造体
typedef struct {
  twai_frame_header_t header;
  uint8_t data[MAX_DATA_SIZE];  // 固定長で持ってるので無駄...キューに入れる時もコピーしちゃう
} rx_queue_item_t;

// --- ISRコールバック群 ---

bool IRAM_ATTR app_twai_tx_done_callback(twai_node_handle_t handle, const twai_tx_done_event_data_t* edata, void* user_ctx) {
  ESP32CanFD* self = (ESP32CanFD*)user_ctx;
  if (self && self->_tx_len_queue) {
    size_t footprint;
    // 送信完了したフレームの占有サイズを取得
    if (xQueueReceiveFromISR(self->_tx_len_queue, &footprint, NULL) == pdTRUE) {
      // クリティカルセクションで保護して空き容量を戻す
      portENTER_CRITICAL_ISR(&self->_mux);
      self->_used_count -= footprint;
      portEXIT_CRITICAL_ISR(&self->_mux);

      BaseType_t high_task_awoken = pdFALSE;
      xSemaphoreGiveFromISR(self->_tx_buf_sem, &high_task_awoken);
      return (high_task_awoken == pdTRUE);
    }
  }
  return false;
}

bool IRAM_ATTR app_twai_rx_done_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t* edata, void* user_ctx) {
  ESP32CanFD* self = (ESP32CanFD*)user_ctx;
  if (!self || !self->_rx_queue) return false;

  rx_queue_item_t item;
  twai_frame_t rx_frame;
  rx_frame.buffer = item.data;
  rx_frame.buffer_len = sizeof(item.data);

  if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK) {
    item.header = rx_frame.header;
    BaseType_t high_task_awoken = pdFALSE;
    xQueueSendFromISR(self->_rx_queue, &item, &high_task_awoken);
    return (high_task_awoken == pdTRUE);
  } else {
    self->_rx_drop_count++; // 溢れた！
  }
  return false;
}

bool IRAM_ATTR app_twai_state_change_callback(twai_node_handle_t handle, const twai_state_change_event_data_t* edata, void* user_ctx) {
  ESP32CanFD* self = (ESP32CanFD*)user_ctx;
  if (self) self->_last_state = edata->new_sta;
  return false;
}

bool IRAM_ATTR app_twai_error_callback(twai_node_handle_t handle, const twai_error_event_data_t* edata, void* user_ctx) {
  ESP32CanFD* self = (ESP32CanFD*)user_ctx;
  if (self) self->_bus_error_count++;
  return false;
}

// --- クラスメソッド実装 ---

ESP32CanFD::ESP32CanFD()
  : _node_handle(NULL), _rx_queue(NULL), _is_fd_enabled(false) {
  memset(&_config, 0, sizeof(_config));
  _config.io_cfg.tx = (gpio_num_t)-1;
  _config.io_cfg.rx = (gpio_num_t)-1;
  _config.io_cfg.quanta_clk_out = (gpio_num_t)-1;
  _config.io_cfg.bus_off_indicator = (gpio_num_t)-1;
  _config.tx_queue_depth = TX_QUEUE_SIZE;
  _config.intr_priority = 1;
}

ESP32CanFD::~ESP32CanFD() {
  end();
}

void ESP32CanFD::setPins(int rxPin, int txPin) {
  _config.io_cfg.rx = (gpio_num_t)rxPin;
  _config.io_cfg.tx = (gpio_num_t)txPin;
}

void ESP32CanFD::setBaudRate(uint32_t nomBaud, uint32_t dataBaud, uint16_t nomSP, uint16_t dataSP, uint16_t nomSSP, uint16_t dataSSP) {
  _config.bit_timing.bitrate = nomBaud;
  _config.bit_timing.sp_permill = nomSP;
  _config.bit_timing.ssp_permill = nomSSP;
  if (dataBaud > 0) {
    _config.data_timing.bitrate = dataBaud;
    _config.data_timing.sp_permill = dataSP;
    _config.data_timing.ssp_permill = dataSSP;
    _is_fd_enabled = true;
  } else {
    _is_fd_enabled = false;
  }
}

void ESP32CanFD::setModeFlags(bool selfTest, bool listenOnly, bool loopback) {
  _config.flags.enable_self_test = selfTest;
  _config.flags.enable_listen_only = listenOnly;
  _config.flags.enable_loopback = loopback;
}

bool ESP32CanFD::begin() {
  if (_node_handle != NULL) return true;

  _head = 0;
  _used_count = 0;
  _tx_len_queue = xQueueCreate(TX_QUEUE_SIZE, sizeof(size_t));
  _rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(rx_queue_item_t));
  _tx_buf_sem = xSemaphoreCreateBinary();

  if (!_tx_len_queue || !_rx_queue || !_tx_buf_sem) return false;

  // 初期状態としてセマフォを1つ与えておく
  xSemaphoreGive(_tx_buf_sem);

  esp_err_t res = twai_new_node_onchip(&_config, &_node_handle);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create TWAI node: %s", esp_err_to_name(res));
    return false;
  }

  twai_event_callbacks_t cbs = {
    .on_tx_done = app_twai_tx_done_callback,
    .on_rx_done = app_twai_rx_done_callback,
    .on_state_change = app_twai_state_change_callback,
    .on_error = app_twai_error_callback,
  };
  twai_node_register_event_callbacks(_node_handle, &cbs, this);

  return (twai_node_enable(_node_handle) == ESP_OK);
}

void ESP32CanFD::end() {
  if (_node_handle) {
    twai_node_disable(_node_handle);
    twai_node_delete(_node_handle);
    _node_handle = NULL;
  }
  if (_tx_len_queue) {
    vQueueDelete(_tx_len_queue);
    _tx_len_queue = NULL;
  }
  if (_tx_buf_sem) {
    vQueueDelete(_tx_buf_sem);
    _tx_buf_sem = NULL;
  }
  if (_rx_queue) {
    vQueueDelete(_rx_queue);
    _rx_queue = NULL;
  }
}

bool ESP32CanFD::recover() {
  if (!_node_handle) return false;
  return (twai_node_recover(_node_handle) == ESP_OK);
}

// --- フィルタ設定 ---

bool ESP32CanFD::setFilter(uint8_t filterId, uint32_t id, uint32_t mask, bool isExtended) {
  if (!_node_handle) return false;
  twai_mask_filter_config_t cfg = { .id = id, .mask = mask, .is_ext = isExtended };
  return (twai_node_config_mask_filter(_node_handle, filterId, &cfg) == ESP_OK);
}

bool ESP32CanFD::setDualFilter(uint8_t filterId, uint32_t id1, uint32_t mask1, uint32_t id2, uint32_t mask2, bool isExtended) {
  if (!_node_handle) return false;
  twai_mask_filter_config_t cfg = twai_make_dual_filter(id1, mask1, id2, mask2, isExtended);
  return (twai_node_config_mask_filter(_node_handle, filterId, &cfg) == ESP_OK);
}

bool ESP32CanFD::setRangeFilter(uint8_t filterId, uint32_t lowId, uint32_t highId, bool isExtended) {
  if (!_node_handle) return false;
  twai_range_filter_config_t cfg = { .range_low = lowId, .range_high = highId, .is_ext = isExtended };
  return (twai_node_config_range_filter(_node_handle, filterId, &cfg) == ESP_OK);
}

// --- 送信処理 ---

ESP32CanFD& ESP32CanFD::beginPacket(uint32_t id) {
  // バッファに空きが出るまで待機
  while (true) {
    portENTER_CRITICAL(&_mux);
    size_t current_used = _used_count;
    portEXIT_CRITICAL(&_mux);

    if (current_used + MAX_DATA_SIZE <= TX_RING_SIZE) break;
    xSemaphoreTake(_tx_buf_sem, pdMS_TO_TICKS(50));
  }

  _txPacketInfo.id = id;
  _txPacketInfo.format = CAN_FMT_AUTO;
  _txPacketInfo.brs = _is_fd_enabled;  // FD有効時はデフォルトで高速化(BRS)を許可
  _txPacketInfo.extended = true;       // 現代的な用途に合わせExtended(29bit)をデフォルト化
  _txBufferIdx = 0;
  return *this;
}

ESP32CanFD& ESP32CanFD::standard() {
  if (_txPacketInfo.id <= 0x7FF) _txPacketInfo.extended = false;
  return *this;
}
ESP32CanFD& ESP32CanFD::extended() {
  _txPacketInfo.extended = true;
  return *this;
}
ESP32CanFD& ESP32CanFD::fd(bool brs) {
  _txPacketInfo.format = CAN_FMT_FD;
  _txPacketInfo.brs = brs;
  return *this;
}
ESP32CanFD& ESP32CanFD::classic() {
  _txPacketInfo.format = CAN_FMT_CLASSIC;
  _txPacketInfo.brs = false;
  return *this;
}

size_t ESP32CanFD::write(uint8_t byte) {
  if (_txBufferIdx < MAX_DATA_SIZE) {
    _txRingBuffer[_head + _txBufferIdx++] = byte;
    return 1;
  }
  return 0;
}

size_t ESP32CanFD::write(const uint8_t* buffer, size_t size) {
  size_t to_write = min(size, (size_t)(MAX_DATA_SIZE - _txBufferIdx));
  memcpy(&_txRingBuffer[_head + _txBufferIdx], buffer, to_write);
  _txBufferIdx += to_write;
  return to_write;
}

int ESP32CanFD::endPacket() {
  if (!_node_handle) return 0;

  twai_frame_t tx_msg = {};
  tx_msg.header.id = _txPacketInfo.id;
  // IDが11bit上限を超えている場合は、強制的にExtended
  tx_msg.header.ide = (_txPacketInfo.id > 0x7FF) ? true : _txPacketInfo.extended;
  tx_msg.buffer = &_txRingBuffer[_head];

  bool use_fd = false;
  if (_is_fd_enabled) {
    // 条件: フォーマット指定がFD、またはAUTOかつ8バイト超過
    if (_txPacketInfo.format == CAN_FMT_FD || (_txPacketInfo.format == CAN_FMT_AUTO && _txBufferIdx > 8)) {
      use_fd = true;
    }
  }

  if (use_fd) {
    tx_msg.header.fdf = 1;
    tx_msg.header.brs = _txPacketInfo.brs;
    // データ長からDLC定数(0~15)を算出
    tx_msg.header.dlc = twaifd_len2dlc(_txBufferIdx);
    // buffer_lenはDLCが表す物理的な長さ(例:13バイトなら16)と一致させる
    tx_msg.buffer_len = twaifd_dlc2len(tx_msg.header.dlc);

    // フレーム長を合わせるためのゼロパディング
    if (tx_msg.buffer_len > _txBufferIdx) {
      memset(&_txRingBuffer[_head + _txBufferIdx], 0, tx_msg.buffer_len - _txBufferIdx);
    }
  } else {
    // Classic CAN: 8バイトが上限
    tx_msg.header.dlc = min((size_t)_txBufferIdx, (size_t)8);
    tx_msg.buffer_len = tx_msg.header.dlc;
  }

  // --- 送信と管理情報の更新を一連のロックで行う ---
  int result = 0;
  portENTER_CRITICAL(&_mux);
  // timeout=0なのでクリティカルセクション内でも呼び出し可能
  if (twai_node_transmit(_node_handle, &tx_msg, 0) == ESP_OK) {
    // 次のパケット開始位置を4バイト境界に揃えるため、占有サイズを切り上げる
    size_t footprint = (tx_msg.buffer_len + 3) & ~3;
    size_t next_head = _head + footprint;
    // ラップアラウンド判定: 次の最大フレーム(64)が入る余地がない場合
    if (TX_RING_SIZE - next_head < 64) {
      // 末尾の隙間を今回の占有分に加算
      footprint += (TX_RING_SIZE - next_head);
      next_head = 0;
    }
    xQueueSendFromISR(_tx_len_queue, &footprint, NULL);
    _used_count += footprint;
    _head = next_head;
    result = 1;
  }
  portEXIT_CRITICAL(&_mux);

  return result;
}

int ESP32CanFD::request(uint32_t id, uint8_t dlc, bool extended) {
  if (!_node_handle) return 0;

  twai_frame_t tx_msg = {};
  tx_msg.header.id = id;
  tx_msg.header.ide = (id > 0x7FF) ? true : extended;
  tx_msg.header.rtr = 1;
  tx_msg.header.dlc = (dlc > 8) ? 8 : dlc;

  // RTRはデータを伴わないため、リングバッファの空き待ち（beginPacket）を介さず
  // 直接クリティカルセクションに入って送信できる。
  int result = 0;
  portENTER_CRITICAL(&_mux);
  if (twai_node_transmit(_node_handle, &tx_msg, 0) == ESP_OK) {
    size_t footprint = 0;
    xQueueSendFromISR(_tx_len_queue, &footprint, NULL);
    result = 1;
  }
  portEXIT_CRITICAL(&_mux);

  return result;
}

// --- 受信・状態取得 ---

int ESP32CanFD::parsePacket() {
  if (!_rx_queue) return 0;
  rx_queue_item_t item;
  if (xQueueReceive(_rx_queue, &item, 0) == pdTRUE) {
    _rxHeader = item.header;
    _rxLength = _rxHeader.fdf ? twaifd_dlc2len(_rxHeader.dlc) : min((int)_rxHeader.dlc, 8);
    memcpy(_rxBuffer, item.data, _rxLength);
    _rxBufferIdx = 0;
    return _rxLength;
  }
  return 0;
}

int ESP32CanFD::available() {
  return _rxLength - _rxBufferIdx;
}
int ESP32CanFD::read() {
  return (_rxBufferIdx < _rxLength) ? _rxBuffer[_rxBufferIdx++] : -1;
}
int ESP32CanFD::peek() {
  return (_rxBufferIdx < _rxLength) ? _rxBuffer[_rxBufferIdx] : -1;
}

uint32_t ESP32CanFD::packetId() {
  return _rxHeader.id;
}
bool ESP32CanFD::packetExtended() {
  return _rxHeader.ide;
}
bool ESP32CanFD::packetRtr() {
  return _rxHeader.rtr;
}
bool ESP32CanFD::packetFdf() {
  return _rxHeader.fdf;
}
bool ESP32CanFD::packetBrs() {
  return _rxHeader.brs;
}
uint8_t ESP32CanFD::packetDlc() {
  return _rxHeader.dlc;
}

uint16_t ESP32CanFD::getTEC() {
  twai_node_status_t status;
  if (_node_handle && twai_node_get_info(_node_handle, &status, NULL) == ESP_OK) return status.tx_error_count;
  return 0;
}

uint16_t ESP32CanFD::getREC() {
  twai_node_status_t status;
  if (_node_handle && twai_node_get_info(_node_handle, &status, NULL) == ESP_OK) return status.rx_error_count;
  return 0;
}