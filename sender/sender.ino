#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <cstring>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define HID_SERVICE_UUID 0x1812
#define HID_REPORT_UUID  0x2A4D

// Diagnostic mode: count real M720 BLE reports but discard their movement data,
// while sending synthetic circular mouse movement through ESP-NOW at 125 Hz.
static constexpr bool TEST_SYNTHETIC_WITH_BLE = true;
static constexpr uint32_t SYNTHETIC_INTERVAL_US = 8000;
static constexpr uint32_t SYNTHETIC_DURATION_MS = 300000; // 5 minutes
static constexpr float SYNTHETIC_TWO_PI = 6.28318530718f;

static constexpr uint8_t ESPNOW_CHANNEL = 6;
static constexpr uint8_t PROTOCOL_VERSION = 1;
static constexpr uint8_t RECEIVER_COUNT = 2;
static constexpr uint8_t PACKET_HID_REPORT = 1;
static constexpr uint8_t PACKET_SWITCH_RECEIVER = 2;
static constexpr uint8_t PACKET_DIAGNOSTICS = 3;
static constexpr uint8_t KEY_SWITCH_RECEIVER_1 = 0x6B; // F16
static constexpr uint8_t KEY_SWITCH_RECEIVER_2 = 0x6A; // F15

uint8_t receiverMacs[RECEIVER_COUNT][6] = {
  {0x90, 0xDA, 0x72, 0x73, 0x4C, 0x18},
  {0xD4, 0x05, 0x92, 0x49, 0x9C, 0x40}
};

struct __attribute__((packed)) HidPacket {
  uint8_t version;
  uint8_t type;
  uint8_t reportId;
  uint8_t reportLength;
  uint8_t data[32];
};
static_assert(sizeof(HidPacket) == 36, "Unexpected HidPacket size");

enum DeviceRole : uint8_t { ROLE_KEYBOARD = 0, ROLE_MOUSE = 1 };

struct DeviceSlot {
  const char* label;
  const char* nameMatch;
  NimBLEClient* client;
  volatile bool connected;
  bool ready;
  volatile bool discoveryPending;
  volatile uint32_t notificationCount;
};

DeviceSlot keyboardSlot = {"KEYBOARD", "keychron", nullptr, false, false, false, 0};
DeviceSlot mouseSlot = {"MOUSE", "m720", nullptr, false, false, false, 0};

struct ReportMapEntry {
  NimBLERemoteCharacteristic* characteristic;
  uint8_t reportId;
  uint8_t reportType;
  DeviceRole role;
};
ReportMapEntry reportMap[16];
volatile uint8_t reportMapCount = 0;

QueueHandle_t hidPacketQueue = nullptr;
SemaphoreHandle_t sendDoneSemaphore = nullptr;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t activeReceiver = 1;
volatile uint32_t reportsQueued = 0, reportsDropped = 0, reportsSent = 0;
volatile uint32_t sendFailures = 0, sendRetries = 0;
volatile uint32_t mouseReportsDiscardedForTest = 0;
volatile uint32_t syntheticReportsGenerated = 0;
volatile uint32_t syntheticReportsQueued = 0;
volatile uint32_t syntheticReportsQueueDrops = 0;
volatile uint32_t sendCallErrors = 0;
volatile uint32_t callbackFailures = 0;
volatile uint32_t callbackTimeouts = 0;
volatile int32_t lastSendError = 0;
volatile int32_t lastCallbackStatus = -1;
volatile uint8_t inFlightType = 0;
volatile uint8_t inFlightReceiver = 0;
volatile uint32_t callbackFailHid = 0;
volatile uint32_t callbackFailSwitch = 0;
volatile uint32_t callbackFailRx1 = 0;
volatile uint32_t callbackFailRx2 = 0;
volatile uint32_t callbackOkHid = 0;
volatile uint32_t callbackOkSwitch = 0;
volatile uint32_t callbackOkRx1 = 0;
volatile uint32_t callbackOkRx2 = 0;
bool wasReceiver1KeyDown = false, wasReceiver2KeyDown = false;
volatile bool sendCallbackSuccess = false;

void recordSendCallback(esp_now_send_status_t status) {
  lastCallbackStatus = (int32_t)status;
  sendCallbackSuccess = (status == ESP_NOW_SEND_SUCCESS);
  if (status == ESP_NOW_SEND_SUCCESS) {
    if (inFlightType == PACKET_SWITCH_RECEIVER) callbackOkSwitch++;
    else callbackOkHid++;
    if (inFlightReceiver == 1) callbackOkRx1++;
    else if (inFlightReceiver == 2) callbackOkRx2++;
  } else {
    if (inFlightType == PACKET_SWITCH_RECEIVER) callbackFailSwitch++;
    else callbackFailHid++;
    if (inFlightReceiver == 1) callbackFailRx1++;
    else if (inFlightReceiver == 2) callbackFailRx2++;
  }
}

struct ReceiverDiagnostics {
  uint32_t received = 0;
  uint32_t keyboard = 0;
  uint32_t mouse = 0;
  uint32_t bad = 0;
  uint32_t queueDrops = 0;
  uint32_t switches = 0;
  uint32_t lastUpdateMs = 0;
};
ReceiverDiagnostics receiverDiag[RECEIVER_COUNT];
portMUX_TYPE diagnosticsMux = portMUX_INITIALIZER_UNLOCKED;

DeviceSlot* slotForRole(DeviceRole role) {
  return role == ROLE_KEYBOARD ? &keyboardSlot : &mouseSlot;
}

bool queueHidReport(uint8_t reportId, const uint8_t* data, size_t length);
void discoverHID(DeviceSlot& slot, DeviceRole role);
void startScan();

bool containsIgnoreCase(const std::string& value, const char* needle) {
  String a(value.c_str()), b(needle);
  a.toLowerCase();
  b.toLowerCase();
  return a.indexOf(b) >= 0;
}

// ESP-NOW callback signature changed with ESP-IDF 5.5.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void onEspNowSent(const wifi_tx_info_t* txInfo, esp_now_send_status_t status) {
  (void)txInfo;
  recordSendCallback(status);
  if (sendDoneSemaphore) xSemaphoreGive(sendDoneSemaphore);
}
#else
void onEspNowSent(const uint8_t* mac, esp_now_send_status_t status) {
  (void)mac;
  recordSendCallback(status);
  if (sendDoneSemaphore) xSemaphoreGive(sendDoneSemaphore);
}
#endif

void onEspNowReceived(
  const esp_now_recv_info_t* info,
  const uint8_t* data,
  int len
) {
  if (!info || !data || len != (int)sizeof(HidPacket)) return;
  if (data[0] != PROTOCOL_VERSION ||
      data[1] != PACKET_DIAGNOSTICS ||
      data[3] != 24) return;

  int receiverIndex = -1;
  for (uint8_t i = 0; i < RECEIVER_COUNT; ++i) {
    if (memcmp(info->src_addr, receiverMacs[i], 6) == 0) {
      receiverIndex = i;
      break;
    }
  }
  if (receiverIndex < 0) return;

  uint32_t counters[6];
  memcpy(counters, data + 4, sizeof(counters));

  portENTER_CRITICAL(&diagnosticsMux);
  receiverDiag[receiverIndex].received = counters[0];
  receiverDiag[receiverIndex].keyboard = counters[1];
  receiverDiag[receiverIndex].mouse = counters[2];
  receiverDiag[receiverIndex].bad = counters[3];
  receiverDiag[receiverIndex].queueDrops = counters[4];
  receiverDiag[receiverIndex].switches = counters[5];
  receiverDiag[receiverIndex].lastUpdateMs = millis();
  portEXIT_CRITICAL(&diagnosticsMux);
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);  // Required for ESP-NOW; no router connection is used.
  esp_err_t err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) {
    Serial.printf("Wi-Fi channel setup failed: %d\n", (int)err);
    return false;
  }
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return false;
  }
  esp_now_register_send_cb(onEspNowSent);
  err = esp_now_register_recv_cb(onEspNowReceived);
  if (err != ESP_OK) {
    Serial.printf("ESP-NOW receive callback registration failed: %d\n", (int)err);
    return false;
  }

  for (uint8_t i = 0; i < RECEIVER_COUNT; i++) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMacs[i], 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;
    if (esp_now_is_peer_exist(receiverMacs[i])) esp_now_del_peer(receiverMacs[i]);
    err = esp_now_add_peer(&peerInfo);
    if (err != ESP_OK) {
      Serial.printf("Adding receiver %u failed: %d\n", i + 1, (int)err);
      return false;
    }
    Serial.printf("Added ESP-NOW receiver %u\n", i + 1);
  }

  hidPacketQueue = xQueueCreate(64, sizeof(HidPacket));
  sendDoneSemaphore = xSemaphoreCreateBinary();
  if (!hidPacketQueue || !sendDoneSemaphore) {
    Serial.println("Could not create ESP-NOW queue/semaphore");
    return false;
  }
  Serial.printf("ESP-NOW channel %u; packet size=%u; queue depth=64\n",
                ESPNOW_CHANNEL, (unsigned)sizeof(HidPacket));
  return true;
}

bool queuePacket(const HidPacket& packet) {
  if (!hidPacketQueue) {
    reportsDropped++;
    return false;
  }
  if (xQueueSend(hidPacketQueue, &packet, 0) != pdTRUE) {
    // Keep the input callback non-blocking. Queue depth is deliberately larger
    // than before; overflow is counted so it is visible in serial diagnostics.
    reportsDropped++;
    return false;
  }
  reportsQueued++;
  return true;
}

bool queueHidReport(uint8_t reportId, const uint8_t* data, size_t length) {
  if (!data || !length || length > sizeof(HidPacket::data)) {
    reportsDropped++;
    return false;
  }
  HidPacket packet = {};
  packet.version = PROTOCOL_VERSION;
  packet.type = PACKET_HID_REPORT;
  packet.reportId = reportId;
  packet.reportLength = (uint8_t)length;
  memcpy(packet.data, data, length);
  return queuePacket(packet);
}

void encodeSyntheticMouseDelta(int16_t dx, int16_t dy, uint8_t* p) {
  uint16_t x = (uint16_t)dx & 0x0FFF;
  uint16_t y = (uint16_t)dy & 0x0FFF;
  p[0] = 0;
  p[1] = 0;
  p[2] = x & 0xFF;
  p[3] = ((x >> 8) & 0x0F) | ((y & 0x0F) << 4);
  p[4] = (y >> 4) & 0xFF;
  p[5] = 0;
  p[6] = 0;
}

void generateSyntheticMouseReport(uint32_t index) {
  const float phase = SYNTHETIC_TWO_PI * ((float)(index % 250) / 250.0f);
  const int16_t dx = (int16_t)lroundf(4.0f * cosf(phase));
  const int16_t dy = (int16_t)lroundf(4.0f * sinf(phase));
  uint8_t data[7];
  encodeSyntheticMouseDelta(dx, dy, data);
  syntheticReportsGenerated++;
  if (queueHidReport(2, data, sizeof(data))) syntheticReportsQueued++;
  else syntheticReportsQueueDrops++;
}

void queueReceiverSwitch(uint8_t receiverId) {
  if (receiverId < 1 || receiverId > RECEIVER_COUNT) return;
  HidPacket packet = {};
  packet.version = PROTOCOL_VERSION;
  packet.type = PACKET_SWITCH_RECEIVER;
  packet.reportLength = 1;
  packet.data[0] = receiverId;
  queuePacket(packet);
}

// All ESP-NOW sends, including neutral reports and receiver-state broadcasts,
// are serialized by the ESP-NOW worker task.
bool sendPacketAndWait(const uint8_t* mac, const HidPacket& packet) {
  if (!sendDoneSemaphore) return false;

  for (uint8_t attempt = 0; attempt < 4; attempt++) {
    while (xSemaphoreTake(sendDoneSemaphore, 0) == pdTRUE) {}
    sendCallbackSuccess = false;
    inFlightType = packet.type;
    inFlightReceiver = (mac[0] == receiverMacs[0][0] &&
                        memcmp(mac, receiverMacs[0], 6) == 0) ? 1 :
                       ((memcmp(mac, receiverMacs[1], 6) == 0) ? 2 : 0);

    esp_err_t err = esp_now_send(mac, (const uint8_t*)&packet, sizeof(packet));
    if (err != ESP_OK) {
      sendFailures++;
      sendCallErrors++;
      lastSendError = (int32_t)err;
      if (attempt < 3) {
        sendRetries++;
        vTaskDelay(pdMS_TO_TICKS(2 + attempt * 3));
        continue;
      }
      Serial.printf("[ESPNOW] send call failed: %d\n", (int)err);
      return false;
    }

    // Never submit another packet until this send has completed.
    if (xSemaphoreTake(sendDoneSemaphore, pdMS_TO_TICKS(100)) != pdTRUE) {
      sendFailures++;
      callbackTimeouts++;
      if (attempt < 3) {
        sendRetries++;
        vTaskDelay(pdMS_TO_TICKS(4 + attempt * 4));
        continue;
      }
      Serial.println("[ESPNOW] completion callback timed out");
      return false;
    }

    if (sendCallbackSuccess) {
      reportsSent++;
      return true;
    }

    sendFailures++;
    callbackFailures++;
    if (attempt < 3) {
      sendRetries++;
      vTaskDelay(pdMS_TO_TICKS(2 + attempt * 3));
    }
  }
  return false;
}

void sendNeutralKeyboardReport(uint8_t receiverId) {
  if (receiverId < 1 || receiverId > RECEIVER_COUNT) return;
  HidPacket packet = {};
  packet.version = PROTOCOL_VERSION;
  packet.type = PACKET_HID_REPORT;
  packet.reportId = 1;
  packet.reportLength = 8;
  sendPacketAndWait(receiverMacs[receiverId - 1], packet);
}

void broadcastReceiverState() {
  HidPacket packet = {};
  packet.version = PROTOCOL_VERSION;
  packet.type = PACKET_SWITCH_RECEIVER;
  packet.reportLength = 1;
  packet.data[0] = activeReceiver;
  for (uint8_t i = 0; i < RECEIVER_COUNT; i++)
    sendPacketAndWait(receiverMacs[i], packet);
}

void processReceiverSwitch(uint8_t receiverId) {
  if (receiverId < 1 || receiverId > RECEIVER_COUNT) return;
  uint8_t oldReceiver;
  portENTER_CRITICAL(&stateMux);
  oldReceiver = activeReceiver;
  portEXIT_CRITICAL(&stateMux);

  if (receiverId != oldReceiver) {
    sendNeutralKeyboardReport(oldReceiver);
    portENTER_CRITICAL(&stateMux);
    activeReceiver = receiverId;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("\n[KVM] Switched receiver %u -> receiver %u\n",
                  oldReceiver, receiverId);
  }
  broadcastReceiverState();
}

void espNowWorker(void* parameter) {
  (void)parameter;
  for (;;) {
    HidPacket packet;
    if (xQueueReceive(hidPacketQueue, &packet, pdMS_TO_TICKS(25)) == pdTRUE) {
      if (packet.version != PROTOCOL_VERSION) continue;

      if (packet.type == PACKET_SWITCH_RECEIVER) {
        if (packet.reportLength >= 1) processReceiverSwitch(packet.data[0]);
      } else if (packet.type == PACKET_HID_REPORT &&
                 packet.reportLength > 0 &&
                 packet.reportLength <= sizeof(packet.data)) {
        uint8_t receiver;
        portENTER_CRITICAL(&stateMux);
        receiver = activeReceiver;
        portEXIT_CRITICAL(&stateMux);
        sendPacketAndWait(receiverMacs[receiver - 1], packet);
      }
    }

    // IMPORTANT TEST MODE: do not periodically broadcast receiver state.
    // Those packets were dominating the ESP-NOW link (roughly 4 failures
    // for every successful SWITCH packet) and can interfere with HID timing.
  }
}

// ---------------- BLE HID notifications ----------------
void hidNotify(NimBLERemoteCharacteristic* characteristic,
               uint8_t* data, size_t length, bool isNotify) {
  (void)isNotify;
  DeviceRole role = ROLE_KEYBOARD;
  uint8_t reportId = 0;
  bool found = false;

  uint8_t mapCount = reportMapCount;
  for (uint8_t i = 0; i < mapCount; i++) {
    if (reportMap[i].characteristic == characteristic &&
        reportMap[i].reportType == 1) {
      role = reportMap[i].role;
      reportId = reportMap[i].reportId;
      found = true;
      break;
    }
  }
  if (!found || !data || !length || length > sizeof(HidPacket::data)) return;

  DeviceSlot* slot = slotForRole(role);
  slot->notificationCount++;

  if (role == ROLE_KEYBOARD) {
    if (reportId != 1 || length != 8) return;

    bool r1 = false, r2 = false;
    for (size_t i = 2; i < 8; i++) {
      if (data[i] == KEY_SWITCH_RECEIVER_1) r1 = true;
      if (data[i] == KEY_SWITCH_RECEIVER_2) r2 = true;
    }
    if (r1 && !wasReceiver1KeyDown) queueReceiverSwitch(1);
    if (r2 && !wasReceiver2KeyDown) queueReceiverSwitch(2);
    wasReceiver1KeyDown = r1;
    wasReceiver2KeyDown = r2;

    uint8_t filtered[8];
    memcpy(filtered, data, sizeof(filtered));
    for (size_t i = 2; i < 8; i++) {
      if (filtered[i] == KEY_SWITCH_RECEIVER_1 ||
          filtered[i] == KEY_SWITCH_RECEIVER_2) filtered[i] = 0;
    }
    queueHidReport(1, filtered, sizeof(filtered));
  } else {
    // M720 report ID 2 is 7 bytes; ignore vendor-specific report ID 17.
    if (reportId == 2 && length == 7) {
      if (TEST_SYNTHETIC_WITH_BLE) {
        // Count the notification, but deliberately discard real movement.
        mouseReportsDiscardedForTest++;
      } else {
        queueHidReport(2, data, length);
      }
    }
  }

  // Avoid per-notification mouse serial output during timing test; USB/serial
  // logging can perturb the very loop timing we are trying to measure.
  if (role == ROLE_KEYBOARD && (slot->notificationCount % 50) == 1)
    Serial.printf("[%s] reports=%lu reportId=%u length=%u\n", slot->label,
      (unsigned long)slot->notificationCount, reportId, (unsigned)length);
}

class SlotClientCallbacks : public NimBLEClientCallbacks {
public:
  explicit SlotClientCallbacks(DeviceRole role) : role_(role) {}
  void onConnect(NimBLEClient* pClient) override {
    DeviceSlot* slot = slotForRole(role_);
    slot->connected = true;
    slot->discoveryPending = true;
    Serial.printf("[%s] Connected; HID discovery pending\n", slot->label);
    (void)pClient;
  }
  void onConnectFail(NimBLEClient* pClient, int reason) override {
    DeviceSlot* slot = slotForRole(role_);
    slot->connected = false;
    slot->ready = false;
    Serial.printf("[%s] Connection failed: %d\n", slot->label, reason);
    (void)pClient;
  }
  void onDisconnect(NimBLEClient* pClient, int reason) override {
    DeviceSlot* slot = slotForRole(role_);
    slot->connected = false;
    slot->ready = false;
    slot->discoveryPending = false;
    Serial.printf("[%s] Disconnected: %d\n", slot->label, reason);
    (void)pClient;
  }
private:
  DeviceRole role_;
};

void discoverHID(DeviceSlot& slot, DeviceRole role) {
  if (!slot.client || !slot.client->isConnected()) return;
  NimBLERemoteService* service =
    slot.client->getService(NimBLEUUID((uint16_t)HID_SERVICE_UUID));
  if (!service) {
    Serial.printf("[%s] HID service not found\n", slot.label);
    return;
  }

  auto characteristics = service->getCharacteristics(true);
  unsigned subscribed = 0;
  for (auto* c : characteristics) {
    if (!c || c->getUUID() != NimBLEUUID((uint16_t)HID_REPORT_UUID)) continue;
    NimBLERemoteDescriptor* ref = c->getDescriptor(NimBLEUUID((uint16_t)0x2908));
    if (!ref) continue;
    std::string value = ref->readValue();
    if (value.length() != 2) continue;

    uint8_t id = (uint8_t)value[0];
    uint8_t type = (uint8_t)value[1];
    if (reportMapCount >= sizeof(reportMap) / sizeof(reportMap[0])) {
      Serial.println("WARNING: BLE report map full");
      break;
    }
    reportMap[reportMapCount++] = {c, id, type, role};

    bool wanted = (type == 1 && c->canNotify() &&
      ((role == ROLE_KEYBOARD && id == 1) ||
       (role == ROLE_MOUSE && id == 2)));
    if (!wanted) continue;
    bool ok = c->subscribe(true, hidNotify);
    Serial.printf("[%s] subscribe report ID %u handle %u: %s\n",
      slot.label, id, c->getHandle(), ok ? "OK" : "FAILED");
    if (ok) subscribed++;
  }
  slot.ready = subscribed > 0 && slot.client && slot.client->isConnected();
  Serial.printf("[%s] HID ready=%s (subscriptions=%u)\n",
    slot.label, slot.ready ? "YES" : "NO", subscribed);
}

// ---------------- Scan and connect ----------------
bool nameMatches(const NimBLEAdvertisedDevice* device, DeviceRole role) {
  if (!device || !device->haveName()) return false;
  return containsIgnoreCase(device->getName(), slotForRole(role)->nameMatch);
}

bool connectSlot(DeviceSlot& slot, DeviceRole role,
                 const NimBLEAdvertisedDevice* device) {
  if (slot.client && (slot.client->isConnected() || slot.connected)) return false;
  if (slot.client) {
    NimBLEDevice::deleteClient(slot.client);
    slot.client = nullptr;
  }
  NimBLEClient* c = NimBLEDevice::createClient();
  if (!c) {
    Serial.printf("[%s] Failed to create BLE client\n", slot.label);
    return false;
  }
  slot.client = c;
  c->setClientCallbacks(new SlotClientCallbacks(role), true);
  Serial.printf("[%s] Connecting to %s (%s), RSSI %d\n",
    slot.label, device->getName().c_str(),
    device->getAddress().toString().c_str(), device->getRSSI());
  if (!c->connect(device, true, true, false)) {
    Serial.printf("[%s] connect() could not start\n", slot.label);
    NimBLEDevice::deleteClient(c);
    slot.client = nullptr;
    return false;
  }
  return true;
}

class MyScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* device) override {
    if (!device) return;
    bool isKeyboard = nameMatches(device, ROLE_KEYBOARD);
    bool isMouse = nameMatches(device, ROLE_MOUSE);
    if (!isKeyboard && !isMouse) return;

    Serial.printf("BLE candidate: %s (%s), RSSI %d\n",
      device->getName().c_str(), device->getAddress().toString().c_str(),
      device->getRSSI());

    if (isKeyboard && (!keyboardSlot.client ||
        (!keyboardSlot.client->isConnected() && !keyboardSlot.connected))) {
      NimBLEDevice::getScan()->stop();
      connectSlot(keyboardSlot, ROLE_KEYBOARD, device);
      return;
    }
    if (isMouse && (!mouseSlot.client ||
        (!mouseSlot.client->isConnected() && !mouseSlot.connected))) {
      NimBLEDevice::getScan()->stop();
      connectSlot(mouseSlot, ROLE_MOUSE, device);
    }
  }
};

void startScan() {
  NimBLEScan* scan = NimBLEDevice::getScan();
  if (!scan) return;
  Serial.println("Scanning for Keychron keyboard and M720 mouse...");
  scan->setScanCallbacks(new MyScanCallbacks(), true);
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(15);
  scan->start(0, false);
}

void printReceiverDiagnostics() {
  ReceiverDiagnostics snapshot[RECEIVER_COUNT];

  portENTER_CRITICAL(&diagnosticsMux);
  memcpy(snapshot, receiverDiag, sizeof(snapshot));
  portEXIT_CRITICAL(&diagnosticsMux);

  const uint32_t now = millis();
  for (uint8_t i = 0; i < RECEIVER_COUNT; ++i) {
    const ReceiverDiagnostics& d = snapshot[i];
    if (d.lastUpdateMs == 0) {
      Serial.printf("[RX%u DIAG] waiting for first report\n", i + 1);
      continue;
    }

    Serial.printf(
      "[RX%u DIAG] age=%lums packets=%lu keys=%lu mouse=%lu bad=%lu queueDrops=%lu switches=%lu\n",
      i + 1,
      (unsigned long)(now - d.lastUpdateMs),
      (unsigned long)d.received,
      (unsigned long)d.keyboard,
      (unsigned long)d.mouse,
      (unsigned long)d.bad,
      (unsigned long)d.queueDrops,
      (unsigned long)d.switches
    );
  }
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("========================================");
  Serial.println(" ESP32-S3 BLE KEYBOARD + MOUSE KVM SENDER");
  Serial.println("========================================");
  if (TEST_SYNTHETIC_WITH_BLE) {
    Serial.println("[TEST] Synthetic mouse + BLE M720 report counting");
    Serial.println("[TEST] Real M720 movement data will be discarded.");
    Serial.println("[TEST] Synthetic reports run for 5 minutes at 125 Hz.");
  } else {
    Serial.println("[TEST] Normal BLE mouse forwarding mode.");
  }

  if (!initEspNow()) {
    Serial.println("ESP-NOW setup failed; check configuration.");
  } else {
    BaseType_t taskResult = xTaskCreatePinnedToCore(
      espNowWorker, "EspNowTx", 4096, nullptr, 3, nullptr, 0);
    if (taskResult != pdPASS) Serial.println("Failed to create ESP-NOW worker task");
  }

  NimBLEDevice::init("KVM Sender");
  NimBLEDevice::setSecurityAuth(true, false, true);
  Serial.println("BLE initialized; searching for names containing Keychron and M720.");
  startScan();
}

void loop() {
  if (keyboardSlot.discoveryPending) {
    keyboardSlot.discoveryPending = false;
    discoverHID(keyboardSlot, ROLE_KEYBOARD);
  }
  if (mouseSlot.discoveryPending) {
    mouseSlot.discoveryPending = false;
    discoverHID(mouseSlot, ROLE_MOUSE);
  }

  uint32_t now = millis();

  // Synthetic reports use the existing queue and ESP-NOW worker.
  if (TEST_SYNTHETIC_WITH_BLE) {
    static bool syntheticStarted = false;
    static bool syntheticFinished = false;
    static uint32_t syntheticStartMs = 0;
    static uint32_t nextSyntheticUs = 0;

    if (!syntheticStarted) {
      syntheticStarted = true;
      syntheticStartMs = now;
      nextSyntheticUs = micros();
      Serial.println("[TEST] 5-minute synthetic stream started.");
    }

    if (!syntheticFinished && (uint32_t)(now - syntheticStartMs) >= SYNTHETIC_DURATION_MS) {
      syntheticFinished = true;
      Serial.println("[TEST] Synthetic stream finished; BLE report counting continues.");
    }

    if (!syntheticFinished) {
      uint32_t nowUs = micros();
      if ((int32_t)(nowUs - nextSyntheticUs) >= 0) {
        generateSyntheticMouseReport(syntheticReportsGenerated);
        nextSyntheticUs += SYNTHETIC_INTERVAL_US;
        if ((int32_t)(nowUs - nextSyntheticUs) >= (int32_t)SYNTHETIC_INTERVAL_US) {
          nextSyntheticUs = nowUs + SYNTHETIC_INTERVAL_US;
        }
      }
    }
  }

  static uint32_t lastStatus = 0;
  if (now - lastStatus >= 5000) {
    lastStatus = now;
    Serial.printf("[STATUS] keyboard=%s/%s reports=%lu | mouse=%s/%s reports=%lu\n",
      keyboardSlot.connected ? "connected" : "down",
      keyboardSlot.ready ? "ready" : "not-ready",
      (unsigned long)keyboardSlot.notificationCount,
      mouseSlot.connected ? "connected" : "down",
      mouseSlot.ready ? "ready" : "not-ready",
      (unsigned long)mouseSlot.notificationCount);
    if (TEST_SYNTHETIC_WITH_BLE) {
      Serial.printf("[SYNTHETIC TEST] generated=%lu queued=%lu queueDrops=%lu | realM720ReportsReadAndDiscarded=%lu\n",
        (unsigned long)syntheticReportsGenerated,
        (unsigned long)syntheticReportsQueued,
        (unsigned long)syntheticReportsQueueDrops,
        (unsigned long)mouseReportsDiscardedForTest);
    }
    Serial.printf("[ESPNOW] queued=%lu sent=%lu dropped=%lu failed=%lu retries=%lu activeReceiver=%u\n",
      (unsigned long)reportsQueued, (unsigned long)reportsSent,
      (unsigned long)reportsDropped, (unsigned long)sendFailures,
      (unsigned long)sendRetries, activeReceiver);
    Serial.printf("[TX DETAIL] callErrors=%lu lastErr=%ld callbackFails=%lu lastCbStatus=%ld timeouts=%lu\n",
      (unsigned long)sendCallErrors, (long)lastSendError,
      (unsigned long)callbackFailures, (long)lastCallbackStatus,
      (unsigned long)callbackTimeouts);
    Serial.printf("[TX BY TYPE] HID ok=%lu fail=%lu | SWITCH ok=%lu fail=%lu\n",
      (unsigned long)callbackOkHid, (unsigned long)callbackFailHid,
      (unsigned long)callbackOkSwitch, (unsigned long)callbackFailSwitch);
    Serial.printf("[TX BY RX] RX1 ok=%lu fail=%lu | RX2 ok=%lu fail=%lu\n",
      (unsigned long)callbackOkRx1, (unsigned long)callbackFailRx1,
      (unsigned long)callbackOkRx2, (unsigned long)callbackFailRx2);
    printReceiverDiagnostics();
  }

  static uint32_t lastScanCheck = 0;
  if (!TEST_SYNTHETIC_WITH_BLE && now - lastScanCheck >= 3000) {
    lastScanCheck = now;
    bool keyboardMissing = !keyboardSlot.client ||
      (!keyboardSlot.client->isConnected() && !keyboardSlot.connected);
    bool mouseMissing = !mouseSlot.client ||
      (!mouseSlot.client->isConnected() && !mouseSlot.connected);
    NimBLEScan* scan = NimBLEDevice::getScan();
    if ((keyboardMissing || mouseMissing) && scan && !scan->isScanning()) startScan();
  }
  // No delay here: keep the main loop available for BLE and timing work.
  yield();
}