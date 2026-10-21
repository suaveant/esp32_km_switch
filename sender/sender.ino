#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ============================================================
// ESP32-S3 BLE keyboard -> ESP-NOW KVM sender
//
// BLE Central / HID Host connects to a BLE keyboard.
// ESP-NOW forwards HID reports to one of two USB HID receivers.
//
// Selector usages in an 8-byte boot-keyboard report:
//   0x6B (F16 / X-cross) -> receiver 1
//   0x6A (F15 / square)  -> receiver 2
//
// Selector keys are removed from forwarded keyboard reports.
// On a switch, a neutral report is sent to the old receiver to
// prevent keys remaining logically held there. Both receivers
// are then told which receiver is active for their status LEDs.
// ============================================================

#define HID_SERVICE_UUID 0x1812
#define HID_REPORT_UUID 0x2A4D
#define HID_BOOT_INPUT_UUID 0x2A22

NimBLEClient* client = nullptr;

bool connected = false;
bool keyboardReady = false;

NimBLERemoteCharacteristic* inputReport = nullptr;
volatile bool discoveryPending = false;
volatile uint32_t keyboardNotificationCount = 0;

// ============================================================
// ESP-NOW packet protocol
// Keep this definition in sync with both receiver sketches.
// ============================================================

enum PacketType : uint8_t {
  PACKET_HID_REPORT = 1,
  PACKET_SWITCH_RECEIVER = 2
};

struct __attribute__((packed)) HidPacket {
  uint8_t version;
  uint8_t type;
  uint8_t reportId;
  uint8_t reportLength;
  uint8_t data[32];
};

static_assert(sizeof(HidPacket) == 36, "Unexpected HidPacket size");

static constexpr uint8_t ESPNOW_CHANNEL = 6;
static constexpr uint8_t PROTOCOL_VERSION = 1;
static constexpr uint8_t RECEIVER_COUNT = 2;

static constexpr uint8_t KEY_SWITCH_RECEIVER_1 = 0x6B;  // F16 / X-cross
static constexpr uint8_t KEY_SWITCH_RECEIVER_2 = 0x6A;  // F15 / square

// Receiver MAC addresses.
uint8_t receiverMacs[RECEIVER_COUNT][6] = {
  { 0x90, 0xDA, 0x72, 0x73, 0x4C, 0x18 },  // Receiver 1
  { 0xD4, 0x05, 0x92, 0x49, 0x9C, 0x40 }   // Receiver 2
};

// This is changed only by loop() when it processes the queued switch.
uint8_t activeReceiver = 1;

QueueHandle_t hidPacketQueue = nullptr;

struct ReportMapEntry {
  NimBLERemoteCharacteristic* characteristic;
  uint8_t reportId;
  uint8_t reportType;
};

ReportMapEntry reportMap[8];
uint8_t reportMapCount = 0;

volatile uint32_t reportsQueued = 0;
volatile uint32_t reportsDropped = 0;
volatile uint32_t reportsSent = 0;

// Forward declarations.
bool queueHidReport(uint8_t reportId, const uint8_t* data, size_t length);
void processEspNowQueue();
void broadcastReceiverState();

// ============================================================
// HID key names
// ============================================================

const char* hidKeyName(uint8_t key) {
  switch (key) {
    case 0x04: return "A";
    case 0x05: return "B";
    case 0x06: return "C";
    case 0x07: return "D";
    case 0x08: return "E";
    case 0x09: return "F";
    case 0x0A: return "G";
    case 0x0B: return "H";
    case 0x0C: return "I";
    case 0x0D: return "J";
    case 0x0E: return "K";
    case 0x0F: return "L";
    case 0x10: return "M";
    case 0x11: return "N";
    case 0x12: return "O";
    case 0x13: return "P";
    case 0x14: return "Q";
    case 0x15: return "R";
    case 0x16: return "S";
    case 0x17: return "T";
    case 0x18: return "U";
    case 0x19: return "V";
    case 0x1A: return "W";
    case 0x1B: return "X";
    case 0x1C: return "Y";
    case 0x1D: return "Z";

    case 0x1E: return "1";
    case 0x1F: return "2";
    case 0x20: return "3";
    case 0x21: return "4";
    case 0x22: return "5";
    case 0x23: return "6";
    case 0x24: return "7";
    case 0x25: return "8";
    case 0x26: return "9";
    case 0x27: return "0";

    case 0x28: return "ENTER";
    case 0x29: return "ESC";
    case 0x2A: return "BACKSPACE";
    case 0x2B: return "TAB";
    case 0x2C: return "SPACE";

    case 0x2D: return "-";
    case 0x2E: return "=";
    case 0x2F: return "[";
    case 0x30: return "]";
    case 0x31: return "\\";
    case 0x33: return ";";
    case 0x34: return "'";
    case 0x35: return "`";
    case 0x36: return ",";
    case 0x37: return ".";
    case 0x38: return "/";

    case 0x39: return "CAPS LOCK";

    case 0x3A: return "F1";
    case 0x3B: return "F2";
    case 0x3C: return "F3";
    case 0x3D: return "F4";
    case 0x3E: return "F5";
    case 0x3F: return "F6";
    case 0x40: return "F7";
    case 0x41: return "F8";
    case 0x42: return "F9";
    case 0x43: return "F10";
    case 0x44: return "F11";
    case 0x45: return "F12";

    case 0x46: return "PRINT SCREEN";
    case 0x47: return "SCROLL LOCK";
    case 0x48: return "PAUSE";
    case 0x49: return "INSERT";
    case 0x4A: return "HOME";
    case 0x4B: return "PAGE UP";
    case 0x4C: return "DELETE";
    case 0x4D: return "END";
    case 0x4E: return "PAGE DOWN";
    case 0x4F: return "RIGHT";
    case 0x50: return "LEFT";
    case 0x51: return "DOWN";
    case 0x52: return "UP";

    case 0x53: return "NUM LOCK";

    case 0x54: return "KEYPAD /";
    case 0x55: return "KEYPAD *";
    case 0x56: return "KEYPAD -";
    case 0x57: return "KEYPAD +";
    case 0x58: return "KEYPAD ENTER";

    case 0x59: return "KEYPAD 1";
    case 0x5A: return "KEYPAD 2";
    case 0x5B: return "KEYPAD 3";
    case 0x5C: return "KEYPAD 4";
    case 0x5D: return "KEYPAD 5";
    case 0x5E: return "KEYPAD 6";
    case 0x5F: return "KEYPAD 7";
    case 0x60: return "KEYPAD 8";
    case 0x61: return "KEYPAD 9";
    case 0x62: return "KEYPAD 0";
    case 0x63: return "KEYPAD .";
    case 0x65: return "APPLICATION";

    case 0x6A: return "F15 (RECEIVER 2 SELECT)";
    case 0x6B: return "F16 (RECEIVER 1 SELECT)";

    case 0xE0: return "LEFT CTRL";
    case 0xE1: return "LEFT SHIFT";
    case 0xE2: return "LEFT ALT";
    case 0xE3: return "LEFT GUI";
    case 0xE4: return "RIGHT CTRL";
    case 0xE5: return "RIGHT SHIFT";
    case 0xE6: return "RIGHT ALT";
    case 0xE7: return "RIGHT GUI";

    default:
      return nullptr;
  }
}

// ============================================================
// ESP-NOW setup and receiver switching
// ============================================================

bool initEspNow() {
  WiFi.mode(WIFI_STA);

  esp_err_t err = esp_wifi_set_channel(
    ESPNOW_CHANNEL,
    WIFI_SECOND_CHAN_NONE);

  if (err != ESP_OK) {
    Serial.printf("Wi-Fi channel setup failed: %d\n", err);
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return false;
  }

  for (uint8_t i = 0; i < RECEIVER_COUNT; i++) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMacs[i], 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;

    if (esp_now_is_peer_exist(receiverMacs[i])) {
      esp_now_del_peer(receiverMacs[i]);
    }

    err = esp_now_add_peer(&peerInfo);
    if (err != ESP_OK) {
      Serial.printf("Adding receiver %u failed: %d\n", i + 1, err);
      return false;
    }

    Serial.printf("Added ESP-NOW receiver %u\n", i + 1);
  }

  hidPacketQueue = xQueueCreate(16, sizeof(HidPacket));
  if (hidPacketQueue == nullptr) {
    Serial.println("Could not create HID packet queue");
    return false;
  }

  Serial.printf("ESP-NOW initialized on channel %u\n", ESPNOW_CHANNEL);
  Serial.printf("HidPacket size: %u bytes\n", (unsigned)sizeof(HidPacket));
  return true;
}

bool queueHidReport(
  uint8_t reportId,
  const uint8_t* data,
  size_t length) {
  if (hidPacketQueue == nullptr || data == nullptr) {
    return false;
  }

  if (length > sizeof(HidPacket::data)) {
    reportsDropped++;
    return false;
  }

  HidPacket packet = {};
  packet.version = PROTOCOL_VERSION;
  packet.type = PACKET_HID_REPORT;
  packet.reportId = reportId;
  packet.reportLength = static_cast<uint8_t>(length);
  memcpy(packet.data, data, length);

  if (xQueueSend(hidPacketQueue, &packet, 0) != pdTRUE) {
    reportsDropped++;
    return false;
  }

  reportsQueued++;
  return true;
}

void queueReceiverSwitch(uint8_t receiverId) {
  if (hidPacketQueue == nullptr ||
      receiverId < 1 ||
      receiverId > RECEIVER_COUNT) {
    return;
  }

  HidPacket packet = {};
  packet.version = PROTOCOL_VERSION;
  packet.type = PACKET_SWITCH_RECEIVER;
  packet.reportId = 0;
  packet.reportLength = 1;
  packet.data[0] = receiverId;

  if (xQueueSend(hidPacketQueue, &packet, 0) != pdTRUE) {
    reportsDropped++;
  }
}

void sendNeutralReportToReceiver(uint8_t receiverId) {
  if (receiverId < 1 || receiverId > RECEIVER_COUNT) {
    return;
  }

  HidPacket packet = {};
  packet.version = PROTOCOL_VERSION;
  packet.type = PACKET_HID_REPORT;
  packet.reportId = 1;
  packet.reportLength = 8;
  // data is zero-filled: all keys and modifiers released.

  esp_err_t result = esp_now_send(
    receiverMacs[receiverId - 1],
    reinterpret_cast<const uint8_t*>(&packet),
    sizeof(packet));

  if (result != ESP_OK) {
    Serial.printf("Neutral report send error: %d\n", result);
  }
}

void broadcastReceiverState() {
  HidPacket packet = {};
  packet.version = PROTOCOL_VERSION;
  packet.type = PACKET_SWITCH_RECEIVER;
  packet.reportId = 0;
  packet.reportLength = 1;
  packet.data[0] = activeReceiver;

  for (uint8_t i = 0; i < RECEIVER_COUNT; i++) {
    esp_now_send(
      receiverMacs[i],
      reinterpret_cast<const uint8_t*>(&packet),
      sizeof(packet));
  }
}

void processReceiverSwitch(uint8_t receiverId) {
  if (receiverId < 1 || receiverId > RECEIVER_COUNT) {
    return;
  }

  if (receiverId == activeReceiver) {
    // Still broadcast the state in case a receiver missed a prior packet.
    broadcastReceiverState();
    return;
  }

  const uint8_t oldReceiver = activeReceiver;

  // Release any held keys at the old host before changing destinations.
  sendNeutralReportToReceiver(oldReceiver);
  delay(2);

  activeReceiver = receiverId;

  Serial.printf(
    "\n[KVM] Switched receiver %u -> receiver %u\n",
    oldReceiver,
    activeReceiver);

  broadcastReceiverState();
}

void processEspNowQueue() {
  if (hidPacketQueue == nullptr) {
    return;
  }

  HidPacket packet;

  // Drain queued work without blocking BLE processing.
  while (xQueueReceive(hidPacketQueue, &packet, 0) == pdTRUE) {
    if (packet.version != PROTOCOL_VERSION) {
      continue;
    }

    if (packet.type == PACKET_SWITCH_RECEIVER) {
      if (packet.reportLength >= 1) {
        processReceiverSwitch(packet.data[0]);
      }
      continue;
    }

    if (packet.type != PACKET_HID_REPORT ||
        packet.reportLength > sizeof(packet.data)) {
      continue;
    }

    uint8_t receiverIndex = activeReceiver - 1;
    esp_err_t result = esp_now_send(
      receiverMacs[receiverIndex],
      reinterpret_cast<const uint8_t*>(&packet),
      sizeof(packet));

    if (result == ESP_OK) {
      reportsSent++;
    } else {
      Serial.printf("ESP-NOW send error: %d\n", result);
    }
  }
}

// ============================================================
// Modifier names
// ============================================================

void printModifiers(uint8_t modifiers) {
  if (!modifiers) {
    return;
  }

  Serial.print("MODIFIERS:");

  if (modifiers & 0x01) Serial.print(" LCTRL");
  if (modifiers & 0x02) Serial.print(" LSHIFT");
  if (modifiers & 0x04) Serial.print(" LALT");
  if (modifiers & 0x08) Serial.print(" LGUI");
  if (modifiers & 0x10) Serial.print(" RCTRL");
  if (modifiers & 0x20) Serial.print(" RSHIFT");
  if (modifiers & 0x40) Serial.print(" RALT");
  if (modifiers & 0x80) Serial.print(" RGUI");

  Serial.println();
}

void printRawReport(uint8_t* data, size_t length) {
  Serial.print("RAW:");
  for (size_t i = 0; i < length; i++) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();
}

// ============================================================
// Keyboard notification
//
// For report ID 1, the expected format is an 8-byte boot report:
//   Byte 0 = modifiers
//   Byte 1 = reserved
//   Byte 2-7 = key usages
//
// F16 / 0x6B selects receiver 1.
// F15 / 0x6A selects receiver 2.
// Both selector usages are filtered from forwarded reports.
// ============================================================

void keyboardNotify(
  NimBLERemoteCharacteristic* characteristic,
  uint8_t* data,
  size_t length,
  bool isNotify) {
  keyboardNotificationCount++;

  uint8_t reportId = 0;
  bool foundInputReport = false;

  for (uint8_t i = 0; i < reportMapCount; i++) {
    if (reportMap[i].characteristic == characteristic &&
        reportMap[i].reportType == 1) {
      reportId = reportMap[i].reportId;
      foundInputReport = true;
      break;
    }
  }

  if (!foundInputReport || length < 1 ||
      length > sizeof(HidPacket::data)) {
    return;
  }

  // Detect rising edges so holding a selector key doesn't repeatedly switch.
  static bool wasReceiver1KeyDown = false;
  static bool wasReceiver2KeyDown = false;

  bool receiver1KeyDown = false;
  bool receiver2KeyDown = false;

  if (reportId == 1 && length == 8) {
    for (size_t i = 2; i < length; i++) {
      if (data[i] == KEY_SWITCH_RECEIVER_1) receiver1KeyDown = true;
      if (data[i] == KEY_SWITCH_RECEIVER_2) receiver2KeyDown = true;
    }

    if (receiver1KeyDown && !wasReceiver1KeyDown) {
      queueReceiverSwitch(1);
    }
    if (receiver2KeyDown && !wasReceiver2KeyDown) {
      queueReceiverSwitch(2);
    }

    wasReceiver1KeyDown = receiver1KeyDown;
    wasReceiver2KeyDown = receiver2KeyDown;
  }

  uint8_t filtered[32] = {};
  memcpy(filtered, data, length);

  if (reportId == 1 && length == 8) {
    for (size_t i = 2; i < length; i++) {
      if (filtered[i] == KEY_SWITCH_RECEIVER_1 ||
          filtered[i] == KEY_SWITCH_RECEIVER_2) {
        filtered[i] = 0;
      }
    }
  }

  // Queue the filtered report after any switch command. Queue ordering means
  // the switch is processed before this selector-free report is forwarded.
  queueHidReport(reportId, filtered, length);

  // Keep serial diagnostics light: print every 25th notification only.
  if ((keyboardNotificationCount % 25) == 1) {
    Serial.printf(
      "[BLE HID] notifications=%lu reportId=%u length=%u\n",
      (unsigned long)keyboardNotificationCount,
      reportId,
      (unsigned)length);
    printRawReport(filtered, length);
  }

  if (reportId == 1 && length >= 2) {
    printModifiers(data[0]);

    for (size_t i = 2; i < length; i++) {
      uint8_t key = data[i];
      if (key == 0 ||
          key == KEY_SWITCH_RECEIVER_1 ||
          key == KEY_SWITCH_RECEIVER_2) {
        continue;
      }

      const char* name = hidKeyName(key);
      if (name) {
        Serial.print("KEY: ");
        Serial.println(name);
      } else {
        Serial.printf("KEY: UNKNOWN HID 0x%02X\n", key);
      }
    }
  }
}

// ============================================================
// Client callbacks
// ============================================================

class MyClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) override {
    Serial.println();
    Serial.println("==============================");
    Serial.println("KEYBOARD CONNECTED");
    Serial.println("==============================");

    connected = true;
    discoveryPending = true;

    Serial.println("Connection callback finished.");
    Serial.println("Service discovery will run from loop().");
  }

  void onConnectFail(NimBLEClient* pClient, int reason) override {
    Serial.printf("Connection FAILED, reason: %d\n", reason);
    connected = false;
  }

  void onDisconnect(NimBLEClient* pClient, int reason) override {
    Serial.println();
    Serial.println("==============================");
    Serial.printf("KEYBOARD DISCONNECTED (%d)\n", reason);
    Serial.println("==============================");

    connected = false;
    keyboardReady = false;
    inputReport = nullptr;
  }
};

// ============================================================
// BLE scan callbacks
// ============================================================

class MyScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* device) override {
    Serial.printf(
      "Found: %-32s %s  RSSI %d\n",
      device->getName().c_str(),
      device->getAddress().toString().c_str(),
      device->getRSSI());

    if (!device->isAdvertisingService(
          NimBLEUUID((uint16_t)HID_SERVICE_UUID))) {
      return;
    }

    Serial.println("  *** HID DEVICE ***");

    if (client != nullptr || connected) {
      Serial.println("Already have a client/connection.");
      return;
    }

    Serial.println("Connecting...");
    NimBLEDevice::getScan()->stop();

    client = NimBLEDevice::createClient();
    if (!client) {
      Serial.println("ERROR: Failed to create BLE client.");
      return;
    }

    client->setClientCallbacks(new MyClientCallbacks(), false);

    Serial.println("Calling connect()...");
    Serial.println("Starting asynchronous connection...");

    if (!client->connect(device, true, true, false)) {
      Serial.println("Could not start connection attempt.");
      NimBLEDevice::deleteClient(client);
      client = nullptr;
      return;
    }

    Serial.println("Connection request submitted.");
  }
};

// ============================================================
// HID service discovery
// ============================================================

void discoverHID() {
  Serial.println("[1] Starting HID service lookup...");

  if (client == nullptr || !client->isConnected()) {
    Serial.println("Client is not connected.");
    return;
  }

  NimBLERemoteService* hidService =
    client->getService(NimBLEUUID((uint16_t)HID_SERVICE_UUID));

  if (!hidService) {
    Serial.println("HID service not found.");
    return;
  }

  Serial.println("HID service found.");

  auto characteristics = hidService->getCharacteristics(true);
  Serial.printf(
    "Found %u HID characteristics.\n",
    (unsigned)characteristics.size());

  reportMapCount = 0;
  unsigned subscribedCount = 0;

  for (auto* c : characteristics) {
    if (!c || c->getUUID() != NimBLEUUID((uint16_t)HID_REPORT_UUID)) {
      continue;
    }

    NimBLERemoteDescriptor* ref =
      c->getDescriptor(NimBLEUUID((uint16_t)0x2908));

    if (!ref) {
      continue;
    }

    std::string value = ref->readValue();
    if (value.length() != 2) {
      continue;
    }

    uint8_t reportId = (uint8_t)value[0];
    uint8_t reportType = (uint8_t)value[1];

    if (reportMapCount < 8) {
      reportMap[reportMapCount++] = { c, reportId, reportType };
    }

    if (reportType != 1 || !c->canNotify()) {
      continue;
    }

    if (!client || !client->isConnected()) {
      Serial.println("Disconnected; stopping subscriptions.");
      break;
    }

    Serial.printf(
      "Subscribing: handle=%u, report ID=%u\n",
      c->getHandle(),
      reportId);

    bool ok = c->subscribe(true, keyboardNotify);
    Serial.printf("  Result: %s\n", ok ? "SUCCESS" : "FAILED");

    if (!client->isConnected()) {
      Serial.println("Disconnected during subscription.");
      break;
    }

    if (ok) {
      subscribedCount++;
    }
  }

  keyboardReady =
    subscribedCount > 0 && client != nullptr && client->isConnected();

  Serial.printf("Input reports subscribed: %u\n", subscribedCount);

  if (keyboardReady) {
    Serial.println("Keyboard reports are ready.");
    Serial.println("F16/X-cross selects receiver 1; F15/square selects receiver 2.");
  }
}

// ============================================================
// Start BLE scanning
// ============================================================

void startScan() {
  Serial.println();
  Serial.println("Scanning for BLE HID keyboards...");

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new MyScanCallbacks(), false);
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(15);
  scan->start(0, false);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println();
  Serial.println("========================================");
  Serial.println(" ESP32-S3 BLE KEYBOARD KVM SENDER");
  Serial.println("========================================");

  if (!initEspNow()) {
    Serial.println("ESP-NOW setup failed; check configuration.");
  }

  NimBLEDevice::init("ESP32-S3 BLE Keyboard Host");
  NimBLEDevice::setSecurityAuth(true, false, true);

  Serial.println("BLE initialized.");
  startScan();

  // The receivers may boot later; loop() repeats this broadcast periodically.
  broadcastReceiverState();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  if (discoveryPending) {
    discoveryPending = false;
    discoverHID();
  }

  processEspNowQueue();

  static uint32_t lastStatus = 0;
  if (millis() - lastStatus >= 5000) {
    lastStatus = millis();

    bool linkUp = client != nullptr && client->isConnected();

    Serial.printf(
      "[STATUS] connected=%s, keyboardReady=%s, notifications=%lu\n",
      linkUp ? "YES" : "NO",
      keyboardReady ? "YES" : "NO",
      (unsigned long)keyboardNotificationCount);

    Serial.printf(
      "[ESPNOW] queued=%lu sent=%lu dropped=%lu activeReceiver=%u\n",
      (unsigned long)reportsQueued,
      (unsigned long)reportsSent,
      (unsigned long)reportsDropped,
      activeReceiver);
  }

  static uint32_t lastReceiverBroadcast = 0;
  if (millis() - lastReceiverBroadcast >= 2000) {
    lastReceiverBroadcast = millis();
    broadcastReceiverState();
  }

  if (!connected && client == nullptr) {
    static uint32_t lastScan = 0;
    if (millis() - lastScan > 3000) {
      lastScan = millis();
      startScan();
    }
  }

  delay(10);
}
