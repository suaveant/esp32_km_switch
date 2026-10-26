#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <USB.h>
#include <USBHIDKeyboard.h>
#include <USBHIDMouse.h>
#include <cstring>
#include <Adafruit_NeoPixel.h>

#define RECEIVER_ID 2  // Change to 2 on the second receiver

Adafruit_NeoPixel pixel(1, 48, NEO_GRB + NEO_KHZ800);

// ============================================================
// ESP-NOW protocol
// MUST match sender/sender.ino
// ============================================================

static constexpr uint8_t PROTOCOL_VERSION = 1;
static constexpr uint8_t PACKET_HID_REPORT = 1;
static constexpr uint8_t PACKET_SWITCH_RECEIVER = 2;
static constexpr uint8_t PACKET_DIAGNOSTICS = 3;

static constexpr uint8_t REPORT_ID_KEYBOARD = 1;
static constexpr uint8_t REPORT_ID_MOUSE = 2;

static constexpr uint32_t WIFI_CHANNEL = 6;

static constexpr size_t PACKET_SIZE = 36;
static constexpr size_t HEADER_SIZE = 4;
static constexpr size_t MAX_REPORT_SIZE = 32;

// Bigger than before so short bursts have more room.
static constexpr uint8_t RX_QUEUE_DEPTH = 64;

// The USB HID worker is deliberately separate from Arduino loop().
static constexpr BaseType_t HID_TASK_CORE = 0;
static constexpr UBaseType_t HID_TASK_PRIORITY = 4;
static constexpr uint32_t HID_TASK_STACK = 4096;

static constexpr uint32_t DIAGNOSTICS_INTERVAL_MS = 2000;

struct __attribute__((packed)) HidPacket {
  uint8_t version;
  uint8_t type;
  uint8_t reportId;
  uint8_t reportLength;
  uint8_t data[MAX_REPORT_SIZE];
};

static_assert(sizeof(HidPacket) == PACKET_SIZE, "Unexpected HidPacket size");

struct RxItem {
  uint8_t length;
  uint8_t data[PACKET_SIZE];
};

// ============================================================
// USB HID devices
// ============================================================

USBHIDKeyboard keyboard;
USBHIDMouse mouse;

QueueHandle_t rxQueue = nullptr;
TaskHandle_t hidTaskHandle = nullptr;

// ============================================================
// Statistics
// ============================================================

volatile uint32_t receivedCount = 0;
volatile uint32_t badPacketCount = 0;
volatile uint32_t queueDropCount = 0;

volatile uint32_t keyboardCount = 0;
volatile uint32_t mouseCount = 0;
volatile uint32_t switchCount = 0;

volatile uint32_t hidTaskPackets = 0;
volatile uint32_t hidTaskMaxQueueDepth = 0;
volatile uint32_t hidTaskMaxRunUs = 0;
volatile uint32_t hidTaskLastCore = 255;

volatile bool clearKeyboardPending = false;

uint8_t senderMac[6] = {};
volatile bool senderMacKnown = false;
bool senderPeerAdded = false;

uint32_t lastDiagnosticsSent = 0;
uint32_t diagnosticsSendFailures = 0;

bool printedBadPacket = false;

// ============================================================
// Active receiver LED
// ============================================================

void setActiveLed(bool active) {
  if (!active) {
    pixel.setPixelColor(0, pixel.Color(0, 18, 0));
  } else {
    pixel.setPixelColor(0, 0);
  }
  pixel.show();
}

// ============================================================
// ESP-NOW receive callback
//
// Keep this extremely short. It only copies the packet into the
// FreeRTOS queue. No USB HID calls happen here.
// ============================================================

void onDataReceived(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {
  if (info != nullptr && !senderMacKnown) {
    memcpy(senderMac, info->src_addr, sizeof(senderMac));
    senderMacKnown = true;
  }

  receivedCount++;

  if (len != PACKET_SIZE) {
    badPacketCount++;

    if (!printedBadPacket) {
      printedBadPacket = true;
      Serial.printf("[BAD RX] Unexpected packet length: %d\n", len);
    }
    return;
  }

  RxItem item;
  item.length = (uint8_t)len;
  memcpy(item.data, data, PACKET_SIZE);

  // Never block inside the ESP-NOW callback.
  if (xQueueSend(rxQueue, &item, 0) != pdTRUE) {
    queueDropCount++;
    clearKeyboardPending = true;
  }
}

// ============================================================
// Release all keyboard keys
// ============================================================

void sendNeutralKeyboardReport() {
  KeyReport report = {};
  keyboard.sendReport(&report);
}

// ============================================================
// Decode signed 12-bit HID value
// ============================================================

int16_t signExtend12(uint16_t value) {
  value &= 0x0FFF;
  if (value & 0x0800) {
    value |= 0xF000;
  }
  return (int16_t)value;
}

// ============================================================
// Process M720 mouse report
//
// M720 report ID 2:
// Byte 0 = button bits 0-7
// Byte 1 = button bits 8-15
// Byte 2 = X bits 0-7
// Byte 3 = X bits 8-11 + Y bits 0-3
// Byte 4 = Y bits 4-11
// Byte 5 = vertical wheel
// Byte 6 = horizontal wheel
// ============================================================

void processMouseReport(
  const uint8_t *payload,
  uint8_t reportLength
) {
  if (reportLength != 7) {
    badPacketCount++;
    Serial.printf("[BAD MOUSE] Expected length 7, got %u\n", reportLength);
    return;
  }

  uint8_t buttons = payload[0] & MOUSE_ALL;

  uint16_t rawX =
    ((uint16_t)payload[2]) |
    (((uint16_t)payload[3] & 0x0F) << 8);

  int16_t x = signExtend12(rawX);

  uint16_t rawY =
    (((uint16_t)payload[3] >> 4) & 0x0F) |
    ((uint16_t)payload[4] << 4);

  int16_t y = signExtend12(rawY);

  int8_t wheel = (int8_t)payload[5];
  int8_t pan = (int8_t)payload[6];


  // Keep the full decoded movement instead of clamping and losing
  // everything beyond the USB mouse API's signed 8-bit range.
  static uint8_t previousButtons = 0;

  if (buttons != previousButtons) {
    mouse.buttons(buttons);
    previousButtons = buttons;
  }

  bool firstChunk = true;

  while (x != 0 || y != 0) {
    int8_t dx = (x > 127) ? 127 : (x < -127) ? -127 : (int8_t)x;
    int8_t dy = (y > 127) ? 127 : (y < -127) ? -127 : (int8_t)y;

    mouse.move(
      dx,
      dy,
      firstChunk ? wheel : 0,
      firstChunk ? pan : 0
    );

    x -= dx;
    y -= dy;
    firstChunk = false;
  }

  // Still send scroll-only reports when there is no X/Y movement.
  if (firstChunk && (wheel != 0 || pan != 0)) {
    mouse.move(0, 0, wheel, pan);
  }

  mouseCount++;
}

// ============================================================
// Process one packet
//
// IMPORTANT: this function is now called only by hidTask(), not
// by Arduino loop() and not by the ESP-NOW callback.
// ============================================================

void processPacket(const RxItem &item) {
  if (item.length != PACKET_SIZE) {
    badPacketCount++;
    return;
  }

  const uint8_t *p = item.data;

  uint8_t version = p[0];
  uint8_t type = p[1];
  uint8_t reportId = p[2];
  uint8_t reportLength = p[3];

  if (version != PROTOCOL_VERSION) {
    badPacketCount++;

    if (!printedBadPacket) {
      printedBadPacket = true;
      Serial.printf(
        "[BAD RX] version=%u type=%u id=%u length=%u\n",
        version, type, reportId, reportLength
      );
    }
    return;
  }

  if (reportLength > MAX_REPORT_SIZE) {
    badPacketCount++;

    if (!printedBadPacket) {
      printedBadPacket = true;
      Serial.printf(
        "[BAD RX] version=%u type=%u id=%u length=%u\n",
        version, type, reportId, reportLength
      );
    }
    return;
  }

  const uint8_t *payload = p + HEADER_SIZE;

  if (type == PACKET_HID_REPORT) {
    if (reportId == REPORT_ID_KEYBOARD && reportLength == 8) {
      KeyReport report = {};
      report.modifiers = payload[0];
      report.reserved = payload[1];

      memcpy(report.keys, payload + 2, 6);

      keyboard.sendReport(&report);
      keyboardCount++;
      return;
    }

    if (reportId == REPORT_ID_MOUSE && reportLength == 7) {
      processMouseReport(payload, reportLength);
      return;
    }

    badPacketCount++;

    if (!printedBadPacket) {
      printedBadPacket = true;

      Serial.printf(
        "[BAD HID] type=%u id=%u length=%u\n",
        type, reportId, reportLength
      );

      Serial.print("[BAD HID] data:");
      for (uint8_t i = 0; i < reportLength; i++) {
        Serial.printf(" %02X", payload[i]);
      }
      Serial.println();
    }

    return;
  }

  if (type == PACKET_SWITCH_RECEIVER) {
    if (reportLength >= 1) {
      uint8_t receiverId = payload[0];
      setActiveLed(receiverId == RECEIVER_ID);
    }

    switchCount++;
    return;
  }

  // Diagnostics packets should normally be sender -> receiver only
  // if a future protocol change needs them. Ignore them here.
  if (type == PACKET_DIAGNOSTICS) {
    return;
  }

  badPacketCount++;

  if (!printedBadPacket) {
    printedBadPacket = true;

    Serial.printf(
      "[BAD TYPE] version=%u type=%u id=%u length=%u\n",
      version, type, reportId, reportLength
    );
  }
}

// ============================================================
// Dedicated USB HID worker
//
// The old receiver drained rxQueue from loop(). This task moves
// the actual keyboard/mouse USB work out of loop(), so Arduino's
// loop task is no longer responsible for HID report timing.
//
// It is pinned to core 0 for this experiment. loop() remains on
// Arduino's normal core. The exact system-task placement is left
// to the Arduino/ESP-IDF runtime.
// ============================================================

void hidTask(void *parameter) {
  (void)parameter;

  Serial.printf(
    "[HID TASK] started on core %d, priority %u\n",
    xPortGetCoreID(),
    (unsigned)uxTaskPriorityGet(nullptr)
  );

  for (;;) {
    RxItem item;

    // Block waiting for work instead of polling.
    if (xQueueReceive(rxQueue, &item, pdMS_TO_TICKS(100)) == pdTRUE) {
      uint32_t startUs = micros();

      uint32_t depth = uxQueueMessagesWaiting(rxQueue);
      uint32_t oldMax = hidTaskMaxQueueDepth;
      if (depth > oldMax) {
        hidTaskMaxQueueDepth = depth;
      }

      processPacket(item);

      uint32_t elapsedUs = micros() - startUs;
      uint32_t oldRun = hidTaskMaxRunUs;
      if (elapsedUs > oldRun) {
        hidTaskMaxRunUs = elapsedUs;
      }

      hidTaskPackets++;
      hidTaskLastCore = xPortGetCoreID();
    }

    // If the callback had to drop a packet, release the keyboard
    // before continuing with normal queued reports.
    if (clearKeyboardPending) {
      clearKeyboardPending = false;
      sendNeutralKeyboardReport();
    }
  }
}

// ============================================================
// Reverse diagnostics link (receiver -> sender)
// ============================================================

void ensureSenderPeer() {
  if (!senderMacKnown || senderPeerAdded) return;

  if (esp_now_is_peer_exist(senderMac)) {
    senderPeerAdded = true;
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, senderMac, sizeof(senderMac));
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  esp_err_t err = esp_now_add_peer(&peerInfo);

  if (err == ESP_OK) {
    senderPeerAdded = true;
    Serial.println("[DIAG] Sender peer registered");
  } else {
    Serial.printf("[DIAG] Adding sender peer failed: %d\n", (int)err);
  }
}

void sendDiagnostics() {
  if (!senderMacKnown || !senderPeerAdded) return;

  HidPacket packet = {};
  packet.version = PROTOCOL_VERSION;
  packet.type = PACKET_DIAGNOSTICS;
  packet.reportLength = 32;

  // 8 counters = 32 bytes.
  const uint32_t counters[8] = {
    (uint32_t)receivedCount,
    (uint32_t)keyboardCount,
    (uint32_t)mouseCount,
    (uint32_t)badPacketCount,
    (uint32_t)queueDropCount,
    (uint32_t)switchCount,
    (uint32_t)hidTaskMaxQueueDepth,
    (uint32_t)hidTaskMaxRunUs
  };

  memcpy(packet.data, counters, sizeof(counters));

  esp_err_t err = esp_now_send(
    senderMac,
    reinterpret_cast<const uint8_t*>(&packet),
    sizeof(packet)
  );

  if (err != ESP_OK) {
    diagnosticsSendFailures++;
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println(" ESP-NOW USB HID KVM Receiver");
  Serial.println(" DEDICATED HID TASK TEST");
  Serial.println("==============================");

#if RECEIVER_ID == 1
  USB.productName("KVM Receiver 1");
#elif RECEIVER_ID == 2
  USB.productName("KVM Receiver 2");
#else
  USB.productName("KVM Receiver");
#endif

  // ==========================================================
  // IMPORTANT:
  //
  // Preserve the working initialization order.
  // ==========================================================

  keyboard.begin();
  mouse.begin();
  USB.begin();

  // ==========================================================
  // LED
  // ==========================================================

  pixel.begin();
  pixel.setBrightness(20);
  setActiveLed(false);

  // ==========================================================
  // RX queue
  // ==========================================================

  rxQueue = xQueueCreate(
    RX_QUEUE_DEPTH,
    sizeof(RxItem)
  );

  if (rxQueue == nullptr) {
    Serial.println("ERROR: Could not create RX queue");

    while (true) {
      delay(1000);
    }
  }

  // ==========================================================
  // Wi-Fi
  // ==========================================================

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_err_t err = esp_wifi_set_channel(
    WIFI_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  if (err != ESP_OK) {
    Serial.printf("Channel setup failed: %d\n", err);
  }

  Serial.print("Wi-Fi MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.printf("Wi-Fi channel: %u\n", WIFI_CHANNEL);

  // ==========================================================
  // ESP-NOW
  // ==========================================================

  err = esp_now_init();

  if (err != ESP_OK) {
    Serial.printf("ESP-NOW init failed: %d\n", err);

    while (true) {
      delay(1000);
    }
  }

  err = esp_now_register_recv_cb(onDataReceived);

  if (err != ESP_OK) {
    Serial.printf("RX callback registration failed: %d\n", err);

    while (true) {
      delay(1000);
    }
  }

  // ==========================================================
  // Dedicated HID task
  // ==========================================================

  BaseType_t taskResult = xTaskCreatePinnedToCore(
    hidTask,
    "HIDTask",
    HID_TASK_STACK,
    nullptr,
    HID_TASK_PRIORITY,
    &hidTaskHandle,
    HID_TASK_CORE
  );

  if (taskResult != pdPASS) {
    Serial.println("ERROR: Could not create HID task");

    while (true) {
      delay(1000);
    }
  }

  Serial.println("USB HID keyboard initialized");
  Serial.println("USB HID mouse initialized");
  Serial.println("ESP-NOW initialized");
  Serial.printf("Receiver ID: %d\n", RECEIVER_ID);
  Serial.printf("RX queue depth: %u\n", RX_QUEUE_DEPTH);
  Serial.printf("HID task core: %d\n", HID_TASK_CORE);
  Serial.println("Waiting for keyboard and mouse reports...");
}

// ============================================================
// LOOP
//
// Deliberately kept light. HID report processing is no longer
// performed here.
// ============================================================

void loop() {
  static uint32_t lastStatus = 0;

  uint32_t now = millis();

  if (now - lastStatus >= 5000) {
    lastStatus = now;

    Serial.printf(
      "[STATUS] core=%d "
      "rx=%lu "
      "keyboard=%lu "
      "mouse=%lu "
      "switches=%lu "
      "bad=%lu "
      "queue_drops=%lu "
      "hid_task=%lu "
      "max_q=%lu "
      "max_hid_us=%lu\n",

      xPortGetCoreID(),

      (unsigned long)receivedCount,
      (unsigned long)keyboardCount,
      (unsigned long)mouseCount,
      (unsigned long)switchCount,
      (unsigned long)badPacketCount,
      (unsigned long)queueDropCount,
      (unsigned long)hidTaskPackets,
      (unsigned long)hidTaskMaxQueueDepth,
      (unsigned long)hidTaskMaxRunUs
    );
  }

  ensureSenderPeer();

  if (now - lastDiagnosticsSent >= DIAGNOSTICS_INTERVAL_MS) {
    lastDiagnosticsSent = now;
    sendDiagnostics();
  }

  // No HID work here.
  delay(5);
}
