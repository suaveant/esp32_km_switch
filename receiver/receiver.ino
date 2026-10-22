#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <USB.h>
#include <USBHIDKeyboard.h>
#include <USBHIDMouse.h>
#include <cstring>
#include <Adafruit_NeoPixel.h>

#define RECEIVER_ID 1  // Change to 2 on the second receiver

Adafruit_NeoPixel pixel(1, 48, NEO_GRB + NEO_KHZ800);

// ============================================================
// ESP-NOW protocol
// MUST match sender/sender.ino
// ============================================================

static constexpr uint8_t PROTOCOL_VERSION = 1;

static constexpr uint8_t PACKET_HID_REPORT = 1;
static constexpr uint8_t PACKET_SWITCH_RECEIVER = 2;

static constexpr uint8_t REPORT_ID_KEYBOARD = 1;
static constexpr uint8_t REPORT_ID_MOUSE = 2;

static constexpr uint8_t WIFI_CHANNEL = 6;

static constexpr size_t PACKET_SIZE = 36;
static constexpr size_t HEADER_SIZE = 4;
static constexpr size_t MAX_REPORT_SIZE = 32;

static constexpr uint8_t RX_QUEUE_DEPTH = 32;

struct __attribute__((packed)) HidPacket {
  uint8_t version;
  uint8_t type;
  uint8_t reportId;
  uint8_t reportLength;
  uint8_t data[MAX_REPORT_SIZE];
};

static_assert(
  sizeof(HidPacket) == PACKET_SIZE,
  "Unexpected HidPacket size"
);

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

// ============================================================
// Statistics
// ============================================================

volatile uint32_t receivedCount = 0;
volatile uint32_t badPacketCount = 0;
volatile uint32_t queueDropCount = 0;

volatile uint32_t keyboardCount = 0;
volatile uint32_t mouseCount = 0;
volatile uint32_t switchCount = 0;

volatile bool clearKeyboardPending = false;

// ============================================================
// Debug
// ============================================================

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
// ============================================================

void onDataReceived(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len) {

  (void)info;

  receivedCount++;

  if (len != PACKET_SIZE) {
    badPacketCount++;

    if (!printedBadPacket) {
      printedBadPacket = true;

      Serial.printf(
        "[BAD RX] Unexpected packet length: %d\n",
        len
      );
    }

    return;
  }

  RxItem item;

  item.length = (uint8_t)len;

  memcpy(
    item.data,
    data,
    PACKET_SIZE
  );

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
//
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
  uint8_t reportLength) {

  if (reportLength != 7) {

    badPacketCount++;

    Serial.printf(
      "[BAD MOUSE] Expected length 7, got %u\n",
      reportLength
    );

    return;
  }

  // ----------------------------------------------------------
  // Buttons
  // ----------------------------------------------------------

  uint8_t buttons =
    payload[0] & MOUSE_ALL;

  // ----------------------------------------------------------
  // X
  // ----------------------------------------------------------

  uint16_t rawX =
    ((uint16_t)payload[2]) |
    (((uint16_t)payload[3] & 0x0F) << 8);

  int16_t x = signExtend12(rawX);

  // ----------------------------------------------------------
  // Y
  // ----------------------------------------------------------

  uint16_t rawY =
    (((uint16_t)payload[3] >> 4) & 0x0F) |
    ((uint16_t)payload[4] << 4);

  int16_t y = signExtend12(rawY);

  // ----------------------------------------------------------
  // Wheel
  // ----------------------------------------------------------

  int8_t wheel =
    (int8_t)payload[5];

  // ----------------------------------------------------------
  // Horizontal wheel
  // ----------------------------------------------------------

  int8_t pan =
    (int8_t)payload[6];

  // ----------------------------------------------------------
  // USB HID mouse uses int8_t relative movement.
  //
  // Clamp the M720's 12-bit movement values to the USB
  // mouse report range.
  // ----------------------------------------------------------

  int8_t dx;

  int8_t dy;

  if (x > 127) {
    dx = 127;
  } else if (x < -127) {
    dx = -127;
  } else {
    dx = (int8_t)x;
  }

  if (y > 127) {
    dy = 127;
  } else if (y < -127) {
    dy = -127;
  } else {
    dy = (int8_t)y;
  }

  // ----------------------------------------------------------
  // Keep USB button state synchronized with the M720.
  //
  // USBHIDMouse maintains the button state internally.
  // buttons() sends a report when the state changes.
  // ----------------------------------------------------------

  static uint8_t previousButtons = 0;

  if (buttons != previousButtons) {

    mouse.buttons(buttons);

    previousButtons = buttons;
  }

  // ----------------------------------------------------------
  // Send movement/wheel/pan.
  //
  // This report includes the current button state.
  // ----------------------------------------------------------

  mouse.move(
    dx,
    dy,
    wheel,
    pan
  );

  mouseCount++;
}

// ============================================================
// Process one packet
// ============================================================

void processPacket(
  const RxItem &item) {

  if (item.length != PACKET_SIZE) {

    badPacketCount++;

    return;
  }

  const uint8_t *p =
    item.data;

  uint8_t version =
    p[0];

  uint8_t type =
    p[1];

  uint8_t reportId =
    p[2];

  uint8_t reportLength =
    p[3];

  // ----------------------------------------------------------
  // Validate common packet header
  // ----------------------------------------------------------

  if (version != PROTOCOL_VERSION) {

    badPacketCount++;

    if (!printedBadPacket) {

      printedBadPacket = true;

      Serial.printf(
        "[BAD RX] version=%u type=%u id=%u length=%u\n",
        version,
        type,
        reportId,
        reportLength
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
        version,
        type,
        reportId,
        reportLength
      );
    }

    return;
  }

  const uint8_t *payload =
    p + HEADER_SIZE;

  // ==========================================================
  // HID REPORT
  // ==========================================================

  if (type == PACKET_HID_REPORT) {

    // --------------------------------------------------------
    // Keyboard
    // --------------------------------------------------------

    if (
      reportId == REPORT_ID_KEYBOARD &&
      reportLength == 8
    ) {

      KeyReport report = {};

      report.modifiers =
        payload[0];

      report.reserved =
        payload[1];

      memcpy(
        report.keys,
        payload + 2,
        6
      );

      keyboard.sendReport(
        &report
      );

      keyboardCount++;

      return;
    }

    // --------------------------------------------------------
    // Mouse
    // --------------------------------------------------------

    if (
      reportId == REPORT_ID_MOUSE &&
      reportLength == 7
    ) {

      processMouseReport(
        payload,
        reportLength
      );

      return;
    }

    // --------------------------------------------------------
    // Unknown HID report
    // --------------------------------------------------------

    badPacketCount++;

    if (!printedBadPacket) {

      printedBadPacket = true;

      Serial.printf(
        "[BAD HID] type=%u id=%u length=%u\n",
        type,
        reportId,
        reportLength
      );

      Serial.print("[BAD HID] data:");

      for (uint8_t i = 0; i < reportLength; i++) {
        Serial.printf(
          " %02X",
          payload[i]
        );
      }

      Serial.println();
    }

    return;
  }

  // ==========================================================
  // RECEIVER SWITCH
  // ==========================================================

  if (type == PACKET_SWITCH_RECEIVER) {

    if (reportLength >= 1) {

      uint8_t receiverId =
        payload[0];

      setActiveLed(
        receiverId == RECEIVER_ID
      );
    }

    switchCount++;

    return;
  }

  // ==========================================================
  // UNKNOWN PACKET TYPE
  // ==========================================================

  badPacketCount++;

  if (!printedBadPacket) {

    printedBadPacket = true;

    Serial.printf(
      "[BAD TYPE] version=%u type=%u id=%u length=%u\n",
      version,
      type,
      reportId,
      reportLength
    );
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
  Serial.println("==============================");

  // ==========================================================
  // USB product name
  // ==========================================================

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

  rxQueue =
    xQueueCreate(
      RX_QUEUE_DEPTH,
      sizeof(RxItem)
    );

  if (rxQueue == nullptr) {

    Serial.println(
      "ERROR: Could not create RX queue"
    );

    while (true) {
      delay(1000);
    }
  }

  // ==========================================================
  // Wi-Fi
  // ==========================================================

  WiFi.mode(WIFI_STA);

  WiFi.disconnect();

  esp_err_t err =
    esp_wifi_set_channel(
      WIFI_CHANNEL,
      WIFI_SECOND_CHAN_NONE
    );

  if (err != ESP_OK) {

    Serial.printf(
      "Channel setup failed: %d\n",
      err
    );
  }

  Serial.print("Wi-Fi MAC: ");
  Serial.println(
    WiFi.macAddress()
  );

  Serial.printf(
    "Wi-Fi channel: %u\n",
    WIFI_CHANNEL
  );

  // ==========================================================
  // ESP-NOW
  // ==========================================================

  err = esp_now_init();

  if (err != ESP_OK) {

    Serial.printf(
      "ESP-NOW init failed: %d\n",
      err
    );

    while (true) {
      delay(1000);
    }
  }

  err =
    esp_now_register_recv_cb(
      onDataReceived
    );

  if (err != ESP_OK) {

    Serial.printf(
      "RX callback registration failed: %d\n",
      err
    );

    while (true) {
      delay(1000);
    }
  }

  Serial.println(
    "USB HID keyboard initialized"
  );

  Serial.println(
    "USB HID mouse initialized"
  );

  Serial.println(
    "ESP-NOW initialized"
  );

  Serial.printf(
    "Receiver ID: %d\n",
    RECEIVER_ID
  );

  Serial.println(
    "Waiting for keyboard and mouse reports..."
  );
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  // ==========================================================
  // Clear keyboard if queue overflow occurred
  // ==========================================================

  if (clearKeyboardPending) {

    clearKeyboardPending = false;

    sendNeutralKeyboardReport();
  }

  // ==========================================================
  // Drain RX queue
  // ==========================================================

  RxItem item;

  while (
    xQueueReceive(
      rxQueue,
      &item,
      0
    ) == pdTRUE
  ) {

    processPacket(item);
  }

  // ==========================================================
  // Status
  // ==========================================================

  static uint32_t lastStatus = 0;

  uint32_t now =
    millis();

  if (
    now - lastStatus >= 5000
  ) {

    lastStatus = now;

    Serial.printf(
      "[STATUS] rx=%lu "
      "keyboard=%lu "
      "mouse=%lu "
      "switches=%lu "
      "bad=%lu "
      "queue_drops=%lu\n",

      (unsigned long)receivedCount,
      (unsigned long)keyboardCount,
      (unsigned long)mouseCount,
      (unsigned long)switchCount,
      (unsigned long)badPacketCount,
      (unsigned long)queueDropCount
    );
  }

  delay(1);
}