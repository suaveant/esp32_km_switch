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
// ESP-NOW protocol shared with the sender
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

static_assert(sizeof(HidPacket) == PACKET_SIZE,
              "Unexpected HidPacket size");

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
    return;
  }

  RxItem item;

  item.length = (uint8_t)len;

  memcpy(item.data, data, PACKET_SIZE);

  // Never wait for the queue inside the Wi-Fi callback.
  if (xQueueSend(rxQueue, &item, 0) != pdTRUE) {
    queueDropCount++;

    // Make sure any held keyboard keys are released if
    // the queue overflows.
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
// Sign-extend a 12-bit signed HID value
// ============================================================

int16_t signExtend12(uint16_t value) {
  value &= 0x0FFF;

  if (value & 0x0800) {
    value |= 0xF000;
  }

  return (int16_t)value;
}

// ============================================================
// Decode Logitech M720 7-byte mouse report
//
// Byte 0:
//   Button bits 0-7
//
// Byte 1:
//   Button bits 8-15
//
// Bytes 2-4:
//   X and Y are signed 12-bit values
//
// Byte 5:
//   Vertical wheel
//
// Byte 6:
//   Horizontal wheel / pan
//
// Layout:
//
//   byte 2       = X bits 0-7
//   byte 3 low   = X bits 8-11
//   byte 3 high  = Y bits 0-3
//   byte 4       = Y bits 4-11
// ============================================================

void processMouseReport(const uint8_t *payload, uint8_t reportLength) {

  if (reportLength != 7) {
    badPacketCount++;
    return;
  }

  // ----------------------------------------------------------
  // Buttons
  //
  // USBHIDMouse supports the standard five mouse buttons:
  //
  // bit 0 = left
  // bit 1 = right
  // bit 2 = middle
  // bit 3 = back
  // bit 4 = forward
  //
  // The M720 exposes more button bits in its report, but the
  // standard Arduino USB mouse descriptor has five buttons.
  // ----------------------------------------------------------

  uint8_t buttons = payload[0] & 0x1F;

  // ----------------------------------------------------------
  // X
  // ----------------------------------------------------------

  uint16_t rawX =
    ((uint16_t)payload[2]) |
    (((uint16_t)payload[3] & 0x0F) << 8);

  // ----------------------------------------------------------
  // Y
  // ----------------------------------------------------------

  uint16_t rawY =
    (((uint16_t)payload[3] >> 4) & 0x0F) |
    ((uint16_t)payload[4] << 4);

  int16_t x = signExtend12(rawX);
  int16_t y = signExtend12(rawY);

  // The Arduino relative mouse API uses int8_t movement.
  //
  // Normal M720 reports fit easily in this range. If Logitech
  // ever produces a larger value, clamp it rather than
  // allowing an integer wrap.
  int8_t mouseX;

  int8_t mouseY;

  if (x > 127) {
    mouseX = 127;
  } else if (x < -127) {
    mouseX = -127;
  } else {
    mouseX = (int8_t)x;
  }

  if (y > 127) {
    mouseY = 127;
  } else if (y < -127) {
    mouseY = -127;
  } else {
    mouseY = (int8_t)y;
  }

  // ----------------------------------------------------------
  // Wheels
  // ----------------------------------------------------------

  int8_t wheel = (int8_t)payload[5];
  int8_t pan = (int8_t)payload[6];

  // ----------------------------------------------------------
  // Update the USB mouse state.
  //
  // buttons() generates a button report if the button state
  // changed. We then send movement/wheel in the same report.
  // To avoid sending a duplicate button-only report followed
  // immediately by movement, directly maintain the internal
  // state through the public API and send movement.
  // ----------------------------------------------------------

  // USBHIDMouse::buttons() is public, but calling it can emit
  // its own report. We need reliable button state even when
  // there is no movement, so use the standard API here.
  //
  // For movement reports, press/release state is maintained
  // internally by the mouse object.
  static uint8_t previousButtons = 0;

  if (buttons != previousButtons) {

    // Release buttons that are no longer pressed.
    uint8_t released = previousButtons & ~buttons;

    if (released) {
      mouse.release(released);
    }

    // Press newly pressed buttons.
    uint8_t pressed = buttons & ~previousButtons;

    if (pressed) {
      mouse.press(pressed);
    }

    previousButtons = buttons;
  }

  // ----------------------------------------------------------
  // Send movement / wheel / pan.
  //
  // This also carries the current button state.
  // ----------------------------------------------------------

  mouse.move(mouseX, mouseY, wheel, pan);

  mouseCount++;
}

// ============================================================
// Process one ESP-NOW packet
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

  if (version != PROTOCOL_VERSION ||
      reportLength > MAX_REPORT_SIZE) {

    badPacketCount++;
    return;
  }

  const uint8_t *payload = p + HEADER_SIZE;

  // ==========================================================
  // HID report
  // ==========================================================

  if (type == PACKET_HID_REPORT) {

    // --------------------------------------------------------
    // Keyboard
    // --------------------------------------------------------

    if (reportId == REPORT_ID_KEYBOARD) {

      if (reportLength != 8) {
        badPacketCount++;
        return;
      }

      // Incoming boot keyboard payload:
      //
      // byte 0 = modifier bits
      // byte 1 = reserved
      // bytes 2-7 = six HID key usages

      KeyReport report = {};

      report.modifiers = payload[0];
      report.reserved = payload[1];

      memcpy(report.keys, payload + 2, 6);

      keyboard.sendReport(&report);

      keyboardCount++;

      return;
    }

    // --------------------------------------------------------
    // Mouse
    // --------------------------------------------------------

    if (reportId == REPORT_ID_MOUSE) {

      if (reportLength != 7) {
        badPacketCount++;
        return;
      }

      processMouseReport(payload, reportLength);

      return;
    }

    // Unknown HID report ID.
    badPacketCount++;

    return;
  }

  // ==========================================================
  // Receiver switch packet
  // ==========================================================

  if (type == PACKET_SWITCH_RECEIVER) {

    if (reportLength >= 1) {
      setActiveLed(payload[0] == RECEIVER_ID);
    }

    switchCount++;

    return;
  }

  // Unknown packet type.
  badPacketCount++;
}

// ============================================================
// Setup
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println(" ESP-NOW USB HID KVM Receiver");
  Serial.println("==============================");

  // ==========================================================
  // USB device name
  //
  // Must be set BEFORE USB.begin().
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
  // Keep the existing initialization order.
  //
  // This was deliberately NOT changed from the working
  // receiver firmware.
  // ==========================================================

  keyboard.begin();
  mouse.begin();
  USB.begin();

  // ==========================================================
  // Status LED
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
  // ESP-NOW / Wi-Fi
  // ==========================================================

  WiFi.mode(WIFI_STA);

  WiFi.disconnect();

  // ESP-NOW operates on the same channel as the sender.

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
  Serial.println(WiFi.macAddress());

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

  err = esp_now_register_recv_cb(
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

  Serial.println("USB HID keyboard initialized");
  Serial.println("USB HID mouse initialized");
  Serial.println("ESP-NOW initialized");
  Serial.printf(
    "Receiver ID: %d\n",
    RECEIVER_ID
  );

  Serial.println(
    "Waiting for keyboard and mouse reports..."
  );
}

// ============================================================
// Main loop
// ============================================================

void loop() {

  // ==========================================================
  // If the queue overflowed, clear any potentially stuck keys.
  // ==========================================================

  if (clearKeyboardPending) {

    clearKeyboardPending = false;

    sendNeutralKeyboardReport();
  }

  // ==========================================================
  // Drain queued reports as quickly as possible.
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
  // Status output
  // ==========================================================

  static uint32_t lastStatus = 0;

  uint32_t now = millis();

  if (now - lastStatus >= 5000) {

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