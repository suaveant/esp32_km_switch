
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <USB.h>
#include <USBHIDKeyboard.h>
#include <cstring>
#include <Adafruit_NeoPixel.h>

#define RECEIVER_ID 1  // Change to 2 on the second receiver

Adafruit_NeoPixel pixel(1, 48, NEO_GRB + NEO_KHZ800);

// ESP-NOW protocol shared with the sender
static constexpr uint8_t PROTOCOL_VERSION = 1;
static constexpr uint8_t PACKET_HID_REPORT = 1;
static constexpr uint8_t PACKET_SWITCH_RECEIVER = 2;
static constexpr uint8_t REPORT_ID_KEYBOARD = 1;

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

USBHIDKeyboard keyboard;
QueueHandle_t rxQueue = nullptr;

volatile uint32_t receivedCount = 0;
volatile uint32_t badPacketCount = 0;
volatile uint32_t queueDropCount = 0;
volatile uint32_t keyboardCount = 0;
volatile uint32_t switchCount = 0;

volatile bool clearKeyboardPending = false;

void setActiveLed(bool active) {
  if (!active) {
    pixel.setPixelColor(0, pixel.Color(0, 18, 0));
  } else {
    pixel.setPixelColor(0, 0);
  }
  pixel.show();
}

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
    clearKeyboardPending = true;
  }
}

void sendNeutralKeyboardReport() {
  KeyReport report = {};
  keyboard.sendReport(&report);
}

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

  if (version != PROTOCOL_VERSION || reportLength > MAX_REPORT_SIZE) {
    badPacketCount++;
    return;
  }

  const uint8_t *payload = p + HEADER_SIZE;

  if (type == PACKET_HID_REPORT) {
    if (reportId != REPORT_ID_KEYBOARD || reportLength != 8) {
      badPacketCount++;
      return;
    }

    // Incoming boot keyboard payload:
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

  if (type == PACKET_SWITCH_RECEIVER) {
    if (reportLength >= 1) {
      setActiveLed(payload[0] == RECEIVER_ID);
    }
    switchCount++;
    return;
  }

  badPacketCount++;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println(" ESP-NOW USB HID Receiver");
  Serial.println("==============================");

  // Native USB HID keyboard.
  keyboard.begin();
  USB.begin();

  pixel.begin();
  pixel.setBrightness(20);
  setActiveLed(false);

  rxQueue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(RxItem));

  if (rxQueue == nullptr) {
    Serial.println("ERROR: Could not create RX queue");
    while (true) {
      delay(1000);
    }
  }

  // ESP-NOW operates on the same channel as the sender.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_err_t err =
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (err != ESP_OK) {
    Serial.printf("Channel setup failed: %d\n", err);
  }

  Serial.print("Wi-Fi MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Wi-Fi channel: %u\n", WIFI_CHANNEL);

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

  Serial.println("USB HID and ESP-NOW initialized");
  Serial.println("Waiting for keyboard reports...");
}

void loop() {
  // If the queue overflowed, clear any potentially stuck keys.
  if (clearKeyboardPending) {
    clearKeyboardPending = false;
    sendNeutralKeyboardReport();
  }

  RxItem item;

  // Drain queued reports as quickly as possible.
  while (xQueueReceive(rxQueue, &item, 0) == pdTRUE) {
    processPacket(item);
  }

  static uint32_t lastStatus = 0;
  uint32_t now = millis();

  if (now - lastStatus >= 5000) {
    lastStatus = now;

    Serial.printf(
      "[STATUS] rx=%lu keyboard=%lu switches=%lu "
      "bad=%lu queue_drops=%lu\n",
      (unsigned long)receivedCount,
      (unsigned long)keyboardCount,
      (unsigned long)switchCount,
      (unsigned long)badPacketCount,
      (unsigned long)queueDropCount);
  }

  delay(1);
}
