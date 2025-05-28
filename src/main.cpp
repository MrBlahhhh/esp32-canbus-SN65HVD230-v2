#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <driver/twai.h>

// Simulator option: Uncomment to simulate RPM, comment to use CAN bus
#define SIMULATE_RPM

// Replace with the receiver ESP32's MAC address
uint8_t receiverMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Update with actual MAC

// TWAI pin definitions using gpio_num_t
#define TWAI_TX_PIN GPIO_NUM_21
#define TWAI_RX_PIN GPIO_NUM_22

// Wi-Fi channel for ESP-Now
#define WIFI_CHANNEL 1

// Timing variables for 10 Hz (100ms interval)
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100; // 100ms = 10 Hz

// Variable to store the latest RPM
uint16_t latestRpm = 0;

// ESP-Now callback function
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for Serial to initialize

#ifndef SIMULATE_RPM
  // Initialize TWAI (CAN) if not simulating
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install TWAI driver");
    while (1);
  }
  
  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start TWAI");
    while (1);
  }
  Serial.println("TWAI initialized");
#else
  Serial.println("RPM simulation enabled");
#endif

  // Initialize Wi-Fi in station mode
  WiFi.mode(WIFI_STA);
  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("Failed to set Wi-Fi channel");
    while (1);
  }
  delay(100); // Small delay to ensure Wi-Fi stability
  Serial.print("Sender MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());

  // Initialize ESP-Now
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-Now");
    while (1);
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Add peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA; // Explicitly set to station interface
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1);
  }
  Serial.println("ESP-Now peer added");
}

void loop() {
  // Declare currentTime once at the start of loop
  unsigned long currentTime = millis();

#ifdef SIMULATE_RPM
  // Simulate RPM: 1000 to 9000 over 10s, then 9000 to 1000 over 10s (20s cycle)
  float cycleTime = (currentTime % 20000) / 1000.0; // Time in seconds (0 to 20s)
  if (cycleTime <= 10.0) {
    // Ramp-up: 1000 to 9000
    latestRpm = 1000 + (uint16_t)(8000 * cycleTime / 10.0);
  } else {
    // Ramp-down: 9000 to 1000
    latestRpm = 9000 - (uint16_t)(8000 * (cycleTime - 10.0) / 10.0);
  }
#else
  // Check for TWAI message
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    if (message.identifier == 0x790 && message.data_length_code >= 4) {
      // Extract RPM (little-endian, unsigned, offset 2, length 2)
      latestRpm = (message.data[3] << 8) | message.data[2]; // Little-endian: data[2] is LSB, data[3] is MSB
    }
  }
#endif

  // Send RPM at 10 Hz (every 100ms)
  if (currentTime - lastSendTime >= sendInterval) {
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&latestRpm, sizeof(latestRpm));
    if (result == ESP_OK) {
      Serial.print("Sent RPM: ");
      Serial.println(latestRpm);
    } else {
      Serial.println("Error sending the data");
    }
    lastSendTime = currentTime;
  }
}