#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <driver/twai.h>

// Simulator option: Uncomment to simulate RPM, comment to use CAN bus
//#define SIMULATE_RPM

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

// === Added for CAN bus debugging ===
unsigned long lastDebugPrint = 0;
const unsigned long debugInterval = 1000; // Debug output every 1s
uint32_t rxMsgCount = 0; // Count of received messages
uint32_t errorCount = 0; // Count of CAN errors
bool canConnected = false; // CAN bus connection status
// ================================

// ESP-Now callback function
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("Last Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  canConnected = true; // Set initial CAN connection status
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
  // Check CAN bus status
  twai_status_info_t status;
  twai_get_status_info(&status);
  
  // Check for bus-off state and attempt recovery
  if (status.state == TWAI_STATE_BUS_OFF) {
    if (canConnected) {
      Serial.println("CAN Bus-off detected, attempting recovery");
      canConnected = false;
      twai_initiate_recovery();
    }
  } else if (status.state == TWAI_STATE_STOPPED) {
    if (canConnected) {
      Serial.println("CAN Bus stopped, restarting");
      canConnected = false;
      twai_start();
    }
  } else if (status.state == TWAI_STATE_RUNNING) {
    canConnected = true;
  }

  // Check for TWAI message
  twai_message_t message;
  esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(10));
  if (result == ESP_OK) {
    rxMsgCount++;
    if (message.identifier == 0x316 && message.data_length_code >= 4) {
      // Extract RPM (little-endian, unsigned, offset 2, length 2)
      uint16_t rawRpm = (message.data[3] << 8) | message.data[2]; // Little-endian: data[2] is LSB, data[3] is MSB
      latestRpm = rawRpm / 6.4; // Apply new RPM formula: value / 6.4
      // Debug: Print received message details
      Serial.printf("CAN Msg: ID=0x%X, DLC=%d, Data=[%02X %02X %02X %02X], Raw RPM=%d, Scaled RPM=%d\n",
                    message.identifier, message.data_length_code,
                    message.data[0], message.data[1], message.data[2], message.data[3], rawRpm, latestRpm);
    }
  } else if (result != ESP_ERR_TIMEOUT) {
    errorCount++;
    Serial.printf("TWAI receive error: 0x%X\n", result);
  }

  // Periodic debug output
  if (currentTime - lastDebugPrint >= debugInterval) {
    Serial.printf("CAN Status: %s, Rx Count: %lu, Errors: %lu, Bus Errors: %lu,  Rx Missed: %lu\n",
                  canConnected ? "Connected" : "Disconnected",
                  rxMsgCount, errorCount, status.bus_error_count, status.rx_missed_count);
    lastDebugPrint = currentTime;
  }
#endif

  // Send RPM at 10 Hz (every 100ms)
  if (currentTime - lastSendTime >= sendInterval) {
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&latestRpm, sizeof(latestRpm));
    if (result == ESP_OK) {
      // Serial.print("Sent RPM: ");
      // Serial.println(latestRpm);
    } else {
      Serial.println("Error sending the data");
    }
    lastSendTime = currentTime;
  }
}