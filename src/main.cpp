#include <esp_now.h>
#include <WiFi.h>
#include <driver/twai.h>

// Replace with the receiver ESP32's MAC address
uint8_t receiverMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Update with actual MAC

// TWAI pin definitions using gpio_num_t
#define TWAI_TX_PIN GPIO_NUM_21
#define TWAI_RX_PIN GPIO_NUM_22

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

  // Initialize TWAI (CAN)
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

  // Initialize ESP-Now
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-Now");
    while (1);
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Add peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1);
  }
}

void loop() {
  // Check for TWAI message
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    if (message.identifier == 0x790 && message.data_length_code >= 4) {
      // Extract RPM (little-endian, unsigned, offset 2, length 2)
      latestRpm = (message.data[3] << 8) | message.data[2]; // Little-endian: data[2] is LSB, data[3] is MSB
    }
  }

  // Send RPM at 10 Hz (every 100ms)
  unsigned long currentTime = millis();
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