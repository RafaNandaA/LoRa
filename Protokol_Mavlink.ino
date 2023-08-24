// Library MAVLink digunakan untuk mengenal protokol data
// Koreksi kesalahan data, konversi satuan, perhitungan sesuai standar, dll.

#include "c_library_v2-master/ardupilotmega/mavlink.h"
#include <HardwareSerial.h>

#define PIN_LED 13 // Indikator LED

HardwareSerial Serial2(USART2);   

class MavlinkBridge {
public:
  MavlinkBridge() : freq_debug(3), freq_sr1_to_sr2(200), freq_sr2_to_sr1(200) {}

  void setupbridge() {
    pinMode(PIN_LED, OUTPUT); // Set pin led sebagai output
    delay(500); // waktu delay LED 500ms
    Serial1.begin(57600); // Set komunikasi serial 1 pada baudrate 57600
    Serial2.begin(57600); // Set komunikasi serial 2 pada baudrate 57600
  }

void loopbridge() {
    if (millis() - last_debug >= 1000 / freq_debug) {
      last_debug = millis();
      digitalWrite(PIN_LED, led_state);
      led_state = !led_state;
    }

    handleSerial1Data();
    handleSerial2Data();
  }

private:
  uint16_t freq_debug;
  uint16_t freq_sr1_to_sr2;
  uint16_t freq_sr2_to_sr1;
  uint32_t last_debug = 0;
  uint32_t last_sr1_to_sr2 = 0;
  uint32_t last_sr2_to_sr1 = 0;
  uint8_t led_state = 1;
  uint8_t adr_mode = 3; // Simulasi pergantian mode 1, 2, atau 3

  void handleSerial1Data() {
    if (millis() - last_sr1_to_sr2 >= 1000 / freq_sr1_to_sr2) {
      last_sr1_to_sr2 = millis();
      uint16_t sr1_av = Serial1.available();
      uint8_t sr1_buff[300];
      uint8_t send_buff[300];
      mavlink_message_t msg; // kontainer tempat pesan baru akan masuk
      mavlink_status_t status;
      if (sr1_av > 0) {
        sr1_av = Serial1.readBytesUntil(0xFE, sr1_buff, sizeof(sr1_buff));
        for (int x = 0; x < sr1_av; x++) {
          if (mavlink_parse_char(MAVLINK_COMM_0, sr1_buff[x], &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
              if (adr_mode == 3) {
                uint16_t len = mavlink_msg_to_send_buffer(send_buff, &msg);
                Serial2.write(send_buff, len);
              }
            } else {
              uint16_t len1 = mavlink_msg_to_send_buffer(send_buff, &msg);
              Serial2.write(send_buff, len1);
            }
          }
        }
      }
    }
  }

  void handleSerial2Data() {
  if (millis() - last_sr2_to_sr1 >= 1000 / freq_sr2_to_sr1) {
    last_sr2_to_sr1 = millis();
    uint16_t sr2_av = Serial2.available();
    uint8_t sr2_buff[300];
    if (sr2_av > 0) {
      Serial2.readBytesUntil(0xFE, sr2_buff, sr2_av);
      Serial1.write(sr2_buff, sr2_av);
      }
    }
  }
};

MavlinkBridge bridge;

void setup() {
  bridge.setupbridge();
}

void loop() {
  bridge.loopbridge();
}
