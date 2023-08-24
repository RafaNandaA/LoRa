#include "fastmavlink-master/c_library/ardupilotmega/ardupilotmega.h"
#include <SPI.h> // The LoRa device is SPI based, so load the SPI library
#include <SX126XLT.h> // Include the appropriate library

SX126XLT LT; // Create a library class instance called LT

#define NSS PA15 // Select pin on LoRa device
#define NRESET PB13 // Reset pin on LoRa device
#define RFBUSY PB12 // SX126X busy pin
#define DIO1 PB10 // DIO1 pin on LoRa device, used for sensing RX and TX done
#define DIO2 -1 // DIO2 pin on LoRa device, not used in this configuration
#define DIO3 -1 // DIO3 pin on LoRa device, not used in this configuration
#define RXEN PB14 // Receiver enable pin on LoRa device
#define TXEN PB15 // Transmitter enable pin on LoRa device
#define SW -1 // SW pin on LoRa device, used to power antenna switch

//#define SERIAL_TX_BUFFER_SIZE 256
//#define SERIAL_RX_BUFFER_SIZE 256

#define LORA_DEVICE DEVICE_SX1268 // Define the LoRa device (DEVICE_SX1261, DEVICE_SX1262, or DEVICE_SX1268)
#define TXpower 10 // LoRa transmit power in dBm

// LED Indicators
#define LED1 PB0
#define LED2 PB1
#define LED3 PB2

//#define BLINK_PIN PB0
//#define BLINK_AP_PIN PB1
//#define BLINK_GCS_PIN PB2

#define SERIAL1 Serial1
HardwareSerial Serial2(USART2);
//uint8_t blink_led1 = 0;
//uint8_t blink_led2 = 0;
//uint8_t blink_led3 = 0;
//#define LED_TOGGLE {digitalWrite(BLINK_PIN, (blink) ? HIGH : LOW); blink = (blink) ? 0 : 1;}
//#define LED_TOGGLE_AP {digitalWrite(BLINK_AP_PIN, (blink_ap) ? HIGH : LOW); blink_ap = (blink_ap) ? 0 : 1;}
//#define LED_TOGGLE_GCS {digitalWrite(BLINK_GCS_PIN, (blink_gcs) ? HIGH : LOW); blink_gcs = (blink_gcs) ? 0 : 1;}
//#define LED1 {digitalWrite(LED1, (blink_led1) ? HIGH : LOW); blink_led1 = (blink_led1) ? 0 : 1;}
//#define LED2 {digitalWrite(LED2, (blink_led2) ? HIGH : LOW); blink_led2 = (blink_led2) ? 0 : 1;}
//#define LED3 {digitalWrite(LED3, (blink_led3) ? HIGH : LOW); blink_led3 = (blink_led3) ? 0 : 1;}
uint16_t serial1_available(void) {
  uint16_t available = SERIAL1.available();
  return (available > 0) ? available : 0;
}

void serial1_read_char(uint8_t* c) {
  *c = SERIAL1.read();
}

uint8_t serial1_has_space(uint16_t count) {
  return (SERIAL1.availableForWrite() >= count) ? 1 : 0;
}

void serial1_write_buf(uint8_t* buf, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    SERIAL1.write(buf[i]);
  }
}

uint32_t get_time_ms() {
  return millis();
}


uint8_t ReceivedPacketL; // Stores the length of the received packet
int8_t PacketRSSI; // Stores the RSSI of the received packet
int8_t PacketSNR; // Stores the signal-to-noise ratio (SNR) of the received packet
uint32_t ReceivedPacketCount;
uint32_t errors;
uint8_t RX_LEN = 250;
uint16_t freq_sr1_to_lora = 200;
uint16_t freq_lora_to_sr1 = 500;
uint32_t last_sr1_to_lora = 0;
uint32_t last_lora_to_sr1 = 0;
uint8_t loraSequence = 0;

// We need a status for each link
// For link 1 and 2, it is required to keep the parser state for receiving
// For link 0, it is required to keep the seq for sending
fmav_status_t status1, status2;

// We need receive working buffer for the two serials
uint8_t sr1_to_lora_buff[1000], lora_to_sr1_buff[1000], parser_buff[1000], buff2[1000], rx_msg_buf[296], rx_msg_buff2[296];

// Receive message structure for the component
fmav_message_t rx_msg1, rx_msg2;

// Some variables we need
uint32_t tlast_ms = 0;

// Define LoRa Modes
enum LoRaMode {
  ADR_MODE_1=1,
  ADR_MODE_2=2,
  ADR_MODE_3=3
};
// Define ADR Mode
uint8_t adr_mode; 


void setup() {
  // Your setup code goes here
  // Set Serial Baudrate
  Serial1.begin(57600);
  Serial2.begin(57600);

  // LED Condition pin
  // pinMode(BLINK_PIN, OUTPUT);
  // pinMode(BLINK_AP_PIN, OUTPUT);
  // pinMode(BLINK_GCS_PIN, OUTPUT);
// pinMode(LED1, OUTPUT);
// pinMode(LED2, OUTPUT);
// pinMode(LED3, OUTPUT);

// digitalWrite(LED1, LOW);
// digitalWrite(LED2, LOW);
// digitalWrite(LED3, LOW);

  // Set SPI Interface for LoRa
  SPI.setMOSI(PB5);
  SPI.setMISO(PB4);
  SPI.setSCLK(PB3);
  SPI.begin();

  // Setup LoRa Init
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RXEN, TXEN, SW, LORA_DEVICE)) {
    delay(1000);
  } 
  else {
    while (1);
  }

  // Setup LoRa Parameter
  LT.setupLoRa(434000000, 0, LORA_SF5, LORA_BW_500, LORA_CR_4_5, LDRO_AUTO);

  // Set LoRa First Time Receive Mode
  ReceivedPacketL = LT.receive(lora_to_sr1_buff, RX_LEN, 0, NO_WAIT);

  // Fast mavlink initialize
  fmav_init(); // let's always call it, even if it currently may not do anything
  fmav_status_reset(&status1);
  fmav_status_reset(&status2);
}

void loop() {
  // Transmit Mode
  if (millis() - last_sr1_to_lora >= 1000 / freq_sr1_to_lora) {
    last_sr1_to_lora = millis();
    uint16_t sr1_av = serial1_available();
    for (uint16_t i = 0; i < sr1_av; i++) {
      uint8_t c;
      serial1_read_char(&c);
      fmav_result_t result1;
      uint8_t res;
      // can return PARSE_RESULT_NONE, PARSE_RESULT_HAS_HEADER, or PARSE_RESULT_OK
      

      res = fmav_parse_to_frame_buf(&result1, sr1_to_lora_buff, &status1, c);

      // A complete frame has been received, not yet validated
      if (res == FASTMAVLINK_PARSE_RESULT_OK) {
        // Can return MSGID_UNKNOWN, LENGTH_ERROR, CRC_ERROR, SIGNATURE_ERROR, or OK
        res = fmav_check_frame_buf(&result1, sr1_to_lora_buff);

        // We want to forward also unknown messages
        if (res == FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN || res == FASTMAVLINK_PARSE_RESULT_OK) {
          fmav_frame_buf_to_msg(&rx_msg1, &result1, sr1_to_lora_buff);

          // Forwarding the received mavlink message to Serial2 (Lora2Serial)
          // Uncomment the following lines if you need to print the received message details
          // Serial2.println("Lora2Serial");
          // Serial2.print("Mavlink Packet Length: ");
          // Serial2.println(rx_msg1.len);
          // Serial2.print("Mavlink Packet Sequence: ");
          // Serial2.println(rx_msg1.seq);
          // Serial2.print("Mavlink MSG ID: ");
          // Serial2.println(rx_msg1.msgid);
          // Serial2.print("Frame Buffer Length: ");
          // Serial2.println(result1.frame_len);

          if (adr_mode == ADR_MODE_1) {
            // Only forward specific messages in adr_mode 1
            switch (rx_msg1.msgid) {
              case FASTMAVLINK_MSG_ID_HEARTBEAT:
              case FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT:
              case FASTMAVLINK_MSG_ID_MISSION_CURRENT:
              case FASTMAVLINK_MSG_ID_MISSION_ITEM_REACHED:
              case FASTMAVLINK_MSG_ID_VFR_HUD:
                fmav_finalize_msg(&rx_msg1, &status1);
                fmav_msg_to_frame_buf(rx_msg_buf, &rx_msg1);
                if (LT.transmit(rx_msg_buf, result1.frame_len + 4, 10000, TXpower, WAIT_TX)) {
                  // Transmission success
                }
                delay(6);
                break;
            }
          } else if (adr_mode == ADR_MODE_2) {
              switch (rx_msg1.msgid) {
                  case FASTMAVLINK_MSG_ID_HEARTBEAT:
                  case FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                  case FASTMAVLINK_MSG_ID_MISSION_CURRENT:
                  case FASTMAVLINK_MSG_ID_MISSION_ITEM_REACHED:
                  case FASTMAVLINK_MSG_ID_VFR_HUD:
                  case FASTMAVLINK_MSG_ID_AHRS:
                  case FASTMAVLINK_MSG_ID_ATTITUDE:
                  case FASTMAVLINK_MSG_ID_MISSION_ACK:
                  case FASTMAVLINK_MSG_ID_MISSION_COUNT:
                  case FASTMAVLINK_MSG_ID_MISSION_ITEM_INT:
                  case FASTMAVLINK_MSG_ID_MISSION_REQUEST:
                  case FASTMAVLINK_MSG_ID_STATUSTEXT:
                      fmav_finalize_msg(&rx_msg1, &status1);
                      fmav_msg_to_frame_buf(rx_msg_buf, &rx_msg1);
                      if (LT.transmit(rx_msg_buf, result1.frame_len + 4, 10000, TXpower, WAIT_TX)) {
                          // Transmission success, do something if needed.
                      }
                      delay(6);
                      break;
              }
          }
          else if (adr_mode == ADR_MODE_3) {
              fmav_finalize_msg(&rx_msg1, &status1);
              fmav_msg_to_frame_buf(rx_msg_buf, &rx_msg1);
              if (result1.frame_len + 4 <= 250) {
                  LT.transmit(rx_msg_buf, result1.frame_len + 4, 0, TXpower, NO_WAIT);
                  delay(6);
              }
              else {
                  LT.transmit(rx_msg_buf, 250, 0, TXpower, NO_WAIT);
                  delay(6);
                  for (int i = 0; i < result1.frame_len + 4 - 250; i++) {
                      buff2[i] = rx_msg_buf[i + 250];
                  }
                  LT.transmit(buff2, result1.frame_len + 4 - 250, 0, TXpower, NO_WAIT);
                  delay(6);
              }
          }

          ReceivedPacketL = LT.receive(lora_to_sr1_buff, RX_LEN, 0, NO_WAIT);
            // Serial2.print(lora_seq);
          }
          //        else {
          //          Serial2.print ("Gagal MSG ID, LEN:");
          //          Serial2.println(result1.frame_len);
          //        }
        }
        else {
          // Serial2.println("Gagal Parsing");
        }
      }
    }
    // Receive
      if (millis() - last_lora_to_sr1 >= 1000 / freq_lora_to_sr1) {
        if (digitalRead(DIO1) == HIGH) {
          ReceivedPacketL = LT.readPacket(lora_to_sr1_buff, 255);
          PacketRSSI = LT.readPacketRSSI();
          PacketSNR = LT.readPacketSNR();
          uint8_t res2;
          fmav_result_t result2;
          if (ReceivedPacketL > 0) {
            for (uint16_t i = 0; i < ReceivedPacketL; i++) {
              // Can return PARSE_RESULT_NONE, PARSE_RESULT_HAS_HEADER, or PARSE_RESULT_OK
              res2 = fmav_parse_to_frame_buf(&result2, parser_buff, &status2, lora_to_sr1_buff[i]);

              // A complete frame has been received, not yet validated
              if (res2 == FASTMAVLINK_PARSE_RESULT_OK) {
                // Can return MSGID_UNKNOWN, LENGTH_ERROR, CRC_ERROR, SIGNATURE_ERROR, or OK
                res2 = fmav_check_frame_buf(&result2, parser_buff);

                // Forwarding the received mavlink message to Serial1 (Serial2Mavlink)
                // We want to forward also unknown messages
                if (res2 == FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN || res2 == FASTMAVLINK_PARSE_RESULT_OK) {
                  fmav_frame_buf_to_msg(&rx_msg2, &result2, parser_buff);
                  fmav_msg_to_frame_buf(rx_msg_buff2, &rx_msg2);

                  // Forward the message to Serial1 (Serial2Mavlink)
                  serial1_write_buf(rx_msg_buff2, result2.frame_len);

                  // Uncomment the following lines if you need to print the received message details
                  // Serial2.println("Lora2Serial");
                  // Serial2.print("Mavlink Packet Length: ");
                  // Serial2.println(rx_msg2.len);
                  // Serial2.print("Mavlink Packet Sequence: ");
                  // Serial2.println(rx_msg2.seq);
                  // Serial2.print("Mavlink MSG ID: ");
                  // Serial2.println(rx_msg2.msgid);
                  // Serial2.print("Frame Buffer Length: ");
                  // Serial2.println(result2.frame_len);
                }
                // else {
                //   Serial2.print("Gagal MSG ID, LEN:");
                //   Serial2.println(result2.frame_len);
                // }
              }
            }
            ReceivedPacketL = LT.receive(lora_to_sr1_buff, RX_LEN, 0, NO_WAIT);
          }
        }
      }
      if (PacketRSSI <= -76) {
        adr_mode = ADR_MODE_1;
        // digitalWrite(LED1, HIGH);
        // digitalWrite(LED2, LOW);
        // digitalWrite(LED3, LOW);
        // delay(5);
      }
      else if (PacketRSSI <= -50 && PacketRSSI > -76) {
        adr_mode = ADR_MODE_2;
        // digitalWrite(LED1, LOW);
        // digitalWrite(LED2, HIGH);
        // digitalWrite(LED3, LOW);
        // delay(5);
      }
      else if (PacketRSSI <= 0 && PacketRSSI > -50) {
        adr_mode = ADR_MODE_3;
        // digitalWrite(LED1, LOW);
        // digitalWrite(LED2, LOW);
        // digitalWrite(LED3, HIGH);
        // delay(5);
      }

}