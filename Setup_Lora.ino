#define Program_Version "V1.1"
#include <SPI.h>
#include <SX126XLT.h>
SX126XLT LT;
#define NSS PA15
#define NRESET PB13
#define RFBUSY PB12
#define DIO1 PB10
#define DIO2 -1
#define DIO3 -1
#define RXEN PB14
#define TXEN PB15
#define SW -1 
#define LORA_DEVICE DEVICE_SX1268
#define RXBUFFER_SIZE 32

uint32_t ReceivedPacketCount;
uint32_t errors;
uint8_t RXBUFFER[RXBUFFER_SIZE];
uint8_t ReceivedPacketL;
int8_t PacketRSSI;
int8_t PacketSNR;


#define PRINT_ERROR(err) \
  do { \
    uint16_t IRQStatus = LT.readIrqStatus(); \
    printElapsedTime(); \
    Serial1.print(F(" ")); \
    errors++; \
    Serial1.print(F(" PacketError")); \
    Serial1.print(F(",RSSI,")); \
    Serial1.print(PacketRSSI); \
    Serial1.print(F("dBm,SNR,")); \
    Serial1.print(PacketSNR); \
    Serial1.print(F("dB,Length,")); \
    Serial1.print(LT.readRXPacketL()); \
    Serial1.print(F(",Packets,")); \
    Serial1.print(ReceivedPacketCount); \
    Serial1.print(F(",Errors,")); \
    Serial1.print(errors); \
    Serial1.print(F(",IRQreg,")); \
    Serial1.print(IRQStatus, HEX); \
    LT.printIrqStatus(); \
  } while (0)

#define PRINT_OK() \
  do { \
    uint16_t IRQStatus = LT.readIrqStatus(); \
    ReceivedPacketCount++; \
    printElapsedTime(); \
    Serial1.print(F(" ")); \
    LT.printASCIIPacket(RXBUFFER, ReceivedPacketL); \
    Serial1.print(F(",RSSI,")); \
    Serial1.print(PacketRSSI); \
    Serial1.print(F("dBm,SNR,")); \
    Serial1.print(PacketSNR); \
    Serial1.print(F("dB,Length,")); \
    Serial1.print(ReceivedPacketL); \
    Serial1.print(F(",Packets,")); \
    Serial1.print(ReceivedPacketCount); \
    Serial1.print(F(",Errors,")); \
    Serial1.print(errors); \
    Serial1.print(F(",IRQreg,")); \
    Serial1.print(IRQStatus, HEX); \
  } while (0)

void setup()
{
  Serial1.begin(9600);
  Serial1.println(F("\n4_LoRa_Receiver Starting\n"));
  SPI.setMOSI(PB5);
  SPI.setMISO(PB4);
  SPI.setSCLK(PB3);
  SPI.begin();

  if (!LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RXEN, TXEN, SW, LORA_DEVICE))
  {
    Serial1.println(F("No device responding"));
    while (1);
  }
  else
  {
    Serial1.println(F("LoRa Device found")); 
    LT.setupLoRa(434000000, 0, LORA_SF7, LORA_BW_125, LORA_CR_4_5, LDRO_AUTO);  //ada variabel frekuensinya
    Serial1.print(F("Receiver ready - RXBUFFER_SIZE "));
    Serial1.println(RXBUFFER_SIZE);
    Serial1.println();
  }
}

void loop()
{
  ReceivedPacketL = LT.receive(RXBUFFER, RXBUFFER_SIZE, 60000, WAIT_RX);
  PacketRSSI = LT.readPacketRSSI();
  PacketSNR = LT.readPacketSNR();

  if (ReceivedPacketL == 0)
  {
    PRINT_ERROR("RXTimeout");
  }
  else
  {
    PRINT_OK();
  }

  Serial1.println();
}

void printElapsedTime()
{
  float seconds = millis() / 1000;
  Serial1.print(seconds, 0);
  Serial1.print(F("s"));
}
