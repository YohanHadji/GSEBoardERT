#include <Arduino.h>
#include <Capsule.h>
#include <LoopbackStream.h>
#include <LoRa.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include "../ERT_RF_Protocol_Interface/ParameterDefinition.h"
#include <config.h>

uint32_t colors[] = {
    0x32A8A0, // Cyan
    0x0000FF, // Blue
    0xFFEA00, // Yellow
    0x00FF00, // Green
    0xFF0000, // Red
    0xCF067C, // Purple
    0xFF0800  // Orange
}; 

static GSE_cmd_status lastGSEStatus;

void handleLoRaUplink(int packetSize);
void handleLoRaCapsuleUplink(uint8_t packetId, uint8_t *dataIn, uint32_t len); 

void handleLoRaDownlink(int packetSize);
void handleLoRaCapsuleDownlink(uint8_t packetId, uint8_t *dataIn, uint32_t len); 

void sendGSETelemetry();

Adafruit_NeoPixel led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // 1 led

LoopbackStream LoRaDownlinkBuffer(1024);
LoopbackStream LoRaUplinkBuffer(1024);

CapsuleStatic LoRaCapsuleDownlink(handleLoRaCapsuleDownlink);
CapsuleStatic LoRaCapsuleUplink(handleLoRaCapsuleUplink);

// So basically, the LoRaUplink HAS to be the global LoRa defined in LoRa.h,
// because we want to use interrupts when a packet from the GS is sent to the GSE,
// that means that everywhere in the code where we use LoRaUplink.something() we actually do 
// LoRa.something() because LoRaUplink is just a reference to LoRa.
static LoRaClass LoRaDownlink;
#define LoRaUplink LoRa

void setup() { 

  SERIAL_TO_PC.begin(SERIAL_TO_PC_BAUD);
  delay(1000);
  {
    SPI1.begin();
    SPI1.setMISO(LORA_DOWNLINK_MISO);
    SPI1.setMOSI(LORA_DOWNLINK_MOSI);
    SPI1.setSCK(LORA_DOWNLINK_SCK);

    LoRaDownlink.setPins(LORA_DOWNLINK_CS, LORA_DOWNLINK_RST, LORA_DOWNLINK_INT0);
    LoRaDownlink.setSPI(SPI1);
    
    if (!LoRaDownlink.begin(LORA_DOWNLINK_FREQ)) {
      if (DEBUG) {
        SERIAL_TO_PC.println("Starting LoRa Downlink failed!");
      }
    }
    else {
      if (DEBUG) {
        SERIAL_TO_PC.println("Starting LoRa Downlink success!");
      }
    }

    LoRaDownlink.setSpreadingFactor(LORA_DOWNLINK_SF);
    LoRaDownlink.setSignalBandwidth(LORA_DOWNLINK_BW);
    LoRaDownlink.setCodingRate4(LORA_DOWNLINK_CR);
    //LoRaDownlink.setPreambleLength(LORA_DOWNLINK_PREAMBLE_LEN);
    //LoRaDownlink.setSyncWord(LORA_DOWNLINK_SYNC_WORD);
    //LoRaDownlink.enableCrc();
    LoRaDownlink.setTxPower(LORA_DOWNLINK_POWER);
    //LoRaDownlink.setOCP(LORA_DOWNLINK_CURRENT_LIMIT);
    LoRaDownlink.receive();  
    //LoRaDownlink.onReceive(handleLoRaDownlink);
  }
  
  {
    SPI.begin(); 
    LoRaUplink.setPins(LORA_UPLINK_CS, LORA_UPLINK_RST, LORA_UPLINK_INT0);
    LoRaUplink.setSPI(SPI);
    
    if (!LoRaUplink.begin(LORA_UPLINK_FREQ)) {
      if (DEBUG) {
        SERIAL_TO_PC.println("Starting LoRa Uplink failed!");
      }
    }
    else {
      if (DEBUG) {
        SERIAL_TO_PC.println("Starting LoRa Uplink success!");
      }
    }

    LoRaUplink.setSpreadingFactor(LORA_UPLINK_SF);
    LoRaUplink.setSignalBandwidth(LORA_UPLINK_BW);
    LoRaUplink.setCodingRate4(LORA_UPLINK_CR);
    //LoRaUplink.setPreambleLength(LORA_UPLINK_PREAMBLE_LEN);
    //LoRaUplink.setSyncWord(LORA_UPLINK_SYNC_WORD);
    //LoRaUplink.enableCrc();
    LoRaUplink.setTxPower(LORA_UPLINK_POWER);
    //LoRa.setOCP(LORA_UPLINK_CURRENT_LIMIT);
    LoRaUplink.receive(); 
    LoRaUplink.onReceive(handleLoRaUplink); 
  }

  pinMode(DOWNLINK_LED, OUTPUT);
  pinMode(UPLINK_LED, OUTPUT);

  led.begin();
  uint32_t ledColor = colors[3];
  led.fill(ledColor);
  led.show();
}

void loop() {
  while (LoRaDownlinkBuffer.available()) {
    LoRaCapsuleDownlink.decode(LoRaDownlinkBuffer.read());
  }
  while (LoRaUplinkBuffer.available()) {
    LoRaCapsuleUplink.decode(LoRaUplinkBuffer.read());
  }
  {
    static unsigned long lastPacketSent;
    if ((millis()-lastPacketSent)>(1000/GSE_TELEMETRY_RATE)) {
      lastPacketSent = millis();
      sendGSETelemetry();
    }
  }
}

void sendGSETelemetry() {

  digitalWrite(DOWNLINK_LED, HIGH);

  uint8_t* buffer = new uint8_t[GSE_cmd_status_size];
  memcpy(buffer, &lastGSEStatus, GSE_cmd_status_size);
  uint8_t* packetToSend = new uint8_t[LoRaCapsuleDownlink.getCodedLen(GSE_cmd_status_size)];
  packetToSend = LoRaCapsuleDownlink.encode(CAPSULE_ID::GSE_TELEMETRY, buffer, GSE_cmd_status_size);
  
  LoRaDownlink.beginPacket();
  LoRaDownlink.write(packetToSend, LoRaCapsuleDownlink.getCodedLen(GSE_cmd_status_size));
  LoRaDownlink.endPacket();

  delete[] buffer;
  delete[] packetToSend;

  digitalWrite(DOWNLINK_LED, LOW);

}

void handleLoRaDownlink(int packetSize) {
  for (int i = 0; i < packetSize; i++) {
    LoRaDownlinkBuffer.write(LoRaDownlink.read());
  }
}

void handleLoRaUplink(int packetSize) {
  digitalWrite(UPLINK_LED, HIGH);
  for (int i = 0; i < packetSize; i++) {
    LoRaUplinkBuffer.write(LoRaUplink.read());
  }
  delay(50);
  digitalWrite(UPLINK_LED, LOW);
}

void handleLoRaCapsuleUplink(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  if (DEBUG) {
    SERIAL_TO_PC.print("Received packet from GS with id: ");
    SERIAL_TO_PC.println(packetId);
  }

  Packet_cmd lastCmd;
  memcpy(&lastCmd, dataIn, sizeof(Packet_cmd));

  switch (packetId) {
    case CAPSULE_ID::GSE_FILLING_N2O:
      if (lastCmd.value == CMD_ACTIVE) {
        lastGSEStatus.fillingN2O = STATUS_ACTIVE;
      } 
      else if (lastCmd.value == CMD_INACTIVE) {
        lastGSEStatus.fillingN2O = STATUS_INACTIVE;
      }
    break;
    case CAPSULE_ID::GSE_VENT:
      if (lastCmd.value == CMD_ACTIVE) {
        lastGSEStatus.vent = STATUS_ACTIVE;
      } 
      else if (lastCmd.value == CMD_INACTIVE) {
        lastGSEStatus.vent = STATUS_INACTIVE;
      }
    break;
  }

  uint32_t ledColor = colors[random(0,7)];
  led.fill(ledColor);
  led.show();
  digitalWrite(DOWNLINK_LED, LOW);
}

// We never "received" anything with the downlink radio.. we just send stuff, still the capsule 
// object needs its callback function to be initialised because Capsule is designed to be
// bidirectional so we just leave it empty.
void handleLoRaCapsuleDownlink(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  if (DEBUG) {
    SERIAL_TO_PC.println("Received packet on the downlink radio.. shouldn't happen");
    }
}