#include <Arduino.h>
#include <Capsule.h>
#include <LoopbackStream.h>
#include <LoRa.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include "../ERT_RF_Protocol_Interface/ParameterDefinition.h"
#include <config.h>

#define GSE_VENT_VALVE_PIN      9
#define GSE_FILLING_VALVE_PIN   8
#define GSE_DISCONNECT_PIN      7

uint32_t colors[] = {
    0x32A8A0, // Cyan
    0x0000FF, // Blue
    0xFFEA00, // Yellow
    0x00FF00, // Green
    0xFF0000, // Red
    0xCF067C, // Purple
    0xFF0800  // Orange
}; 

static PacketGSE_downlink lastGSE;

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

	// 1st LoRa initialization
    {
        SPI1.begin();
        SPI1.setMISO(LORA_DOWNLINK_MISO);
        SPI1.setMOSI(LORA_DOWNLINK_MOSI);
        SPI1.setSCK(LORA_DOWNLINK_SCK);

        LoRaDownlink.setPins(LORA_DOWNLINK_CS, LORA_DOWNLINK_RST, LORA_DOWNLINK_INT0);
        LoRaDownlink.setSPI(SPI1);

        if (!LoRaDownlink.begin(LORA_DOWNLINK_FREQ)) {
#ifdef DEBUG
            SERIAL_TO_PC.println("Starting LoRa Downlink failed!");
#endif
        } else {
#ifdef DEBUG
            SERIAL_TO_PC.println("Starting LoRa Downlink success!");
#endif
        }

        LoRaDownlink.setSpreadingFactor(LORA_DOWNLINK_SF);
        LoRaDownlink.setSignalBandwidth(LORA_DOWNLINK_BW);
        LoRaDownlink.setCodingRate4(LORA_DOWNLINK_CR);
        // LoRaDownlink.setPreambleLength(LORA_DOWNLINK_PREAMBLE_LEN);
        // LoRaDownlink.setSyncWord(LORA_DOWNLINK_SYNC_WORD);
        // LoRaDownlink.enableCrc();
        LoRaDownlink.setTxPower(LORA_DOWNLINK_POWER);
        // LoRaDownlink.setOCP(LORA_DOWNLINK_CURRENT_LIMIT);
        LoRaDownlink.receive();
        // LoRaDownlink.onReceive(handleLoRaDownlink);
    }

	// 2nd LoRa initialization
    {
        SPI.begin();
        LoRaUplink.setPins(LORA_UPLINK_CS, LORA_UPLINK_RST, LORA_UPLINK_INT0);
        LoRaUplink.setSPI(SPI);

        if (!LoRaUplink.begin(LORA_UPLINK_FREQ)) {
#ifdef DEBUG
            SERIAL_TO_PC.println("Starting LoRa Uplink failed!");
#endif
        } else {
#ifdef DEBUG
            SERIAL_TO_PC.println("Starting LoRa Uplink success!");
#endif
        }

        LoRaUplink.setSpreadingFactor(LORA_UPLINK_SF);
        LoRaUplink.setSignalBandwidth(LORA_UPLINK_BW);
        LoRaUplink.setCodingRate4(LORA_UPLINK_CR);
        // LoRaUplink.setPreambleLength(LORA_UPLINK_PREAMBLE_LEN);
        // LoRaUplink.setSyncWord(LORA_UPLINK_SYNC_WORD);
        // LoRaUplink.enableCrc();
        LoRaUplink.setTxPower(LORA_UPLINK_POWER);
        // LoRa.setOCP(LORA_UPLINK_CURRENT_LIMIT);
        LoRaUplink.receive();
        LoRaUplink.onReceive(handleLoRaUplink);
    }

    pinMode(DOWNLINK_LED, OUTPUT);
    pinMode(UPLINK_LED, OUTPUT);

    pinMode(GSE_FILLING_VALVE_PIN, OUTPUT);
    digitalWrite(GSE_FILLING_VALVE_PIN, LOW);
    pinMode(GSE_VENT_VALVE_PIN, OUTPUT);
    digitalWrite(GSE_VENT_VALVE_PIN, LOW);
    pinMode(GSE_DISCONNECT_PIN, OUTPUT);
    digitalWrite(GSE_DISCONNECT_PIN, LOW);

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
        if ((millis() - lastPacketSent) > (1000 / GSE_TELEMETRY_RATE)) {
            lastPacketSent = millis();
            sendGSETelemetry();
        }
    }
}

void sendGSETelemetry() {
    digitalWrite(DOWNLINK_LED, HIGH);

    lastGSE.tankPressure = 1013 + sin(millis() / 10000.0) * 100;
    //Serial.println(lastGSE.tankPressure);

    uint8_t *buffer = new uint8_t[packetGSE_downlink_size];
    memcpy(buffer, &lastGSE, packetGSE_downlink_size);

    uint8_t *packetToSend = new uint8_t[LoRaCapsuleDownlink.getCodedLen(packetGSE_downlink_size)];
    packetToSend = LoRaCapsuleDownlink.encode(CAPSULE_ID::GSE_TELEMETRY, buffer, packetGSE_downlink_size);

    LoRaDownlink.beginPacket();
    LoRaDownlink.write(packetToSend, LoRaCapsuleDownlink.getCodedLen(packetGSE_downlink_size));
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
    for (int i = 0; i < packetSize; i++) {
        LoRaUplinkBuffer.write(LoRaUplink.read());
    }
}

void handleLoRaCapsuleUplink(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
	digitalWrite(UPLINK_LED, HIGH);
#ifdef DEBUG
    SERIAL_TO_PC.print("Received packet from GS with id: ");
    SERIAL_TO_PC.println(packetId);
#endif

    av_uplink_t uplink_packet;
    memcpy(&uplink_packet, dataIn, av_uplink_size);
	
    switch (packetId) {
        case CAPSULE_ID::GS_CMD: {
            switch (uplink_packet.order_id) {
                case CMD_ID::GSE_FILLING_N2O:
                    lastGSE.status.fillingN2O = uplink_packet.order_value;
                    if (uplink_packet.order_value == ACTIVE) {
                        digitalWrite(GSE_FILLING_VALVE_PIN, HIGH);
                    } else if (uplink_packet.order_value == INACTIVE) {
                        digitalWrite(GSE_FILLING_VALVE_PIN, LOW);
                    }
                    break;
                case CMD_ID::GSE_VENT:
                    lastGSE.status.vent = uplink_packet.order_value;
                    if (uplink_packet.order_value == ACTIVE) {
                        digitalWrite(GSE_VENT_VALVE_PIN, HIGH);
                    } else if (uplink_packet.order_value == INACTIVE) {
                        digitalWrite(GSE_VENT_VALVE_PIN, LOW);
                    }
                    break;
                case CMD_ID::AV_CMD_DISCONNECT:
                    if (uplink_packet.order_value == ACTIVE) {
                        digitalWrite(GSE_DISCONNECT_PIN, HIGH);
                    }
                    break;
            }
            break;
        }
    }
	delay(50); // delay to see the LED
    digitalWrite(UPLINK_LED, LOW);

    uint32_t ledColor = colors[random(0, 7)];
    led.fill(ledColor);
    led.show();
    digitalWrite(DOWNLINK_LED, LOW);
}

// We never "receive" anything with the downlink radio.. we just send stuff, still the capsule
// object needs its callback function to be initialised because Capsule is designed to be
// bidirectional so we just leave it empty.
void handleLoRaCapsuleDownlink(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
#ifdef DEBUG
    SERIAL_TO_PC.println("Received packet on the downlink radio.. shouldn't happen");
#endif
}