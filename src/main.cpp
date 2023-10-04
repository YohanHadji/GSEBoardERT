#include <Arduino.h>
#include <Capsule.h>
#include <LoopbackStream.h>
#include <LoRa.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <HX711.h> // load cell

#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include "../ERT_RF_Protocol_Interface/ParameterDefinition.h"
#include <config.h>

#define DISCONNECTING_TIMEOUT    20000      // 10 sec for each pin in successive mode 

#define GSE_VENT_VALVE_PIN       16
#define GSE_FILLING_VALVE_PIN    17
#define GSE_DISCONNECT_PIN_1     14
#define GSE_DISCONNECT_PIN_2     15  // redundance

#define DIO_0_PIN         (2)
#define DIO_1_PIN         (3)
#define DIO_2_PIN         (4)
#define DIO_3_PIN         (5)
#define DIO_4_PIN         (6)
#define DIO_5_PIN         (7)
#define DIO_6_PIN         (8)
#define DIO_7_PIN         (9)
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

// Load cell
#define LOAD_CELL_PIN_SCK       7
#define LOAD_CELL_PIN_DOUT      8

// Pressure sensor
#define FILLING_PRESSURE_PIN    A6
#define TANK_PRESSURE_PIN       A5

#define V_SUPPLY 	            (5000.0) //mv
#define P_MAX		            (60.0)   //bar
#define P_MIN      	            (1.0)    //bar

float convert_to_pressure(float ADC_value) {
    // Serial.println("val: " + String(ADC_value));
    float voltage = ADC_value / 1024.0 * 3300.0; // mV
    // Serial.println("Volateg: " + String(voltage));
    float voltage_sensor = voltage * ((150.0 + 51.0) / 150.0);  // voltage divider with R=150k and 51k Ohm
    // Serial.println("V_ADC: " + String(voltage_sensor));
    float delta_p = P_MAX - P_MIN;
    float v_max = 0.8 * V_SUPPLY;
    float v_min = 0.1 * V_SUPPLY;
    return (delta_p * (voltage_sensor - v_min)) / v_max + P_MIN;
}

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
static int disconnect_time_1 = DISCONNECTING_TIMEOUT; // more than few seconds otherwise ON on start
static int disconnect_time_2 = DISCONNECTING_TIMEOUT; // more than few seconds otherwise ON on start
static bool disconnect1_active = false;
static bool disconnect2_active = false;

void handleLoRaUplink(int packetSize);
void handleLoRaCapsuleUplink(uint8_t packetId, uint8_t *dataIn, uint32_t len); 

void handleLoRaDownlink(int packetSize);
void handleLoRaCapsuleDownlink(uint8_t packetId, uint8_t *dataIn, uint32_t len); 

void sendGSETelemetry();
void read_temperature();

Adafruit_NeoPixel led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // 1 led

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 tempSensor = Adafruit_MAX31865(DIO_4_PIN, DIO_2_PIN,DIO_1_PIN,DIO_0_PIN);

// load cell
HX711 loadcell;

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

    lastGSE.status.fillingN2O = INACTIVE;
    lastGSE.status.vent = INACTIVE;

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
#if GSE_DOWNLINK_INVERSE_IQ
        LoRaDownlink.enableInvertIQ();
#else
        LoRaDownlink.disableInvertIQ();
#endif
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
#if UPLINK_INVERSE_IQ
        LoRaUplink.enableInvertIQ();
#else
        LoRaUplink.disableInvertIQ();
#endif
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
    pinMode(GSE_DISCONNECT_PIN_1, OUTPUT);
    digitalWrite(GSE_DISCONNECT_PIN_1, LOW);
    pinMode(GSE_DISCONNECT_PIN_2, OUTPUT);
    digitalWrite(GSE_DISCONNECT_PIN_2, LOW);

    led.begin();
    uint32_t ledColor = colors[3];
    led.fill(ledColor);
    led.show();

    tempSensor.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary

    loadcell.begin(LOAD_CELL_PIN_DOUT, LOAD_CELL_PIN_SCK);
    // loadcell.set_scale(25.0);
    Serial.println("Setup done!");
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
    if (disconnect1_active && millis() - disconnect_time_1 > DISCONNECTING_TIMEOUT) {
        digitalWrite(GSE_DISCONNECT_PIN_1, LOW);
        digitalWrite(GSE_DISCONNECT_PIN_2, HIGH);
        disconnect_time_2 = millis();
        disconnect1_active = false;
        disconnect2_active = true;
    }
    if (disconnect2_active && millis() - disconnect_time_2 > DISCONNECTING_TIMEOUT) {
        digitalWrite(GSE_DISCONNECT_PIN_2, LOW);
        disconnect2_active = false;
    }
}

void sendGSETelemetry() {
    digitalWrite(DOWNLINK_LED, HIGH);

    // lastGSE.tankPressure = 1013 + sin(millis() / 10000.0) * 100;
    lastGSE.tankPressure = convert_to_pressure(analogRead(TANK_PRESSURE_PIN));
    // Serial.println("Tank pressure: " + String(lastGSE.tankPressure));
    // lastGSE.fillingPressure = 1013 + cos(millis() / 10000.0) * 100;
    lastGSE.fillingPressure = convert_to_pressure(analogRead(FILLING_PRESSURE_PIN));
    // Serial.println("Filling pressure: " + String(lastGSE.fillingPressure));
    // lastGSE.tankTemperature = 20 + sin(millis() / 10000.0) * 10;
    lastGSE.disconnectActive = disconnect1_active or disconnect2_active;

    read_temperature();

    if (loadcell.is_ready()) {
        lastGSE.loadcellRaw = loadcell.read();
        Serial.println("load cell: " + String(lastGSE.loadcellRaw));
    } else {
        lastGSE.loadcellRaw = 0;
        Serial.println("HX711 not found, fail!");
    }

    uint8_t* packetToSend = LoRaCapsuleDownlink.encode(CAPSULE_ID::GSE_TELEMETRY, (uint8_t*) &lastGSE, packetGSE_downlink_size);

    LoRaDownlink.beginPacket();
    LoRaDownlink.write(packetToSend, LoRaCapsuleDownlink.getCodedLen(packetGSE_downlink_size));
    LoRaDownlink.endPacket();

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

    if (packetId == CAPSULE_ID::GS_CMD) {
        switch (uplink_packet.order_id) {
            case CMD_ID::GSE_CMD_FILLING_N2O:
                lastGSE.status.fillingN2O = uplink_packet.order_value;
                if (uplink_packet.order_value == ACTIVE) {
                    digitalWrite(GSE_FILLING_VALVE_PIN, HIGH);
                } else if (uplink_packet.order_value == INACTIVE) {
                    digitalWrite(GSE_FILLING_VALVE_PIN, LOW);
                }
                break;
            case CMD_ID::GSE_CMD_VENT:
                lastGSE.status.vent = uplink_packet.order_value;
                if (uplink_packet.order_value == ACTIVE) { // valve normally close
                    digitalWrite(GSE_VENT_VALVE_PIN, HIGH);
                } else if (uplink_packet.order_value == INACTIVE) {
                    digitalWrite(GSE_VENT_VALVE_PIN, LOW);
                }
                break;
            case CMD_ID::GSE_CMD_DISCONNECT:
                if (uplink_packet.order_value == ACTIVE) {
                    digitalWrite(GSE_DISCONNECT_PIN_1, HIGH);
                    // digitalWrite(GSE_DISCONNECT_PIN_2, HIGH); // 10 sec later
                    disconnect_time_1 = millis();
                    disconnect1_active = true;
                    disconnect2_active = false;
                    digitalWrite(GSE_DISCONNECT_PIN_2, LOW);
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

// from Lucas Pallez
void read_temperature() {
    uint16_t rtd = tempSensor.readRTD();
    float ratio = rtd;
    ratio /= 32768;
    // Serial.print("RTD value: ");
    // Serial.println(rtd);
    // Serial.print("Ratio = ");
    // Serial.println(ratio, 8);
    // Serial.print("Resistance = ");
    // Serial.println(RREF * ratio, 8);
    // Serial.print("Temperature = ");
    // Serial.println(tempSensor.temperature(RNOMINAL, RREF));
    lastGSE.tankTemperature = tempSensor.temperature(RNOMINAL, RREF);

    // Check and print any faults
    uint8_t fault = tempSensor.readFault();
    if (fault) {
        Serial.print("Fault 0x");
        Serial.println(fault, HEX);
        if (fault & MAX31865_FAULT_HIGHTHRESH) {
            Serial.println("RTD High Threshold");
        }
        if (fault & MAX31865_FAULT_LOWTHRESH) {
            Serial.println("RTD Low Threshold");
        }
        if (fault & MAX31865_FAULT_REFINLOW) {
            Serial.println("REFIN- > 0.85 x Bias");
        }
        if (fault & MAX31865_FAULT_REFINHIGH) {
            Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
        }
        if (fault & MAX31865_FAULT_RTDINLOW) {
            Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
        }
        if (fault & MAX31865_FAULT_OVUV) {
            Serial.println("Under/Over voltage");
        }
        tempSensor.clearFault();
    }
}