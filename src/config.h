#include "../ERT_RF_Protocol_Interface/ParameterDefinition.h"

#define DEBUG   // comment if not

#define SERIAL_TO_PC    Serial
#define SERIAL_TO_PC_BAUD 115200

#define GSE_TELEMETRY_RATE 2

#define LORA_UPLINK_FREQ         UPLINK_FREQUENCY
#define LORA_UPLINK_POWER        UPLINK_POWER
#define LORA_UPLINK_BW           UPLINK_BW
#define LORA_UPLINK_SF           UPLINK_SF
#define LORA_UPLINK_CR           UPLINK_CR
#define LORA_UPLINK_PREAMBLE_LEN UPLINK_PREAMBLE_LEN
#define LORA_UPLINK_SYNC_WORD    UPLINK_SYNC_WORD
#define LORA_UPLINK_CRC          UPLINK_CRC

#define LORA_DOWNLINK_FREQ         GSE_DOWNLINK_FREQUENCY
#define LORA_DOWNLINK_POWER        GSE_DOWNLINK_POWER
#define LORA_DOWNLINK_BW           GSE_DOWNLINK_BW
#define LORA_DOWNLINK_SF           GSE_DOWNLINK_SF
#define LORA_DOWNLINK_CR           GSE_DOWNLINK_CR
#define LORA_DOWNLINK_PREAMBLE_LEN GSE_DOWNLINK_PREAMBLE_LEN
#define LORA_DOWNLINK_SYNC_WORD    GSE_DOWNLINK_SYNC_WORD
#define LORA_DOWNLINK_CRC          GSE_DOWNLINK_CRC

#define LORA_CURRENT_LIMIT 120

// PIN/GPIO Definition
#define LORA_UPLINK_SCK                13
#define LORA_UPLINK_MOSI               11
#define LORA_UPLINK_MISO               12
#define LORA_UPLINK_CS                 10
#define LORA_UPLINK_INT0               22
//#define LORA_UPLINK_INT5             39
#define LORA_UPLINK_RST                36

#define LORA_DOWNLINK_SCK                27
#define LORA_DOWNLINK_MOSI               26
#define LORA_DOWNLINK_MISO               39
#define LORA_DOWNLINK_CS                 38
#define LORA_DOWNLINK_INT0               23
//#define LORA_UPLINK_INT5               39
#define LORA_DOWNLINK_RST                37

#define NEOPIXEL_PIN            0

#define DOWNLINK_LED 28
#define UPLINK_LED 29