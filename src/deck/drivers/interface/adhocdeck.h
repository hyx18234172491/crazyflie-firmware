#ifndef __ADHOCDECK_H__
#define __ADHOCDECK_H__

#include "libdw3000.h"
#include "mac_802_15_4.h"
#include "dwTypes.h"
#include "queue.h"

/* Function Switch */
#define ENABLE_BUS_BOARDING_SCHEME
#define ENABLE_PHR_EXT_MODE

#define SPEED_OF_LIGHT 299702547
#define MAX_TIMESTAMP 1099511627776  // 2**40
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define FRAME_LEN_STD 127
#define FRAME_LEN_EXT 1023
#ifdef ENABLE_PHR_EXT_MODE
#define FRAME_LEN_MAX FRAME_LEN_EXT
#else
#define FRAME_LEN_MAX FRAME_LEN_STD
#endif

/* Queue Constants */
#define TX_QUEUE_SIZE 10 // TODO 5
#define RX_QUEUE_SIZE 20
#define TX_QUEUE_ITEM_SIZE sizeof(uwbPacket_t)
#define RX_QUEUE_ITEM_SIZE sizeof(uwbPacketWithTimestamp_t)
#define RX_BUFFER_SIZE RX_QUEUE_ITEM_SIZE  // RX_BUFFER_SIZE ≤ FRAME_LEN_MAX

/* Ranging Constants */
#define RANGING_INTERVAL_MIN 20 // default 20
#define RANGING_INTERVAL_MAX 500 // default 500
#define Tf_BUFFER_POOL_SIZE (4 * RANGING_INTERVAL_MAX / RANGING_INTERVAL_MIN)
#define TX_PERIOD_IN_MS 100

/* Packet */
#define PACKET_SIZE FRAME_LEN_MAX
#define PAYLOAD_SIZE (PACKET_SIZE - sizeof(Packet_Header_t))

/* TX options */
static dwt_txconfig_t txconfig_options = {
    .PGcount = 0x0,
    .PGdly = 0x34,
    .power = 0xfdfdfdfd
};

/* PHR configuration */
static dwt_config_t config = {
    5,            /* Channel number. */
    DWT_PLEN_128, /* Preamble length. Used in TX only. */
    DWT_PAC8,     /* Preamble acquisition chunk size. Used in RX only. */
    9,            /* TX preamble code. Used in TX only. */
    9,            /* RX preamble code. Used in RX only. */
    1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for
          non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
#ifdef ENABLE_PHR_EXT_MODE
    DWT_PHRMODE_EXT, /* Extended PHY header mode. */
#else
    DWT_PHRMODE_STD, /* Standard PHY header mode. */
#endif
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size).
                      Used in RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64, /* STS length, see allowed values in Enum dwt_sts_lengths_e
                     */
    DWT_PDOA_M0     /* PDOA mode off */
};

/* UWB packet definition */
typedef enum {
  RANGING = 0,
  DATA = 1,
} MESSAGE_TYPE;

typedef struct {
  mhr_802_15_4_t mac;    // mac header
  union {
    uint16_t header;
    struct {
      MESSAGE_TYPE type: 6;
      uint16_t length: 10;
    };
  };
} __attribute__((packed)) Packet_Header_t;

typedef struct {
  Packet_Header_t header; // Packet header
  uint8_t payload[PAYLOAD_SIZE]
} __attribute__((packed)) uwbPacket_t;

typedef struct {
  uwbPacket_t packet;
  dwTime_t rxTime;
} __attribute__((packed)) uwbPacketWithTimestamp_t;

typedef void (*UWBCallback)(void *);

typedef struct {
  MESSAGE_TYPE type;
  QueueHandle_t queue;
  UWBCallback preprocess;
} uwbPacketHandler;

/* UWB operations */
int uwbSendPacket(uwbPacket_t *packet);
int uwbReceivePacket(MESSAGE_TYPE type, uwbPacket_t *packet);
void uwbRegisterHandler(uwbPacketHandler *handler);
#endif