#include"setStruct.h"

#define PACKET_MAX_LENGTH 125 //got this value by testing
#define PACKET_PAYLOAD_MAX_SIZE 104 //(maxPacketLength-MAC802154_HEADER_LENGTH)
#define MESSAGE_MAX_LENGTH 100 //(payLoadMaxSize-sizeof(olsr_packet_hdr_t))
#define MESSAGE_PAYLOAD_MAX_SIZE  84 //(messageMaxSize-sizeof(olsr_message_hdr_t))
#define LINK_ADDRESS_MAX_NUM 10
#define LINK_MESSAGE_MAX_NUM ((MESSAGE_PAYLOAD_MAX_SIZE-sizeof(olsrHelloMessageHeader_t))\
                              /sizeof(olsrLinkMessage_t)) 
#define TC_PAYLOAD_MAX_NUM ((MESSAGE_PAYLOAD_MAX_SIZE-2)/sizeof(olsrTopologyMessageUint_t))
#define DATA_PAYLOAD_MAX_NUM  MESSAGE_PAYLOAD_MAX_SIZE-sizeof(olsrDataMessageHeader_t)
#define TS_PAYLOAD_MAX_NUM  9 //((MESSAGE_PAYLOAD_MAX_SIZE-sizeof(olsrTsMessageHeader_t))/sizeof(olsrTsMessageBodyUnit_t))

#define OLSR_HELLO_INTERVAL 2000
#define OLSR_NEIGHB_HOLD_TIME (3*OLSR_HELLO_INTERVAL)
#define OLSR_TC_INTERVAL 5000
#define OLSR_NS_INTERVAL 1000
#define TS_INTERVAL 20 //must be in range: 20 - 500
#define TS_INTERVAL_MIN 20 //default 20
#define TS_INTERVAL_MAX 500 //default 500
#define TS_OTSPOOL_MAXSIZE (4*TS_INTERVAL_MAX/TS_INTERVAL)
#define OLSR_TOP_HOLD_TIME (3*OLSR_TC_INTERVAL)
#define OLSR_DUP_HOLD_TIME 10000
#define OLSR_ROUTING_SET_HOLD_TIME 10000
#define OLSR_RANGING_TABLE_HOLD_TIME 10000
#define OLSR_DUP_CLEAR_INTERVAL 30000
#define OLSR_LINK_CLEAR_INTERVAL 5000
#define OLSR_MPR_SELECTOR_CLEAR_INTERVAL 6000

#define OLSR_NEIGHBOR2HOP_CLEAR_INTERVAL 5000

#define OLSR_MS_CLEAR_INTERVAL 4000

#define OLSR_TOP_CLEAR_INTERVAL 3000

/// Unspecified link type.
#define OLSR_UNSPEC_LINK        0
/// Asymmetric link type.
#define OLSR_ASYM_LINK          1
/// Symmetric link type.
#define OLSR_SYM_LINK           2
/// Lost link type.
#define OLSR_LOST_LINK          3

/// Not neighbor type.
#define OLSR_NOT_NEIGH          0
/// Symmetric neighbor type.
#define OLSR_SYM_NEIGH          1
/// Asymmetric neighbor type.
#define OLSR_MPR_NEIGH          2



#define MAX_TIMESTAMP 1099511627776 //2**40

typedef enum{
    HELLO_MESSAGE = 1,
    TC_MESSAGE = 2,
    DATA_MESSAGE = 3,
    TS_MESSAGE= 4,
    NS_MESSAGE = 5,
} olsrMessageType_t;
//1
typedef struct{
   olsrMessageType_t m_messageType;
   uint16_t m_vTime; //The validity time.
   uint16_t m_messageSize;
   olsrAddr_t m_originatorAddress;
   olsrAddr_t m_relayAddress;
   olsrAddr_t m_destinationAddress;
   uint8_t m_reserved;
   uint8_t m_timeToLive;
   uint8_t m_hopCount;
   uint16_t m_messageSeq;
} __attribute__((packed)) olsrMessageHeader_t; //16bytes
//message 
typedef struct
{
    olsrMessageHeader_t m_messageHeader; //16bytes
    uint8_t m_messagePayload[MESSAGE_MAX_LENGTH-sizeof(olsrMessageHeader_t)];//84bytes
    //int content_size;
} __attribute__((packed)) olsrMessage_t;

//hello message
typedef struct{
    uint8_t m_linkMessageNumber;
    uint16_t m_hTime;
    uint8_t m_willingness;
} __attribute__((packed)) olsrHelloMessageHeader_t;//4bytes

typedef struct{
    uint8_t m_linkCode;
    uint8_t m_reserved;
    uint16_t m_addressUsedSize;
#ifdef USER_ROUTING
    olsrWeight_t m_weight;
#endif
    olsrAddr_t m_addresses; //this item may be vector if multi-interface support is needed.
} __attribute__((packed)) olsrLinkMessage_t; //6

typedef struct{
    olsrHelloMessageHeader_t  m_helloHeader;
    olsrLinkMessage_t m_linkMessage[LINK_MESSAGE_MAX_NUM];
} __attribute__((packed)) olsrHelloMessage_t;

static uint16_t g_staticMessageSeq = 0;