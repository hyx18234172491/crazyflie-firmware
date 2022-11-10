#ifndef _SWARM_RANGING_H_
#define _SWARM_RANGING_H_
#include "adhocdeck.h"
#include "ranging_struct.h"


/* Ranging Constants */
#define RANGING_INTERVAL_MIN 20 // default 20
#define RANGING_INTERVAL_MAX 500 // default 500
#define Tf_BUFFER_POOL_SIZE (4 * RANGING_INTERVAL_MAX / RANGING_INTERVAL_MIN)
#define TX_PERIOD_IN_MS 100

/* Ranging Operations */
void rangingInit();
int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                        Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                        Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);
void processRangingMessage(Ranging_Message_With_Timestamp_t  *rangingMessageWithTimestamp);
void generateRangingMessage(Ranging_Message_t *rangingMessage);

#endif