#ifndef MERGE_WIRE_TUNABLES_H
#define MERGE_WIRE_TUNABLES_H

#define MAX_CHUNK_SIZE        255  // data_len is a uint8_t
#define MW_JAM_BITS           0    // >0: jam the bus with a break on clash
#define MW_FWD_BRK_BITS       12   // regenerated break width (>= 1 frame);

// rs485 burst policy
#define QUIET_BITS        35   // carrier sense: 3.5 chars of bus silence
#define DE_SETTLE_US      2    // transceiver driver-enable settle time
#define RXFIFO_FULL_THR   64
#define RX_TOUT_BITS      22
#define TXFIFO_EMPTY_THR  16
#define MW_MAX_RETRIES    8    // chunk replays after collisions, then drop

#endif //MERGE_WIRE_TUNABLES_H
