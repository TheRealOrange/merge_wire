#ifndef MERGE_WIRE_TUNABLES_H
#define MERGE_WIRE_TUNABLES_H

#include "sdkconfig.h"

// structural constant, NOT tunable: data_len in bridge_uart_data_t is a uint8_t
#define MAX_CHUNK_SIZE        255

// Every knob below is exposed in the component Kconfig (menuconfig ->
// "merge_wire UART <-> RS485 bridge" -> "Timing and thresholds"); the
// fallbacks keep the sources compiling in trees without the Kconfig.

#ifdef CONFIG_MERGE_WIRE_QUIET_BITS
#define QUIET_BITS        CONFIG_MERGE_WIRE_QUIET_BITS
#else
#define QUIET_BITS        35   // carrier sense: 3.5 chars of bus silence (Modbus T3.5)
#endif

#ifdef CONFIG_MERGE_WIRE_DE_SETTLE_US
#define DE_SETTLE_US      CONFIG_MERGE_WIRE_DE_SETTLE_US
#else
#define DE_SETTLE_US      2    // transceiver driver-enable settle time
#endif

#ifdef CONFIG_MERGE_WIRE_FWD_BRK_BITS
#define MW_FWD_BRK_BITS   CONFIG_MERGE_WIRE_FWD_BRK_BITS
#else
#define MW_FWD_BRK_BITS   12   // regenerated break width; >= 22 for DMX512
#endif

#ifdef CONFIG_MERGE_WIRE_JAM_BITS
#define MW_JAM_BITS       CONFIG_MERGE_WIRE_JAM_BITS
#else
#define MW_JAM_BITS       0    // >0: jam the bus with a break on clash
#endif

#ifdef CONFIG_MERGE_WIRE_MAX_RETRIES
#define MW_MAX_RETRIES    CONFIG_MERGE_WIRE_MAX_RETRIES
#else
#define MW_MAX_RETRIES    8    // chunk replays after collisions, then drop
#endif

#ifdef CONFIG_MERGE_WIRE_FEED_CAP
#define MW_FEED_CAP       CONFIG_MERGE_WIRE_FEED_CAP
#else
#define MW_FEED_CAP       32   // max bytes per rs485 TX FIFO fill pass
#endif

#ifdef CONFIG_MERGE_WIRE_RXFIFO_FULL_THR
#define RXFIFO_FULL_THR   CONFIG_MERGE_WIRE_RXFIFO_FULL_THR
#else
#define RXFIFO_FULL_THR   64
#endif

#ifdef CONFIG_MERGE_WIRE_RX_TOUT_BITS
#define RX_TOUT_BITS      CONFIG_MERGE_WIRE_RX_TOUT_BITS
#else
#define RX_TOUT_BITS      22
#endif

#ifdef CONFIG_MERGE_WIRE_TXFIFO_EMPTY_THR
#define TXFIFO_EMPTY_THR  CONFIG_MERGE_WIRE_TXFIFO_EMPTY_THR
#else
#define TXFIFO_EMPTY_THR  16
#endif

#endif //MERGE_WIRE_TUNABLES_H