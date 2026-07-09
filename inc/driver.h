#ifndef MERGE_WIRE_DRIVER_H
#define MERGE_WIRE_DRIVER_H

#include "merge_wire.h"

#include "esp_err.h"
#include "esp_timer.h"

#include "esp_private/critical_section.h"
#include "freertos/ringbuf.h"
#include "hal/uart_hal.h"
#include "hal/uart_types.h"

#define MERGE_WIRE_TAG "merge_wire"

// data chunks in the ringbuf, max size of 255 bytes per chunk
typedef struct {
  uint8_t tx_brk_len; // if this is >0, there has to be NO DATA in this chunk, break only
  uint8_t data_len;
  uint8_t data[];
} bridge_uart_data_t;

// RS485 direction/burst state, DE is wired to RTS using uart_hal_set_rts(hal, 0)
// drives the pin HIGH (DE asserted), set_rts(hal, 1) releases it.
typedef enum {
  BRIDGE_RS485_IDLE = 0,  /*!< DE released, listening*/
  BRIDGE_RS485_TX,        /*!< DE asserted, chunk in flight*/
  BRIDGE_RS485_BRK,       /*!< sending a forwarded break, DE held through it*/
  BRIDGE_RS485_JAM,       /*!< collision + jam break (MW_JAM_BITS > 0), DE held*/
  BRIDGE_RS485_DRAIN,     /*!< collision: DE dropped, waiting for the shifter to idle*/
} bridge_rs485_state_t;

typedef struct {
  uart_port_t producer_port;
  uart_port_t consumer_port;

  // bridging ringbufs from either UART
  RingbufHandle_t bridging_ringbuf;        /*!< Bridging buffer from producer to consumer*/
  bridge_uart_data_t *chunk_in_flight;     /*!< Consumer current chunk in flight to be sent*/
  uint8_t *tx_ptr;                         /*!< Consumer current chunk in flight data send ptr*/
  uint32_t chunk_rem_len;                  /*!< Consumer current chunk in flight remaining length to be sent*/

  // buffer health
  bool buffer_full_flg;                    /*!< Bridging buffer full flag*/
  int buffered_len;                        /*!< Bridging buffer cached data length in bytes*/

  // fifo data buffers
  uint8_t *producer_rx_data_buf;           /*!< Producer data buffer to stash RX FIFO data or store stashed chunks to be sent to consumer*/

  // break send flags
  bool consumer_tx_waiting_brk;            /*!< Flag to indicate that producer TX FIFO is ready to send break signal after FIFO is empty, do not push data into TX FIFO right now*/
  bool tx_brk_stashed_flg;                 /*!< Flag to indicate to send a break signal in ring buf once space available (stashed brk) for consumer*/
  uint8_t tx_brk_stashed_len;              /*!< Stashed break signal cycle length/number for consumer*/
} bridge_port_obj_t;

typedef struct {
  uart_port_t fd_uart_num;                 /*!< Full-duplex UART port number*/
  uart_port_t rs485_uart_num;              /*!< RS485 UART port number*/
  intr_handle_t fd_intr_handle;            /*!< Full-duplex UART interrupt handle*/
  intr_handle_t rs485_intr_handle;         /*!< RS485 UART interrupt handle*/

  bridge_port_obj_t fd_uart_to_rs485;      /*!< Full-duplex UART RX to RS485 TX buffers and state*/
  bridge_port_obj_t rs485_to_fd_uart;      /*!< RS485 RX to full-duplex UART TX buffers and state*/

  // rs485 burst machine
  bridge_rs485_state_t r_state;            /*!< RS485 UART FSM state*/
  uint8_t burst_retries;                   /*!< RS485 UART replay count of the chunk in flight*/
  bool kick_scheduled_flg;                 /*!< RS485 UART whether we have a next kick scheduled*/
  uint64_t kick_scheduled_us;              /*!< RS485 UART time in microseconds when the next kick is scheduled*/
  int64_t last_bus_rx_us;                  /*!< RS485 UART last RS485 bus activity*/
  uint32_t quiet_us;                       /*!< RS485 UART carrier-sense window*/
  uint32_t rnd;                            /*!< RS485 UART xorshift32 for backoff jitter*/
  esp_timer_handle_t kick;                 /*!< RS485 UART deferred pump / backoff timer*/

  // diagnostics: inspect in a debugger; wire a getter later if wanted
  uint32_t collisions, retries_total, tx_giveups, breaks_forwarded;

  /* one lock for ALL shared bridge state: both ISRs and the kick timer
   * callback take it, so correctness is independent of core and level.
   * explicit portMUX because this struct is heap-allocated: a calloc'd
   * zero portMUX is NOT a valid unlocked spinlock.                          */
  portMUX_TYPE lock;

  // semaphores
  SemaphoreHandle_t fd_rx_mux;             /*!< Full-duplex UART RX data mutex*/
  SemaphoreHandle_t fd_tx_mux;             /*!< Full-duplex UART TX mutex*/
  SemaphoreHandle_t fd_tx_fifo_sem;        /*!< Full-duplex UART TX FIFO semaphore*/
  SemaphoreHandle_t fd_tx_done_sem;        /*!< Full-duplex UART TX done semaphore*/
  SemaphoreHandle_t fd_tx_brk_sem;         /*!< Full-duplex UART TX send break done semaphore*/

  SemaphoreHandle_t rs485_tx_done_sem;     /*!< RS485 UART TX done semaphore*/
  SemaphoreHandle_t rs485_tx_brk_sem;      /*!< RS485 UART TX send break done semaphore*/
} bridge_context_t;

typedef struct {
  uart_port_t port_id;                     /*!< Self UART port number*/
  bridge_context_t *uart_ctx;              /*!< UART RS485 bridge context object*/
} bridge_port_context_t;

// modelled after the esp-idf uart driver code
typedef struct {
  uart_port_t port_id;
  uart_hal_context_t hal;        /*!< UART hal context*/
  soc_module_clk_t sclk_sel;     /*!< UART port clock source selection*/
  DECLARE_CRIT_SECTION_LOCK_IN_STRUCT(spinlock)
  bool hw_flwctrl_enabled;
  gpio_num_t tx_io_num;
  gpio_num_t rx_io_num;
  gpio_num_t rts_io_num;
  gpio_num_t cts_io_num;
  gpio_num_t dtr_io_num;
  gpio_num_t dsr_io_num;
  uint64_t io_reserved_mask;
} bridge_uart_port_context_t;

/**
 * @brief Install the ISR-driven bridge core on two already-configured ports.
 *
 * Both ports must have been through merge_wire_config() (params + pins) and
 * must NOT have the esp-idf uart driver installed: this call sets the RS485
 * collision-detect register mode, releases DE, programs the FIFO thresholds
 * and allocates both UART interrupt sources for the bridge ISRs.
 *
 * @param fd_uart_cfg            full-duplex port description
 * @param rs485_uart_cfg         RS485 port description
 * @param rx_buffer_size         RS485 -> full-duplex ring buffer size
 * @param tx_buffer_size         full-duplex -> RS485 ring buffer size
 *                               (larger: it absorbs collision backoff)
 * @param fd_intr_alloc_flags    ESP_INTR_FLAG_* for the full-duplex ISR
 * @param rs485_intr_alloc_flags ESP_INTR_FLAG_* for the RS485 ISR
 */
esp_err_t bridge_driver_install(fullduplex_uart_config_t *fd_uart_cfg,
                                rs485_uart_config_t *rs485_uart_cfg,
                                int rs485_to_uart_buffer_size, int uart_to_rs485_buffer_size,
                                int fd_intr_alloc_flags, int rs485_intr_alloc_flags);

esp_err_t bridge_driver_set_pins(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin,
                                 gpio_num_t rts_pin, gpio_num_t cts_pin,
                                 gpio_num_t dtr_pin, gpio_num_t dsr_pin);

/** Snapshot of the bridge counters; pass either bridged port number. */
void bridge_driver_get_stats(uart_port_t any_bridge_port, bridge_stats_t *out);

#endif //MERGE_WIRE_DRIVER_H