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
  uart_port_t fd_uart_num;                 /*!< Full-duplex UART port number*/
  uart_port_t rs485_uart_num;              /*!< RS485 UART port number*/
  intr_handle_t fd_intr_handle;            /*!< Full-duplex UART interrupt handle*/
  intr_handle_t rs485_intr_handle;         /*!< RS485 UART interrupt handle*/

  // bridging ringbufs from either UART
  RingbufHandle_t rs485_to_uart_ring_buf;  /*!< RS485 RX to full-duplex UART TX ring buffer handle*/
  RingbufHandle_t uart_to_rs485_ring_buf;  /*!< Full-duplex UART RX to RS485 TX ring buffer handle*/
  // chunks to be sent in flight from the ringbuf
  bridge_uart_data_t *rs485_to_uart_chunk; /*!< RS485 RX to full-duplex UART TX chunk in flight*/
  bridge_uart_data_t *uart_to_rs485_chunk; /*!< Full-duplex UART RX to RS485 TX chunk in flight*/
  uint8_t *fd_tx_ptr;                      /*!< RS485 RX to full-duplex UART TX send ptr*/
  uint8_t *rs485_tx_ptr;                   /*!< Full-duplex UART RX to RS485 TX send ptr*/
  uint32_t rs485_to_uart_chunk_rem_len;   /*!< Remaining data length of the current processing chunk of the transaction to be sent from full-duplex UART*/
  uint32_t uart_to_rs485_chunk_rem_len;   /*!< Remaining data length of the current processing chunk of the transaction to be sent from RS485 UART*/

  // buffer health
  bool rs485_to_uart_buffer_full_flg;      /*!< RS485 RX to full-duplex UART TX ring buffer full flag. */
  bool uart_to_rs485_buffer_full_flg;      /*!< Full-duplex UART RX to RS485 TX ring buffer full flag. */
  int rs485_to_uart_buffered_len;          /*!< RS485 RX to full-duplex UART TX cached data length */
  int uart_to_rs485_buffered_len;          /*!< Full-duplex UART RX to RS485 TX cached data length */

  // fifo data buffers
  uint8_t *fd_rx_data_buf;                 /*!< Data buffer to stash full-duplex UART FIFO data*/
  uint8_t *rs485_rx_data_buf;              /*!< Data buffer to stash RS485 UART FIFO data*/

  // rs485 burst machine
  bridge_rs485_state_t r_state;
  uint8_t burst_retries;                   /*!< replays of the chunk in flight */
  int64_t last_bus_rx_us;                  /*!< last RS485 bus activity        */
  uint32_t quiet_us;                       /*!< carrier-sense window           */
  uint32_t rnd;                            /*!< xorshift32 for backoff jitter  */
  esp_timer_handle_t kick;                 /*!< deferred pump / backoff timer  */

  // diagnostics: inspect in a debugger; wire a getter later if wanted
  uint32_t collisions, retries_total, tx_giveups, breaks_forwarded;

  /* one lock for ALL shared bridge state: both ISRs and the kick timer
   * callback take it, so correctness is independent of core and level.
   * explicit portMUX because this struct is heap-allocated: a calloc'd
   * zero portMUX is NOT a valid unlocked spinlock.                          */
  portMUX_TYPE lock;

  // break send flags
  uint8_t fd_tx_waiting_brk;               /*!< Full-duplex UART flag to indicate that TX FIFO is ready to send break signal after FIFO is empty, do not push data into TX FIFO right now.*/
  uint8_t rs485_tx_waiting_brk;            /*!< RS485 UART flag to indicate to send a break signal in the end of the item sending procedure */
  uint8_t fd_tx_brk_flg;                   /*!< Full-duplex UART flag to indicate to send a break signal in ring buf once empty */
  uint8_t fd_tx_brk_len;                   /*!< Full-duplex UART TX break signal cycle length/number */
  uint8_t rs485_tx_brk_flg;                /*!< RS485 UART flag to indicate to send a break signal in ring buf once empty */
  uint8_t rs485_tx_brk_len;                /*!< RS485 UART TX break signal cycle length/number */

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