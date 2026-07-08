#include "driver.h"

#include <sys/param.h>

#include "esp_intr_types.h"
#include "esp_log.h"
#include "merge_wire.h"
#include "driver/uart.h"
#include "esp_private/critical_section.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "hal/uart_hal.h"
#include "soc/gpio_num.h"

#define UART_ENTER_CRITICAL_SAFE(spinlock)   esp_os_enter_critical_safe(spinlock)
#define UART_EXIT_CRITICAL_SAFE(spinlock)    esp_os_exit_critical_safe(spinlock)
#define UART_ENTER_CRITICAL_ISR(spinlock)    esp_os_enter_critical_isr(spinlock)
#define UART_EXIT_CRITICAL_ISR(spinlock)     esp_os_exit_critical_isr(spinlock)
#define UART_ENTER_CRITICAL(spinlock)        esp_os_enter_critical(spinlock)
#define UART_EXIT_CRITICAL(spinlock)         esp_os_exit_critical(spinlock)

#ifdef CONFIG_MERGE_WIRE_ISR_IN_IRAM
#define BRIDGE_ISR_ATTR     IRAM_ATTR
#define BRIDGE_MALLOC_CAPS  (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#else
#define BRIDGE_ISR_ATTR
#define BRIDGE_MALLOC_CAPS  MALLOC_CAP_DEFAULT
#endif

#define MAX_CHUNK_SIZE        255  /* data_len is a uint8_t                   */
#define QUIET_BITS            35   /* carrier sense: 3.5 chars before TX      */
#define DE_SETTLE_US          2    /* transceiver driver-enable time          */
#define RXFIFO_FULL_THR       64
#define RX_TOUT_BITS          22
#define TXFIFO_EMPTY_THR      16
#define MW_MAX_RETRIES        8    /* chunk replays before giving up          */
#define MW_JAM_BITS           0    /* >0: jam the bus with a break on clash   */
#define MW_FWD_BRK_BITS       12   /* regenerated break width (>= 1 frame);   */

#define RS485_ERR_MASK (UART_INTR_RS485_CLASH | UART_INTR_RS485_FRM_ERR | \
UART_INTR_RS485_PARITY_ERR)

// data chunks in the ringbuf, max size of 255 bytes per chunk
typedef struct {
  uint8_t tx_brk_len; // if this is >0, there has to be NO DATA in this chunk, break only
  uint8_t data_len;
  uint8_t data[];
} uart_bridge_data_t;

typedef struct {
  uart_port_t fd_uart_num;                 /*!< Full-duplex UART port number*/
  uart_port_t rs485_uart_num;              /*!< RS485 UART port number*/
  intr_handle_t fd_intr_handle;            /*!< Full-duplex UART interrupt handle*/
  intr_handle_t rs485_intr_handle;         /*!< RS485 UART interrupt handle*/

  // bridging ringbufs from either UART
  RingbufHandle_t rs485_to_uart_ring_buf;  /*!< RS485 RX to full-duplex UART TX ring buffer handle*/
  RingbufHandle_t uart_to_rs485_ring_buf;  /*!< Full-duplex UART RX to RS485 TX ring buffer handle*/
  // chunks to be sent in flight from the ringbuf
  uart_bridge_data_t *rs485_to_uart_chunk; /*!< RS485 RX to full-duplex UART TX chunk in flight*/
  uart_bridge_data_t *uart_to_rs485_chunk; /*!< Full-duplex UART RX to RS485 TX chunk in flight*/
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
} uart_port_context_t;

#define UART_CONTEXT_INIT_DEF(uart_num) { \
.port_id = uart_num, \
.hal.dev = UART_LL_GET_HW(uart_num), \
.sclk_sel = -1, \
INIT_CRIT_SECTION_LOCK_IN_STRUCT(spinlock) \
.hw_flwctrl_enabled = false, \
.tx_io_num = -1, \
.rx_io_num = -1, \
.rts_io_num = -1, \
.cts_io_num = -1, \
.dtr_io_num = -1, \
.dsr_io_num = -1, \
.io_reserved_mask = 0, \
}

static bridge_port_context_t driver_uart_ctx[UART_NUM_MAX] = {0};
static uart_port_context_t uart_context[UART_NUM_MAX] = {
  UART_CONTEXT_INIT_DEF(UART_NUM_0),
  UART_CONTEXT_INIT_DEF(UART_NUM_1),
#if SOC_UART_HP_NUM > 2
  UART_CONTEXT_INIT_DEF(UART_NUM_2),
#endif
#if SOC_UART_HP_NUM > 3
  UART_CONTEXT_INIT_DEF(UART_NUM_3),
#endif
#if SOC_UART_HP_NUM > 4
  UART_CONTEXT_INIT_DEF(UART_NUM_4),
#endif
#if (SOC_UART_LP_NUM >= 1)
  UART_CONTEXT_INIT_DEF(LP_UART_NUM_0),
#endif
};

static uint32_t BRIDGE_ISR_ATTR bridge_enable_tx_write_fifo(uart_port_t uart_num, const uint8_t *pbuf, uint32_t len)
{
  uint32_t sent_len = 0;
  UART_ENTER_CRITICAL_SAFE(&(uart_context[uart_num].spinlock));
  uart_hal_write_txfifo(&(uart_context[uart_num].hal), pbuf, len, &sent_len);
  UART_EXIT_CRITICAL_SAFE(&(uart_context[uart_num].spinlock));
  return sent_len;
}

static bool bridge_try_send_ring_buf(RingbufHandle_t ringbuf, const uint8_t *data, uint32_t len, bool *buf_full_flag, uart_port_t producer_port, BaseType_t *HPTaskAwoken, bool *need_yield) {
  BaseType_t sent = pdFALSE;
  // try to send the data to the ring buffer
  sent = xRingbufferSendFromISR(ringbuf, data, len, HPTaskAwoken);
  *need_yield |= (*HPTaskAwoken == pdTRUE);
  if (sent == pdFALSE) {
    // failed to send to the ring buffer, possibly full?
    // set the buffer full flag to indicate we have a chunk waiting
    *buf_full_flag = true;
    // disable the RX interrupts so we wait for the buffer to clear
    // the interrupts will only be re-enabled once the rs485 bus services the queue
    UART_ENTER_CRITICAL_ISR(&(uart_context[producer_port].spinlock));
    uart_hal_disable_intr_mask(&(uart_context[producer_port].hal),
                               UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                               UART_INTR_BRK_DET);
    UART_EXIT_CRITICAL_ISR(&(uart_context[producer_port].spinlock));
    // failed to push to buffer
    return false;
  }

  // successfully pushed data to the bridging buffer
  // return success
  return true;
}

static void bridge_service_producer(RingbufHandle_t ringbuf, uart_bridge_data_t *stashed, bool *buf_full_flag, uint8_t *brk_flg, uint8_t brk_len, uart_port_t producer_port, BaseType_t *HPTaskAwoken, bool *need_yield) {
  if (*buf_full_flag == true) {
    // buffer was previously full and we have a stashed chunk waiting
    // and possibly a stashed break too
    int stashed_len = stashed->data_len + sizeof(uart_bridge_data_t);

    BaseType_t sent = pdFALSE;
    // try to send the data to the ring buffer
    sent = xRingbufferSendFromISR(ringbuf, stashed, stashed_len, HPTaskAwoken);
    *need_yield |= (*HPTaskAwoken == pdTRUE);
    if (sent == pdTRUE) {
      // we have successfully queued the send to the buffer
      // check if we have a BRK waiting to be queued
      if (*brk_flg) {
        // we have a break waiting, try to send that
        // try to send the break to the ring buffer
        // notice we reuse the stashed because we know we just cleared it
        stashed->tx_brk_len = brk_len;
        stashed->data_len = 0;
        sent = xRingbufferSendFromISR(ringbuf, stashed, sizeof(uart_bridge_data_t), HPTaskAwoken);
        *need_yield |= (*HPTaskAwoken == pdTRUE);
        if (sent == pdFALSE) {
          // break did not send, wait till next time bridge_service_producer is called
          // but clear the brk waiting flag because we fed the brk into the stashed buffer already
          // so we treat it like a regular chunk
          *brk_flg = false;
          // early return to prevent clearing the flags erroneously
          return;
        }

        // successfully sent break
      }

      // pending events all succesfully sent, clear the buffer full flag
      *buf_full_flag = false;
      // and ONLY NOW we re-enable the interrupts
      UART_ENTER_CRITICAL_ISR(&(uart_context[producer_port].spinlock));
      uart_hal_ena_intr_mask(&(uart_context[producer_port].hal),
                               UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                               UART_INTR_BRK_DET);
      UART_EXIT_CRITICAL_ISR(&(uart_context[producer_port].spinlock));
    }
    // if not, we will try again next time and keep the stashed data
  }
}

/* drain a port's RX FIFO into its stash buffer as one chunk. If BRK_DET is
 * in the status, strip the break's artifact NUL (heuristic: it is the last
 * byte drained //TODO: verify if the crossing protocol can end in 0x00
 * and report the break so the caller can queue a break chunk AFTER data.   */
static void BRIDGE_ISR_ATTR bridge_read_fifo_chunk(uart_port_t port, uint8_t *stash_buf,
                                                   uint32_t status, bool *got_brk)
{
  uart_bridge_data_t *chunk = (uart_bridge_data_t *)stash_buf;
  int rx_fifo_len = MIN((int)uart_hal_get_rxfifo_len(&(uart_context[port].hal)),
                        MAX_CHUNK_SIZE);
  uart_hal_read_rxfifo(&(uart_context[port].hal), chunk->data, &rx_fifo_len);
  *got_brk = (status & UART_INTR_BRK_DET) != 0;
  if (*got_brk && rx_fifo_len > 0 && chunk->data[rx_fifo_len - 1] == 0x00) {
    rx_fifo_len--;
  }
  chunk->tx_brk_len = 0;
  chunk->data_len = (uint8_t)rx_fifo_len;
}

static void BRIDGE_ISR_ATTR bridge_uart_intr_handler(void *param) {
  bridge_port_context_t *p_port_ctx = param;
  uint8_t uart_num = p_port_ctx->port_id;
  bridge_context_t *c = p_port_ctx->uart_ctx;

  bool rx_buffer_free = true;
  uint32_t tx_fifo_rem = 0;
  uint32_t uart_intr_status = 0;
  BaseType_t HPTaskAwoken = 0;
  bool need_yield = false;

  // interrupt handler for UART RX to RS485 TX
  while ((uart_intr_status = uart_hal_get_intsts_mask(&(uart_context[uart_num].hal))) != 0) {
    if (uart_intr_status & UART_INTR_TXFIFO_EMPTY) {
      // tx fifo empty, clear the intr mask while we are handling txfifo empty
      UART_ENTER_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
      uart_hal_disable_intr_mask(&(uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);
      UART_EXIT_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
      uart_hal_clr_intsts_mask(&(uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);

      // if we have a break condition waiting, its normal for the txfifo to run empty
      // so we ignore and continue the interrupt handling
      if (c->fd_tx_waiting_brk) {
        continue;
      }

      // whether to enable txfifo empty interrupt after we fill the fifo
      bool en_tx_flg = false;
      tx_fifo_rem = uart_hal_get_txfifo_len(&(uart_context[uart_num].hal));

      // no break waiting, check if we have any data waiting to be sent
      while (tx_fifo_rem > 0) {
        // check if we have a chunk in flight
        if (c->rs485_to_uart_chunk_rem_len == 0) {
          // no chunk in flight, grab a new chunk from the ringbuf
          size_t size;
          uart_bridge_data_t *chunk = xRingbufferReceiveFromISR(c->rs485_to_uart_ring_buf, &size);
          if (chunk) {
            // received data from the ringbuf
            if (chunk->tx_brk_len > 0) {
              // no data in this chunk, set up break
              uart_hal_clr_intsts_mask(&(uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
              UART_ENTER_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
              uart_hal_tx_break(&(uart_context[uart_num].hal), chunk->tx_brk_len);
              uart_hal_ena_intr_mask(&(uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
              UART_EXIT_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
              c->fd_tx_waiting_brk = 1;

              //do not enable TX empty interrupt
              en_tx_flg = false;

              // return the item to the ISR since we are done processing it
              vRingbufferReturnItemFromISR(c->rs485_to_uart_ring_buf, chunk, &HPTaskAwoken);
              need_yield |= (HPTaskAwoken == pdTRUE);
              bridge_service_producer(
                c->rs485_to_uart_ring_buf,
                (uart_bridge_data_t *) c->rs485_rx_data_buf,
                &c->rs485_to_uart_buffer_full_flg,
                &c->rs485_tx_brk_flg, c->rs485_tx_brk_len,
                c->rs485_uart_num,
                &HPTaskAwoken, &need_yield);

              // break from the loop to handle the tx brk send
              break;
            }

            // this is a data chunk
            c->rs485_to_uart_chunk_rem_len = chunk->data_len;
            c->rs485_to_uart_chunk = chunk;
            c->fd_tx_ptr = chunk->data;
          } else {
            // cannot get data from ring buffer, return;
            break;
          }
        }

        if (c->rs485_to_uart_chunk_rem_len > 0) {
          // fill the TX fifo from the chunk
          uint32_t send_len = bridge_enable_tx_write_fifo(uart_num, (const uint8_t *) c->fd_tx_ptr,
            MIN(c->rs485_to_uart_chunk_rem_len, tx_fifo_rem));

          c->fd_tx_ptr += send_len;
          c->rs485_to_uart_chunk_rem_len -= send_len;
          tx_fifo_rem -= send_len;

          if (c->rs485_to_uart_chunk_rem_len == 0) {
            vRingbufferReturnItemFromISR(c->rs485_to_uart_ring_buf, c->rs485_to_uart_chunk, &HPTaskAwoken);
            need_yield |= (HPTaskAwoken == pdTRUE);
            c->fd_tx_ptr = NULL;
            bridge_service_producer(
                c->rs485_to_uart_ring_buf,
                (uart_bridge_data_t *) c->rs485_rx_data_buf,
                &c->rs485_to_uart_buffer_full_flg,
                &c->rs485_tx_brk_flg, c->rs485_tx_brk_len,
                c->rs485_uart_num,
                &HPTaskAwoken, &need_yield);
          }

          // enable TX empty interrupt to handle subsequent chunks
          en_tx_flg = true;
        }
      }

      if (en_tx_flg) {
        uart_hal_clr_intsts_mask(&(uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);
        UART_ENTER_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
        uart_hal_ena_intr_mask(&(uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);
        UART_EXIT_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
      }
    } else if (uart_intr_status & (UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                                   UART_INTR_BRK_DET)) {
      // either the RX timed out or the RX FIFO is full
      // or a break is detected (with its associated NUL in the buffer
      // which we will strip and queue a break chunk
      bool got_brk = false;
      if (c->uart_to_rs485_buffer_full_flg == false) {
        bridge_read_fifo_chunk(uart_num, c->fd_rx_data_buf, uart_intr_status, &got_brk);
        uart_bridge_data_t *chunk = (uart_bridge_data_t *)c->fd_rx_data_buf;


        // after we copy the data from the fifo into the rx data buffer, we clear the intr status
        uart_hal_clr_intsts_mask(&(uart_context[uart_num].hal),
                                 UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                                 UART_INTR_BRK_DET);

        if (chunk->data_len > 0) {
          // try to send the data to the ring buffer
          bool sent = bridge_try_send_ring_buf(
            c->uart_to_rs485_ring_buf,
            (const uint8_t *) chunk, chunk->data_len + sizeof(uart_bridge_data_t),
            &c->uart_to_rs485_buffer_full_flg,
            uart_num, &HPTaskAwoken, &need_yield);
          if (sent) {
            // successfully pushed data to the bridging buffer
            UART_ENTER_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
            c->uart_to_rs485_buffered_len += chunk->data_len;
            UART_EXIT_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
            rx_buffer_free = true;
          } else {
            // rx buffer containes stashed data, not free
            rx_buffer_free = false;
          }
        }

        if (got_brk) {
          // try to send the break to the ring buffer
          if (rx_buffer_free) {
            // the rx buf is already sent so we can use it to store the brk
            uart_bridge_data_t *brk = (uart_bridge_data_t *) c->fd_rx_data_buf;
            brk->tx_brk_len = MW_FWD_BRK_BITS;
            brk->data_len = 0;
            // try to send the data to the ring buffer
            bridge_try_send_ring_buf(
              c->uart_to_rs485_ring_buf,
              (const uint8_t *) brk, sizeof(uart_bridge_data_t),
              &c->uart_to_rs485_buffer_full_flg,
              uart_num, &HPTaskAwoken, &need_yield);
            // if failed, the data is already stashed inside the rx buffer, no need to set flags
          } else {
            // failed to send the data to the buffer, stash the brk without trying to send it
            // for when the queue is serviced, to preserve ordering (in case data fails but ringbuf has space for brk)
            c->rs485_tx_brk_flg = 1;
            c->rs485_tx_brk_len = MW_FWD_BRK_BITS;
          }
        }
      } else {
        // the bridging buffer is full
        // we DO NOT empty the fifo buffer
        // but we disable the interrupts (it should have already been disabled)
        UART_ENTER_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
        uart_hal_disable_intr_mask(&(uart_context[uart_num].hal), UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
        UART_EXIT_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
        uart_hal_clr_intsts_mask(&(uart_context[uart_num].hal), UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);

        // now we wait for the rs485 bus to service the queue in order to re-enable interrupts
      }
    } else if (uart_intr_status & UART_INTR_RXFIFO_OVF) {
      // When fifo overflows, we reset the fifo.
      UART_ENTER_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
      uart_hal_rxfifo_rst(&(uart_context[uart_num].hal));
      UART_EXIT_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
      uart_hal_clr_intsts_mask(&(uart_context[uart_num].hal), UART_INTR_RXFIFO_OVF);
    } else if (uart_intr_status & UART_INTR_TX_BRK_DONE) {
      UART_ENTER_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
      uart_hal_tx_break(&(uart_context[uart_num].hal), 0);
      uart_hal_disable_intr_mask(&(uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
      if (c->fd_tx_waiting_brk == 1) {
        uart_hal_ena_intr_mask(&(uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);
      }
      UART_EXIT_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
      uart_hal_clr_intsts_mask(&(uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
      if (c->fd_tx_waiting_brk == 1) {
        c->fd_tx_waiting_brk = 0;
      } else {
        xSemaphoreGiveFromISR(c->fd_tx_brk_sem, &HPTaskAwoken);
        need_yield |= (HPTaskAwoken == pdTRUE);
      }
    } else if (uart_intr_status & UART_INTR_TX_DONE) {
      // Workaround for RS485: If the RS485 half duplex mode is active
      // and transmitter is in idle state then reset received buffer and reset RTS pin
      // skip this behavior for other UART modes
      uart_hal_clr_intsts_mask(&(uart_context[uart_num].hal), UART_INTR_TX_DONE);
      UART_ENTER_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
      uart_hal_disable_intr_mask(&(uart_context[uart_num].hal), UART_INTR_TX_DONE);

      UART_EXIT_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
      xSemaphoreGiveFromISR(c[uart_num].fd_tx_done_sem, &HPTaskAwoken);
      need_yield |= (HPTaskAwoken == pdTRUE);
    } else {
      uart_hal_clr_intsts_mask(&(uart_context[uart_num].hal), uart_intr_status); /*simply clear all other intr status*/
    }
  }
  if (need_yield) {
    portYIELD_FROM_ISR();
  }
}

static void BRIDGE_ISR_ATTR bridge_rs485_intr_handler(void *param) {
  bridge_port_context_t *p_uart = param;
  uint8_t uart_num = p_uart->port_id;
  bridge_context_t *c = p_uart->uart_ctx;

  uint32_t uart_intr_status = 0;
  BaseType_t HPTaskAwoken = 0;
  bool need_yield = false;

  // interrupt handler for RS485 RX to UART TX
  while ((uart_intr_status = uart_hal_get_intsts_mask(&(uart_context[uart_num].hal))) != 0) {
    if (uart_intr_status & UART_INTR_TXFIFO_EMPTY) {
      // tx fifo empty
    } else if ((uart_intr_status & UART_INTR_RXFIFO_TOUT) || (uart_intr_status & UART_INTR_RXFIFO_FULL)) {

    } else if (uart_intr_status & UART_INTR_RXFIFO_OVF) {

    } else if (uart_intr_status & UART_INTR_BRK_DET) {

    } else if (uart_intr_status & UART_INTR_FRAM_ERR) {

    } else if (uart_intr_status & UART_INTR_PARITY_ERR) {

    } else if (uart_intr_status & UART_INTR_TX_BRK_DONE) {

    } else if (uart_intr_status & UART_INTR_TX_BRK_IDLE) {

    } else if (uart_intr_status & UART_INTR_CMD_CHAR_DET) {
      uart_hal_clr_intsts_mask(&(uart_context[uart_num].hal), UART_INTR_CMD_CHAR_DET);
    } else if ((uart_intr_status & UART_INTR_RS485_PARITY_ERR)
                   || (uart_intr_status & UART_INTR_RS485_FRM_ERR)
                   || (uart_intr_status & UART_INTR_RS485_CLASH)) {
      // here we handle the collision detection
      // where do we enable the interrupts?
    } else if (uart_intr_status & UART_INTR_TX_DONE) {
      if (uart_hal_is_tx_idle(&(uart_context[uart_num].hal)) != true) {
        // The TX_DONE interrupt is triggered but transmit is active
        // then postpone interrupt processing for next interrupt
      } else {
        // Workaround for RS485: If the RS485 half duplex mode is active
        // and transmitter is in idle state then reset received buffer and reset RTS pin
        // skip this behavior for other UART modes
        uart_hal_clr_intsts_mask(&(uart_context[uart_num].hal), UART_INTR_TX_DONE);
        UART_ENTER_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
        uart_hal_disable_intr_mask(&(uart_context[uart_num].hal), UART_INTR_TX_DONE);

        // specific for RS485 mode
        uart_hal_rxfifo_rst(&(uart_context[uart_num].hal));
        uart_hal_set_rts(&(uart_context[uart_num].hal), 1);

        UART_EXIT_CRITICAL_ISR(&(uart_context[uart_num].spinlock));
        xSemaphoreGiveFromISR(c[uart_num].rs485_tx_done_sem, &HPTaskAwoken);
        need_yield |= (HPTaskAwoken == pdTRUE);
      }
    } else {
      uart_hal_clr_intsts_mask(&(uart_context[uart_num].hal), uart_intr_status); /*simply clear all other intr status*/
    }
  }
  if (need_yield) {
    portYIELD_FROM_ISR();
  }
}

static void bridge_free_driver_ctx(bridge_context_t *uart_ctx) {
  if (uart_ctx) {
    if (uart_ctx->fd_rx_data_buf) heap_caps_free(uart_ctx->fd_rx_data_buf);
    if (uart_ctx->rs485_rx_data_buf) heap_caps_free(uart_ctx->rs485_rx_data_buf);

    if (uart_ctx->rs485_to_uart_ring_buf) vRingbufferDeleteWithCaps(uart_ctx->rs485_to_uart_ring_buf);
    if (uart_ctx->uart_to_rs485_ring_buf) vRingbufferDeleteWithCaps(uart_ctx->uart_to_rs485_ring_buf);

    if (uart_ctx->fd_tx_mux) vSemaphoreDeleteWithCaps(uart_ctx->fd_tx_mux);
    if (uart_ctx->fd_rx_mux) vSemaphoreDeleteWithCaps(uart_ctx->fd_rx_mux);
    if (uart_ctx->fd_tx_brk_sem) vSemaphoreDeleteWithCaps(uart_ctx->fd_tx_brk_sem);
    if (uart_ctx->fd_tx_done_sem) vSemaphoreDeleteWithCaps(uart_ctx->fd_tx_done_sem);
    if (uart_ctx->fd_tx_fifo_sem) vSemaphoreDeleteWithCaps(uart_ctx->fd_tx_fifo_sem);

    if (uart_ctx->rs485_tx_done_sem) vSemaphoreDeleteWithCaps(uart_ctx->rs485_tx_done_sem);
    if (uart_ctx->rs485_tx_brk_sem) vSemaphoreDeleteWithCaps(uart_ctx->rs485_tx_brk_sem);

    heap_caps_free(uart_ctx);
  }
}

static bridge_context_t *bridge_alloc_driver_ctx(uart_port_t uart_num, uart_port_t rs485_num, int buffer_size) {
  bridge_context_t *uart_ctx = heap_caps_calloc(1, sizeof(bridge_context_t), BRIDGE_MALLOC_CAPS);
  if (!uart_ctx) {
    return NULL;
  }

  // set the respective uart numbers belonging to the bridge
  uart_ctx->fd_uart_num = uart_num;
  uart_ctx->rs485_uart_num = rs485_num;

  // allocate the buffer to store the FIFO data of the UART peripheral
  uart_ctx->fd_rx_data_buf = heap_caps_calloc(UART_HW_FIFO_LEN(uart_num) + sizeof(uart_bridge_data_t), sizeof(uint32_t), BRIDGE_MALLOC_CAPS);
  uart_ctx->rs485_rx_data_buf = heap_caps_calloc(UART_HW_FIFO_LEN(rs485_num) + sizeof(uart_bridge_data_t), sizeof(uint32_t), BRIDGE_MALLOC_CAPS);
  if (!uart_ctx->fd_rx_data_buf || !uart_ctx->rs485_rx_data_buf) {
    goto err;
  }

  // allocate the glue buffers for bridging data
  // both are allocated as nosplit buffers in order to store the break condition
  uart_ctx->rs485_to_uart_ring_buf = xRingbufferCreateWithCaps(buffer_size, RINGBUF_TYPE_NOSPLIT, BRIDGE_MALLOC_CAPS);
  uart_ctx->uart_to_rs485_ring_buf = xRingbufferCreateWithCaps(buffer_size, RINGBUF_TYPE_NOSPLIT, BRIDGE_MALLOC_CAPS);
  if (!uart_ctx->rs485_to_uart_ring_buf || !uart_ctx->uart_to_rs485_ring_buf) {
    goto err;
  }

  uart_ctx->fd_tx_mux = xSemaphoreCreateMutexWithCaps(BRIDGE_MALLOC_CAPS);
  uart_ctx->fd_rx_mux = xSemaphoreCreateMutexWithCaps(BRIDGE_MALLOC_CAPS);
  uart_ctx->fd_tx_brk_sem = xSemaphoreCreateBinaryWithCaps(BRIDGE_MALLOC_CAPS);
  uart_ctx->fd_tx_done_sem = xSemaphoreCreateBinaryWithCaps(BRIDGE_MALLOC_CAPS);
  uart_ctx->fd_tx_fifo_sem = xSemaphoreCreateBinaryWithCaps(BRIDGE_MALLOC_CAPS);
  if (!uart_ctx->fd_tx_mux || !uart_ctx->fd_rx_mux || !uart_ctx->fd_tx_brk_sem ||
          !uart_ctx->fd_tx_done_sem || !uart_ctx->fd_tx_fifo_sem) {
    goto err;
  }

  uart_ctx->rs485_tx_brk_sem = xSemaphoreCreateBinaryWithCaps(BRIDGE_MALLOC_CAPS);
  uart_ctx->rs485_tx_done_sem = xSemaphoreCreateBinaryWithCaps(BRIDGE_MALLOC_CAPS);
  if (!uart_ctx->rs485_tx_brk_sem || !uart_ctx->rs485_tx_done_sem) {
    goto err;
  }

  return uart_ctx;

err:
    bridge_free_driver_ctx(uart_ctx);
  return NULL;
}

esp_err_t bridge_driver_install(fullduplex_uart_config_t *fd_uart_cfg, rs485_uart_config_t *rs485_uart_cfg, int buffer_size, int fd_intr_alloc_flags, int rs485_intr_alloc_flags) {
  uart_port_t uart_num = fd_uart_cfg->uart_num;
  uart_port_t rs485_num = rs485_uart_cfg->uart_num;
  if (driver_uart_ctx[uart_num].uart_ctx == NULL && driver_uart_ctx[rs485_num].uart_ctx == NULL) {
    // create the driver object and assign to both the
    // full-duplex UART port and the RS485 UART port
    bridge_context_t *uart_ctx = bridge_alloc_driver_ctx(uart_num, rs485_num, buffer_size);
    if (uart_ctx == NULL) {
      ESP_LOGE(MERGE_WIRE_TAG, "UART driver malloc error");
      return ESP_FAIL;
    }
    driver_uart_ctx[uart_num].port_id = uart_num;
    driver_uart_ctx[uart_num].uart_ctx = uart_ctx;
    driver_uart_ctx[rs485_num].port_id = uart_num;
    driver_uart_ctx[rs485_num].uart_ctx = uart_ctx;

    uart_ctx->fd_tx_waiting_brk = 0;
    uart_ctx->rs485_tx_waiting_brk = 0;

    uart_ctx->fd_tx_ptr = NULL;
    uart_ctx->rs485_tx_ptr = NULL;
    uart_ctx->uart_to_rs485_chunk_rem_len = 0;
    uart_ctx->rs485_to_uart_chunk_rem_len = 0;
    uart_ctx->uart_to_rs485_buffered_len = 0;
    uart_ctx->rs485_to_uart_buffered_len = 0;

    uart_ctx->fd_tx_brk_flg = 0;
    uart_ctx->fd_tx_brk_len = 0;
    uart_ctx->rs485_tx_brk_flg = 0;
    uart_ctx->rs485_tx_brk_len = 0;
  } else {
    ESP_LOGE(MERGE_WIRE_TAG, "UART driver already installed on one or more of supplied UART port nums!");
    return ESP_FAIL;
  }

  return ESP_OK;
}


esp_err_t bridge_driver_set_pins(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t rts_pin, gpio_num_t cts_pin, gpio_num_t dtr_pin, gpio_num_t dsr_pin) {
  esp_err_t ret;
  // we use the esp-idf driver function to ensure the IOMUX and gpio matrix is
  // properly configured, no point doing repeated work
  ESP_RETURN_ON_FALSE(((ret = uart_set_pin(
      uart_num,
      tx_pin, rx_pin,
      rts_pin, cts_pin,
      dtr_pin, dsr_pin
      )) == ESP_OK), ret, MERGE_WIRE_TAG, "uart_set_pin error");

  // but we need to know the pin numbers as well for software control of the
  // rts/cts/dtr/dsr pins if need be, so we store them in our own context
  uart_context[uart_num].tx_io_num = tx_pin;
  uart_context[uart_num].rx_io_num = rx_pin;
  uart_context[uart_num].rts_io_num = rts_pin;
  uart_context[uart_num].cts_io_num = cts_pin;
  uart_context[uart_num].dtr_io_num = dtr_pin;
  uart_context[uart_num].dsr_io_num = dsr_pin;

  return ESP_OK;
}
