#include "driver_common.h"

#include <sys/param.h>

#include "esp_intr_types.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "merge_wire.h"
#include "tunables.h"
#include "driver/uart.h"
#include "esp_private/critical_section.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "hal/uart_hal.h"
#include "hal/uart_periph.h"
#include "soc/gpio_num.h"

#define RS485_ERR_MASK (UART_INTR_RS485_CLASH | UART_INTR_RS485_FRM_ERR | \
UART_INTR_RS485_PARITY_ERR)


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

bridge_port_context_t merge_wire_driver_uart_ctx[UART_NUM_MAX] = {0};
bridge_uart_port_context_t merge_wire_uart_context[UART_NUM_MAX] = {
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

uint32_t BRIDGE_ISR_ATTR bridge_enable_tx_write_fifo(uart_port_t uart_num, const uint8_t *pbuf, uint32_t len) {
  uint32_t sent_len = 0;
  UART_ENTER_CRITICAL_SAFE(&(merge_wire_uart_context[uart_num].spinlock));
  uart_hal_write_txfifo(&(merge_wire_uart_context[uart_num].hal), pbuf, len, &sent_len);
  UART_EXIT_CRITICAL_SAFE(&(merge_wire_uart_context[uart_num].spinlock));
  return sent_len;
}

bool BRIDGE_ISR_ATTR bridge_try_send_ring_buf(bridge_port_obj_t *ctx, const uint8_t *data, uint32_t len, BaseType_t *HPTaskAwoken, bool *need_yield) {
  uart_port_t producer_port = ctx->producer_port;
  BaseType_t sent = pdFALSE;
  // try to send the data to the ring buffer
  sent = xRingbufferSendFromISR(ctx->bridging_ringbuf, data, len, HPTaskAwoken);
  *need_yield |= (*HPTaskAwoken == pdTRUE);
  if (sent == pdFALSE) {
    // failed to send to the ring buffer, possibly full?
    // set the buffer full flag to indicate we have a chunk waiting
    ctx->buffer_full_flg = true;
    // disable the RX interrupts so we wait for the buffer to clear
    // the interrupts will only be re-enabled once the rs485 bus services the queue
    UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[producer_port].spinlock));
    uart_hal_disable_intr_mask(&(merge_wire_uart_context[producer_port].hal),
                               UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                               UART_INTR_BRK_DET);
    UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[producer_port].spinlock));
    // failed to push to buffer
    return false;
  }

  // successfully pushed data to the bridging buffer
  // return success
  return true;
}

void BRIDGE_ISR_ATTR bridge_service_producer(bridge_port_obj_t *ctx, BaseType_t *HPTaskAwoken, bool *need_yield) {
  uart_port_t producer_port = ctx->producer_port;
  if (ctx->buffer_full_flg == true) {
    // buffer was previously full and we have a stashed chunk waiting
    // and possibly a stashed break too
    bridge_uart_data_t *stashed = (bridge_uart_data_t *) ctx->producer_rx_data_buf;
    int stashed_len = stashed->data_len + (int)sizeof(bridge_uart_data_t);

    BaseType_t sent = pdFALSE;
    // try to send the data to the ring buffer
    sent = xRingbufferSendFromISR(ctx->bridging_ringbuf, stashed, stashed_len, HPTaskAwoken);
    *need_yield |= (*HPTaskAwoken == pdTRUE);
    if (sent == pdTRUE) {
      // we have successfully queued the send to the buffer
      // check if we have a BRK waiting to be queued
      if (ctx->tx_brk_stashed_flg) {
        // we have a break waiting, try to send that
        // try to send the break to the ring buffer
        // notice we reuse the stashed because we know we just cleared it
        stashed->tx_brk_len = ctx->tx_brk_stashed_len;
        stashed->data_len = 0;
        sent = xRingbufferSendFromISR(ctx->bridging_ringbuf, stashed, sizeof(bridge_uart_data_t), HPTaskAwoken);
        *need_yield |= (*HPTaskAwoken == pdTRUE);
        if (sent == pdFALSE) {
          // break did not send, wait till next time bridge_service_producer is called
          // but clear the brk waiting flag because we fed the brk into the stashed buffer already
          // so we treat it like a regular chunk
          ctx->tx_brk_stashed_flg = false;
          // early return to prevent clearing the flags erroneously
          return;
        }

        // successfully sent break
      }

      // pending events all succesfully sent, clear the buffer full flag
      ctx->buffer_full_flg = false;
      // and ONLY NOW we re-enable the interrupts
      UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[producer_port].spinlock));
      uart_hal_ena_intr_mask(&(merge_wire_uart_context[producer_port].hal),
                               UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                               UART_INTR_BRK_DET);
      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[producer_port].spinlock));
    }
    // if not, we will try again next time and keep the stashed data
  }
}

void BRIDGE_ISR_ATTR bridge_arm_brk_chunk(bridge_port_obj_t *ctx, bridge_uart_data_t *chunk, BaseType_t *HPTaskAwoken, bool *need_yield) {
  uart_port_t consumer_port = ctx->consumer_port;
  uart_hal_clr_intsts_mask(&(merge_wire_uart_context[consumer_port].hal), UART_INTR_TX_BRK_DONE);
  UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[consumer_port].spinlock));
  uart_hal_tx_break(&(merge_wire_uart_context[consumer_port].hal), chunk->tx_brk_len);
  uart_hal_ena_intr_mask(&(merge_wire_uart_context[consumer_port].hal), UART_INTR_TX_BRK_DONE);
  UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[consumer_port].spinlock));
  ctx->consumer_tx_waiting_brk = true;
  ctx->breaks_tx++;
  vRingbufferReturnItemFromISR(ctx->bridging_ringbuf, chunk, HPTaskAwoken);
  *need_yield |= (*HPTaskAwoken == pdTRUE);
  bridge_service_producer(ctx, HPTaskAwoken, need_yield);
}

void BRIDGE_ISR_ATTR bridge_fill_tx_fifo(bridge_port_obj_t *ctx, bool defer_return, bool *brk_waiting_int_ena, bool *chunk_in_flight, bool *ringbuf_empty, BaseType_t *HPTaskAwoken, bool *need_yield) {
  uart_port_t consumer_port = ctx->consumer_port;
  // whether or not we enabled the break interrupts because a break is scheduled
  if (brk_waiting_int_ena) *brk_waiting_int_ena = false;
  // whether or not there is still a chunk in flight
  if (chunk_in_flight) *chunk_in_flight = false;
  // whether or not the bridging ringbuf is empty
  if (ringbuf_empty) *ringbuf_empty = false;

  // if we have a break condition waiting, its normal for the txfifo to run empty
  // so we ignore and continue the interrupt handling
  if (ctx->consumer_tx_waiting_brk) {
    // signal that we have a break waiting
    // and that we enabled the break interrupts
    if (brk_waiting_int_ena) *brk_waiting_int_ena = true;
    return;
  }

  uint32_t tx_fifo_rem = uart_hal_get_txfifo_len(&(merge_wire_uart_context[consumer_port].hal));
  if (defer_return && tx_fifo_rem > MW_FEED_CAP) {
    // rs485 consumer only: the TX FIFO cannot be flushed (UART1/2 erratum),
    // so everything queued at the instant of a collision drains onto the
    // dead bus before cleanup can finish. Capping each fill pass bounds
    // that residue (32 B ~ 2.8 ms at 115200 vs ~11 ms for a full FIFO);
    // the TXFIFO_EMPTY watermark keeps streaming gap-free regardless.
    tx_fifo_rem = MW_FEED_CAP;
  }

  // no break waiting, check if we have any data waiting to be sent
  while (tx_fifo_rem > 0) {
    // check if we have a chunk in flight
    if (ctx->chunk_rem_len == 0 && ctx->chunk_in_flight == NULL) {
      // no chunk in flight, grab a new chunk from the ringbuf.
      // (under defer_return a fully-fed chunk keeps chunk_in_flight set until
      // the TX_DONE commit -- we must NOT pull past it)
      size_t size;
      bridge_uart_data_t *chunk = xRingbufferReceiveFromISR(ctx->bridging_ringbuf, &size);
      if (chunk) {
        // received data from the ringbuf
        if (chunk->tx_brk_len > 0) {
          // no data in this chunk: it is an in-band break condition
          bridge_arm_brk_chunk(ctx, chunk, HPTaskAwoken, need_yield);
          // signal that we have a break waiting
          // and that we enabled the break interrupts
          if (brk_waiting_int_ena) *brk_waiting_int_ena = true;

          // break from the loop to handle the tx brk send
          break;
        }

        // this is a data chunk
        ctx->chunk_rem_len = chunk->data_len;
        ctx->chunk_in_flight = chunk;
        ctx->tx_ptr = chunk->data;
      } else {
        // cannot get data from ring buffer, return;
        if (ringbuf_empty) *ringbuf_empty = true;
        break;
      }
    }

    if (ctx->chunk_rem_len > 0) {
      // fill the TX fifo from the chunk
      uint32_t send_len = bridge_enable_tx_write_fifo(consumer_port, ctx->tx_ptr, MIN(ctx->chunk_rem_len, tx_fifo_rem));
      if (chunk_in_flight) *chunk_in_flight = true;
      ctx->tx_bytes += send_len;

      ctx->tx_ptr += send_len;
      ctx->chunk_rem_len -= send_len;
      tx_fifo_rem -= send_len;

      if (ctx->chunk_rem_len == 0) {
        if (defer_return) {
          // rs485 consumer: the chunk is FED but not yet VERIFIED on the wire.
          // keep it un-returned so the collision path can rewind (or drop) it;
          // the TX_DONE(idle) commit returns it. do not pull the next chunk.
          break;
        }
        vRingbufferReturnItemFromISR(ctx->bridging_ringbuf, ctx->chunk_in_flight, HPTaskAwoken);
        *need_yield |= (*HPTaskAwoken == pdTRUE);
        ctx->chunk_in_flight = NULL;
        ctx->tx_ptr = NULL;
        bridge_service_producer(ctx, HPTaskAwoken, need_yield);
        // chunk fully sent
      }
      // chunk partially sent
    }
  }
}

// drain a port's RX FIFO into its stash buffer as one chunk. If BRK_DET is
// in the status, strip the break's artifact NUL (heuristic: it is the last
// byte drained TODO: verify if the crossing protocol can end in 0x00
// and report the break so the caller can queue a break chunk AFTER data.
void BRIDGE_ISR_ATTR bridge_read_fifo_chunk(uart_port_t port, uint8_t *stash_buf, uint32_t status, bool *got_brk) {
  bridge_uart_data_t *chunk = (bridge_uart_data_t *)stash_buf;
  int rx_fifo_len = MIN((int)uart_hal_get_rxfifo_len(&(merge_wire_uart_context[port].hal)),
                        MAX_CHUNK_SIZE);
  uart_hal_read_rxfifo(&(merge_wire_uart_context[port].hal), chunk->data, &rx_fifo_len);
  *got_brk = (status & UART_INTR_BRK_DET) != 0;
  if (*got_brk && rx_fifo_len > 0 && chunk->data[rx_fifo_len - 1] == 0x00) {
    rx_fifo_len--;
  }
  chunk->tx_brk_len = 0;
  chunk->data_len = (uint8_t)rx_fifo_len;
}

static void bridge_free_driver_ctx(bridge_context_t *uart_ctx) {
  if (uart_ctx) {
    if (uart_ctx->kick) esp_timer_delete(uart_ctx->kick);
    if (uart_ctx->fd_uart_to_rs485.producer_rx_data_buf) heap_caps_free(uart_ctx->fd_uart_to_rs485.producer_rx_data_buf);
    if (uart_ctx->rs485_to_fd_uart.producer_rx_data_buf) heap_caps_free(uart_ctx->rs485_to_fd_uart.producer_rx_data_buf);

    if (uart_ctx->fd_uart_to_rs485.bridging_ringbuf) vRingbufferDeleteWithCaps(uart_ctx->fd_uart_to_rs485.bridging_ringbuf);
    if (uart_ctx->rs485_to_fd_uart.bridging_ringbuf) vRingbufferDeleteWithCaps(uart_ctx->rs485_to_fd_uart.bridging_ringbuf);

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

static bridge_context_t *bridge_alloc_driver_ctx(uart_port_t uart_num, uart_port_t rs485_num, int rs485_to_uart_buffer_size, int uart_to_rs485_buffer_size) {
  bridge_context_t *ctx = heap_caps_calloc(1, sizeof(bridge_context_t), BRIDGE_MALLOC_CAPS);
  if (!ctx) {
    return NULL;
  }

  // set the respective uart numbers belonging to the bridge
  ctx->fd_uart_num = uart_num;
  ctx->rs485_uart_num = rs485_num;

  // allocate the buffer to store the FIFO data of the UART peripheral
  ctx->fd_uart_to_rs485.producer_rx_data_buf = heap_caps_calloc(UART_HW_FIFO_LEN(uart_num) + sizeof(bridge_uart_data_t), sizeof(uint8_t), BRIDGE_MALLOC_CAPS);
  ctx->rs485_to_fd_uart.producer_rx_data_buf = heap_caps_calloc(UART_HW_FIFO_LEN(rs485_num) + sizeof(bridge_uart_data_t), sizeof(uint8_t), BRIDGE_MALLOC_CAPS);
  if (!ctx->fd_uart_to_rs485.producer_rx_data_buf || !ctx->rs485_to_fd_uart.producer_rx_data_buf) {
    goto err;
  }

  // allocate the glue buffers for bridging data
  // both are allocated as nosplit buffers in order to store the break condition
  ctx->fd_uart_to_rs485.bridging_ringbuf = xRingbufferCreateWithCaps(rs485_to_uart_buffer_size, RINGBUF_TYPE_NOSPLIT, BRIDGE_MALLOC_CAPS);
  ctx->rs485_to_fd_uart.bridging_ringbuf = xRingbufferCreateWithCaps(uart_to_rs485_buffer_size, RINGBUF_TYPE_NOSPLIT, BRIDGE_MALLOC_CAPS);
  if (!ctx->fd_uart_to_rs485.bridging_ringbuf || !ctx->rs485_to_fd_uart.bridging_ringbuf) {
    goto err;
  }

  ctx->fd_tx_mux = xSemaphoreCreateMutexWithCaps(BRIDGE_MALLOC_CAPS);
  ctx->fd_rx_mux = xSemaphoreCreateMutexWithCaps(BRIDGE_MALLOC_CAPS);
  ctx->fd_tx_done_sem = xSemaphoreCreateBinaryWithCaps(BRIDGE_MALLOC_CAPS);
  ctx->fd_tx_fifo_sem = xSemaphoreCreateBinaryWithCaps(BRIDGE_MALLOC_CAPS);
  if (!ctx->fd_tx_mux || !ctx->fd_rx_mux ||
          !ctx->fd_tx_done_sem || !ctx->fd_tx_fifo_sem) {
    goto err;
  }

  ctx->rs485_tx_brk_sem = xSemaphoreCreateBinaryWithCaps(BRIDGE_MALLOC_CAPS);
  ctx->rs485_tx_done_sem = xSemaphoreCreateBinaryWithCaps(BRIDGE_MALLOC_CAPS);
  if (!ctx->rs485_tx_brk_sem || !ctx->rs485_tx_done_sem) {
    goto err;
  }

  esp_timer_create_args_t targs = { .callback = bridge_rs485_kick_cb, .arg = ctx, .name = "mw_kick" };
  if (esp_timer_create(&targs, &ctx->kick) != ESP_OK) {
    goto err;
  }

  return ctx;

err:
    bridge_free_driver_ctx(ctx);
  return NULL;
}

esp_err_t bridge_driver_install(fullduplex_uart_config_t *fd_uart_cfg, rs485_uart_config_t *rs485_uart_cfg, int rs485_to_uart_buffer_size, int uart_to_rs485_buffer_size, int fd_intr_alloc_flags, int rs485_intr_alloc_flags) {
  uart_port_t uart_num = fd_uart_cfg->uart_num;
  uart_port_t rs485_num = rs485_uart_cfg->uart_num;
  if (merge_wire_driver_uart_ctx[uart_num].uart_ctx == NULL && merge_wire_driver_uart_ctx[rs485_num].uart_ctx == NULL) {
    // create the driver object and assign to both the
    // full-duplex UART port and the RS485 UART port
    bridge_context_t *ctx = bridge_alloc_driver_ctx(uart_num, rs485_num, rs485_to_uart_buffer_size, uart_to_rs485_buffer_size);
    if (ctx == NULL) {
      ESP_LOGE(MERGE_WIRE_TAG, "UART driver malloc error");
      return ESP_FAIL;
    }
    merge_wire_driver_uart_ctx[uart_num].port_id = uart_num;
    merge_wire_driver_uart_ctx[uart_num].uart_ctx = ctx;
    merge_wire_driver_uart_ctx[rs485_num].port_id = rs485_num;
    merge_wire_driver_uart_ctx[rs485_num].uart_ctx = ctx;

    ctx->fd_uart_to_rs485.producer_port = uart_num;
    ctx->fd_uart_to_rs485.consumer_port = rs485_num;
    ctx->rs485_to_fd_uart.producer_port = rs485_num;
    ctx->rs485_to_fd_uart.consumer_port = uart_num;

    ctx->fd_uart_to_rs485.consumer_tx_waiting_brk = false;
    ctx->rs485_to_fd_uart.consumer_tx_waiting_brk = false;

    ctx->fd_uart_to_rs485.tx_ptr = NULL;
    ctx->fd_uart_to_rs485.chunk_rem_len = 0;
    ctx->fd_uart_to_rs485.buffered_len = 0;
    ctx->rs485_to_fd_uart.tx_ptr = NULL;
    ctx->rs485_to_fd_uart.chunk_rem_len = 0;
    ctx->rs485_to_fd_uart.buffered_len = 0;

    ctx->fd_uart_to_rs485.tx_brk_stashed_flg = false;
    ctx->fd_uart_to_rs485.tx_brk_stashed_len = 0;
    ctx->rs485_to_fd_uart.tx_brk_stashed_flg = false;
    ctx->rs485_to_fd_uart.tx_brk_stashed_len = 0;

    // burst machine + lock initial state
    ctx->lock = (portMUX_TYPE) portMUX_INITIALIZER_UNLOCKED;
    ctx->rnd = esp_random() | 1;                 // xorshift seed != 0
    ctx->kick_scheduled_flg = 0;
    ctx->kick_scheduled_us = 0;
    ctx->r_state = BRIDGE_RS485_IDLE;
    ctx->last_bus_rx_us = esp_timer_get_time();
    // carrier-sense quiet us calculated based on the baud rate
    ctx->quiet_us = (QUIET_BITS * 1000000UL) / (uint32_t) rs485_uart_cfg->baud_rate + 1;

    // RS485 register mode: collision-detect set = hardware echo suppression
    // NOTE: it leaves sw_rts = 0 (DE ASSERTED, holding the bus!) -> release now
    uart_hal_set_mode(&(merge_wire_uart_context[rs485_num].hal), UART_MODE_RS485_COLLISION_DETECT);
    DE_RELEASE(rs485_num);

    // set the latency / streaming thresholds on both ports
    uart_hal_set_rxfifo_full_thr(&(merge_wire_uart_context[uart_num].hal), RXFIFO_FULL_THR);
    uart_hal_set_rx_timeout(&(merge_wire_uart_context[uart_num].hal), RX_TOUT_BITS);
    uart_hal_set_txfifo_empty_thr(&(merge_wire_uart_context[uart_num].hal), TXFIFO_EMPTY_THR);
    uart_hal_set_rxfifo_full_thr(&(merge_wire_uart_context[rs485_num].hal), RXFIFO_FULL_THR);
    uart_hal_set_rx_timeout(&(merge_wire_uart_context[rs485_num].hal), RX_TOUT_BITS);
    uart_hal_set_txfifo_empty_thr(&(merge_wire_uart_context[rs485_num].hal), TXFIFO_EMPTY_THR);

    // quiesce, then take both interrupt sources for the bridge ISRs
    uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_LL_INTR_MASK);
    uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_LL_INTR_MASK);
    uart_hal_disable_intr_mask(&(merge_wire_uart_context[rs485_num].hal), UART_LL_INTR_MASK);
    uart_hal_clr_intsts_mask(&(merge_wire_uart_context[rs485_num].hal), UART_LL_INTR_MASK);

    esp_err_t ret = esp_intr_alloc(uart_periph_signal[uart_num].irq,
                                   fd_intr_alloc_flags | BRIDGE_INTR_FLAGS,
                                   bridge_uart_intr_handler, &merge_wire_driver_uart_ctx[uart_num],
                                   &ctx->fd_intr_handle);
    if (ret != ESP_OK) {
      merge_wire_driver_uart_ctx[uart_num].uart_ctx = NULL;
      merge_wire_driver_uart_ctx[rs485_num].uart_ctx = NULL;
      bridge_free_driver_ctx(ctx);
      return ret;
    }
    ret = esp_intr_alloc(uart_periph_signal[rs485_num].irq,
                         rs485_intr_alloc_flags | BRIDGE_INTR_FLAGS,
                         bridge_rs485_intr_handler, &merge_wire_driver_uart_ctx[rs485_num],
                         &ctx->rs485_intr_handle);
    if (ret != ESP_OK) {
      esp_intr_free(ctx->fd_intr_handle);
      merge_wire_driver_uart_ctx[uart_num].uart_ctx = NULL;
      merge_wire_driver_uart_ctx[rs485_num].uart_ctx = NULL;
      bridge_free_driver_ctx(ctx);
      return ret;
    }

    // arm the always-on RX side interrupts
    // TX interrupts are armed on demand
    uart_hal_ena_intr_mask(&(merge_wire_uart_context[uart_num].hal),
                           UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT |
                           UART_INTR_RXFIFO_OVF | UART_INTR_BRK_DET);
    // for RS485, we additionally enable the
    // UART_INTR_RS485_CLASH | UART_INTR_RS485_FRM_ERR | UART_INTR_RS485_PARITY_ERR
    // interrupts, which we need to use for detecting collisions
    uart_hal_ena_intr_mask(&(merge_wire_uart_context[rs485_num].hal),
                           UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT |
                           UART_INTR_RXFIFO_OVF | UART_INTR_BRK_DET | RS485_ERR_MASK);

    ESP_LOGI(MERGE_WIRE_TAG, "bridge up: fd=UART%d <-> rs485=UART%d, quiet=%luus, jam=%d",
             uart_num, rs485_num, (unsigned long) ctx->quiet_us, MW_JAM_BITS);
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
  merge_wire_uart_context[uart_num].tx_io_num = tx_pin;
  merge_wire_uart_context[uart_num].rx_io_num = rx_pin;
  merge_wire_uart_context[uart_num].rts_io_num = rts_pin;
  merge_wire_uart_context[uart_num].cts_io_num = cts_pin;
  merge_wire_uart_context[uart_num].dtr_io_num = dtr_pin;
  merge_wire_uart_context[uart_num].dsr_io_num = dsr_pin;

  return ESP_OK;
}


void bridge_driver_get_stats(uart_port_t any_bridge_port, bridge_stats_t *out) {
  bridge_context_t *ctx = merge_wire_driver_uart_ctx[any_bridge_port].uart_ctx;
  if (ctx == NULL || out == NULL) {
    return;
  }
  portENTER_CRITICAL(&ctx->lock);
  out->fd_rx_bytes        = ctx->fd_uart_to_rs485.rx_bytes;
  out->rs485_tx_bytes     = ctx->fd_uart_to_rs485.tx_bytes;
  out->rs485_rx_bytes     = ctx->rs485_to_fd_uart.rx_bytes;
  out->fd_tx_bytes        = ctx->rs485_to_fd_uart.tx_bytes;
  out->collisions         = ctx->collisions;
  out->retries            = ctx->retries_total;   // always 0 under COLLISION_DROP
  out->tx_giveups         = ctx->tx_giveups;      // under COLLISION_DROP: every collision-dropped chunk
  out->breaks_forwarded   = ctx->fd_uart_to_rs485.breaks_tx + ctx->rs485_to_fd_uart.breaks_tx;
  out->breaks_dropped     = 0;  // no silent-drop path: a failed break send stashes or parks, never vanishes
  out->fd_rx_overflows    = ctx->fd_rx_overflows;
  out->rs485_rx_overflows = ctx->rs485_rx_overflows;
  out->fd_line_errors     = 0;  // FRAM/PARITY not enabled on the fd port
  out->rs485_line_errors  = ctx->rs485_line_errors;
  portEXIT_CRITICAL(&ctx->lock);
}