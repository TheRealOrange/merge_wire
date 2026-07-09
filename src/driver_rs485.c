#include <sys/param.h>

#include "driver_common.h"
#include "tunables.h"

#define RS485_ERR_MASK (UART_INTR_RS485_CLASH | UART_INTR_RS485_FRM_ERR | \
UART_INTR_RS485_PARITY_ERR)

static uint32_t BRIDGE_ISR_ATTR xrnd(bridge_context_t *c) {
  uint32_t x = c->rnd;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  return c->rnd = x;
}

// return the in-flight chunk to the ring (COMMIT or give-up) and service the
// full-duplex producer, whose stashed chunk may be waiting for this space
// call with c->lock held
static void BRIDGE_ISR_ATTR bridge_rs485_return_chunk(bridge_context_t *ctx, BaseType_t *HPTaskAwoken, bool *need_yield) {
  bridge_port_obj_t *port_obj = &ctx->fd_uart_to_rs485;
  vRingbufferReturnItemFromISR(port_obj->bridging_ringbuf, port_obj->chunk_in_flight, HPTaskAwoken);
  *need_yield |= (*HPTaskAwoken == pdTRUE);
  port_obj->chunk_in_flight = NULL;
  port_obj->tx_ptr = NULL;
  port_obj->chunk_rem_len = 0;
  ctx->burst_retries = 0;
  bridge_service_producer(port_obj, ctx->fd_uart_num, HPTaskAwoken, need_yield);
}

// arm a forwarded break, the UART hardware appends it once the TX FIFO
// finishes to preserve the data and break ordering in the burst
// DE stays
static void BRIDGE_ISR_ATTR bridge_rs485_set_fwd_brk(bridge_context_t *ctx, uint8_t bits) {
  uart_port_t n = ctx->rs485_uart_num;
  ctx->r_state = BRIDGE_RS485_BRK;
  UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[n].spinlock));
  uart_hal_disable_intr_mask(&(merge_wire_uart_context[n].hal), UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_DONE);
  uart_hal_clr_intsts_mask(&(merge_wire_uart_context[n].hal), UART_INTR_TX_BRK_DONE);
  uart_hal_tx_break(&(merge_wire_uart_context[n].hal), bits);
  uart_hal_ena_intr_mask(&(merge_wire_uart_context[n].hal), UART_INTR_TX_BRK_DONE);
  UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[n].spinlock));
  ctx->breaks_forwarded++;
}

/* collision aftermath: rewind the in-flight chunk for replay (or drop it
 * after MW_MAX_RETRIES) and arm a randomized binary-exponential backoff.
 * call with c->lock held, r_state already back to R_IDLE.                    */
static void BRIDGE_ISR_ATTR bridge_rs485_schedule_retry(bridge_context_t *ctx, BaseType_t *HPTaskAwoken, bool *need_yield) {
  ctx->burst_retries++;
  ctx->retries_total++;
  if (ctx->burst_retries > MW_MAX_RETRIES) {
    ctx->tx_giveups++;
    bridge_rs485_return_chunk(ctx, HPTaskAwoken, need_yield);  // drop the chunk
  } else {
    ctx->fd_uart_to_rs485.tx_ptr = ctx->fd_uart_to_rs485.chunk_in_flight->data;          // rewind, replay
    ctx->fd_uart_to_rs485.chunk_rem_len = ctx->fd_uart_to_rs485.chunk_in_flight->data_len;
  }
  uint32_t exp = ctx->burst_retries > 4 ? 4 : ctx->burst_retries;
  uint64_t backoff = (uint64_t) ctx->quiet_us * (1 + (xrnd(ctx) % (1u << exp)));

  // if there is already a timer running and it is for a time later than the backoff
  // do not try to set an earlier timer
  if (ctx->kick_scheduled_flg && ctx->kick_scheduled_us < backoff + esp_timer_get_time()) {
    // we have a kick scheduled and the current timer will expire sooner than the
    // time we want to backoff for, so we restart the timer
    if (esp_timer_restart(ctx->kick, backoff) == ESP_ERR_INVALID_STATE) {
      // timer expired so we set a new timer
      if (esp_timer_start_once(ctx->kick, backoff) == ESP_OK) {
        ctx->kick_scheduled_flg = true;
      }
    }
  } else if (!ctx->kick_scheduled_flg) {
    // no kick scheduled
    if (esp_timer_start_once(ctx->kick, backoff) == ESP_OK) {
      ctx->kick_scheduled_flg = true;
    }
  }

  if (ctx->kick_scheduled_flg) {
    esp_timer_get_expiry_time(ctx->kick, &ctx->kick_scheduled_us);
  }
}

void BRIDGE_ISR_ATTR bridge_rs485_carrier_sense_try_tx(bridge_context_t *ctx, BaseType_t *HPTaskAwoken, bool *need_yield) {
  // we want to check if the bus has been idle long enough
  // QUIET_BITS elapsed so that we know the bus is idle and
  // it is safe for us to attempt to TX
  int64_t us_from_last_rx = esp_timer_get_time() - ctx->last_bus_rx_us;
  uint64_t quiet_check_us = ctx->last_bus_rx_us + ctx->quiet_us;
  uint64_t timer_kick_us = MAX(1, (int64_t)ctx->quiet_us - us_from_last_rx);

  if (us_from_last_rx < (int64_t)ctx->quiet_us
    || uart_hal_get_rxfifo_len(&(merge_wire_uart_context[ctx->rs485_uart_num].hal)) != 0) {
    // the quiet time has not yet elapsed
    // OR
    // the quiet time has elapsed
    // check if there is any data in the RX buffer
    // if there is, then the quiet time needs to be restarted
    // because it could mean that RX_TOUT has not yet fired and
    // RX_FULL threshold has not been reached but there is new data

    // check if there is a kick currently scheduled and push it back to the
    // last_bus_rx_us + quiet_us
    if (ctx->kick_scheduled_flg && ctx->kick_scheduled_us < quiet_check_us) {
      // current timer is too soon, restart it
      // ensure the timer is in the future
      if (esp_timer_restart(ctx->kick, timer_kick_us) == ESP_ERR_INVALID_STATE) {
        // timer expired so we set a new timer
        if (esp_timer_start_once(ctx->kick, timer_kick_us) == ESP_OK) {
          ctx->kick_scheduled_flg = true;
        }
      }
    } else {
      // no kick scheduled, schedule a new kick
      if (esp_timer_start_once(ctx->kick, timer_kick_us) == ESP_OK) {
        ctx->kick_scheduled_flg = true;
      }
    }
    if (ctx->kick_scheduled_flg) {
      esp_timer_get_expiry_time(ctx->kick, &ctx->kick_scheduled_us);
    }
  } else {
    // the quiet time has elapsed AND there is no data in the RX FIFO
    // the bus is truly quiet so we try to TX here

  }
}

void BRIDGE_ISR_ATTR bridge_rs485_kick_cb(void *arg) {          // esp_timer task context
  bridge_context_t *c = arg;
  BaseType_t HPTaskAwoken = 0;
  bool need_yield = false;
  portENTER_CRITICAL(&c->lock);
  // TODO: feed the TX fifo here
  portEXIT_CRITICAL(&c->lock);
  (void) need_yield;                             // scheduler handles task ctx
}

void BRIDGE_ISR_ATTR bridge_rs485_intr_handler(void *param) {
  bridge_port_context_t *p_uart = param;
  uint8_t uart_num = p_uart->port_id;
  bridge_context_t *ctx = p_uart->uart_ctx;
  bridge_port_obj_t *port_obj = &ctx->rs485_to_fd_uart;

  uint32_t uart_intr_status = 0;
  BaseType_t HPTaskAwoken = 0;
  bool need_yield = false;
  bool postponed = false;
  bool rx_buffer_free = true;

  // grab the lock for the RS485 state machine
  portENTER_CRITICAL_ISR(&ctx->lock);
  while (!postponed &&
         (uart_intr_status = uart_hal_get_intsts_mask(&(merge_wire_uart_context[uart_num].hal))) != 0) {
    if (uart_intr_status & RS485_ERR_MASK) {
      // handle collision or rs485 line errors FIRST so garbage is flushed before the
      // RX branch below can forward it

      // any rs485 errors are only meaningful while WE transmit
      // during our own break (R_BRK/R_JAM) the loopback reports real
      // framing errors, while idle it is just noise
      if (ctx->r_state == BRIDGE_RS485_TX) {
        ctx->collisions++;
        UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        uart_hal_rxfifo_rst(&(merge_wire_uart_context[uart_num].hal));
        // the TX FIFO residue CANNOT be flushed (UART1/2 errata woops) and will
        // shift out against foreign bus data: mask the storm until idle
        uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal),
                                   RS485_ERR_MASK | UART_INTR_TXFIFO_EMPTY);
#if MW_JAM_BITS > 0
        // if we want to notify all parties on the bus of the clash, we send BRK to jam the bus
        uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_DONE);
        uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
        uart_hal_tx_break(&(merge_wire_uart_context[uart_num].hal), MW_JAM_BITS);
        uart_hal_ena_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
        UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        ctx->r_state = R_JAM;          // DE stays, transmit jam so everyone sees the clash
#else
        // if we do not want to jam, we disable sending so the buffer is not TX'ed
        UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        // release de so the driver stops controlling the bus and nothing gets sent
        DE_RELEASE(uart_num);
        // set state to BRIDGE_RS485_DRAIN to wait for TX to drain
        ctx->r_state = BRIDGE_RS485_DRAIN;
        // TX_DONE(idle) finishes the cleanup
#endif
      }
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), RS485_ERR_MASK);
    } else if (uart_intr_status & (UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                                   UART_INTR_BRK_DET)) {
      // bus data to full-duplex UART
      // our own DATA echo never lands here (rely on tx_rx_en hardware
      // suppression), BUT our own BREAK echo does, so breaks are only
      // forwardable while the burst machine is idle
      bool got_brk = false;
      if (!port_obj->buffer_full_flg) {
        bridge_read_fifo_chunk(uart_num, port_obj->producer_rx_data_buf, uart_intr_status, &got_brk);
        bridge_uart_data_t *chunk = (bridge_uart_data_t *) port_obj->producer_rx_data_buf;

        // clear the interrupt status now that we read the fifo
        uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal),
                                 UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                                 UART_INTR_BRK_DET);

        // gate the BRK signal behind whether the bus is idle
        // this ignores any breaks we sent which get echo'ed back to ue
        got_brk = got_brk && (ctx->r_state == BRIDGE_RS485_IDLE);

        if (chunk->data_len > 0) {
          bool sent = bridge_try_send_ring_buf(
            port_obj, uart_num,
            (const uint8_t *) chunk, chunk->data_len + sizeof(bridge_uart_data_t),
            &HPTaskAwoken, &need_yield);
          if (sent) {
            UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
            port_obj->buffered_len += chunk->data_len;
            UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
            rx_buffer_free = true;
          } else {
            // rx buffer contains stashed data, not free
            rx_buffer_free = false;
          }
        }

        if (got_brk) {
          if (rx_buffer_free) {
            // the rx buf is already sent so we can use it to store the brk
            bridge_uart_data_t *brk = (bridge_uart_data_t *) port_obj->producer_rx_data_buf;
            brk->tx_brk_len = MW_FWD_BRK_BITS;
            brk->data_len = 0;
            bridge_try_send_ring_buf(
            port_obj, uart_num,
              (const uint8_t *) brk, sizeof(bridge_uart_data_t),
              &HPTaskAwoken, &need_yield);
            // if failed, the brk is already stashed inside the rx buffer
          } else {
            // failed to send the data to the buffer, stash the brk without trying to send it
            // for when the queue is serviced, to preserve ordering (in case data fails but ringbuf has space for brk)
            port_obj->tx_brk_stashed_flg = true;
            port_obj->tx_brk_stashed_len = MW_FWD_BRK_BITS;
          }
        }

        // wake the fd TX drain (unless it is mid-break)
        if (!ctx->fd_uart_to_rs485.producer_tx_waiting_brk) {
          UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[ctx->fd_uart_num].spinlock));
          uart_hal_ena_intr_mask(&(merge_wire_uart_context[ctx->fd_uart_num].hal), UART_INTR_TXFIFO_EMPTY);
          UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[ctx->fd_uart_num].spinlock));
        }
      } else {
        // bridging buffer full: leave data in the hardware FIFO and mask
        // with RTS flow control on this port the filling FIFO would throttle
        // the peer, but on a multi-drop RS485 bus there is nobody to throttle,
        // so the OVF handler below is the actual safety net here
        UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal),
                                   UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT |
                                   UART_INTR_BRK_DET);
        UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal),
                                 UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
      }
      // remember bus activity for carrier sense
      // the frame edge that raised this interrupt may be our chance to transmit
      ctx->last_bus_rx_us = esp_timer_get_time();

      // schedule a kick to check if the bus has been idle
    } else if (uart_intr_status & UART_INTR_RXFIFO_OVF) {
      UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_rxfifo_rst(&(merge_wire_uart_context[uart_num].hal));
      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_RXFIFO_OVF);
    } else if (uart_intr_status & UART_INTR_TXFIFO_EMPTY) {
      // we cannot just load the TX FIFO willy nilly the moment it is empty unlike the full-duplex UART
      // we have to check if the bus is currently busy
      if (ctx->r_state == BRIDGE_RS485_TX ) {
        // we are currently already TX'ing so it is safe to push data onto the bus
      }
    } else if (uart_intr_status & UART_INTR_TX_DONE) {
      if (uart_hal_is_tx_idle(&(merge_wire_uart_context[uart_num].hal)) != true) {
        // TX_DONE fires when the UART TX FIFO is empty
        // but the UART transmission could not yet be complete (not yet put onto bus)
        // so we DO NOT release DE and let the TX_DONE interrupt flag stay so that
        // on the next loop we check again on uart_hal_is_tx_idle
        postponed = true;
      } else if (ctx->r_state == BRIDGE_RS485_TX) {
        if (port_obj->chunk_rem_len == 0) {
          // there is a chunk in flight currently
          // TODO: try to feed data from the chunk in flight to the TX fifo
        }
      } else if (ctx->r_state == BRIDGE_RS485_DRAIN) {
        // state machine was waiting for the TX to drain, DE has been released so
        // the TX buffer was emptying but the data is not being sent into the bus
        // TX_DONE means the dying TX has fully shifted out: flush, unmask, back off, replay
        UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        uart_hal_rxfifo_rst(&(merge_wire_uart_context[uart_num].hal));
        uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal),
                                 RS485_ERR_MASK | UART_INTR_TX_DONE);
        uart_hal_ena_intr_mask(&(merge_wire_uart_context[uart_num].hal), RS485_ERR_MASK);
        uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_DONE);
        UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));

        // set the bus state to IDLE since we are no longer TX'ing
        // and schedule a retry to try sending the chunk again
        ctx->r_state = BRIDGE_RS485_IDLE;
        bridge_rs485_schedule_retry(ctx, &HPTaskAwoken, &need_yield);
      } else {
        uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_DONE);
      }
    } else if (uart_intr_status & UART_INTR_TX_BRK_DONE) {
      // our break (forwarded or jam) finished
      UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_tx_break(&(merge_wire_uart_context[uart_num].hal), 0);
      uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
      // purge our own break's echo/NUL and its framing artifacts
      uart_hal_rxfifo_rst(&(merge_wire_uart_context[uart_num].hal));
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal),
                               RS485_ERR_MASK | UART_INTR_BRK_DET | UART_INTR_TX_BRK_DONE);
      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));

      if (ctx->r_state == BRIDGE_RS485_BRK) {
        ctx->r_state = BRIDGE_RS485_IDLE;
        size_t size;
        bridge_uart_data_t *chunk = xRingbufferReceiveFromISR(ctx->fd_uart_to_rs485.bridging_ringbuf, &size);
        if (chunk && chunk->tx_brk_len > 0) {
          // next chunk is a BRK to be sent
          uint8_t bits = chunk->tx_brk_len;
          vRingbufferReturnItemFromISR(ctx->fd_uart_to_rs485.bridging_ringbuf, chunk, &HPTaskAwoken);
          need_yield |= (HPTaskAwoken == pdTRUE);
          bridge_rs485_set_fwd_brk(ctx, bits);
        } else if (chunk) {
          // next chunk is a data chunk
          // set the chunk in flight to this chunk and prime the TX
          ctx->fd_uart_to_rs485.chunk_in_flight = chunk;
          ctx->fd_uart_to_rs485.tx_ptr = chunk->data;
          ctx->fd_uart_to_rs485.chunk_rem_len = chunk->data_len;
          ctx->r_state = BRIDGE_RS485_TX;

          // try to feed the TX fifo
          // TODO: feed tx fifo here
        } else {
          DE_RELEASE(uart_num);
        }
      } else if (ctx->r_state == BRIDGE_RS485_JAM) {
        DE_RELEASE(uart_num);
        UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        uart_hal_ena_intr_mask(&(merge_wire_uart_context[uart_num].hal), RS485_ERR_MASK);
        UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        ctx->r_state = BRIDGE_RS485_IDLE;
        bridge_rs485_schedule_retry(ctx, &HPTaskAwoken, &need_yield);
      }
    } else if (uart_intr_status & UART_INTR_CMD_CHAR_DET) {
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_CMD_CHAR_DET);
    } else {
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), uart_intr_status); /*simply clear all other intr status*/
    }
  }
  // release the lock for the RS485 state machine
  portEXIT_CRITICAL_ISR(&ctx->lock);
  if (need_yield) {
    portYIELD_FROM_ISR();
  }
}
