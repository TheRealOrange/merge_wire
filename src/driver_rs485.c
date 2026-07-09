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
static void BRIDGE_ISR_ATTR bridge_rs485_return_chunk(bridge_context_t *c, BaseType_t *HPTaskAwoken, bool *need_yield) {
  vRingbufferReturnItemFromISR(c->uart_to_rs485_ring_buf, c->uart_to_rs485_chunk, HPTaskAwoken);
  *need_yield |= (*HPTaskAwoken == pdTRUE);
  c->uart_to_rs485_chunk = NULL;
  c->rs485_tx_ptr = NULL;
  c->uart_to_rs485_chunk_rem_len = 0;
  c->burst_retries = 0;
  bridge_service_producer(c->uart_to_rs485_ring_buf,
                          (bridge_uart_data_t *) c->fd_rx_data_buf,
                          &c->uart_to_rs485_buffer_full_flg,
                          &c->rs485_tx_brk_flg, c->rs485_tx_brk_len,
                          c->fd_uart_num, HPTaskAwoken, need_yield);
}

// arm a forwarded break, the UART hardware appends it once the TX FIFO
// finishes to preserve the data and break ordering in the burst
// DE stays
static void BRIDGE_ISR_ATTR bridge_rs485_set_fwd_brk(bridge_context_t *c, uint8_t bits) {
  uart_port_t n = c->rs485_uart_num;
  c->r_state = BRIDGE_RS485_BRK;
  UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[n].spinlock));
  uart_hal_disable_intr_mask(&(merge_wire_uart_context[n].hal), UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_DONE);
  uart_hal_clr_intsts_mask(&(merge_wire_uart_context[n].hal), UART_INTR_TX_BRK_DONE);
  uart_hal_tx_break(&(merge_wire_uart_context[n].hal), bits);
  uart_hal_ena_intr_mask(&(merge_wire_uart_context[n].hal), UART_INTR_TX_BRK_DONE);
  UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[n].spinlock));
  c->breaks_forwarded++;
}

/* collision aftermath: rewind the in-flight chunk for replay (or drop it
 * after MW_MAX_RETRIES) and arm a randomized binary-exponential backoff.
 * call with c->lock held, r_state already back to R_IDLE.                    */
static void BRIDGE_ISR_ATTR bridge_rs485_schedule_retry(bridge_context_t *c, BaseType_t *HPTaskAwoken, bool *need_yield) {
  c->burst_retries++;
  c->retries_total++;
  if (c->burst_retries > MW_MAX_RETRIES) {
    c->tx_giveups++;
    bridge_rs485_return_chunk(c, HPTaskAwoken, need_yield);  // drop the chunk
  } else {
    c->rs485_tx_ptr = c->uart_to_rs485_chunk->data;          // rewind, replay
    c->uart_to_rs485_chunk_rem_len = c->uart_to_rs485_chunk->data_len;
  }
  uint32_t exp = c->burst_retries > 4 ? 4 : c->burst_retries;
  uint64_t backoff = (uint64_t) c->quiet_us * (1 + (xrnd(c) % (1u << exp)));
  if (esp_timer_start_once(c->kick, backoff) == ESP_ERR_INVALID_STATE) {
    esp_timer_restart(c->kick, backoff);
  }
}

void bridge_rs485_kick_cb(void *arg) {          // esp_timer task context
  bridge_context_t *c = (bridge_context_t *) arg;
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
  bridge_context_t *c = p_uart->uart_ctx;

  uint32_t uart_intr_status = 0;
  BaseType_t HPTaskAwoken = 0;
  bool need_yield = false;
  bool postponed = false;
  bool rx_buffer_free = true;

  portENTER_CRITICAL_ISR(&c->lock);
  while (!postponed &&
         (uart_intr_status = uart_hal_get_intsts_mask(&(merge_wire_uart_context[uart_num].hal))) != 0) {
    if (uart_intr_status & RS485_ERR_MASK) {
      // handle collision or rs485 line errors FIRST so garbage is flushed before the
      // RX branch below can forward it
      // any rs485 errors are only meaningful while WE transmit
      // during our own break (R_BRK/R_JAM) the loopback reports real
      // framing errors, while idle it is just noise
      if (c->r_state == BRIDGE_RS485_TX) {
        c->collisions++;
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
        c->r_state = R_JAM;          // DE stays, transmit jam so everyone sees the clash
#else
        // if we do not want to jam, we disable sending so the buffer is not TX'ed
        UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        // release de so the driver stops controlling the bus and nothing gets sent
        DE_RELEASE(uart_num);
        // set state to BRIDGE_RS485_DRAIN to wait for TX to drain
        c->r_state = BRIDGE_RS485_DRAIN;
        // TX_DONE(idle) finishes the cleanup
#endif
      }
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), RS485_ERR_MASK);
    } else if (uart_intr_status & (UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                                   UART_INTR_BRK_DET)) {
      // bus data -> fd
      // our own DATA echo never lands here (rely on tx_rx_en hardware
      // suppression), BUT our own BREAK echo does, so breaks are only
      // forwardable while the burst machine is idle
      bool got_brk = false;
      if (c->rs485_to_uart_buffer_full_flg == false) {
        bridge_read_fifo_chunk(uart_num, c->rs485_rx_data_buf, uart_intr_status, &got_brk);
        bridge_uart_data_t *chunk = (bridge_uart_data_t *) c->rs485_rx_data_buf;

        // clear the interrupt status now that we read the fifo
        uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal),
                                 UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                                 UART_INTR_BRK_DET);

        // gate the BRK signal behind whether the bus is idle
        // this ignores any breaks we sent which get echo'ed back to ue
        got_brk = got_brk && (c->r_state == BRIDGE_RS485_IDLE);

        if (chunk->data_len > 0) {
          bool sent = bridge_try_send_ring_buf(
            c->rs485_to_uart_ring_buf,
            (const uint8_t *) chunk, chunk->data_len + sizeof(bridge_uart_data_t),
            &c->rs485_to_uart_buffer_full_flg,
            uart_num, &HPTaskAwoken, &need_yield);
          if (sent) {
            UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
            c->rs485_to_uart_buffered_len += chunk->data_len;
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
            bridge_uart_data_t *brk = (bridge_uart_data_t *) c->rs485_rx_data_buf;
            brk->tx_brk_len = MW_FWD_BRK_BITS;
            brk->data_len = 0;
            bridge_try_send_ring_buf(
              c->rs485_to_uart_ring_buf,
              (const uint8_t *) brk, sizeof(bridge_uart_data_t),
              &c->rs485_to_uart_buffer_full_flg,
              uart_num, &HPTaskAwoken, &need_yield);
            // if failed, the brk is already stashed inside the rx buffer
          } else {
            // data itself is stashed: park the break BEHIND it to keep order
            c->fd_tx_brk_flg = 1;
            c->fd_tx_brk_len = MW_FWD_BRK_BITS;
          }
        }

        // wake the fd TX drain (unless it is mid-break)
        if (!c->fd_tx_waiting_brk) {
          UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[c->fd_uart_num].spinlock));
          uart_hal_ena_intr_mask(&(merge_wire_uart_context[c->fd_uart_num].hal), UART_INTR_TXFIFO_EMPTY);
          UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[c->fd_uart_num].spinlock));
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
      // remember bus activity for carrier sense; the frame edge that raised
      // this interrupt may be our chance to transmit
      c->last_bus_rx_us = esp_timer_get_time();

      // we just finished RX-ing
      // check if we can TX
      // TODO: feed TX fifo here
    } else if (uart_intr_status & UART_INTR_RXFIFO_OVF) {
      UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_rxfifo_rst(&(merge_wire_uart_context[uart_num].hal));
      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_RXFIFO_OVF);
    } else if (uart_intr_status & UART_INTR_TXFIFO_EMPTY) {

    } else if (uart_intr_status & UART_INTR_TX_DONE) {
      if (uart_hal_is_tx_idle(&(merge_wire_uart_context[uart_num].hal)) != true) {
        // TX_DONE leads the shifter: DO NOT clear the status; exit the ISR
        // and let the level interrupt re-fire once the line is truly done
        postponed = true;
      } else if (c->r_state == BRIDGE_RS485_TX) {
        if (c->uart_to_rs485_chunk_rem_len == 0) {
          // there is a chunk in flight currently
          // TODO: try to feed data from the chunk in flight to the TX fifo
        }
      } else if (c->r_state == BRIDGE_RS485_DRAIN) {
        // the dying TX has fully shifted out: flush, unmask, back off, replay
        UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        uart_hal_rxfifo_rst(&(merge_wire_uart_context[uart_num].hal));
        uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal),
                                 RS485_ERR_MASK | UART_INTR_TX_DONE);
        uart_hal_ena_intr_mask(&(merge_wire_uart_context[uart_num].hal), RS485_ERR_MASK);
        uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_DONE);
        UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        c->r_state = BRIDGE_RS485_IDLE;
        bridge_rs485_schedule_retry(c, &HPTaskAwoken, &need_yield);
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

      if (c->r_state == BRIDGE_RS485_BRK) {
        c->r_state = BRIDGE_RS485_IDLE;
        size_t size;
        bridge_uart_data_t *chunk = xRingbufferReceiveFromISR(c->uart_to_rs485_ring_buf, &size);
        if (chunk && chunk->tx_brk_len > 0) {
          // next chunk is a BRK to be sent
          uint8_t bits = chunk->tx_brk_len;
          vRingbufferReturnItemFromISR(c->uart_to_rs485_ring_buf, chunk, &HPTaskAwoken);
          need_yield |= (HPTaskAwoken == pdTRUE);
          bridge_rs485_set_fwd_brk(c, bits);
        } else if (chunk) {
          // next chunk is a data chunk
          // set the chunk in flight to this chunk and prime the TX
          c->uart_to_rs485_chunk = chunk;
          c->rs485_tx_ptr = chunk->data;
          c->uart_to_rs485_chunk_rem_len = chunk->data_len;
          c->r_state = BRIDGE_RS485_TX;

          // try to feed the TX fifo
          // TODO: feed tx fifo here
        } else {
          DE_RELEASE(uart_num);
        }
      } else if (c->r_state == BRIDGE_RS485_JAM) {
        DE_RELEASE(uart_num);
        UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        uart_hal_ena_intr_mask(&(merge_wire_uart_context[uart_num].hal), RS485_ERR_MASK);
        UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        c->r_state = BRIDGE_RS485_IDLE;
        bridge_rs485_schedule_retry(c, &HPTaskAwoken, &need_yield);
      }
    } else if (uart_intr_status & UART_INTR_CMD_CHAR_DET) {
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_CMD_CHAR_DET);
    } else {
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), uart_intr_status); /*simply clear all other intr status*/
    }
  }
  portEXIT_CRITICAL_ISR(&c->lock);
  if (need_yield) {
    portYIELD_FROM_ISR();
  }
}
