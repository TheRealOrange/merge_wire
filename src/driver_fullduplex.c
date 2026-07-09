#include <sys/param.h>

#include "driver_common.h"
#include "tunables.h"

void BRIDGE_ISR_ATTR bridge_uart_intr_handler(void *param) {
  bridge_port_context_t *p_port_ctx = param;
  uint8_t uart_num = p_port_ctx->port_id;
  bridge_context_t *ctx = p_port_ctx->uart_ctx;
  bridge_port_obj_t *port_obj = &ctx->fd_uart_to_rs485;

  bool rx_buffer_free = true;
  bool brk_waiting = false;
  bool chunk_in_flight = false;
  bool ringbuf_empty = false;
  uint32_t uart_intr_status = 0;
  BaseType_t HPTaskAwoken = 0;
  bool need_yield = false;

  // ALL shared bridge state (port objects, flags, the burst machine) lives
  // under one lock: hold it for the whole handler, exactly as the rs485
  // handler does, so cross-core visibility is never load-bearing
  portENTER_CRITICAL_ISR(&ctx->lock);

  // interrupt handler for UART RX to RS485 TX
  while ((uart_intr_status = uart_hal_get_intsts_mask(&(merge_wire_uart_context[uart_num].hal))) != 0) {
    if (uart_intr_status & UART_INTR_TXFIFO_EMPTY) {
      // tx fifo empty, clear the intr mask while we are handling txfifo empty
      UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);
      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);

      // try to fill the TX FIFO from the producer to consumer bridging ring buffer
      bridge_fill_tx_fifo(&ctx->rs485_to_fd_uart, false, &brk_waiting, &chunk_in_flight, &ringbuf_empty, &HPTaskAwoken, &need_yield);

      if ((chunk_in_flight || !ringbuf_empty) && !brk_waiting) {
        // if there is still a chunk in flight, or the ringbuf is not empty AND there is
        // no break currently waiting to be sent, then re-enable the TXFIFO_EMPTY interrupt
        uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);
        UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        uart_hal_ena_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);
        UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      }
    } else if (uart_intr_status & (UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                                   UART_INTR_BRK_DET)) {
      // either the RX timed out or the RX FIFO is full
      // or a break is detected (with its associated NUL in the buffer
      // which we will strip and queue a break chunk
      bool got_brk = false;
      if (!port_obj->buffer_full_flg) {
        bridge_read_fifo_chunk(uart_num, port_obj->producer_rx_data_buf, uart_intr_status, &got_brk);
        bridge_uart_data_t *chunk = (bridge_uart_data_t *)port_obj->producer_rx_data_buf;


        // after we copy the data from the fifo into the rx data buffer, we clear the intr status
        uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal),
                                 UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                                 UART_INTR_BRK_DET);

        if (chunk->data_len > 0) {
          // try to send the data to the ring buffer
          bool sent = bridge_try_send_ring_buf(
            port_obj,
            (const uint8_t *) chunk, chunk->data_len + sizeof(bridge_uart_data_t),
            &HPTaskAwoken, &need_yield);
          if (sent) {
            // successfully pushed data to the bridging buffer
            UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
            port_obj->buffered_len += chunk->data_len;
            port_obj->rx_bytes += chunk->data_len;
            UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
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
            bridge_uart_data_t *brk = (bridge_uart_data_t *) port_obj->producer_rx_data_buf;
            brk->tx_brk_len = MW_FWD_BRK_BITS;
            brk->data_len = 0;
            // try to send the data to the ring buffer
            bridge_try_send_ring_buf(
              port_obj,
              (const uint8_t *) brk, sizeof(bridge_uart_data_t),
              &HPTaskAwoken, &need_yield);
            // if failed, the data is already stashed inside the rx buffer, no need to set flags
          } else {
            // failed to send the data to the buffer, stash the brk without trying to send it
            // for when the queue is serviced, to preserve ordering (in case data fails but ringbuf has space for brk)
            port_obj->tx_brk_stashed_flg = true;
            port_obj->tx_brk_stashed_len = MW_FWD_BRK_BITS;
          }
        }
      } else {
        // the bridging buffer is full
        // we DO NOT empty the fifo buffer
        // but we disable the interrupts (it should have already been disabled)
        UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
        UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);

        // now we wait for the rs485 bus to service the queue in order to re-enable interrupts
      }

      // here we can check if the rs485 uart has space in its fifo to push data to reduce latency
      // grab the lock for the RS485 state machine
      bridge_rs485_carrier_sense_try_tx(ctx, &HPTaskAwoken, &need_yield);
      need_yield |= (HPTaskAwoken == pdTRUE);
    } else if (uart_intr_status & UART_INTR_RXFIFO_OVF) {
      // When fifo overflows, we reset the fifo.
      UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_rxfifo_rst(&(merge_wire_uart_context[uart_num].hal));
      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_RXFIFO_OVF);
      ctx->fd_rx_overflows++;
    } else if (uart_intr_status & UART_INTR_TX_BRK_DONE) {
      UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_tx_break(&(merge_wire_uart_context[uart_num].hal), 0);
      uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
      if (ctx->rs485_to_fd_uart.consumer_tx_waiting_brk) {
        // we were waiting for a brk to send, re-enable TXFIFO_EMPTY interrupt
        uart_hal_ena_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);
      }
      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
      ctx->rs485_to_fd_uart.consumer_tx_waiting_brk = false;
    } else if (uart_intr_status & UART_INTR_TX_DONE) {
      // full-duplex TX completion: nothing to do for the bridge itself
      // (no DE, no echo); signal any task waiting on the semaphore
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_DONE);
      UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_DONE);

      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      xSemaphoreGiveFromISR(ctx->fd_tx_done_sem, &HPTaskAwoken);
      need_yield |= (HPTaskAwoken == pdTRUE);
    } else {
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), uart_intr_status); /*simply clear all other intr status*/
    }
  }
  portEXIT_CRITICAL_ISR(&ctx->lock);
  if (need_yield) {
    portYIELD_FROM_ISR();
  }
}