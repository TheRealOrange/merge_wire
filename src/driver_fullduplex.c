#include <sys/param.h>

#include "driver_common.h"
#include "tunables.h"

void BRIDGE_ISR_ATTR bridge_uart_intr_handler(void *param) {
  bridge_port_context_t *p_port_ctx = param;
  uint8_t uart_num = p_port_ctx->port_id;
  bridge_context_t *c = p_port_ctx->uart_ctx;

  bool rx_buffer_free = true;
  uint32_t tx_fifo_rem = 0;
  uint32_t uart_intr_status = 0;
  BaseType_t HPTaskAwoken = 0;
  bool need_yield = false;

  // interrupt handler for UART RX to RS485 TX
  while ((uart_intr_status = uart_hal_get_intsts_mask(&(merge_wire_uart_context[uart_num].hal))) != 0) {
    if (uart_intr_status & UART_INTR_TXFIFO_EMPTY) {
      // tx fifo empty, clear the intr mask while we are handling txfifo empty
      UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);
      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);

      // if we have a break condition waiting, its normal for the txfifo to run empty
      // so we ignore and continue the interrupt handling
      if (c->fd_tx_waiting_brk) {
        continue;
      }

      // whether to enable txfifo empty interrupt after we fill the fifo
      bool en_tx_flg = false;
      tx_fifo_rem = uart_hal_get_txfifo_len(&(merge_wire_uart_context[uart_num].hal));

      // no break waiting, check if we have any data waiting to be sent
      while (tx_fifo_rem > 0) {
        // check if we have a chunk in flight
        if (c->rs485_to_uart_chunk_rem_len == 0) {
          // no chunk in flight, grab a new chunk from the ringbuf
          size_t size;
          bridge_uart_data_t *chunk = xRingbufferReceiveFromISR(c->rs485_to_uart_ring_buf, &size);
          if (chunk) {
            // received data from the ringbuf
            if (chunk->tx_brk_len > 0) {
              // no data in this chunk, set up break
              uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
              UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
              uart_hal_tx_break(&(merge_wire_uart_context[uart_num].hal), chunk->tx_brk_len);
              uart_hal_ena_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
              UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
              c->fd_tx_waiting_brk = 1;

              //do not enable TX empty interrupt
              en_tx_flg = false;

              // return the item to the ISR since we are done processing it
              vRingbufferReturnItemFromISR(c->rs485_to_uart_ring_buf, chunk, &HPTaskAwoken);
              need_yield |= (HPTaskAwoken == pdTRUE);
              bridge_service_producer(
                c->rs485_to_uart_ring_buf,
                (bridge_uart_data_t *) c->rs485_rx_data_buf,
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
                (bridge_uart_data_t *) c->rs485_rx_data_buf,
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
      if (c->uart_to_rs485_buffer_full_flg == false) {
        bridge_read_fifo_chunk(uart_num, c->fd_rx_data_buf, uart_intr_status, &got_brk);
        bridge_uart_data_t *chunk = (bridge_uart_data_t *)c->fd_rx_data_buf;


        // after we copy the data from the fifo into the rx data buffer, we clear the intr status
        uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal),
                                 UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL |
                                 UART_INTR_BRK_DET);

        if (chunk->data_len > 0) {
          // try to send the data to the ring buffer
          bool sent = bridge_try_send_ring_buf(
            c->uart_to_rs485_ring_buf,
            (const uint8_t *) chunk, chunk->data_len + sizeof(bridge_uart_data_t),
            &c->uart_to_rs485_buffer_full_flg,
            uart_num, &HPTaskAwoken, &need_yield);
          if (sent) {
            // successfully pushed data to the bridging buffer
            UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
            c->uart_to_rs485_buffered_len += chunk->data_len;
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
            bridge_uart_data_t *brk = (bridge_uart_data_t *) c->fd_rx_data_buf;
            brk->tx_brk_len = MW_FWD_BRK_BITS;
            brk->data_len = 0;
            // try to send the data to the ring buffer
            bridge_try_send_ring_buf(
              c->uart_to_rs485_ring_buf,
              (const uint8_t *) brk, sizeof(bridge_uart_data_t),
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
        UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
        UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
        uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);

        // now we wait for the rs485 bus to service the queue in order to re-enable interrupts
      }

      // here we can check if the rs485 buffer has space in its fifo to push data

    } else if (uart_intr_status & UART_INTR_RXFIFO_OVF) {
      // When fifo overflows, we reset the fifo.
      UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_rxfifo_rst(&(merge_wire_uart_context[uart_num].hal));
      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_RXFIFO_OVF);
    } else if (uart_intr_status & UART_INTR_TX_BRK_DONE) {
      UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_tx_break(&(merge_wire_uart_context[uart_num].hal), 0);
      uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
      if (c->fd_tx_waiting_brk == 1) {
        uart_hal_ena_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);
      }
      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
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
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_DONE);
      UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_disable_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_DONE);

      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      xSemaphoreGiveFromISR(c[uart_num].fd_tx_done_sem, &HPTaskAwoken);
      need_yield |= (HPTaskAwoken == pdTRUE);
    } else {
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), uart_intr_status); /*simply clear all other intr status*/
    }
  }
  if (need_yield) {
    portYIELD_FROM_ISR();
  }
}
