#include <sys/param.h>

#include "driver_common.h"
#include "tunables.h"

void BRIDGE_ISR_ATTR bridge_uart_intr_handler(void *param) {
  bridge_port_context_t *p_port_ctx = param;
  uint8_t uart_num = p_port_ctx->port_id;
  bridge_context_t *ctx = p_port_ctx->uart_ctx;
  bridge_port_obj_t *port_obj = &ctx->fd_uart_to_rs485;

  bool rx_buffer_free = true;
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
            port_obj, uart_num,
            (const uint8_t *) chunk, chunk->data_len + sizeof(bridge_uart_data_t),
            &HPTaskAwoken, &need_yield);
          if (sent) {
            // successfully pushed data to the bridging buffer
            UART_ENTER_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
            port_obj->buffered_len += chunk->data_len;
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
              port_obj, uart_num,
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
      if (port_obj->producer_tx_waiting_brk) {
        uart_hal_ena_intr_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TXFIFO_EMPTY);
      }
      UART_EXIT_CRITICAL_ISR(&(merge_wire_uart_context[uart_num].spinlock));
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), UART_INTR_TX_BRK_DONE);
      if (port_obj->producer_tx_waiting_brk) {
        port_obj->producer_tx_waiting_brk = false;
      } else {
        xSemaphoreGiveFromISR(ctx->fd_tx_brk_sem, &HPTaskAwoken);
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
      xSemaphoreGiveFromISR(ctx[uart_num].fd_tx_done_sem, &HPTaskAwoken);
      need_yield |= (HPTaskAwoken == pdTRUE);
    } else {
      uart_hal_clr_intsts_mask(&(merge_wire_uart_context[uart_num].hal), uart_intr_status); /*simply clear all other intr status*/
    }
  }
  if (need_yield) {
    portYIELD_FROM_ISR();
  }
}
