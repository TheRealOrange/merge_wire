#ifndef MERGE_WIRE_DRIVER_COMMON_H
#define MERGE_WIRE_DRIVER_COMMON_H
#include "driver.h"

extern bridge_port_context_t merge_wire_driver_uart_ctx[UART_NUM_MAX];
extern bridge_uart_port_context_t merge_wire_uart_context[UART_NUM_MAX];

// if we want this to be in IRAM, we need to ensure the memory allocated
// that will be accessed during interrupts is only in IRAM
#ifdef CONFIG_MERGE_WIRE_ISR_IN_IRAM
#define BRIDGE_ISR_ATTR     IRAM_ATTR
#define BRIDGE_MALLOC_CAPS  (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#define BRIDGE_INTR_FLAGS   ESP_INTR_FLAG_IRAM
#else
#define BRIDGE_ISR_ATTR
#define BRIDGE_MALLOC_CAPS  MALLOC_CAP_DEFAULT
#define BRIDGE_INTR_FLAGS   0
#endif

#define UART_ENTER_CRITICAL_SAFE(spinlock)   esp_os_enter_critical_safe(spinlock)
#define UART_EXIT_CRITICAL_SAFE(spinlock)    esp_os_exit_critical_safe(spinlock)
#define UART_ENTER_CRITICAL_ISR(spinlock)    esp_os_enter_critical_isr(spinlock)
#define UART_EXIT_CRITICAL_ISR(spinlock)     esp_os_exit_critical_isr(spinlock)
#define UART_ENTER_CRITICAL(spinlock)        esp_os_enter_critical(spinlock)
#define UART_EXIT_CRITICAL(spinlock)         esp_os_exit_critical(spinlock)

uint32_t bridge_enable_tx_write_fifo(uart_port_t uart_num, const uint8_t *pbuf, uint32_t len);
void bridge_fill_tx_fifo(bridge_port_obj_t *ctx, bool defer_return, bool *brk_waiting_int_ena, bool *chunk_in_flight, bool *ringbuf_empty, BaseType_t *HPTaskAwoken, bool *need_yield);
bool bridge_try_send_ring_buf(bridge_port_obj_t *ctx, const uint8_t *data, uint32_t len, BaseType_t *HPTaskAwoken, bool *need_yield);
void bridge_service_producer(bridge_port_obj_t *ctx, BaseType_t *HPTaskAwoken, bool *need_yield);

// arm an in-band break chunk on the consumer port: hardware appends the
// break after whatever already sits in the TX FIFO. Returns the chunk to
// the ring and services the producer
// Caller manages r_state/DE
void bridge_arm_brk_chunk(bridge_port_obj_t *ctx, bridge_uart_data_t *chunk, BaseType_t *HPTaskAwoken, bool *need_yield);

void bridge_read_fifo_chunk(uart_port_t port, uint8_t *stash_buf, uint32_t status, bool *got_brk);
void bridge_rs485_carrier_sense_try_tx(bridge_context_t *ctx, BaseType_t *HPTaskAwoken, bool *need_yield);
void bridge_rs485_kick_cb(void *arg);

void bridge_uart_intr_handler(void *param);
void bridge_rs485_intr_handler(void *param);

// setting RTS 0 means RTS is de-asserted
// RTS is active LOW, so this results in DE (wired to RTS) being asserted
// since RTS de-asserted means it is HIGH (DE active, the rs485 driver controls the bus)
// setting RTS 1 means RTS is asserted
// RTS is active LOW, so this results in DE (wired to RTS) being de-asserted
// since RTS asserted means it is LOW (DE released, the rs485 driver relinquishes the bus)
#define DE_ASSERT(n) { uart_hal_set_rts(&(merge_wire_uart_context[(n)].hal), 0); }
#define DE_RELEASE(n) { uart_hal_set_rts(&(merge_wire_uart_context[(n)].hal), 1); }

#endif //MERGE_WIRE_DRIVER_COMMON_H