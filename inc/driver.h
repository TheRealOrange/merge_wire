#ifndef MERGE_WIRE_DRIVER_H
#define MERGE_WIRE_DRIVER_H
#include "esp_err.h"
#include "esp_check.h"
#include "merge_wire.h"
#include "hal/uart_types.h"

#define MERGE_WIRE_TAG "merge_wire"

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
                                int rx_buffer_size, int tx_buffer_size,
                                int fd_intr_alloc_flags, int rs485_intr_alloc_flags);

esp_err_t bridge_driver_set_pins(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin,
                                 gpio_num_t rts_pin, gpio_num_t cts_pin,
                                 gpio_num_t dtr_pin, gpio_num_t dsr_pin);

/** Snapshot of the bridge counters; pass either bridged port number. */
void bridge_driver_get_stats(uart_port_t any_bridge_port, bridge_stats_t *out);

#endif //MERGE_WIRE_DRIVER_H