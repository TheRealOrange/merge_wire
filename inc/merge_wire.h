#ifndef MERGE_WIRE_H
#define MERGE_WIRE_H

#include "esp_err.h"
#include "driver/uart.h"
#include "soc/gpio_num.h"

typedef struct {
  uart_port_t uart_num;
  int baud_rate;
  gpio_num_t tx_pin;   /* -> transceiver DI                                  */
  gpio_num_t rx_pin;   /* <- transceiver RO; /RE tied to GND                 */
  gpio_num_t de_pin;   /* -> transceiver DE; driven via this port's RTS      */
} rs485_uart_config_t;

typedef struct {
  uart_port_t uart_num;
  uart_config_t uart_config;
  gpio_num_t tx_pin;
  gpio_num_t rx_pin;
  gpio_num_t cts_pin;
  gpio_num_t rts_pin;
  gpio_num_t dtr_pin;
  gpio_num_t dsr_pin;
} fullduplex_uart_config_t;

typedef struct {
  uint32_t fd_rx_bytes, fd_tx_bytes;
  uint32_t rs485_rx_bytes, rs485_tx_bytes;
  uint32_t collisions, retries, tx_giveups;
  uint32_t breaks_forwarded, breaks_dropped;
  uint32_t fd_rx_overflows, rs485_rx_overflows;
  uint32_t fd_line_errors, rs485_line_errors;
} bridge_stats_t;

/**
 * @brief configures the UART bridge, enables the UART HW for the RS485 port
 * @param uart_cfg config struct defining parameters for the full-duplex UART port, should NOT
 * have an esp-idf uart driver installed and should NOT be configured yet (merge_wire handles it)
 * @param rs485_cfg config struct defining parameters for the RS485 port
 * @return
 */
esp_err_t merge_wire_config(fullduplex_uart_config_t uart_cfg, rs485_uart_config_t rs485_cfg);

/**
 * @brief takes driver control for the RS485 port and the UART port
 * @param rx_buffer_size RS485 receive buffer, full-duplex UART send buffer
 * @param tx_buffer_size full-duplex UART receive buffer, RS485 send buffer (should be larger to allow for collision-backoff)
 * @param rs485_intr_alloc_flags interrupt handler allocation flags for RS485 interrupts
 * @param uart_intr_alloc_flags interrupt handler allocation flags for UART interrupts
 * @return
 */
esp_err_t merge_wire_driver_install(int rx_buffer_size, int tx_buffer_size, int rs485_intr_alloc_flags, int uart_intr_alloc_flags);

/** @brief snapshot of the bridge diagnostic counters */
esp_err_t merge_wire_get_stats(bridge_stats_t *out);

#endif // MERGE_WIRE_H