#include <stdio.h>

#include "merge_wire.h"

#include "soc/gpio_num.h"
#include "driver/uart.h"
#include "freertos/queue.h"

/**
 * this is a demo application to show the usage of merge_wire to create a UART bridge between
 * a RS485 half-duplex UART and a full-duplex TX/RX UART
 */
#define TAG "BRIDGE_EXAMPLE_APP"

void app_main(void) {
  // set up the main TX/RX UART
  fullduplex_uart_config_t main_uart_config = {
    .uart_num = CONFIG_FULL_DUPLEX_UART_PORT_NUM,
    .uart_config = {
      .baud_rate = CONFIG_FULL_DUPLEX_UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl =
#if CONFIG_FULL_DUPLEX_UART_HW_FLOWCTRL
        UART_HW_FLOWCTRL_CTS_RTS,
#else
        UART_HW_FLOWCTRL_DISABLE,
#endif
      .rx_flow_ctrl_thresh = 122,
    },
    .tx_pin = CONFIG_FULL_DUPLEX_UART_TX_IO,
    .rx_pin = CONFIG_FULL_DUPLEX_UART_RX_IO,
#if FULL_DUPLEX_UART_HW_FLOWCTRL
    .cts_pin = CONFIG_FULL_DUPLEX_UART_CTS_IO,
    .rts_pin = CONFIG_FULL_DUPLEX_UART_RTS_IO,
    .dtr_pin = CONFIG_FULL_DUPLEX_UART_DTR_IO,
    .dsr_pin = CONFIG_FULL_DUPLEX_UART_DSR_IO,
#endif
  };

  rs485_uart_config_t rs485_uart_config = {
    .uart_num = CONFIG_RS485_UART_PORT_NUM,
    .baud_rate = CONFIG_RS485_UART_BAUD_RATE,
    .tx_pin = CONFIG_RS485_TX_IO,
    .rx_pin = CONFIG_RS485_RX_IO,
    .de_pin = CONFIG_RS485_DIRECTION_IO,
  };

  // init the merge_wire serial bridge
  merge_wire_config(main_uart_config, rs485_uart_config);
}
