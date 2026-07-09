#include <stdio.h>
#include "merge_wire.h"
#include "driver.h"

#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "soc/soc_caps.h"

// full-duplex uart
static fullduplex_uart_config_t main_uart_ctx;

// rs485 uart
static rs485_uart_config_t rs485_uart_ctx;

// whether the library has been configured, gates the driver install
static bool configured = false;

esp_err_t merge_wire_config(fullduplex_uart_config_t uart_cfg, rs485_uart_config_t rs485_cfg) {
  esp_err_t ret;

  // check for valid configuration
  ESP_RETURN_ON_FALSE((uart_cfg.uart_num < UART_NUM_MAX), ESP_FAIL, MERGE_WIRE_TAG, "full-duplex uart_num error");
  // RS485 must be a HIGH-POWER uart: the LP UART (last port number on
  // C5/C6/P4) has no RS485 mode and runs from a slow LP clock
  ESP_RETURN_ON_FALSE((rs485_cfg.uart_num < SOC_UART_HP_NUM), ESP_FAIL, MERGE_WIRE_TAG, "rs485 uart_num error (LP UART cannot do RS485)");
  ESP_RETURN_ON_FALSE(
    (uart_cfg.uart_num != rs485_cfg.uart_num),
    ESP_FAIL, MERGE_WIRE_TAG,
    "rs485 and full-duplex uart have to be different"
    );

  main_uart_ctx = uart_cfg;
  rs485_uart_ctx = rs485_cfg;

  // set up the full-duplex UART
  // configure UART parameters
  // (leaving .source_clk unset in uart_config is fine: uart_param_config
  //  treats 0 as "use the default clock source")
  ESP_RETURN_ON_FALSE(((ret = uart_param_config(main_uart_ctx.uart_num, &main_uart_ctx.uart_config)) == ESP_OK), ret, MERGE_WIRE_TAG, "uart_param_config error");
  ESP_RETURN_ON_FALSE(((ret = bridge_driver_set_pins(
    main_uart_ctx.uart_num,
    main_uart_ctx.tx_pin,
    main_uart_ctx.rx_pin,
    main_uart_ctx.rts_pin,
    main_uart_ctx.cts_pin,
    main_uart_ctx.dtr_pin,
    main_uart_ctx.dsr_pin
    )) == ESP_OK), ret, MERGE_WIRE_TAG, "uart_set_pin error");

  // set up the RS485 UART
  uart_config_t uart_config = {
    .baud_rate = rs485_cfg.baud_rate,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,   // never on the RS485 port: the
                                             // flow engine would own the RTS
                                             // pin our ISR needs for DE
    .rx_flow_ctrl_thresh = 64,
  };
  // configure UART parameters
  ESP_RETURN_ON_FALSE(((ret = uart_param_config(rs485_uart_ctx.uart_num, &uart_config)) == ESP_OK), ret, MERGE_WIRE_TAG, "uart_param_config error");

  // set UART pins for RS485 UART
  //
  // DE goes on RTS on EVERY target. The bridge core steers direction by
  // toggling conf0.sw_rts from its ISR (sw_rts = 0 drives the RTS pin HIGH
  // = DE asserted). The "hardware controls DTR automatically" behaviour in
  // the esp-idf docs belongs to the driver's RS485_HALF_DUPLEX mode on
  // newer UART IP -- we run the collision-detect register set driverlessly,
  // where nothing toggles DTR, and the classic ESP32 has no DTR output on
  // UART1/2 in any case. Wiring DE to DTR here would leave the transceiver
  // permanently disabled.
  ESP_RETURN_ON_FALSE(((ret = bridge_driver_set_pins(
    rs485_uart_ctx.uart_num,
    rs485_cfg.tx_pin,
    rs485_cfg.rx_pin,
    rs485_cfg.de_pin,        /* RTS -> DE                                   */
    UART_PIN_NO_CHANGE,      /* CTS                                         */
    UART_PIN_NO_CHANGE,      /* DTR                                         */
    UART_PIN_NO_CHANGE       /* DSR                                         */
    )) == ESP_OK), ret, MERGE_WIRE_TAG, "uart_set_pin error");

  // we do not install the uart driver for either uart because we handle the
  // peripherals ourselves
  configured = true;
  return ESP_OK;
}

esp_err_t merge_wire_driver_install(int rx_buffer_size, int tx_buffer_size, int rs485_intr_alloc_flags, int uart_intr_alloc_flags) {
  ESP_RETURN_ON_FALSE((configured), ESP_ERR_INVALID_STATE, MERGE_WIRE_TAG, "not configured yet!");
  ESP_RETURN_ON_FALSE((rx_buffer_size >= 512 && tx_buffer_size >= 512), ESP_ERR_INVALID_ARG, MERGE_WIRE_TAG,
                      "buffers must hold several chunks (>= 512 bytes)");
  return bridge_driver_install(&main_uart_ctx, &rs485_uart_ctx,
                               rx_buffer_size, tx_buffer_size,
                               uart_intr_alloc_flags, rs485_intr_alloc_flags);
}

esp_err_t merge_wire_get_stats(bridge_stats_t *out) {
  ESP_RETURN_ON_FALSE((configured && out != NULL), ESP_ERR_INVALID_STATE, MERGE_WIRE_TAG, "not installed");
  bridge_driver_get_stats(main_uart_ctx.uart_num, out);
  return ESP_OK;
}
