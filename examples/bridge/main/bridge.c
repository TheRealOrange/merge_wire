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

// bridging (glue) buffer sizes. The UART -> RS485 buffer should be the larger
// one: it absorbs data queued while the bus is busy, while collision backoff
// is in progress, and (under the RETRY policy) while a chunk is replayed.
#define BRIDGE_RS485_TO_UART_BUF_SIZE   (2048)
#define BRIDGE_UART_TO_RS485_BUF_SIZE   (4096)

// how often to dump the bridge counters
#define BRIDGE_STATS_PERIOD_MS          (5000)

void app_main(void) {
  // full-duplex UART config
  fullduplex_uart_config_t main_uart_config = {
    .uart_num = CONFIG_FULL_DUPLEX_UART_PORT_NUM,
    .uart_config = {
      .baud_rate = CONFIG_FULL_DUPLEX_UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl =
#if CONFIG_FULL_DUPLEX_UART_HW_FLOWCTRL
        // CTS_RTS enables BOTH halves. The CTS half gates our transmitter on
        // the peer's RTS: if that wire is not actually connected, the main
        // TX stalls forever waiting for permission. Only enable with both
        // handshake lines physically wired.
        UART_HW_FLOWCTRL_CTS_RTS,
#else
        UART_HW_FLOWCTRL_DISABLE,
#endif
      // RTS deasserts when the RX FIFO reaches this fill level. 64 leaves 64
      // bytes of hardware slack for the peer's in-flight bytes -- USB-serial
      // adapters can be slow to react to RTS.
      .rx_flow_ctrl_thresh = 64,
    },
    .tx_pin = CONFIG_FULL_DUPLEX_UART_TX_IO,
    .rx_pin = CONFIG_FULL_DUPLEX_UART_RX_IO,
    // IMPORTANT: default every handshake pin to NO_CHANGE. A zero-initialized
    // gpio_num_t is GPIO0 (a strapping pin!) and would get routed by
    // uart_set_pin. Flow-control pins are assigned below only when enabled.
    .cts_pin = UART_PIN_NO_CHANGE,
    .rts_pin = UART_PIN_NO_CHANGE,
    // DTR/DSR stay unrouted on purpose: the classic ESP32 only has these
    // signals on UART0 (uart_set_pin refuses them on UART1/2), and no target
    // drives them automatically. If your full-duplex peer needs modem
    // signalling, drive plain GPIOs from application logic instead.
    .dtr_pin = UART_PIN_NO_CHANGE,
    .dsr_pin = UART_PIN_NO_CHANGE,
  };
#if CONFIG_FULL_DUPLEX_UART_HW_FLOWCTRL
  main_uart_config.cts_pin = CONFIG_FULL_DUPLEX_UART_CTS_IO;
  main_uart_config.rts_pin = CONFIG_FULL_DUPLEX_UART_RTS_IO;
#endif

  // rs485 UART config
  rs485_uart_config_t rs485_uart_config = {
    .uart_num = CONFIG_RS485_UART_PORT_NUM,
    .baud_rate = CONFIG_RS485_UART_BAUD_RATE,
    .tx_pin = CONFIG_RS485_TX_IO,
    .rx_pin = CONFIG_RS485_RX_IO,
    .de_pin = CONFIG_RS485_DIRECTION_IO,
  };

  // bring the bridge up
  ESP_LOGI(TAG, "merge_wire bridge: fd=UART%d @%d <-> rs485=UART%d @%d",
           main_uart_config.uart_num, main_uart_config.uart_config.baud_rate,
           rs485_uart_config.uart_num, rs485_uart_config.baud_rate);
#if CONFIG_MERGE_WIRE_COLLISION_DROP
  ESP_LOGI(TAG, "collision policy: DROP (frame-unaware: collided chunk is "
                "discarded, recovery belongs to the end-to-end protocol)");
#else
  ESP_LOGI(TAG, "collision policy: RETRY (commit-when-verified: collided "
                "chunk is rewound and replayed with exponential backoff)");
#endif

  // configure parameters + pins for both ports
  // ENSURE no esp-idf uart driver is installed on either port
  ESP_ERROR_CHECK(merge_wire_config(main_uart_config, rs485_uart_config));

  // install the bridge driver
  ESP_ERROR_CHECK(merge_wire_driver_install(BRIDGE_RS485_TO_UART_BUF_SIZE,
                                            BRIDGE_UART_TO_RS485_BUF_SIZE,
                                            0 /* rs485_intr_alloc_flags */,
                                            0 /* uart_intr_alloc_flags  */));

  ESP_LOGI(TAG, "bridge running; data path is interrupt-driven from here on");

  bridge_stats_t prev = {0};
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(BRIDGE_STATS_PERIOD_MS));

    bridge_stats_t st;
    if (merge_wire_get_stats(&st) != ESP_OK) {
      continue;
    }
    uint32_t fd_rx_rate    = (st.fd_rx_bytes    - prev.fd_rx_bytes)    * 1000u / BRIDGE_STATS_PERIOD_MS;
    uint32_t rs485_rx_rate = (st.rs485_rx_bytes - prev.rs485_rx_bytes) * 1000u / BRIDGE_STATS_PERIOD_MS;

    ESP_LOGI(TAG, "fd rx/tx %lu/%lu B (%lu B/s in) | rs485 rx/tx %lu/%lu B (%lu B/s in)",
             (unsigned long) st.fd_rx_bytes, (unsigned long) st.fd_tx_bytes,
             (unsigned long) fd_rx_rate,
             (unsigned long) st.rs485_rx_bytes, (unsigned long) st.rs485_tx_bytes,
             (unsigned long) rs485_rx_rate);
    ESP_LOGI(TAG, "coll %lu retry %lu giveup %lu | brk fwd %lu | ovf fd/485 %lu/%lu | lineerr %lu",
             (unsigned long) st.collisions, (unsigned long) st.retries,
             (unsigned long) st.tx_giveups, (unsigned long) st.breaks_forwarded,
             (unsigned long) st.fd_rx_overflows, (unsigned long) st.rs485_rx_overflows,
             (unsigned long) st.rs485_line_errors);
    prev = st;
  }
}