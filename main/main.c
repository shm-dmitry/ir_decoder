#include "driver/rmt.h"
#include "ir_tools/ir_tools.h"
#include "esp_err.h"
#include "esp_log.h"
#include "string.h"

#define TAG "ir_decode"
#define GPIO_NUMBER 26

void app_main(void)
{
    uint32_t addr = 0;
    uint32_t cmd = 0;
    size_t length = 0;
    bool repeat = false;
    RingbufHandle_t rb = NULL;
    rmt_item32_t *items = NULL;

    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(GPIO_NUMBER, RMT_CHANNEL_0);

    rmt_config(&rmt_rx_config);
    rmt_driver_install(RMT_CHANNEL_0, 1000, 0);

    ir_parser_config_t ir_parser_config = IR_PARSER_DEFAULT_CONFIG((ir_dev_t)RMT_CHANNEL_0);
    ir_parser_config.flags |= IR_TOOLS_FLAGS_PROTO_EXT; // Using extended IR protocols (both NEC and RC5 have extended version)

    ir_parser_t *ir_parser_nec = ir_parser_rmt_new_nec(&ir_parser_config);
    ir_parser_t *ir_parser_rc5 = ir_parser_rmt_new_rc5(&ir_parser_config);

    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(RMT_CHANNEL_0, &rb);
    assert(rb != NULL);
    // Start receive
    rmt_rx_start(RMT_CHANNEL_0, true);
    while (1) {
        items = (rmt_item32_t *) xRingbufferReceive(rb, &length, portMAX_DELAY);
        if (items) {
            length /= 4; // one RMT = 4 Bytes
            if (ir_parser_nec->input(ir_parser_nec, items, length) == ESP_OK) {
                if (ir_parser_nec->get_scan_code(ir_parser_nec, &addr, &cmd, &repeat) == ESP_OK) {
                    ESP_LOGI(TAG, "NEC Scan Code %s --- addr: 0x%04x cmd: 0x%04x", repeat ? "(repeat)" : "", addr, cmd);
                }
            }

            if (ir_parser_rc5->input(ir_parser_rc5, items, length) == ESP_OK) {
                if (ir_parser_rc5->get_scan_code(ir_parser_rc5, &addr, &cmd, &repeat) == ESP_OK) {
                    ESP_LOGI(TAG, "RC5 Scan Code %s --- addr: 0x%04x cmd: 0x%04x", repeat ? "(repeat)" : "", addr, cmd);
                }
            }

            //after parsing the data, return spaces to ringbuffer.
            vRingbufferReturnItem(rb, (void *) items);
        }
    }

    ir_parser_nec->del(ir_parser_nec);
    ir_parser_rc5->del(ir_parser_rc5);

    rmt_driver_uninstall(RMT_CHANNEL_0);
}

