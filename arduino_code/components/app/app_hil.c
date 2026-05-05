#include "app_hil.h"
#include "app_context.h"
#include "ekf.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "math.h"

/** Message structure for communication between the USB CDC ACM callback and the HIL task. It contains the command identifier, 
 * a data buffer for the received payload, the length of the received data, and the index of the CDC device interface that received 
 * the data. */
typedef struct {
    uint8_t cmd;    // Command identifier
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];     // Data buffer
    size_t buf_len;                                     // Number of bytes received
    uint8_t itf;                                        // Index of CDC device interface
} app_message_t;


static QueueHandle_t hil_queue;
static uint8_t rx_buf[124];

/**
 * @brief Callback function for handling incoming USB CDC ACM data. It reads the received data, identifies the command, and sends a message to the HIL queue for further processing based on the command type.
 * @param itf The index of the CDC device interface that received the data
 * @param event The event structure containing information about the received data
 */
static void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event) 
{
    size_t rx_size = 0;
    if (tinyusb_cdcacm_read(itf, rx_buf, sizeof(rx_buf), &rx_size) != ESP_OK)
        return;

    if (rx_size == 0)
        return;

    uint8_t cmd = rx_buf[0];

    /* ---------- START ---------- */
    if (cmd == HIL_CMD_START && rx_size == 1 + 3 * sizeof(float)) {
        app_message_t msg = {
            .cmd = cmd,
            .buf_len = 1 + 3*sizeof(float),
            .itf = itf
        };
        memcpy(msg.buf, &rx_buf[1], msg.buf_len);
        xQueueSend(hil_queue, &msg, 0);
        return;
    }

    /* ---------- STOP ---------- */
    if (cmd == HIL_CMD_STOP) {
        app_message_t msg = {
            .cmd = cmd,
            .buf_len = 0,
            .itf = itf
        };
        xQueueSend(hil_queue, &msg, 0);
        return;
    }

    /* ---------- HIL MEAS ---------- */
    if (cmd == HIL_CMD_NEW_MEAS && rx_size == 1 + 11 * sizeof(float)) {
        app_message_t msg = {
            .cmd = cmd,
            .buf_len = 11*sizeof(float),
            .itf = itf
        };
        memcpy(msg.buf, &rx_buf[1], msg.buf_len);
        xQueueSend(hil_queue, &msg, 0);
        return;
    }
}

/* Public API*/

void app_hil_init(void) 
{
    hil_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(hil_queue);

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = NULL,
        .hs_configuration_descriptor = NULL,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = NULL,
#endif
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
};

void app_hil_loop(app_context_t *ctx) 
{    
    float dt = 1.0f/HIL_FREQ_HZ;

    for(;;) {
        app_message_t msg;
        if (!xQueueReceive(hil_queue, &msg, portMAX_DELAY))
            continue;

        switch(msg.cmd) {
            case HIL_CMD_START:
                memcpy(ctx->target_enu, msg.buf, sizeof(ctx->target_enu));

                /* ekf initialization */
                ctx->total_distance = sqrtf(ctx->target_enu[0]*ctx->target_enu[0] + ctx->target_enu[1]*ctx->target_enu[1] + ctx->target_enu[2]*ctx->target_enu[2]);
                ctx->time_s = 0.0f;
                ekf_init(ctx->ekf_x, ctx->ekf_P);

                /* acknowledgment */
                uint8_t ack = 0x88;
                tinyusb_cdcacm_write_queue(msg.itf, &ack, sizeof(ack));
                tinyusb_cdcacm_write_flush(msg.itf, 0);

                ctx->running = true;       
                break;
            
            case HIL_CMD_STOP:
                ctx->running = false;
                break;

            case HIL_CMD_NEW_MEAS:
                if(!ctx->running) continue;

                float meas[11];
                memcpy(meas, msg.buf, sizeof(meas));
                ctx->time_s = ctx->time_s + dt;

                ekf_processing(ctx, meas, dt);

                float tx[19];
                tx[0] = ctx->actuators.cm;
                tx[1] = ctx->actuators.cn;
                tx[2] = ctx->actuators.cant;
                memcpy(&tx[3], ctx->ekf_x, 16*sizeof(float));
                tinyusb_cdcacm_write_queue(msg.itf, (const uint8_t *) tx, sizeof(tx));
                tinyusb_cdcacm_write_flush(msg.itf, 0);
                break;

            default:
                break;
        }
    }
}