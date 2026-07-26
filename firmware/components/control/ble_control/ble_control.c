#include "ble_control.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "web_control.h"

static const char *TAG = "ble_control";

/* 5f6d1000-8f3e-4c21-a7b2-3d4e5f607182 */
static const ble_uuid128_t s_service_uuid =
    BLE_UUID128_INIT(0x82, 0x71, 0x60, 0x5f, 0x4e, 0x3d, 0xb2, 0xa7,
                     0x21, 0x4c, 0x3e, 0x8f, 0x00, 0x10, 0x6d, 0x5f);
/* 5f6d1001-8f3e-4c21-a7b2-3d4e5f607182 */
static const ble_uuid128_t s_command_uuid =
    BLE_UUID128_INIT(0x82, 0x71, 0x60, 0x5f, 0x4e, 0x3d, 0xb2, 0xa7,
                     0x21, 0x4c, 0x3e, 0x8f, 0x01, 0x10, 0x6d, 0x5f);
/* 5f6d1002-8f3e-4c21-a7b2-3d4e5f607182 */
static const ble_uuid128_t s_telemetry_uuid =
    BLE_UUID128_INIT(0x82, 0x71, 0x60, 0x5f, 0x4e, 0x3d, 0xb2, 0xa7,
                     0x21, 0x4c, 0x3e, 0x8f, 0x02, 0x10, 0x6d, 0x5f);

static uint8_t s_own_addr_type;
static uint16_t s_command_handle;
static uint16_t s_telemetry_handle;
static volatile uint16_t s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile bool s_notify_enabled;

static int format_telemetry(char *buffer, size_t size)
{
    web_control_command_t command = {0};
    web_control_telemetry_t telemetry = {0};
    web_control_get_snapshot(&command, &telemetry);
    unsigned flags = 0;
    flags |= telemetry.uwb_ok ? 1U << 0 : 0;
    flags |= telemetry.lidar_ok ? 1U << 1 : 0;
    flags |= telemetry.ultrasonic_left_ok ? 1U << 2 : 0;
    flags |= telemetry.ultrasonic_right_ok ? 1U << 3 : 0;
    flags |= telemetry.fsr_ok ? 1U << 4 : 0;
    flags |= telemetry.battery_ok ? 1U << 5 : 0;
    flags |= telemetry.encoder_ok ? 1U << 6 : 0;
    const char *state = telemetry.state == NULL ? "BOOT" : telemetry.state;
    return snprintf(
        buffer, size,
        "T,%u,%u,%u,%u,%u,%u,%s,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%d,%d",
        command.mode == WEB_CONTROL_MODE_AUTO ? 0U : 1U,
        command.estop_latched ? 1U : 0U,
        command.client_alive ? 1U : 0U,
        command.follow_speed_pct, command.follow_turn_pct,
        command.remote_speed_pct, state, flags,
        telemetry.target_distance_m, telemetry.target_bearing_rad,
        telemetry.front_clearance_m, telemetry.left_clearance_m,
        telemetry.right_clearance_m, telemetry.battery_percent,
        telemetry.battery_voltage_v, telemetry.fsr_weight_kg,
        telemetry.measured_linear_mps, telemetry.measured_angular_rps,
        telemetry.left_pulse_us, telemetry.right_pulse_us);
}

static int apply_command(const char *command)
{
    if (strcmp(command, "H") == 0) {
        web_control_heartbeat();
        return 0;
    }
    if (strcmp(command, "E") == 0) {
        web_control_emergency_stop();
        return 0;
    }
    if (strcmp(command, "A") == 0) {
        web_control_arm();
        return 0;
    }
    unsigned mode = 0;
    if (sscanf(command, "M,%u", &mode) == 1 && mode <= 1) {
        return web_control_set_mode(mode == 0 ? WEB_CONTROL_MODE_AUTO
                                              : WEB_CONTROL_MODE_MANUAL) == ESP_OK
                   ? 0
                   : BLE_ATT_ERR_UNLIKELY;
    }
    int linear = 0;
    int angular = 0;
    if (sscanf(command, "D,%d,%d", &linear, &angular) == 2 &&
        linear >= -100 && linear <= 100 && angular >= -100 && angular <= 100) {
        return web_control_set_manual((float)linear / 100.0f,
                                      (float)angular / 100.0f) == ESP_OK
                   ? 0
                   : BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    unsigned follow = 0;
    unsigned turn = 0;
    unsigned remote = 0;
    if (sscanf(command, "S,%u,%u,%u", &follow, &turn, &remote) == 3 &&
        follow <= 100 && turn <= 100 && remote <= 100) {
        return web_control_set_speeds((uint8_t)follow, (uint8_t)turn,
                                      (uint8_t)remote) == ESP_OK
                   ? 0
                   : BLE_ATT_ERR_UNLIKELY;
    }
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
}

static int command_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    char command[64];
    uint16_t length = 0;
    const int rc = ble_hs_mbuf_to_flat(ctxt->om, command,
                                       sizeof(command) - 1, &length);
    if (rc != 0 || length == 0 || length >= sizeof(command)) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    command[length] = '\0';
    while (length > 0 &&
           (command[length - 1] == '\r' || command[length - 1] == '\n')) {
        command[--length] = '\0';
    }
    return apply_command(command);
}

static int telemetry_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }
    char value[256];
    const int length = format_telemetry(value, sizeof(value));
    if (length < 0 || length >= (int)sizeof(value)) {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return os_mbuf_append(ctxt->om, value, length) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_svc_def s_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &s_command_uuid.u,
                .access_cb = command_access,
                .val_handle = &s_command_handle,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &s_telemetry_uuid.u,
                .access_cb = telemetry_access,
                .val_handle = &s_telemetry_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {0},
        },
    },
    {0},
};

static int gap_event(struct ble_gap_event *event, void *arg);

static void advertise(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "advertisement fields failed: %d", rc);
        return;
    }

    struct ble_hs_adv_fields response = {0};
    response.uuids128 = (ble_uuid128_t *)&s_service_uuid;
    response.num_uuids128 = 1;
    response.uuids128_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&response);
    if (rc != 0) {
        ESP_LOGE(TAG, "scan response failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &params, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "advertising failed: %d", rc);
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_connection_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "phone connected handle=%u", s_connection_handle);
        } else {
            advertise();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "phone disconnected reason=%d", event->disconnect.reason);
        s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
        s_notify_enabled = false;
        advertise();
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_telemetry_handle) {
            s_notify_enabled = event->subscribe.cur_notify != 0;
            ESP_LOGI(TAG, "telemetry notify=%d", s_notify_enabled);
        }
        return 0;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU=%u", event->mtu.value);
        return 0;
    default:
        return 0;
    }
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "host reset reason=%d", reason);
}

static void on_sync(void)
{
    const int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "address inference failed: %d", rc);
        return;
    }
    advertise();
    ESP_LOGI(TAG, "advertising as SmartSuitcase");
}

static void host_task(void *argument)
{
    (void)argument;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void telemetry_task(void *argument)
{
    (void)argument;
    char value[256];
    while (true) {
        if (s_notify_enabled &&
            s_connection_handle != BLE_HS_CONN_HANDLE_NONE) {
            const int length = format_telemetry(value, sizeof(value));
            if (length > 0 && length < (int)sizeof(value)) {
                struct os_mbuf *om = ble_hs_mbuf_from_flat(value, length);
                if (om != NULL) {
                    const int rc = ble_gatts_notify_custom(
                        s_connection_handle, s_telemetry_handle, om);
                    if (rc != 0 && rc != BLE_HS_ENOTCONN) {
                        ESP_LOGW(TAG, "notify failed: %d", rc);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

esp_err_t ble_control_init(void)
{
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        return ret;
    }
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sm_our_key_dist = 0;
    ble_hs_cfg.sm_their_key_dist = 0;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(s_services);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(s_services);
    }
    if (rc == 0) {
        rc = ble_svc_gap_device_name_set("SmartSuitcase");
    }
    if (rc == 0) {
        rc = ble_att_set_preferred_mtu(247);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT initialization failed: %d", rc);
        return ESP_FAIL;
    }
    nimble_port_freertos_init(host_task);
    if (xTaskCreate(telemetry_task, "ble_telemetry", 4096, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
