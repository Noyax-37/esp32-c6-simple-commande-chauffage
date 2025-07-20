/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "esp_zigbee_chauffage.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_server.h"
#include <lwip/ip_addr.h>
#include "esp_coexist.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include <stdlib.h> // Pour malloc et free

static const char *TAG = "ESP_ZIGBEE_CHAUFFAGE";

// Déclarations préalables
static void esp_zb_task(void *pvParameters);
static httpd_handle_t start_webserver(void);
static void read_thermostat_attributes(void);

// Paramètres modifiables
int16_t last_temperature = INT16_MIN;
int16_t last_setpoint = INT16_MIN;
int16_t input_setpoint = INT16_MIN;
uint16_t high_hysteresis = 10;
uint16_t low_hysteresis = 10;
uint16_t input_high_hyst = 10;
uint16_t input_low_hyst = 10;
uint8_t running_state = 0; // Nouvel état de fonctionnement du thermostat

static uint8_t last_command_sent = 0xFF;

static int s_retry_num = 0;
static bool wifi_failed = false;
static TaskHandle_t zb_task_handle = NULL;

static void bdb_start_top_level_commissioning_wrapper(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    case ESP_ZB_BDB_SIGNAL_STEERING:
        ESP_LOGI(TAG, "Signal: %s, Status: %s (0x%x)", esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status), err_status);
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Joined Zigbee network successfully (PAN ID: 0x%04hx, Channel: %d, Short Address: 0x%04hx)",
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            read_thermostat_attributes();
        } else {
            ESP_LOGW(TAG, "Network %s failed with status: %s (0x%x). Check Zigbee2MQTT configuration or enable permit join.",
                     esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status), err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_wrapper, ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s (0x%x)", 
                 esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status), err_status);
        break;
    }
}

static void send_on_off_command(uint8_t command_id)
{
    if (command_id == last_command_sent) {
        ESP_LOGI(TAG, "Skipping %s command to Shelly relay (0x%04x, endpoint %d): already in this state",
                 (command_id == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF", RELAY_CHAUFF, RELAY_BINDING_EP);
        return;
    }

    esp_zb_zcl_on_off_cmd_t cmd_req;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = RELAY_CHAUFF;
    cmd_req.zcl_basic_cmd.dst_endpoint = RELAY_BINDING_EP;
    cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd_req.on_off_cmd_id = command_id;

    ESP_LOGI(TAG, "Sending %s command to Shelly relay (0x%04x, endpoint %d)", 
             (command_id == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF", RELAY_CHAUFF, RELAY_BINDING_EP);

    esp_zb_lock_acquire(portMAX_DELAY);
    uint8_t tsn = esp_zb_zcl_on_off_cmd_req(&cmd_req);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "Sent %s command to Shelly relay (0x%04x, endpoint %d) with TSN 0x%02x", 
             (command_id == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF", RELAY_CHAUFF, RELAY_BINDING_EP, tsn);

    last_command_sent = command_id;
}

static void read_thermostat_attributes(void)
{
    esp_zb_zcl_read_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = THERMOSTAT,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        .manuf_specific = 0,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .dis_defalut_resp = 0,
        .manuf_code = 0,
        .attr_number = 4,
    };

    uint16_t attrs[] = {
        ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
        ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
        0x0101, // High hysteresis
        0x0102  // Low hysteresis
    };
    cmd.attr_field = attrs;

    esp_zb_zcl_read_attr_cmd_req(&cmd);
    ESP_LOGI(TAG, "Sent read request for thermostat attributes");
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, 
                        "Received message: error status(%d)", message->status);

    ESP_LOGI(TAG, "Received report from address(0x%04x) src endpoint(%d) to dst endpoint(%d) cluster(0x%04x)", 
             message->src_address.u.short_addr, message->src_endpoint, message->dst_endpoint, message->cluster);
    ESP_LOGI(TAG, "Report information: attribute(0x%04x), type(0x%02x), value(%d)", 
             message->attribute.id, message->attribute.data.type, 
             message->attribute.data.value ? *(int16_t *)message->attribute.data.value : 0);

    if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT && message->src_address.u.short_addr == THERMOSTAT) {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID) {
            running_state = *(uint8_t *)message->attribute.data.value;
            ESP_LOGI(TAG, "Thermostat 0x%04x Running State: %s", 
                     message->src_address.u.short_addr, (running_state == 1) ? "heat" : "idle");
        } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID) {
            last_temperature = *(int16_t *)message->attribute.data.value;
            ESP_LOGI(TAG, "Thermostat 0x%04x Local Temperature: %d.%d °C", 
                     message->src_address.u.short_addr, 
                     last_temperature / 100, abs(last_temperature % 100));
        } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
            last_setpoint = *(int16_t *)message->attribute.data.value;
            input_setpoint = last_setpoint; // Synchronisation initiale
            ESP_LOGI(TAG, "Thermostat 0x%04x Occupied Heating Setpoint: %d.%d °C", 
                     message->src_address.u.short_addr, 
                     last_setpoint / 100, abs(last_setpoint % 100));
        } else if (message->attribute.id == 0x0101) {
            high_hysteresis = *(uint16_t *)message->attribute.data.value;
            input_high_hyst = high_hysteresis;
            ESP_LOGI(TAG, "Thermostat 0x%04x High Hysteresis: %d.%d °C", 
                     message->src_address.u.short_addr, 
                     high_hysteresis / 100, abs(high_hysteresis % 100));
        } else if (message->attribute.id == 0x0102) {
            low_hysteresis = *(uint16_t *)message->attribute.data.value;
            input_low_hyst = low_hysteresis;
            ESP_LOGI(TAG, "Thermostat 0x%04x Low Hysteresis: %d.%d °C", 
                     message->src_address.u.short_addr, 
                     low_hysteresis / 100, abs(low_hysteresis % 100));
        }

        if (last_temperature != INT16_MIN && last_setpoint != INT16_MIN) {
            if (last_temperature <= last_setpoint - (int16_t)low_hysteresis) {
                send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_ON_ID);
            } else if (last_temperature >= last_setpoint + (int16_t)high_hysteresis) {
                send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID);
            }
        }
    }

    return ESP_OK;
}

static void write_thermostat_attributes(int16_t new_setpoint, uint16_t new_high_hyst, uint16_t new_low_hyst)
{
    esp_zb_zcl_write_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = THERMOSTAT,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        .manuf_specific = 0,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .dis_defalut_resp = 0,
        .manuf_code = 0,
        .attr_number = 3,
    };

    esp_zb_zcl_attribute_t attrs[3];
    attrs[0].id = ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID;
    attrs[0].data.value = &new_setpoint;
    attrs[0].data.type = 0x29;
    attrs[1].id = 0x0101;
    attrs[1].data.value = &new_high_hyst;
    attrs[1].data.type = 0x21;
    attrs[2].id = 0x0102;
    attrs[2].data.value = &new_low_hyst;
    attrs[2].data.type = 0x21;
    cmd.attr_field = attrs;

    esp_zb_zcl_write_attr_cmd_req(&cmd);
    ESP_LOGI(TAG, "Sent write request for setpoint=%d, high_hyst=%d, low_hyst=%d", new_setpoint, new_high_hyst, new_low_hyst);
    // Forcer une relecture après écriture
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Attendre une seconde pour laisser le temps à la réponse
    read_thermostat_attributes();
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
    {
        esp_zb_zcl_cmd_default_resp_message_t *resp = (esp_zb_zcl_cmd_default_resp_message_t *)message;
        ESP_LOGI(TAG, "Received ZCL Default Response from address(0x%04x) endpoint(%d) cluster(0x%04x) command(0x%02x) status(0x%02x)",
                 resp->info.src_address.u.short_addr, resp->info.src_endpoint, resp->info.cluster, 
                 resp->resp_to_cmd, resp->status_code);
        if (resp->info.src_address.u.short_addr == RELAY_CHAUFF && resp->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            ESP_LOGI(TAG, "Command %s (0x%02x) to Shelly relay (0x%04x) %s",
                     (resp->resp_to_cmd == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF", resp->resp_to_cmd,
                     RELAY_CHAUFF, (resp->status_code == 0) ? "succeeded" : "failed");
        }
        break;
    }
    default:
        ESP_LOGW(TAG, "Received Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static esp_err_t example_set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type)
{
    if (addr && (addr != IPADDR_NONE)) {
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = addr;
        dns.ip.type = IPADDR_TYPE_V4;
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns));
    }
    return ESP_OK;
}

static void example_set_static_ip(esp_netif_t *netif)
{
    if (esp_netif_dhcpc_stop(netif) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop dhcp client");
        return;
    }
    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(EXAMPLE_STATIC_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(EXAMPLE_STATIC_NETMASK_ADDR);
    ip.gw.addr = ipaddr_addr(EXAMPLE_STATIC_GW_ADDR);
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ip info");
        return;
    }
    ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", EXAMPLE_STATIC_IP_ADDR, EXAMPLE_STATIC_NETMASK_ADDR, EXAMPLE_STATIC_GW_ADDR);
    ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    esp_netif_t *netif = (esp_netif_t *)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        example_set_static_ip(netif);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Disconnected from Wi-Fi, retrying... (Attempt %d/%d)", s_retry_num + 1, WIFI_MAX_RETRIES);
        ESP_LOGE(TAG, "!! STA Disconnected! Reason: %d", event->reason);

        if (s_retry_num < WIFI_MAX_RETRIES) {
            s_retry_num++;
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "Max Wi-Fi retries reached (%d), starting Zigbee fallback.", WIFI_MAX_RETRIES);
            wifi_failed = true;
            if (zb_task_handle == NULL) {
                xTaskCreate(esp_zb_task, "Zigbee_main", 4096 * 4, NULL, 2, &zb_task_handle);
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected to Wi-Fi, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        ESP_LOGI(TAG, "Free heap size after IP: %lu bytes", esp_get_free_heap_size());
        if (zb_task_handle == NULL) {
            xTaskCreate(esp_zb_task, "Zigbee_main", 4096 * 4, NULL, 2, &zb_task_handle);
        }
        httpd_handle_t server = start_webserver();
        if (server == NULL) {
            ESP_LOGE(TAG, "Failed to start web server");
        } else {
            ESP_LOGI(TAG, "Web server started successfully");
        }
        if (esp_coex_wifi_i154_enable() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable Wi-Fi and 802.15.4 coexistence");
        } else {
            ESP_LOGI(TAG, "Wi-Fi and 802.15.4 coexistence enabled");
        }
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        sta_netif,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        sta_netif,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static esp_err_t get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Received request: URI=%s, Content-Length=%d, last_temperature=%d, last_setpoint=%d, high_hyst=%d, low_hyst=%d",
             req->uri, req->content_len, last_temperature, last_setpoint, high_hysteresis, low_hysteresis);

    FILE *f = fopen("/spiffs/index.html", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open index.html");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    rewind(f);
    ESP_LOGI(TAG, "Index.html size: %ld bytes", file_size);
    if (file_size > 4095) {
        ESP_LOGE(TAG, "Index.html too large (%ld bytes), max is 4095 bytes", file_size);
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *response = (char *)calloc(8192, 1);
    if (response == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for response");
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    char *updated_response = (char *)calloc(8192, 1);
    if (updated_response == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for updated_response");
        free(response);
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t len = fread(response, 1, 4095, f);
    fclose(f);
    ESP_LOGI(TAG, "Read %u bytes from index.html", len);

    if (len <= 0) {
        ESP_LOGE(TAG, "Failed to read index.html");
        free(response);
        free(updated_response);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    response[len] = '\0';
    ESP_LOGI(TAG, "First 50 chars of response: %.*s", 50, response);

    char temp_str[16], setpoint_str[16], high_hyst_str[16], low_hyst_str[16], relay_str[4], running_state_str[5];
    snprintf(temp_str, sizeof(temp_str), "%.1f", last_temperature != INT16_MIN ? last_temperature / 100.0 : 0.0);
    snprintf(setpoint_str, sizeof(setpoint_str), "%.1f", last_setpoint != INT16_MIN ? last_setpoint / 100.0 : 0.0);
    snprintf(high_hyst_str, sizeof(high_hyst_str), "%.1f", high_hysteresis / 100.0);
    snprintf(low_hyst_str, sizeof(low_hyst_str), "%.1f", low_hysteresis / 100.0);
    snprintf(relay_str, sizeof(relay_str), "%s", last_command_sent == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID ? "ON" : "OFF");
    snprintf(running_state_str, sizeof(running_state_str), "%s", running_state == 1 ? "heat" : "idle");
    ESP_LOGI(TAG, "String lengths: temp=%u, setpoint=%u, high_hyst=%u, low_hyst=%u, relay=%u, running=%u",
             strlen(temp_str), strlen(setpoint_str), strlen(high_hyst_str), strlen(low_hyst_str), strlen(relay_str), strlen(running_state_str));

    int res_len = snprintf(updated_response, 8192,
                           response,
                           temp_str, setpoint_str, relay_str, running_state_str, setpoint_str, high_hyst_str, low_hyst_str);
    if (res_len >= 8192) {
        ESP_LOGE(TAG, "Buffer overflow in get_handler, response truncated, res_len=%d", res_len);
        free(response);
        free(updated_response);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Response length: %d bytes", res_len);

    httpd_resp_set_type(req, "text/html"); // Assure l'encodage UTF-8 par défaut avec httpd
    httpd_resp_send(req, updated_response, strlen(updated_response));
    free(response);
    free(updated_response);
    return ESP_OK;
}

static esp_err_t post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Received POST request: URI=%s, Content-Length=%d", req->uri, req->content_len);

    char buf[100];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)-1));
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        buf[ret] = '\0';
        remaining -= ret;
    }

    char setpoint_str[10], high_hyst_str[10], low_hyst_str[10];
    int16_t new_setpoint = last_setpoint; // Utilise last_setpoint comme base
    uint16_t new_high_hyst = high_hysteresis;
    uint16_t new_low_hyst = low_hysteresis;

    if (httpd_query_key_value(buf, "setpoint", setpoint_str, sizeof(setpoint_str)) == ESP_OK) {
        new_setpoint = (int16_t)(atof(setpoint_str) * 100 + 0.05); // Ajoute 0.05 pour arrondir correctement à 0.1
        ESP_LOGI(TAG, "New setpoint received: %.1f °C, stored as %d", atof(setpoint_str), new_setpoint);
    }
    if (httpd_query_key_value(buf, "high_hyst", high_hyst_str, sizeof(high_hyst_str)) == ESP_OK) {
        new_high_hyst = (uint16_t)(atof(high_hyst_str) * 100 + 0.05);
        if (new_high_hyst > 0x03E8) new_high_hyst = 0x03E8;
        ESP_LOGI(TAG, "New high hysteresis received: %.1f °C, stored as %d", atof(high_hyst_str), new_high_hyst);
    }
    if (httpd_query_key_value(buf, "low_hyst", low_hyst_str, sizeof(low_hyst_str)) == ESP_OK) {
        new_low_hyst = (uint16_t)(atof(low_hyst_str) * 100 + 0.05);
        if (new_low_hyst > 0x03E8) new_low_hyst = 0x03E8;
        ESP_LOGI(TAG, "New low hysteresis received: %.1f °C, stored as %d", atof(low_hyst_str), new_low_hyst);
    }

    write_thermostat_attributes(new_setpoint, new_high_hyst, new_low_hyst);
    httpd_resp_send(req, "Parameters updated", strlen("Parameters updated"));
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        return NULL;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 114688; // Augmenté à 114688 pour plus d'espace
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t get_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = get_handler,
            .user_ctx = NULL
        };
        httpd_uri_t post_uri = {
            .uri = "/update",
            .method = HTTP_POST,
            .handler = post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &get_uri);
        httpd_register_uri_handler(server, &post_uri);
        ESP_LOGI(TAG, "Web server started on port %d", config.server_port);
    } else {
        ESP_LOGE(TAG, "Failed to start web server");
    }
    return server;
}

static void esp_zb_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Free heap size at Zigbee task start: %lu bytes", esp_get_free_heap_size());
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    uint8_t zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    uint16_t app_version = 1;
    uint16_t stack_version = 0x0003;
    uint8_t hw_version = 1;
    char manu_name[] = ESP_MANUFACTURER_NAME;
    char model_id[] = ESP_MODEL_IDENTIFIER;
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &app_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &stack_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &hw_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manu_name);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model_id);

    esp_zb_attribute_list_t *esp_zb_thermostat_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT);
    esp_zb_attribute_list_t *esp_zb_on_off_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);

    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_thermostat_cluster(esp_zb_cluster_list, esp_zb_thermostat_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(true));
    ESP_LOGI(TAG, "Free heap size after Zigbee start: %lu bytes", esp_get_free_heap_size());

    while (1) {
        esp_zb_stack_main_loop();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    wifi_init();
}