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

static const char *TAG = "ESP_ZIGBEE_CHAUFFAGE";

// Paramètres modifiables
int16_t last_setpoint = DEFAULT_OCCUPIED_HEATING_SETPOINT;
int16_t hysteresis_minus = HYSTERESIS_MOINS;
int16_t hysteresis_plus = HYSTERESIS_PLUS;

// État global du relais
static uint8_t last_command_sent = 0xFF; // Valeur initiale invalide pour forcer la première commande

// Variables pour gérer les tentatives Wi-Fi
static int wifi_retry_count = 0;
static bool wifi_failed = false;

// Déclaration préalable de start_webserver pour éviter les erreurs d'ordre
static httpd_handle_t start_webserver(void);

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
        } else {
            ESP_LOGW(TAG, "Network %s failed with status: %s (0x%x). Check Zigbee2MQTT configuration or enable permit join.",
                     esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status), err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_wrapper, ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000); // Délai de 5 secondes
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
    // Éviter d'envoyer la même commande consécutivement
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

    last_command_sent = command_id; // Mettre à jour l'état global
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
             message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0);

    if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT && message->src_address.u.short_addr == THERMOSTAT) {
        static int16_t last_temperature = INT16_MIN; // Valeur invalide initiale

        if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID) {
            uint8_t running_state = *(uint8_t *)message->attribute.data.value;
            ESP_LOGI(TAG, "Thermostat 0x%04x Running State: %s", 
                     message->src_address.u.short_addr, (running_state == 1) ? "heat" : "idle");

            if (running_state == 1) {
                send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_ON_ID);
            } else {
                send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID);
            }
        } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID) {
            last_temperature = *(int16_t *)message->attribute.data.value;
            ESP_LOGI(TAG, "Thermostat 0x%04x Local Temperature: %d.%d °C (seuil de chauffe %d.%d)", 
                     message->src_address.u.short_addr, 
                     last_temperature / 100, abs(last_temperature % 100), 
                     last_setpoint / 100, abs(last_setpoint % 100));

            if (last_temperature <= last_setpoint - hysteresis_minus) {
                send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_ON_ID);
            } else if (last_temperature >= last_setpoint + hysteresis_plus) {
                send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID);
            }
        } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
            last_setpoint = *(int16_t *)message->attribute.data.value;
            ESP_LOGI(TAG, "Thermostat 0x%04x Occupied Heating Setpoint: %d.%d °C (température mesurée %d.%d)", 
                     message->src_address.u.short_addr, 
                     last_setpoint / 100, abs(last_setpoint % 100), 
                     last_temperature / 100, abs(last_temperature % 100));

            if (last_temperature != INT16_MIN) {
                if (last_temperature <= last_setpoint - hysteresis_minus) {
                    send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_ON_ID);
                } else if (last_temperature >= last_setpoint + hysteresis_plus) {
                    send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID);
                }
            } else {
                ESP_LOGI(TAG, "Skipping ON/OFF decision: temperature not yet initialized");
            }
        }
    }

    return ESP_OK;
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

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Disconnected from Wi-Fi, retrying... (Attempt %d/%d)", wifi_retry_count + 1, WIFI_MAX_RETRIES);
        ESP_LOGE(TAG, "!! STA Disconnected! Reason: %d", event->reason);

        if (wifi_retry_count < WIFI_MAX_RETRIES) {
            wifi_retry_count++;
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "Max Wi-Fi retries reached (%d), starting Zigbee fallback.", WIFI_MAX_RETRIES);
            wifi_failed = true;
            ESP_ERROR_CHECK(esp_zb_start(true)); // Lancer Zigbee après échec
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected to Wi-Fi, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_count = 0; // Réinitialiser le compteur en cas de succès
        ESP_ERROR_CHECK(esp_zb_start(true)); // Lancer Zigbee après connexion Wi-Fi
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Délai de 1 seconde
        httpd_handle_t server = start_webserver();
        if (server == NULL) {
            ESP_LOGE(TAG, "Web server failed to start, retrying in 2 seconds");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            server = start_webserver(); // Tentative de redémarrage
            if (server == NULL) {
                ESP_LOGE(TAG, "Web server failed to start after retry");
            }
        }
        ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size()); // Correction ici
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
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
    static int16_t last_temperature = INT16_MIN; // Synchronisé avec zb_attribute_reporting_handler
    char response[1024]; // Augmenter la taille du buffer
    snprintf(response, sizeof(response),
             "<html><body>"
             "<h1>Thermostat Control</h1>"
             "<p>Temperature: %.2f °C</p>"
             "<p>Setpoint: %.2f °C</p>"
             "<p>Hysteresis (-): %.2f °C</p>"
             "<p>Hysteresis (+): %.2f °C</p>"
             "<p>Relay State: %s</p>"
             "<form method='POST' action='/update'>"
             "<label>Setpoint: <input type='number' name='setpoint' step='0.01' value='%.2f'></label><br>"
             "<label>Hysteresis (-): <input type='number' name='hyst_minus' step='0.01' value='%.2f'></label><br>"
             "<label>Hysteresis (+): <input type='number' name='hyst_plus' step='0.01' value='%.2f'></label><br>"
             "<input type='submit' value='Update'>"
             "</form>"
             "<script>"
             "setInterval(function() {"
             "  fetch('/').then(response => response.text()).then(data => document.body.innerHTML = data);"
             "}, 5000);"
             "</script>"
             "</body></html>",
             last_temperature != INT16_MIN ? last_temperature / 100.0 : 0.0,
             last_setpoint / 100.0,
             hysteresis_minus / 100.0,
             hysteresis_plus / 100.0,
             last_command_sent == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID ? "ON" : "OFF",
             last_setpoint / 100.0,
             hysteresis_minus / 100.0,
             hysteresis_plus / 100.0);

    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static esp_err_t post_handler(httpd_req_t *req)
{
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

    char setpoint_str[10], hyst_minus_str[10], hyst_plus_str[10];
    if (httpd_query_key_value(buf, "setpoint", setpoint_str, sizeof(setpoint_str)) == ESP_OK) {
        last_setpoint = (int16_t)(atof(setpoint_str) * 100);
        ESP_LOGI(TAG, "Updated setpoint to %.2f °C", last_setpoint / 100.0);
    }
    if (httpd_query_key_value(buf, "hyst_minus", hyst_minus_str, sizeof(hyst_minus_str)) == ESP_OK) {
        hysteresis_minus = (int16_t)(atof(hyst_minus_str) * 100);
        ESP_LOGI(TAG, "Updated hysteresis (-) to %.2f °C", hysteresis_minus / 100.0);
    }
    if (httpd_query_key_value(buf, "hyst_plus", hyst_plus_str, sizeof(hyst_plus_str)) == ESP_OK) {
        hysteresis_plus = (int16_t)(atof(hyst_plus_str) * 100);
        ESP_LOGI(TAG, "Updated hysteresis (+) to %.2f °C", hysteresis_plus / 100.0);
    }

    httpd_resp_send(req, "Parameters updated", strlen("Parameters updated"));
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 8080; // Changer pour le port 8080
    httpd_handle_t server = NULL;

    ESP_LOGI(TAG, "Starting web server on port %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Web server started successfully");
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
    } else {
        ESP_LOGE(TAG, "Failed to start web server");
    }
    return server;
}

static void esp_zb_task(void *pvParameters)
{
    // Initialiser la pile Zigbee
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // Créer le cluster Basic avec attributs standard
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

    // Créer les clusters client Thermostat et On/Off
    esp_zb_attribute_list_t *esp_zb_thermostat_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT);
    esp_zb_attribute_list_t *esp_zb_on_off_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);

    // Créer la liste des clusters
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_thermostat_cluster(esp_zb_cluster_list, esp_zb_thermostat_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    // Créer la liste des endpoints
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

    // Enregistrer l'appareil
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK); // Utiliser le canal 11

    // Initialiser Wi-Fi (démarré dans wifi_init, Zigbee démarré dans wifi_event_handler)
    wifi_init();

    // Pas de boucle d'attente ici, laisser wifi_event_handler gérer le démarrage
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096 * 4, NULL, 5, NULL); // Augmenter la taille de la pile à 16 KB
}