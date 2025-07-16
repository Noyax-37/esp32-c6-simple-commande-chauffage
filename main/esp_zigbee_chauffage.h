/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#ifndef ESP_ZIGBEE_CHAUFFAGE_H
#define ESP_ZIGBEE_CHAUFFAGE_H

#include "esp_zigbee_core.h"

// Configuration Wi-Fi
#define WIFI_SSID "Bbox-417260CF-Legacy"
#define WIFI_PASSWORD "KqphCaz6QgxZc697GS"
#define WIFI_MAX_RETRIES 10

// Paramètres IP statique
#define STATIC_IP_ADDR0 192
#define STATIC_IP_ADDR1 168
#define STATIC_IP_ADDR2 1
#define STATIC_IP_ADDR3 160  // Choisissez une IP dans la plage de votre réseau, ex. 192.168.1.98

#define STATIC_GW_ADDR0 192
#define STATIC_GW_ADDR1 168
#define STATIC_GW_ADDR2 1
#define STATIC_GW_ADDR3 1   // Adresse de la passerelle (votre routeur)

#define STATIC_NETMASK_ADDR0 255
#define STATIC_NETMASK_ADDR1 255
#define STATIC_NETMASK_ADDR2 255
#define STATIC_NETMASK_ADDR3 0  // Masque de sous-réseau standard

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false       /* enable the install code policy for security */
#define HA_ONOFF_SWITCH_ENDPOINT        10          /* esp switch device endpoint */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     (1l << 11)  /* Zigbee primary channel mask use in the example */
#define RELAY_CHAUFF                    0xB377      /* Relay for chauffage */
#define RELAY_BINDING_EP                1           /* Endpoint for binding to the relay */
#define DEFAULT_OCCUPIED_HEATING_SETPOINT 2100      /* en centièmes de degré */
#define THERMOSTAT                      0xD5AA      /* Thermostat device ID */
#define HYSTERESIS_MOINS                10          /* Hysteresis inférieur en centièmes de degré */
#define HYSTERESIS_PLUS                 10          /* Hysteresis supérieur en centièmes de degré */

// Attributs personnalisés dans le cluster Basic
#define ZCL_THERMOSTAT_ATTR_RELAY_STATE          0xF000

// Adresses IEEE (64 bits) pour le thermostat et le relais
static const esp_zb_ieee_addr_t THERMOSTAT_IEEE_ADDR = {0x00, 0x0d, 0x6f, 0x00, 0x0b, 0xf8, 0xb7, 0xd7}; // Adresse IEEE du thermostat 0x000d6f000bf8b7d7
static const esp_zb_ieee_addr_t RELAY_IEEE_ADDR = {0x7c, 0x2c, 0x67, 0xff, 0xfe, 0x75, 0xc2, 0x8c}; // Adresse IEEE du relais Shelly 0x7c2c67fffe75c28c

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x09""ESPRESSIF"         /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x07"CONFIG_IDF_TARGET    /* Customized model identifier */

#define ESP_ZB_ZR_CONFIG()                                       \
    {                                                           \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,               \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,       \
        .nwk_cfg.zczr_cfg = {                                   \
            .max_children = 10,                                 \
        },                                                      \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }

#endif /* ESP_ZIGBEE_CHAUFFAGE_H */