idf_component_register(
    SRCS
    "esp_zigbee_chauffage.c"
    INCLUDE_DIRS "."
)
# Ajouter SPIFFS
spiffs_create_partition_image(spiffs ./spiffs FLASH_IN_PROJECT)