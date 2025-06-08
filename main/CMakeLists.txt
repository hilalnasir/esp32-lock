idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi nvs_flash mqtt esp_netif
    EMBED_FILES "aws_root_ca.pem" "client.crt" "client.key"
)
