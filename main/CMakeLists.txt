idf_component_register(SRCS "extreme.c"
                    INCLUDE_DIRS "."
                    REQUIRES ieee802154 kroschuthread_protocol esp_timer nvs_flash
                    PRIV_REQUIRES esp_hw_support
)

