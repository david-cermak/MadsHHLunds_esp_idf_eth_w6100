set(srcs "src/esp_eth_mac_w6100.c"
         "src/esp_eth_phy_w6100.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS include
                    REQUIRES esp_eth)
