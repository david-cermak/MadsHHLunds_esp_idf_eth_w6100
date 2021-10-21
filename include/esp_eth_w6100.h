/**
 * @brief w6100 config type is the same as w5500
 *
 */
typedef eth_w5500_config_t eth_w6100_config_t;

/**
 * @brief Default w6100 configuration is the same as w5500
 *
 */
#define ETH_W5500_DEFAULT_CONFIG(spi_device) \
    {                                        \
        .spi_hdl = spi_device,               \
        .int_gpio_num = 4,                   \
    }

/**
* @brief Create W6100 Ethernet MAC instance
*
* @param w6100_config: W6100 specific configuration
* @param mac_config: Ethernet MAC configuration
*
* @return
*      - instance: create MAC instance successfully
*      - NULL: create MAC instance failed because some error occurred
*/
esp_eth_mac_t *esp_eth_mac_new_w6100(const eth_w6100_config_t *w6100_config, const eth_mac_config_t *mac_config);

/**
* @brief Create a PHY instance of W5500
*
* @param[in] config: configuration of PHY
*
* @return
*      - instance: create PHY instance successfully
*      - NULL: create PHY instance failed because some error occurred
*/
esp_eth_phy_t *esp_eth_phy_new_w6100(const eth_phy_config_t *config);






