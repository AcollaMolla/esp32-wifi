#include <stdio.h>
#include <esp_wifi.h>
#include "esp_log.h"
#include <string.h>
#include "nvs_flash.h"
#include <driver/gpio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DEFAULT_SCAN_LIST_SIZE 20
#define BLINK_GPIO GPIO_NUM_2
#define BEACON_SSID_OFFSET 38
#define SRCADDR_OFFSET 10
#define BSSID_OFFSET 16
#define SEQNUM_OFFSET 22

static void blink_led(void);
static void wifi_send_deauth(wifi_ap_record_t ap_info);

static const char *TAG_SCAN = "wifi-scan";
static const char *TAG_LED = "led";
static const char *TAG_DEAUTH = "wifi-deauth";
static const char *TAG_BEACON = "wifi-beacon";
static uint8_t s_led_state = 0;
static bool wifi_driver_configured = false;

static uint8_t deauth_frame_default[26] = {
	0xc, 0x0c, 0x3a, 0x01,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xf0, 0xff, 0x02, 0x00
};

static uint8_t beacon_raw[] = {
	0x80, 0x00,							// 0-1: Frame Control
	0x00, 0x00,							// 2-3: Duration
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,				// 4-9: Destination address (broadcast)
	0xba, 0xde, 0xaf, 0xfe, 0x00, 0x06,				// 10-15: Source address
	0xba, 0xde, 0xaf, 0xfe, 0x00, 0x06,				// 16-21: BSSID
	0x00, 0x00,							// 22-23: Sequence / fragment number
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,			// 24-31: Timestamp (GETS OVERWRITTEN TO 0 BY HARDWARE)
	0x64, 0x00,							// 32-33: Beacon interval
	0x31, 0x04,							// 34-35: Capability info
	0x00, 0x00, /* FILL CONTENT HERE */				// 36-38: SSID parameter set, 0x00:length:content
	0x01, 0x08, 0x82, 0x84,	0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24,	// 39-48: Supported rates
	0x03, 0x01, 0x01,						// 49-51: DS Parameter set, current channel 1 (= 0x01),
	0x05, 0x04, 0x01, 0x02, 0x00, 0x00,				// 52-57: Traffic Indication Map
	
};

char *rick_ssids[] = {
	"01 Never gonna give you up",
	"02 Never gonna let you down",
	"03 Never gonna run around",
	"04 and desert you",
	"05 Never gonna make you cry",
	"06 Never gonna say goodbye",
	"07 Never gonna tell a lie",
	"08 and hurt you"
};

esp_err_t event_handler(void *ctx, system_event_t *event){
	return ESP_OK;
}

//Unfourtanly, sending DEAUTH frames is not yet possible in esp_wifi_80211_tx API...
//For more info: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv417esp_wifi_80211_tx16wifi_interface_tPKvib
static void wifi_send_deauth(wifi_ap_record_t ap_info){
	ESP_LOGI(TAG_DEAUTH, "Sending DEAUTH frame to AP %s\n", ap_info.ssid);
	ESP_ERROR_CHECK(esp_wifi_set_channel(ap_info.primary, WIFI_SECOND_CHAN_NONE));
	vTaskDelay(10/portTICK_PERIOD_MS);

	deauth_frame_default[10] = ap_info.bssid[0];
	deauth_frame_default[11] = ap_info.bssid[1];
	deauth_frame_default[12] = ap_info.bssid[2];
	deauth_frame_default[13] = ap_info.bssid[3];
	deauth_frame_default[14] = ap_info.bssid[4];
	deauth_frame_default[15] = ap_info.bssid[5];

	deauth_frame_default[16] = ap_info.bssid[0];
	deauth_frame_default[17] = ap_info.bssid[1];
	deauth_frame_default[18] = ap_info.bssid[2];
	deauth_frame_default[19] = ap_info.bssid[3];
	deauth_frame_default[20] = ap_info.bssid[4];
	deauth_frame_default[21] = ap_info.bssid[5];

	ESP_LOGI(TAG_DEAUTH, "Sending 3 deauth frames to AP\n");
	esp_wifi_80211_tx(ESP_IF_WIFI_STA, deauth_frame_default, sizeof(deauth_frame_default), false);
	esp_wifi_80211_tx(ESP_IF_WIFI_STA, deauth_frame_default, sizeof(deauth_frame_default), false);
	esp_wifi_80211_tx(ESP_IF_WIFI_STA, deauth_frame_default, sizeof(deauth_frame_default), false);
}

void wifi_send_beacon(void *pvParameters){
	uint8_t line = 0;
	while(true){
		blink_led();
		vTaskDelay(250/portTICK_PERIOD_MS);
		uint8_t beacon_rick[200];
		memcpy(beacon_rick, beacon_raw, BEACON_SSID_OFFSET - 1);
		beacon_rick[BEACON_SSID_OFFSET - 1] = strlen(rick_ssids[line]);
		memcpy(&beacon_rick[BEACON_SSID_OFFSET], rick_ssids[line], strlen(rick_ssids[line]));
		memcpy(&beacon_rick[BEACON_SSID_OFFSET + strlen(rick_ssids[line])], &beacon_raw[BEACON_SSID_OFFSET], sizeof(beacon_raw) - BEACON_SSID_OFFSET);

		ESP_LOGI(TAG_BEACON, "beacon_rick size %d\n", sizeof(beacon_rick));
		ESP_LOGI(TAG_BEACON, "frame len %d\n", sizeof(beacon_raw));
		ESP_LOGI(TAG_BEACON, "ssid len bytes %d\n", sizeof(rick_ssids[line]));
		ESP_LOGI(TAG_BEACON, "ssid len %d\n", strlen(rick_ssids[line]));
		esp_wifi_80211_tx(ESP_IF_WIFI_AP, beacon_rick, sizeof(beacon_raw) + strlen(rick_ssids[line]), false);
		line++;
	}
}

static void configure_led(void){
	ESP_LOGI(TAG_LED, "Blink GPIO LED config");
	gpio_reset_pin(BLINK_GPIO);
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void blink_led(void){
	if(s_led_state == 0){
		s_led_state = 1;
	}
	else{
		s_led_state = 0;
	}
	gpio_set_level(BLINK_GPIO, s_led_state);
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

	// Init dummy AP to specify a channel and get WiFi hardware into
	// a mode where we can send the actual fake beacon frames.
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	wifi_config_t ap_config = {
		.ap = {
			.ssid = "esp32-beaconspam",
			.ssid_len = 0,
			.password = "dummypassword",
			.channel = 1,
			.authmode = WIFI_AUTH_WPA2_PSK,
			.ssid_hidden = 1,
			.max_connection = 4,
			.beacon_interval = 60000
		}
	};

	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
	configure_led();
	xTaskCreate(&wifi_send_beacon, "wifi_send_beacon", 4096, NULL, 5, NULL);
}