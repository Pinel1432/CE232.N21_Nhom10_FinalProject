/* WiFi station Example

 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_intr_alloc.h"

//thư viện cần để dùng mqtt
#include "mqtt_client.h"

//thư viện đọc rfid
#include "rc522.h"
#include "string.h"

#include "ssd1306.h"
#include "font8x8_basic.h"
//servo
#include <time.h>
#include <math.h>
#include <driver/gpio.h>
#include <driver/ledc.h>

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
uint32_t MQTT_CONNEECTED = 0;   //cờ MQTT, nếu thành công thì giá trị 1

/* The examples use WiFi configuration that you can set via project configuration menu

 If you'd rather not, just change the below entries to strings with
 the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
 */
#define EXAMPLE_ESP_WIFI_SSID      "Pea_0010"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10 // cố gắng connect 10 lần

//define tên của topic
#define MQTT_TOPIC "/test"
#define BARRIER_TOPIC "barrier"
#define ID_TOPIC "snapshot"
#define SPACE_TOPIC "space"
//URL của mqtt broker
#define CONFIG_BROKER_URL "mqtt://mqtt.flespi.io"

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

//static const char *SERVO_TAG = "Servo: ";
#define SERVO_PIN 17
#define ServoMsMin 0.5
#define ServoMsMax 2.5
#define ServoMs90 ((ServoMsMax+ServoMsMin)/2.0)

static const char *RFID_TAG = "RFID Read";
char ID[14];

#define BARRIER_PIN GPIO_NUM_18
#define IT_PIN_OUT 32
#define LED_PIN 2

// các pin đọc chỗ trống
#define PIN_1 2
#define PIN_2 4
#define PIN_3 5
#define PIN_4 18

char space[5] = "0000";
char barrier_control[8];
char topic_data[8];
char topic_name[8];
SSD1306_t dev;
bool state = 1; //led state
void oled_task();
void init_oled();

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

void RFID_IN_handler(uint8_t*);
void RFID_OUT_handler(uint8_t*);
void init_rfid();
void servoDeg90(gpio_num_t);
void servoDeg0(gpio_num_t);
void init_gpio();

void check_barrier();
//void check_barrier_out();


/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;   //biến static retry count

static void event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data) {
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
		ESP_LOGI(TAG, "Trying to connect with Wi-Fi\n");
	} else if (event_base == WIFI_EVENT
			&& event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		} else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG, "connect to the AP fail");
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t *event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

void wifi_init_sta(void) {
	s_wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;
	ESP_ERROR_CHECK(
			esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
	ESP_ERROR_CHECK(
			esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

	wifi_config_t wifi_config = { .sta = { .ssid = EXAMPLE_ESP_WIFI_SSID,
			.password = EXAMPLE_ESP_WIFI_PASS,
			/* Setting a password implies station will connect to all security modes including WEP/WPA.
			 * However these modes are deprecated and not advisable to be used. Incase your Access point
			 * doesn't support WPA2, these mode can be enabled by commenting below line */
			.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
			.sae_pwe_h2e = WPA3_SAE_PWE_BOTH, }, };
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(TAG, "wifi_init_sta finished.");

	/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
	 * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
	WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
	pdFALSE,
	pdFALSE,
	portMAX_DELAY);

	/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
	 * happened. */
	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
				EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
				EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
	} else {
		ESP_LOGE(TAG, "UNEXPECTED EVENT");
	}

	/* The event will not be processed after unregister */
	ESP_ERROR_CHECK(
			esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
					instance_got_ip));
	ESP_ERROR_CHECK(
			esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
	vEventGroupDelete(s_wifi_event_group);
}
//hàm xử lí của mqtt
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
		int32_t event_id, void *event_data) {
	ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base,
			event_id);
	esp_mqtt_event_handle_t event = event_data;
	esp_mqtt_client_handle_t client = event->client;
	int msg_id;
	switch ((esp_mqtt_event_id_t) event_id) {
	case MQTT_EVENT_CONNECTED:
		MQTT_CONNEECTED = 1;
		ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
		//esp in out thì sub
		msg_id = esp_mqtt_client_subscribe(client, BARRIER_TOPIC, 0);

		msg_id=esp_mqtt_client_subscribe(client, SPACE_TOPIC, 0);
		ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

		break;
	case MQTT_EVENT_DISCONNECTED:
		MQTT_CONNEECTED = 0;
		ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
		break;

	case MQTT_EVENT_SUBSCRIBED:
		ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
		printf("TOPIC = %.*s\r\n", event->topic_len, event->topic);
//        printf("CURRDATA = %.*s\r\n", event->data_len, event->data);
		break;
	case MQTT_EVENT_UNSUBSCRIBED:
		ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_PUBLISHED:
		ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_DATA:


		sprintf(topic_data, "%.*s", event->data_len, event->data);

		sprintf(topic_name, "%.*s", event->topic_len, event->topic);
		ESP_LOGI("Test: ","%s %s", topic_name,topic_data);


		//code for esp in
		if (!strcmp(topic_data, "IN")) {
				check_barrier();
		}
		//code for esp out
		if (!strcmp(topic_data, "OUT")) {
			check_barrier();
		}
		if (!strcmp(topic_name, "space")) {
			oled_task(topic_data);
		}

		break;
	case MQTT_EVENT_ERROR:
		ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
		break;
	default:
		ESP_LOGI(TAG, "Other event id:%d", event->event_id);
		break;
	}
}

esp_mqtt_client_handle_t client = NULL;

static void mqtt_app_start(void) {
	ESP_LOGI(TAG, "STARTING MQTT");
	esp_mqtt_client_config_t mqttConfig = { /*.uri = CONFIG_BROKER_URL*/
	.host = "192.168.137.32", .port = 1883, .username = "esp1", .password =
			"012345678", .client_id = "hello_1", };

	client = esp_mqtt_client_init(&mqttConfig);
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler,
			client);
	esp_mqtt_client_start(client);


}

void Publisher_Task() {


	if (MQTT_CONNEECTED) {

		//esp in out
		esp_mqtt_client_publish(client, ID_TOPIC, ID, 0, 0, 0);


	}
}
void app_main(void) {
//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
	servoDeg0(SERVO_PIN);
	wifi_init_sta();
	mqtt_app_start();
init_rfid();
//	init_oled();



//code cho esp32_2_space
//	init_gpio();
//	read_space();
}

void init_rfid() {
//while(1){

	const rc522_start_args_t RFID_IN = { .miso_io = 21, .mosi_io = 23, .sck_io =
			19, .sda_io = 22, //.callback = &RFID_IN_handler,
			.callback = &RFID_OUT_handler,

	// Uncomment next line for attaching RC522 to SPI2 bus. Default is VSPI_HOST (SPI3)
			.spi_host_id = HSPI_HOST };

	rc522_start(RFID_IN);
//vTaskDelay(2000/portTICK_PERIOD_MS);
//rc522_pause();
//			rc522_pause();
//			vTaskDelay(2000/portTICK_PERIOD_MS);
//
//	const rc522_start_args_t RFID_OUT = { .miso_io = 26, .mosi_io = 27,
//			.sck_io = 14, .sda_io = 12, .callback = &RFID_OUT_handler,
//
//			// Uncomment next line for attaching RC522 to SPI2 bus. Default is VSPI_HOST (SPI3)
//			.spi_host_id = HSPI_HOST };
//	rc522_start(RFID_OUT);
//			vTaskDelay(2000/portTICK_PERIOD_MS);
//			rc522_destroy();

//}
}

void RFID_IN_handler(uint8_t *sn) {

	sprintf(ID, "I%X%X%X%X%X", sn[0], sn[1], sn[2], sn[3], sn[4]);
	ESP_LOGI(RFID_TAG, "%s", ID);
	Publisher_Task();


}

void RFID_OUT_handler(uint8_t *sn) {

	sprintf(ID, "O%X%X%X%X%X", sn[0], sn[1], sn[2], sn[3], sn[4]);
	ESP_LOGI(RFID_TAG, "%s", ID);
	Publisher_Task();

}

void servoDeg0(gpio_num_t pinServo) {

	ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_LOW_SPEED_MODE,
			.timer_num = LEDC_TIMER_0, .duty_resolution = LEDC_TIMER_13_BIT,
			.freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK };
	ledc_timer_config(&ledc_timer);
	ledc_channel_config_t ledc_channel = { .speed_mode = LEDC_LOW_SPEED_MODE,
			.channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .intr_type =
					LEDC_INTR_DISABLE, .gpio_num = pinServo, .duty = 0,
			.hpoint = 0 };
	ledc_channel_config(&ledc_channel);
	int duty = (int) (100.0 * (ServoMsMin / 20.0) * 81.91);
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

}

void servoDeg90(gpio_num_t pinServo) {
	ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_LOW_SPEED_MODE,
			.timer_num = LEDC_TIMER_0, .duty_resolution = LEDC_TIMER_13_BIT,
			.freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK };
	ledc_timer_config(&ledc_timer);
	ledc_channel_config_t ledc_channel = { .speed_mode = LEDC_LOW_SPEED_MODE,
			.channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .intr_type =
					LEDC_INTR_DISABLE, .gpio_num = pinServo, .duty = 0,
			.hpoint = 0 };
	ledc_channel_config(&ledc_channel);
	int duty = (int) (100.0 * (ServoMs90 / 20.0) * 81.91);

	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

}

void init_gpio() {
	gpio_pad_select_gpio(BARRIER_PIN);
	gpio_set_direction(BARRIER_PIN, GPIO_MODE_INPUT);



}



void check_barrier() {
	servoDeg90(SERVO_PIN);
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	while (!gpio_get_level(BARRIER_PIN)) {
		vTaskDelay(2000 / portTICK_PERIOD_MS);
		if (gpio_get_level(BARRIER_PIN)) {
			servoDeg0(SERVO_PIN);
			break;
		}
	}
	if (gpio_get_level(BARRIER_PIN))
		servoDeg0(SERVO_PIN);
}

void oled_task(char *space_recei) {
	char A1[7], A2[7], B1[7], B2[7];

	if (space_recei[0]=='1') {
		strcpy(A1, "A1: x");
		ESP_LOGI("Test","%s",A1);
	} else {
		strcpy(A1, "A1: .");
		ESP_LOGI("Test","%s",A1);
	};

	if (space_recei[1]=='1') {
		strcpy(A2, "A2: x");
	} else {
		strcpy(A2, "A2: .");
	};

	if (space_recei[2]=='1') {
		strcpy(B1, "B1: x");
	} else {
		strcpy(B1, "B1: .");
	};

	if (space_recei[3]=='1') {
		strcpy(B2, "B2: x");
	} else {
		strcpy(B2, "B2: .");
	};
	char text3[25];
	char text4[25];
	sprintf(text3, "%s   %s      ", A1, B1);
	sprintf(text4, "%s   %s      ", A2, B2);
	// INITIATING OLED
	char text1[] = " CHO CON TRONG: ";
	char text2[] = "(x: cho da dung)";

	ssd1306_display_text(&dev, 0, text1, sizeof(text1), false);
	ssd1306_display_text(&dev, 1, text2, sizeof(text2), false);
	ssd1306_display_text(&dev, 3, text3, sizeof(text3), false);
	ssd1306_display_text(&dev, 4, text4, sizeof(text4), false);

}

void init_oled() {

	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
	ssd1306_init(&dev, 128, 64);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	oled_task("0000");
}

