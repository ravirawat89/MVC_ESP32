  /*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This file is for gatt server. It can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server_service_table demo.
* Client demo will enable gatt_server_service_table's notify after connection. Then two devices will exchange
* data.
*
****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <rom/ets_sys.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "nvs.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"

#include "esp_sleep.h"
//#include "esp_deep_sleep.h"
#include "esp_log.h"
#include "esp32/ulp.h"
#include "driver/touch_pad.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
//#include "soc/rtc.h"


#define GPIO_OUTPUT_IO_LED    2

#define GPIO_OUTPUT_IO_SOLP1	12
//#define GPIO_OUTPUT_IO_SOLN1	27

#define GPIO_OUTPUT_IO_SOLP2	13            //12
//#define GPIO_OUTPUT_IO_SOLN2	13
//#define GPIO_OUTPUT_PIN_SEL_SOL1  ((1ULL<<GPIO_OUTPUT_IO_SOLP1) | (1ULL<<GPIO_OUTPUT_IO_SOLN1))

#define GPIO_OUTPUT_IO_SOLP3	14            //25
//#define GPIO_OUTPUT_IO_SOLN3	26

#define GPIO_OUTPUT_IO_SOLP4	27              //32
//#define GPIO_OUTPUT_IO_SOLN4	33

#define GPIO_INPUT_IO_SOL1_FS1   25  //34  
#define GPIO_INPUT_IO_SOL2_FS2   26  //35
#define GPIO_INPUT_IO_SOL3_FS3   32  //36
#define GPIO_INPUT_IO_SOL4_FS4   33  //39

#define PCNT_TEST_UNIT      PCNT_UNIT_0

#define ESP_INTR_FLAG_DEFAULT 0
#define PCNT_H_LIM_VAL      30000



#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
//#define SAMPLE_DEVICE_NAME          "ESP_GATTS_DEMO"
#define SAMPLE_DEVICE_NAME          "Pebble"
#define SVC_INST_ID                 0

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 100
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

#define wakeup_time_sec             18


//static int cnt = 0;

static RTC_DATA_ATTR bool bleConnected = false;   //boolean variable for BLE device connection
static RTC_DATA_ATTR bool fillingWater_v1 = false, fillingWater_v2 = false, fillingWater_v3 = false, fillingWater_v4 = false;  //bool variaable for valve turn ON status
static RTC_DATA_ATTR bool countStart1 = false, countStart2 = false, countStart3 = false, countStart4 = false;                  //bool variables for four flow sensor count start 
RTC_DATA_ATTR static int16_t count = 0;																						   
RTC_DATA_ATTR static uint64_t flowCount1 = 0;	//variable to count flow sensor 1 pulses for valve 1
RTC_DATA_ATTR static uint64_t flowCount2 = 0;	//variable to count flow sensor 2 pulses for valve 2
RTC_DATA_ATTR static uint64_t flowCount3 = 0;	//variable to count flow sensor 3 pulses for valve 3
RTC_DATA_ATTR static uint64_t flowCount4 = 0;	//variable to count flow sensor 4 pulses for valve 4
RTC_DATA_ATTR static volatile int waterVolume1 = 0,	waterVolume2 = 0, waterVolume3 = 0, waterVolume4 = 0;	//variables for four different valves water quantity discharge
RTC_DATA_ATTR static volatile int countStop1 = 0, countStop2 = 0, countStop3 = 0, countStop4 = 0;			// variable to store flow counter value to as per quantity discharge
static int cal = 1;

//static RTC_DATA_ATTR struct timeval sleep_enter_time;

static uint8_t adv_config_done       = 0;

//uint16_t heart_rate_handle_table[HRS_IDX_NB];
uint16_t aquarius_handle_table[AQUARIUS_IDX_NB];

static struct timeval systemTimeVal;
static struct timezone systemTimeZone;

static time_t calendarNow;
static struct tm timeinfo;
//static RTC_DATA_ATTR uint8_t arrayLength;

int32_t OFFSET = 0;
float SCALE = 1;
RTC_DATA_ATTR static uint32_t weightData = 0;
RTC_DATA_ATTR static uint8_t  Tubby_calibrate = 0;	     //variable just for common android app
RTC_DATA_ATTR static uint8_t  deviceCode = 2;                //1-Pebble_SVC , 2-Pebble_MVC , 3-Tubby

static RTC_DATA_ATTR struct timeAlarm {
  bool active;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint16_t duration;
  uint16_t volume;
}startAlarm1[28], stopAlarm1[28], startAlarm2[28], stopAlarm2[28], startAlarm3[28], stopAlarm3[28], startAlarm4[28], stopAlarm4[28];


typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

//#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power*/
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF, 0x00,
        /* device name */
        0x0f, 0x09, 'A', 'q', 'u', 'a', 'r', 'i', 'u', 's'
};
static uint8_t raw_scan_rsp_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF,0x00
};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    //0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC
    0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = false,
    .min_interval        = 0x20,
    .max_interval        = 0x40,
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    //.flag              = 0,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = false,
    .min_interval        = 0x20,
    .max_interval        = 0x40,
    .appearance          = 0x00,
    .manufacturer_len    = 0,			// TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL,		// &test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 16,
    .p_service_uuid      = service_uuid,
    //.flag              = 0,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = 
{
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_AQUARIUS  = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_BATTERY      = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_CURRENT_TIME = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_POTS         = 0xFF03;
static const uint16_t GATTS_CHAR_UUID_NEW_TIME_POINT = 0xFF04;
static const uint16_t GATTS_CHAR_UUID_COMMAND      = 0xFF05;
static const uint16_t GATTS_CHAR_UUID_LOG_EVENT    = 0xFF06;


static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write          = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static const uint8_t char_value[10]                 = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA};


/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[AQUARIUS_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC_AQUARIUS]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_AQUARIUS), (uint8_t *)&GATTS_SERVICE_UUID_AQUARIUS}},

    /* Characteristic Declaration */
    [IDX_CHAR_BATTERY]     =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_BATTERY] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_BATTERY, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Client Characteristic Configuration Descriptor */
    /*[IDX_CHAR_CFG_A]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},*/

    /* Characteristic Declaration */
    [IDX_CHAR_CURRENT_TIME]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_CURRENT_TIME]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_CURRENT_TIME, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Characteristic Declaration */
    [IDX_CHAR_COMMAND]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_COMMAND]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_COMMAND, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /*Characteristic Declaration */
    [IDX_CHAR_POTS]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_POTS] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_POTS, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /*Characteristic Declaration */
    [IDX_CHAR_NEW_TIME_POINT]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_NEW_TIME_POINT] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_NEW_TIME_POINT, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /*Characteristic Declaration */
    [IDX_CHAR_LOG_EVENT]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_LOG_EVENT] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_LOG_EVENT, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connetion params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for(int i = 0; i < AQUARIUS_IDX_NB ; i++){
        if(handle == aquarius_handle_table[i])
		{
            return i;
        }
    }

    return error;
}

static void solenoid_valve1_open()                     //Turn ON valve 1
{
	ESP_LOGI(GATTS_TABLE_TAG, "Opening Solenoid...");
	rtc_gpio_hold_dis(GPIO_OUTPUT_IO_SOLP1);
	rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLP1, 1);
	rtc_gpio_hold_en(GPIO_OUTPUT_IO_SOLP1);
	printf("Valve1: Solenoid Open\n");
}

static void solenoid_valve1_close()					//Turn OFF valve 1
{
	ESP_LOGI(GATTS_TABLE_TAG, "Closing Solenoid...");
	rtc_gpio_hold_dis(GPIO_OUTPUT_IO_SOLP1);
	rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLP1, 0);
	rtc_gpio_hold_en(GPIO_OUTPUT_IO_SOLP1);
	printf("Valve1: Solenoid Close\n");
}

static void solenoid_valve2_open()					//Turn ON valve 2
{
	ESP_LOGI(GATTS_TABLE_TAG, "Opening Solenoid...");
	rtc_gpio_hold_dis(GPIO_OUTPUT_IO_SOLP2);
	rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLP2, 1);
	rtc_gpio_hold_en(GPIO_OUTPUT_IO_SOLP2);
	printf("Valve2: Solenoid Open\n");
}

static void solenoid_valve2_close()					//Turn OFF valve 2
{
	ESP_LOGI(GATTS_TABLE_TAG, "Closing Solenoid...");
	rtc_gpio_hold_dis(GPIO_OUTPUT_IO_SOLP2);
	rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLP2, 0);
	rtc_gpio_hold_en(GPIO_OUTPUT_IO_SOLP2);
	printf("Valve2: Solenoid Close\n");
}

static void solenoid_valve3_open()					//Turn ON valve 3
{
	ESP_LOGI(GATTS_TABLE_TAG, "Opening Solenoid...");
	rtc_gpio_hold_dis(GPIO_OUTPUT_IO_SOLP3);
	rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLP3, 1);
	rtc_gpio_hold_en(GPIO_OUTPUT_IO_SOLP3);
	printf("Valve3: Solenoid Open\n");
}

static void solenoid_valve3_close()					//Turn OFF valve 3
{
	ESP_LOGI(GATTS_TABLE_TAG, "Closing Solenoid...");
	rtc_gpio_hold_dis(GPIO_OUTPUT_IO_SOLP3);
	rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLP3, 0);
	rtc_gpio_hold_en(GPIO_OUTPUT_IO_SOLP3);
	printf("Valve3: Solenoid Close\n");
}

static void solenoid_valve4_open()					//Turn ON valve 4
{
	ESP_LOGI(GATTS_TABLE_TAG, "Opening Solenoid...");
	rtc_gpio_hold_dis(GPIO_OUTPUT_IO_SOLP4);
	rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLP4, 1);
	rtc_gpio_hold_en(GPIO_OUTPUT_IO_SOLP4);
	printf("Valve4: Solenoid Open\n");
}

static void solenoid_valve4_close()					//Turn OFF valve 4
{
	ESP_LOGI(GATTS_TABLE_TAG, "Closing Solenoid...");
	rtc_gpio_hold_dis(GPIO_OUTPUT_IO_SOLP4);
	rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLP4, 0);
	rtc_gpio_hold_en(GPIO_OUTPUT_IO_SOLP4);
	printf("Valve4: Solenoid Close\n");
}

 static void water_discharge()                   // Check if flow counter value reached to count value according to discharge quantity of water 
 {
	 if(fillingWater_v1	&& (!countStart1))
	{
		 if(waterVolume1 >= 2000)
		 {
			countStop1 = waterVolume1/1.3;
			printf("Solenoid close count= %d\n",countStop1);
			waterVolume1 = 0;
			countStart1 = true;
			
		 }
		 else if(waterVolume1 < 2000 && waterVolume1 != 0)
		 {
			countStop1 = waterVolume1/1.4;
			printf("Solenoid close count= %d\n",countStop1);
			waterVolume1 = 0;
			countStart1 = true;
		 }
	 }

	  if(fillingWater_v2 && (!countStart2))
	 {

		 if(waterVolume2 >= 2000)
		 {
			countStop2 = waterVolume2/1.3;
			printf("Solenoid close count= %d\n",countStop2);
			waterVolume2 = 0;
			countStart2 = true;
			
		 }
		 else if(waterVolume2 < 2000 && waterVolume2 != 0)
		 {
			countStop2 = waterVolume2/1.4;
			printf("Solenoid close count= %d\n",countStop2);
			waterVolume2 = 0;
			countStart2 = true;
		 }
	 }

	  if(fillingWater_v3 && (!countStart3))
	 {
		
		 if(waterVolume3 >= 2000)
		 {
			countStop3 = waterVolume3/1.3;
			printf("Solenoid close count= %d\n",countStop3);
			waterVolume3 = 0;
			countStart3 = true;
			
		 }
		 else if(waterVolume3< 2000 && waterVolume1 != 0)
		 {
			countStop3 = waterVolume3/1.4;
			printf("Solenoid close count= %d\n",countStop3);
			waterVolume3 = 0;
			countStart3 = true;
		 }

	 }

	
	  if(fillingWater_v4 && (!countStart4))
	 {

		 if(waterVolume4 >= 2000)
		 {
			countStop4 = waterVolume4/1.3;
			printf("Solenoid close count= %d\n",countStop4);
			waterVolume4 = 0;
			countStart4 = true;
			
		 }
		 else if(waterVolume4 < 2000 && waterVolume4 != 0)
		 {
			countStop4= waterVolume4/1.4;
			printf("Solenoid close count= %d\n",countStop4);
			waterVolume4 = 0;
			countStart4 = true;
		 }

	 }
 }

static void read_nvs_data()
{
	esp_err_t err;
	printf("\nOpening Non-Volatile Storage (NVS) handle... ");
	nvs_handle my_handle;
	err = nvs_open("storage", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) 
	{
		printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	} 
	else 
	{
		printf("Done\n");
		// Read
		printf("Reading flow count from NVS ... ");
		//int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
		err = nvs_get_u64(my_handle, "restart_counter", &flowCount1);
		switch (err) 
		{
			case ESP_OK:
				printf("Done\n");
				printf("Count value from NVS = %llu\n", flowCount1);
				break;
			case ESP_ERR_NVS_NOT_FOUND:
				printf("The value is not initialized yet!\n");
				break;
			default :
				printf("Error (%s) reading!\n", esp_err_to_name(err));
		}
	 }
}

static void write_nvs_data()
{
	esp_err_t err;
	// Open
	printf("\nOpening Non-Volatile Storage (NVS) handle... ");
	nvs_handle my_handle;
	err = nvs_open("storage", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) 
	{
		printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	} 
	else 
	{
		printf("Done\n");
		// Write
		err = nvs_set_u64(my_handle, "restart_counter", flowCount1);
		printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

		// Commit written value.
		// After setting any values, nvs_commit() must be called to ensure changes are written
		// to flash storage. Implementations may write to storage at other times,
		// but this is not guaranteed.
		printf("Committing updates in NVS ... ");
		err = nvs_commit(my_handle);
		printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

		// Close
		nvs_close(my_handle);
	}
 } 


 static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	uint8_t res = 0xff;   // Test to find characteristics
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #endif
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, AQUARIUS_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            //ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");

			ESP_LOGI(GATTS_TABLE_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
			esp_gatt_rsp_t rsp;
			memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
			rsp.attr_value.handle = param->read.handle;
			rsp.attr_value.len = 6;       //sizeof(char_value);
			rsp.attr_value.value[0] = (uint8_t) (weightData >> 24);                //weightData variable for Tubby, weightData =0 for PebbleMVC
			rsp.attr_value.value[1] = (uint8_t) (weightData >> 16);                
			rsp.attr_value.value[2] = (uint8_t) (weightData >> 8);
			rsp.attr_value.value[3] = (uint8_t) weightData;
			rsp.attr_value.value[4] = (uint8_t) Tubby_calibrate;
			rsp.attr_value.value[5] = (uint8_t) deviceCode;

			//res = find_char_and_desr_index(param->read.handle);
			res = (int) param->read.handle;
            if(res == 42)
			{
                //TODO:client read the status characteristic
				printf("Device for display : %d\n",deviceCode);
				esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            }
       	    break;
        case ESP_GATTS_WRITE_EVT:

			//res = find_char_and_desr_index(param->write.handle);
            if (!param->write.is_prep){
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);


                uint16_t dataHandle = (int)param->write.handle;
                uint8_t* dataValue = param->write.value;

                gpio_config_t io_conf;

                switch (dataHandle) {
                  case 44:
                    ESP_LOGI("gatt write event","Current Time written");
                    uint32_t systemTime = 0;
                    int index;
                    for(index=0;index<4;index++) {  
                      ESP_LOGI("dataValue","%d", *dataValue);
                      systemTime = systemTime << 8;
                      systemTime = systemTime + *dataValue;
                      ESP_LOGI("current system time","%d", systemTime);
                      dataValue++;
                    }

                    systemTimeVal.tv_sec = systemTime;
                    systemTimeVal.tv_usec = 0;
                    systemTimeZone.tz_minuteswest = -330;
                    systemTimeZone.tz_dsttime = 0;
                    settimeofday(&systemTimeVal, NULL);

                    time(&calendarNow);
                    localtime_r(&calendarNow, &timeinfo);
                    char strftime_buf[64];

                    // Set timezone to Indian Standard Time and print local time
                    //setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
                    setenv("TZ", "IST-5:30", 0);
                    tzset();
                    localtime_r(&calendarNow, &timeinfo);
                    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
                    ESP_LOGI("Current Time","The current date/time in India is: %s", strftime_buf);

                  break;

                  case 46:
                    ESP_LOGI("gatt write event","Valve Command written : %d", *dataValue);
					uint8_t k;
                    switch (*dataValue) 
					{
                      case 17:
							ESP_LOGI("Command","Valve 1: Flush Open");
							printf("Valve 1: Flush Open\n");
							if(!fillingWater_v1)
							{
								fillingWater_v1 = true;
								rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
								rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
								rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
								//Reset flow counter
								flowCount1 = 0;
								solenoid_valve1_open();                //Open solenoid
							}
							else
								printf("Valve 1 already open...\n");
						
						break;

                      case 18:
							ESP_LOGI("Command","Valve 1: Start");
							printf("PebbleMVC is resumed...\n");
							
						break;

					  case 19:
							ESP_LOGI("Command","Valve 1: Stop");   
							printf("Deactivate all alarms of valve 1\n");
							for(k = 0; k < 28; k++)
							{
                              startAlarm1[k].active = 0;
							  stopAlarm1[k].active = 0;
							}

						break;

					  case 20:
							ESP_LOGI("Command","Valve 1: Pause");
							printf("PebbleMVC is paused...\n");

						break;


                      case 21:
							ESP_LOGI("Command","Valve 1: Flush Close");
							printf("Valve 1: Flush Close\n");
							fillingWater_v1 = false;
							rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
							rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);
							rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
							solenoid_valve1_close();

						break;

					  case 33:
							ESP_LOGI("Command","Valve 2: Flush Open");
							printf("Valve 2: Flush Open\n");
							if(!fillingWater_v2)
							{
								fillingWater_v2 = true;
								//Reset flow counter
								flowCount2 = 0;
								rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
								rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
								rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
							}
							else
								printf("Valve 2 already open...\n");

							solenoid_valve2_open();

						break;

                      case 34:
							ESP_LOGI("Command","Valve 2: Start");
							printf("PebbleMVC is resumed...\n");
							
							
						break;

					  case 35:
							ESP_LOGI("Command","Valve 2: Stop");   
							printf("Deactivate all alarms of valve 2\n");
							for(k = 0; k < 28; k++)
							{
                              startAlarm2[k].active = 0;
							  stopAlarm2[k].active = 0;
							}						

						break;

					  case 36:
							ESP_LOGI("Command","Valve 2: Pause");
							printf("PebbleMVC is paused...\n");

						break;


                      case 37:
							ESP_LOGI("Command","Valve 2: Flush Close");
							printf("Valve 2: Flush Close\n");
							fillingWater_v2 = false;
							rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
							rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
							rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
							solenoid_valve2_close();

						break;

					  case 49:
							ESP_LOGI("Command","Valve 3: Flush Open");
							printf("Valve 3: Flush Open\n");
							if(!fillingWater_v3)
							{
								fillingWater_v3 = true;
								//Reset flow counter
								flowCount3 = 0;
								rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
								rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
								rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
								solenoid_valve3_open();
							}
							else
								printf("Valve 3 already open...\n");

						break;

                      case 50:
							ESP_LOGI("Command","Valve 3: Start");
							printf("PebbleMVC is resumed...\n");							
							
						break;

					  case 51:
							ESP_LOGI("Command","Valve 3: Stop");   
							printf("Deactivate all alarms of valve 3\n");
							for(k = 0; k < 28; k++)
							{
                              startAlarm3[k].active = 0;
							  stopAlarm3[k].active = 0;
							}
						break;

					  case 52:
							ESP_LOGI("Command","Valve 3: Pause");
							printf("PebbleMVC is paused...\n");

						break;


                      case 53:
							ESP_LOGI("Command","Valve 3: Flush Close");
							printf("Valve 3: Flush Close\n");
							fillingWater_v3 = false;
							rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
							rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
							rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
							solenoid_valve3_close();

						break;

					  case 65:
							ESP_LOGI("Command","Valve 4: Flush Open");
							printf("Valve 4: Flush Open\n");
							if(!fillingWater_v4)
							{
								fillingWater_v4 = true;
								//Reset flow counter
								flowCount4 = 0;
								rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
								rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
								rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
								solenoid_valve4_open();
							}
							else
								printf("Valve 4 already open...\n");

						break;

                      case 66:
							ESP_LOGI("Command","Valve 4: Start");
							printf("PebbleMVC is resumed...\n");
							
							
						break;

					  case 67:
							ESP_LOGI("Command","Valve 4: Stop");   
							printf("Deactivate all alarms of valve 4\n");
							for(k = 0; k < 28; k++)
							{
                              startAlarm4[k].active = 0;
							  stopAlarm4[k].active = 0;
							}
							
						break;

					  case 68:
							ESP_LOGI("Command","Valve 4: Pause");
							printf("PebbleMVC is paused...\n");

						break;

                      case 69:
							ESP_LOGI("Command","Valve 4: Flush Close");
							printf("Valve 4: Flush Close\n");
							fillingWater_v4 = false;
							rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
							rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
							rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
							solenoid_valve4_close();

						break;
                      
						  default:
							ESP_LOGI("Command","Unknown Command");
						break;
                    }
					break;

                  case 48:
                    ESP_LOGI("gatt write event","Number of Pots written: %d",dataValue[0]);
	
                  break;

                  case 50:
                    ESP_LOGI("gatt write event","New Time Point written: Active=%d Index=%d Day=%d Hour=%d Minute=%d DurationMSB=%d DurationLSB=%d VolumeMSB=%d VolumeLSB=%d Valve number=%d",dataValue[4],dataValue[0],dataValue[1],dataValue[2],dataValue[3],dataValue[5],dataValue[6],dataValue[7],dataValue[8],dataValue[9]);
                    printf("Valve %d time points received.\n",dataValue[9]);

					if(dataValue[0] == 0)
					{
						if(dataValue[9] == 1)
						{
							for(k = 0; k < 28; k++)            //Deactivate all alarms to set new alarms
							{
							  startAlarm1[k].active = 0;
							  stopAlarm1[k].active = 0;
							}
							printf("Valve %d : Old Time points erased..\n",dataValue[9]);
						}

						else if(dataValue[9] == 2)
						{
							for(k = 0; k < 28; k++)            //Deactivate all alarms to set new alarms
							{
							  startAlarm2[k].active = 0;
							  stopAlarm2[k].active = 0;
							}
							printf("Valve %d : Old Time points erased..\n",dataValue[9]);
						}

						else if(dataValue[9] == 3)
						{
							for(k = 0; k < 28; k++)            //Deactivate all alarms to set new alarms
							{
							  startAlarm3[k].active = 0;
							  stopAlarm3[k].active = 0;
							}
							printf("Valve %d : Old Time points erased..\n",dataValue[9]);
						}

						else if(dataValue[9] == 4)
						{
							for(k = 0; k < 28; k++)            //Deactivate all alarms to set new alarms
							{
							  startAlarm4[k].active = 0;
							  stopAlarm4[k].active = 0;
							}
							printf("Valve %d : Old Time points erased..\n",dataValue[9]);
						}
					}
					else
					{
						if(dataValue[9] == 1)
						{
							startAlarm1[dataValue[0]].active = dataValue[4];
							startAlarm1[dataValue[0]].day = dataValue[1]-1;
							if(startAlarm1[dataValue[0]].day == 0)
								startAlarm1[dataValue[0]].day = 7;
							startAlarm1[dataValue[0]].hour = dataValue[2];
							startAlarm1[dataValue[0]].minute = dataValue[3];
							startAlarm1[dataValue[0]].duration = (256 * (uint8_t)dataValue[5]) + (uint8_t)dataValue[6];
							startAlarm1[dataValue[0]].volume = (256 * (uint8_t)dataValue[7]) + (uint8_t)dataValue[8];
							ESP_LOGI("gatt event","Duration=%d Volume=%d\n",startAlarm1[dataValue[0]].duration,startAlarm1[dataValue[0]].volume);
						}
						else if(dataValue[9] == 2)
						{
							startAlarm2[dataValue[0]].active = dataValue[4];
							startAlarm2[dataValue[0]].day = dataValue[1]-1;
							if(startAlarm2[dataValue[0]].day == 0)
								startAlarm2[dataValue[0]].day = 7;
							startAlarm2[dataValue[0]].hour = dataValue[2];
							startAlarm2[dataValue[0]].minute = dataValue[3];
							startAlarm2[dataValue[0]].duration = (256 * (uint8_t)dataValue[5]) + (uint8_t)dataValue[6];
							startAlarm2[dataValue[0]].volume = (256 * (uint8_t)dataValue[7]) + (uint8_t)dataValue[8];
							ESP_LOGI("gatt event","Duration=%d Volume=%d\n",startAlarm2[dataValue[0]].duration,startAlarm2[dataValue[0]].volume);	
						}

						else if(dataValue[9] == 3)
						{
							startAlarm3[dataValue[0]].active = dataValue[4];
							startAlarm3[dataValue[0]].day = dataValue[1]-1;
							if(startAlarm3[dataValue[0]].day == 0)
								startAlarm3[dataValue[0]].day = 7;
							startAlarm3[dataValue[0]].hour = dataValue[2];
							startAlarm3[dataValue[0]].minute = dataValue[3];
							startAlarm3[dataValue[0]].duration = (256 * (uint8_t)dataValue[5]) + (uint8_t)dataValue[6];
							startAlarm3[dataValue[0]].volume = (256 * (uint8_t)dataValue[7]) + (uint8_t)dataValue[8];
							ESP_LOGI("gatt event","Duration=%d Volume=%d\n",startAlarm3[dataValue[0]].duration,startAlarm3[dataValue[0]].volume);
						}

						else if(dataValue[9] == 4)
						{
							startAlarm4[dataValue[0]].active = dataValue[4];
							startAlarm4[dataValue[0]].day = dataValue[1]-1;
							if(startAlarm4[dataValue[0]].day == 0)
								startAlarm4[dataValue[0]].day = 7;
							startAlarm4[dataValue[0]].hour = dataValue[2];
							startAlarm4[dataValue[0]].minute = dataValue[3];
							startAlarm4[dataValue[0]].duration = (256 * (uint8_t)dataValue[5]) + (uint8_t)dataValue[6];
							startAlarm4[dataValue[0]].volume = (256 * (uint8_t)dataValue[7]) + (uint8_t)dataValue[8];
							ESP_LOGI("gatt event","Duration=%d Volume=%d\n",startAlarm4[dataValue[0]].duration,startAlarm4[dataValue[0]].volume);
						}
					}
                  break;

                  default:
                    ESP_LOGI("gatt write event","Unknown Handle");
                  break;
                }
                
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else{
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d", param->conf.status);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            bleConnected = true;
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            bleConnected = false;
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = %d", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != AQUARIUS_IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to AQUARIUS_IDX_NB(%d)", param->add_attr_tab.num_handle, AQUARIUS_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(aquarius_handle_table, param->add_attr_tab.handles, sizeof(aquarius_handle_table));
                esp_ble_gatts_start_service(aquarius_handle_table[IDX_SVC_AQUARIUS]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


static void checkAlarms() 
{
  time(&calendarNow);
  localtime_r(&calendarNow, &timeinfo);
  char strftime_buf[64];
  // Set timezone to Indian Standard Time and print local time
  //setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
  setenv("TZ", "IST-5:30", 0);
  tzset();
  localtime_r(&calendarNow, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  ESP_LOGI("Checking Alarms","The current date/time in India is: %s. (%d) %2d:%2d", strftime_buf, timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min);

  uint8_t i = 0;
  for(i=0; i<29; i++)
 {

	if(startAlarm1[i].active && (timeinfo.tm_wday == startAlarm1[i].day) && (timeinfo.tm_hour == startAlarm1[i].hour) && (timeinfo.tm_min == startAlarm1[i].minute)) 
	{
		ESP_LOGI("Valve 1: Alarm Match","Alarm 0 : Hour = %d Minute = %d", timeinfo.tm_hour, timeinfo.tm_min);
		fillingWater_v1 = true;

		//************Start flow counter****************************
		//Reset flow counter
		flowCount1 = 0;	

		rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
		rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);
		rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
		
		solenoid_valve1_open();					//Open solenoid on start alarm match
		startAlarm1[i].active = 0;
		if(startAlarm1[i].volume == 0)
		{
			waterVolume1 = 0;
			stopAlarm1[i].active = 1;   //Active Stop alarm for related Start alarm
			stopAlarm1[i].day = startAlarm1[i].day;
			stopAlarm1[i].hour = startAlarm1[i].hour;
			stopAlarm1[i].minute = startAlarm1[i].minute + startAlarm1[i].duration;
			if(stopAlarm1[i].minute >=60) 
			{
			  stopAlarm1[i].hour = startAlarm1[i].hour + (stopAlarm1[i].minute / 60);
			  stopAlarm1[i].minute = stopAlarm1[i].minute % 60;
			  if(stopAlarm1[i].hour > 23) 
			  {
				  stopAlarm1[i].hour = 0;
			  }
			}
		}
		else
		 {	
			waterVolume1 = startAlarm1[i].volume;
			printf("Water volume for solenoid close: %d ml\n",waterVolume1);
		 }
    }

	if(startAlarm2[i].active && (timeinfo.tm_wday == startAlarm2[i].day) && (timeinfo.tm_hour == startAlarm2[i].hour) && (timeinfo.tm_min == startAlarm2[i].minute))
	{
		ESP_LOGI("Valve 2: Alarm Match","Alarm 0 : Hour = %d Minute = %d", timeinfo.tm_hour, timeinfo.tm_min);
		fillingWater_v2 = true;

		//************Start flow counter****************************
		//Reset flow counter
		flowCount2 = 0;	

		rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
		rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);
		rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
		
		solenoid_valve2_open();					//Open solenoid Valve 2

		startAlarm2[i].active = 0;
		if(startAlarm2[i].volume == 0)
		{
			waterVolume2 = 0;
			stopAlarm2[i].active = 1;   //Active Stop alarm for related Start alarm
			stopAlarm2[i].day = startAlarm2[i].day;
			stopAlarm2[i].hour = startAlarm2[i].hour;
			stopAlarm2[i].minute = startAlarm2[i].minute + startAlarm2[i].duration;
			if(stopAlarm2[i].minute >=60) 
			{
			  stopAlarm2[i].hour = startAlarm2[i].hour + (stopAlarm2[i].minute / 60);
			  stopAlarm2[i].minute = stopAlarm2[i].minute % 60;
			  if(stopAlarm2[i].hour > 23) 
			  {
				  stopAlarm2[i].hour = 0;
			  }
			}
		}
		else
		 {	
			waterVolume2 = startAlarm2[i].volume;
			printf("Water volume for solenoid close: %d ml\n",waterVolume2);
		 }
	}

	if(startAlarm3[i].active && (timeinfo.tm_wday == startAlarm3[i].day) && (timeinfo.tm_hour == startAlarm3[i].hour) && (timeinfo.tm_min == startAlarm3[i].minute))
	{
		ESP_LOGI("Valve 3: Alarm Match","Alarm 0 : Hour = %d Minute = %d", timeinfo.tm_hour, timeinfo.tm_min);
		fillingWater_v3 = true;

		//************Start flow counter****************************
		//Reset flow counter
		flowCount3 = 0;		

		rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
		rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);
		rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
		
		solenoid_valve3_open();					//Open solenoid Valve 2

		startAlarm3[i].active = 0;
		if(startAlarm3[i].volume == 0)
		{
			waterVolume3 = 0;
			stopAlarm3[i].active = 1;   //Active Stop alarm for related Start alarm
			stopAlarm3[i].day = startAlarm3[i].day;
			stopAlarm3[i].hour = startAlarm3[i].hour;
			stopAlarm3[i].minute = startAlarm3[i].minute + startAlarm3[i].duration;
			if(stopAlarm3[i].minute >=60) 
			{
			  stopAlarm3[i].hour = startAlarm3[i].hour + (stopAlarm3[i].minute / 60);
			  stopAlarm3[i].minute = stopAlarm3[i].minute % 60;
			  if(stopAlarm3[i].hour > 23) 
			  {
				  stopAlarm3[i].hour = 0;
			  }
			}
		}
		else
		 {	
			waterVolume3 = startAlarm3[i].volume;
			printf("Water volume for solenoid close: %d ml\n",waterVolume3);
		 }
	}

	if(startAlarm4[i].active && (timeinfo.tm_wday == startAlarm4[i].day) && (timeinfo.tm_hour == startAlarm4[i].hour) && (timeinfo.tm_min == startAlarm4[i].minute))
	{
		ESP_LOGI("Valve 4: Alarm Match","Alarm 0 : Hour = %d Minute = %d", timeinfo.tm_hour, timeinfo.tm_min);
		fillingWater_v4 = true;

		//************Start flow counter****************************
		//Reset flow counter
		flowCount4 = 0;		

		rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
		rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);
		rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
		
		solenoid_valve4_open();					//Open solenoid Valve 2

		startAlarm4[i].active = 0;
		if(startAlarm4[i].volume == 0)
		{
			waterVolume4 = 0;
			stopAlarm4[i].active = 1;   //Active Stop alarm for related Start alarm
			stopAlarm4[i].day = startAlarm4[i].day;
			stopAlarm4[i].hour = startAlarm4[i].hour;
			stopAlarm4[i].minute = startAlarm4[i].minute + startAlarm4[i].duration;
			if(stopAlarm4[i].minute >=60) 
			{
			  stopAlarm4[i].hour = startAlarm4[i].hour + (stopAlarm4[i].minute / 60);
			  stopAlarm4[i].minute = stopAlarm4[i].minute % 60;
			  if(stopAlarm4[i].hour > 23) 
			  {
				  stopAlarm4[i].hour = 0;
			  }
			}
		}
		else
		 {	
			waterVolume4 = startAlarm4[i].volume;
			printf("Water volume for solenoid close: %d ml\n",waterVolume4);
		 }
	}
  }
}


static void checkStopAlarms() {
  time(&calendarNow);
  localtime_r(&calendarNow, &timeinfo);
  char strftime_buf[64];

  // Set timezone to Indian Standard Time and print local time
  //setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
  setenv("TZ", "IST-5:30", 0);
  tzset();
  localtime_r(&calendarNow, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  ESP_LOGI("Checking Alarms","The current date/time in India is: %s. (%d) %2d:%2d", strftime_buf, timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min);

  uint8_t j=0;
  for(j = 0; j < 29; j++)
  {
	  if(stopAlarm1[j].active && (timeinfo.tm_wday == stopAlarm1[j].day) && (timeinfo.tm_hour == stopAlarm1[j].hour) && (timeinfo.tm_min == stopAlarm1[j].minute)) 
	 {
		ESP_LOGI("Valve 1: Alarm Match","Stopping Alarm : Hour = %d Minute = %d", timeinfo.tm_hour, timeinfo.tm_min);
		fillingWater_v1 = false;
	
		rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
		rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
		rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
      
		solenoid_valve1_close();			//close solenoid on stop alarm
		stopAlarm1[j].active = 0;
	  }

	  if(stopAlarm2[j].active && (timeinfo.tm_wday == stopAlarm2[j].day) && (timeinfo.tm_hour == stopAlarm2[j].hour) && (timeinfo.tm_min == stopAlarm2[j].minute)) 
	 {
		ESP_LOGI("Valve 2: Alarm Match","Stopping Alarm : Hour = %d Minute = %d", timeinfo.tm_hour, timeinfo.tm_min);
		fillingWater_v2 = false;
	
		rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
		rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
		rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
     
		solenoid_valve2_close();			//close solenoid on stop alarm
		stopAlarm2[j].active = 0;
	  }

	   if(stopAlarm3[j].active && (timeinfo.tm_wday == stopAlarm3[j].day) && (timeinfo.tm_hour == stopAlarm3[j].hour) && (timeinfo.tm_min == stopAlarm3[j].minute)) 
	 {
		ESP_LOGI("Valve 3: Alarm Match","Stopping Alarm : Hour = %d Minute = %d", timeinfo.tm_hour, timeinfo.tm_min);
		fillingWater_v3 = false;
	
		rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
		rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
		rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);

		solenoid_valve3_close();			//close solenoid on stop alarm
		stopAlarm3[j].active = 0;

	  }

	   if(stopAlarm4[j].active && (timeinfo.tm_wday == stopAlarm4[j].day) && (timeinfo.tm_hour == stopAlarm4[j].hour) && (timeinfo.tm_min == stopAlarm4[j].minute)) 
	 {
		ESP_LOGI("Valve 4: Alarm Match","Stopping Alarm : Hour = %d Minute = %d", timeinfo.tm_hour, timeinfo.tm_min);
		fillingWater_v4 = false;
	
		rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
		rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);  //rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
		rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
        
		solenoid_valve4_close();			//close solenoid on stop alarm
		stopAlarm4[j].active = 0;

	  }
  }
}

void IRAM_ATTR FlowSensor1_isr_handler(void* arg)				 //ISR for flow sensor interrupt
{
    uint32_t gpio_num = (uint32_t) arg;
	flowCount1++;
}

void IRAM_ATTR FlowSensor2_isr_handler(void* arg)				 //ISR for flow sensor interrupt
{
    uint32_t gpio_num = (uint32_t) arg;
	flowCount2++;
}

void IRAM_ATTR FlowSensor3_isr_handler(void* arg)				 //ISR for flow sensor interrupt
{
    uint32_t gpio_num = (uint32_t) arg;
	flowCount3++;
}

void IRAM_ATTR FlowSensor4_isr_handler(void* arg)				 //ISR for flow sensor interrupt
{
    uint32_t gpio_num = (uint32_t) arg;
	flowCount4++;
}


void app_main()
{
  esp_err_t ret;

  /* Initialize NVS. */
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) 
  {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );

  //*******************************************4- Flow Sensors*****************************************************************
	
	ret = gpio_set_direction(GPIO_INPUT_IO_SOL1_FS1, GPIO_MODE_INPUT);       // set GPIO pin as flow sensor input interrupt pin to count pulses
	if(ret) {
	  ESP_LOGI("GPIO","Could not set direction of GPIO 25");
	  return;
	}
	 ret = gpio_pulldown_en(GPIO_INPUT_IO_SOL1_FS1);
	if(ret) {
	  ESP_LOGI("GPIO","Could not enable pull up of GPIO 25");
	  return;
	}

	
	ret = gpio_set_direction(GPIO_INPUT_IO_SOL2_FS2, GPIO_MODE_INPUT);
	if(ret) {
	  ESP_LOGI("GPIO","Could not set direction of GPIO 26");
	  return;
	}
	 ret = gpio_pulldown_en(GPIO_INPUT_IO_SOL2_FS2);
	if(ret) {
	  ESP_LOGI("GPIO","Could not enable pull up of GPIO 26");
	  return;
	}

	
	ret = gpio_set_direction(GPIO_INPUT_IO_SOL3_FS3, GPIO_MODE_INPUT);
	if(ret) {
	  ESP_LOGI("GPIO","Could not set direction of GPIO 32");
	  return;
	}
	 ret = gpio_pulldown_en(GPIO_INPUT_IO_SOL3_FS3);
	if(ret) {
	  ESP_LOGI("GPIO","Could not enable pull up of GPIO 32");
	  return;
	}

	
	ret = gpio_set_direction(GPIO_INPUT_IO_SOL4_FS4, GPIO_MODE_INPUT);
	if(ret) {
	  ESP_LOGI("GPIO","Could not set direction of GPIO 33");
	  return;
	}
	 ret = gpio_pulldown_en(GPIO_INPUT_IO_SOL4_FS4);
	if(ret) {
	  ESP_LOGI("GPIO","Could not enable pull up of GPIO 33");
	  return;
	}
	
	gpio_intr_enable(GPIO_INPUT_IO_SOL1_FS1);
	gpio_intr_enable(GPIO_INPUT_IO_SOL2_FS2);
	gpio_intr_enable(GPIO_INPUT_IO_SOL3_FS3);
	gpio_intr_enable(GPIO_INPUT_IO_SOL4_FS4);

	gpio_set_intr_type(GPIO_INPUT_IO_SOL1_FS1, GPIO_INTR_POSEDGE);
	gpio_set_intr_type(GPIO_INPUT_IO_SOL2_FS2, GPIO_INTR_POSEDGE);
	gpio_set_intr_type(GPIO_INPUT_IO_SOL3_FS3, GPIO_INTR_POSEDGE);
	gpio_set_intr_type(GPIO_INPUT_IO_SOL4_FS4, GPIO_INTR_POSEDGE);

	  //install gpio isr service
	gpio_install_isr_service(0);

	 //hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_INPUT_IO_SOL1_FS1, FlowSensor1_isr_handler, (void*) GPIO_INPUT_IO_SOL1_FS1);

	gpio_isr_handler_add(GPIO_INPUT_IO_SOL2_FS2, FlowSensor2_isr_handler, (void*) GPIO_INPUT_IO_SOL2_FS2);

	gpio_isr_handler_add(GPIO_INPUT_IO_SOL3_FS3, FlowSensor3_isr_handler, (void*) GPIO_INPUT_IO_SOL3_FS3);

	gpio_isr_handler_add(GPIO_INPUT_IO_SOL4_FS4, FlowSensor4_isr_handler, (void*) GPIO_INPUT_IO_SOL4_FS4);

//****************************************************************************************************************************

    switch (esp_sleep_get_wakeup_cause()) 
	{
      case ESP_SLEEP_WAKEUP_EXT1: {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask != 0) {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            printf("Wake up from User Button on GPIO %d\n", pin);
        } else {
            printf("Wake up from GPIO\n");
        }

        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed", __func__);
            return;
        }

        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed", __func__);
            return;
        }

        ret = esp_bluedroid_init();
        if (ret) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed", __func__);
            return;
        }

        ret = esp_bluedroid_enable();
        if (ret) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed", __func__);
            return;
        }

        ret = esp_ble_gatts_register_callback(gatts_event_handler);
        if (ret){
            ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
            return;
        }

        ret = esp_ble_gap_register_callback(gap_event_handler);
        if (ret){
            ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
            return;
        }

        ret = esp_ble_gatts_app_register(ESP_APP_ID);
        if (ret){
            ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
            return;
        }

        esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
        if (local_mtu_ret){
            ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
        }

        const int ext_wakeup_pin_1 = 15;
        const uint64_t ext_wakeup_pin_1_mask = 1 << ext_wakeup_pin_1;
        const int ext_wakeup_pin_2 = 4;
        const uint64_t ext_wakeup_pin_2_mask = 1 << ext_wakeup_pin_2;

        ESP_LOGI("External Interrupts","Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
        esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ALL_LOW);

        while (1) 
		{

			checkAlarms();
			checkStopAlarms();
			water_discharge();
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			
          while(bleConnected || fillingWater_v1) 
		  {

            time(&calendarNow);
            localtime_r(&calendarNow, &timeinfo);
            char strftime_buf[64];

            // Set timezone to Indian Standard Time and print local time
            //setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
            setenv("TZ", "IST-5:30", 0);
            tzset();
            localtime_r(&calendarNow, &timeinfo);
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            //ESP_LOGI("Current Time","The current date/time in India is: %s. (%d) %2d:%2d", strftime_buf, timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            checkAlarms();
			checkStopAlarms();
			water_discharge();

            while(fillingWater_v1 || fillingWater_v2 || fillingWater_v3 || fillingWater_v4) 
			{
				if((flowCount1 >= countStop1) && countStart1)             // check if flow counter reaches to countStop to stop valve
				{
					//pcnt_counter_pause(PCNT_TEST_UNIT);
					printf("Current counter value : %llu\n",flowCount1);
					rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
					rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
					rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
					solenoid_valve1_close();
					fillingWater_v1 = false;
					countStart1 = false;
					countStop1 = 0;
					//Reset flow counter
					flowCount1 = 0;
				 }
				 if((flowCount2 >= countStop2) && countStart2)
				{
					//pcnt_counter_pause(PCNT_TEST_UNIT);
					printf("Current counter value : %llu\n",flowCount2);
					rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
					rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
					rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
					solenoid_valve2_close();
					fillingWater_v2 = false;
					countStart2 = false;
					countStop2 = 0;
					//Reset flow counter
					flowCount2 = 0;
				 }
				 
				 if((flowCount3 >= countStop3) && countStart3)
				{
					//pcnt_counter_pause(PCNT_TEST_UNIT);
					printf("Current counter value : %llu\n",flowCount3);
					rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
					rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
					rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
					solenoid_valve3_close();
					fillingWater_v3 = false;
					countStart3 = false;
					countStop3 = 0;
					//Reset flow counter
					flowCount3 = 0;
				 }

				 if((flowCount4 >= countStop4) && countStart4)
				{
					//pcnt_counter_pause(PCNT_TEST_UNIT);
					printf("Current counter value : %llu\n",flowCount4);
					rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
					rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
					rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
					solenoid_valve4_close();
					fillingWater_v4 = false;
					countStart4 = false;
					countStop4 = 0;
					//Reset flow counter
					flowCount4 = 0;
				 }
				vTaskDelay(1000 / portTICK_RATE_MS);
				//printf("Vale 1: count = %llu\n",flowCount1);
				//printf("Vale 2: count = %llu\n",flowCount2);
				//printf("Vale 3: count = %llu\n",flowCount3);
				//printf("Vale 4: count = %llu\n",flowCount4);
				checkAlarms();
				checkStopAlarms();
				water_discharge();
			}
          }
        }

        break;
       }

      case ESP_SLEEP_WAKEUP_TIMER:  {
			ESP_LOGI("app_main","timer based wake up");

			vTaskDelay(2000 / portTICK_RATE_MS);
			checkAlarms();
			checkStopAlarms();
			water_discharge();
			while(fillingWater_v1 || fillingWater_v2 || fillingWater_v3 || fillingWater_v4) 
			{
				if((flowCount1 >= countStop1) && countStart1)
				{
					//pcnt_counter_pause(PCNT_TEST_UNIT);
					printf("Current counter value : %llu\n",flowCount1);
					rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
					rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
					rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
					solenoid_valve1_close();
					fillingWater_v1 = false;
					countStart1 = false;
					countStop1 = 0;
					//Reset flow counter
					flowCount1 = 0;
				 }
				 if((flowCount2 >= countStop2) && countStart2)
				{
					//pcnt_counter_pause(PCNT_TEST_UNIT);
					printf("Current counter value : %llu\n",flowCount2);
					rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
					rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
					rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
					solenoid_valve2_close();
					fillingWater_v2 = false;
					countStart2 = false;
					countStop2 = 0;
					//Reset flow counter
					flowCount2 = 0;
				 }
				 
				 if((flowCount3 >= countStop3) && countStart3)
				{
					//pcnt_counter_pause(PCNT_TEST_UNIT);
					printf("Current counter value : %llu\n",flowCount3);
					rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
					rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
					rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
					solenoid_valve3_close();
					fillingWater_v3 = false;
					countStart3 = false;
					countStop3 = 0;
					//Reset flow counter
					flowCount3 = 0;
				 }

				 if((flowCount4 >= countStop4) && countStart4)
				{
					//pcnt_counter_pause(PCNT_TEST_UNIT);
					printf("Current counter value : %llu\n",flowCount4);
					rtc_gpio_hold_dis(GPIO_OUTPUT_IO_LED);
					rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 0);
					rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
					solenoid_valve4_close();
					fillingWater_v4 = false;
					countStart4 = false;
					countStop4 = 0;
					//Reset flow counter
					flowCount4 = 0;
				 }
				 checkAlarms();
				 checkStopAlarms();
				 water_discharge();
			}
		
        const int ext_wakeup_pin_1 = 15;
        const uint64_t ext_wakeup_pin_1_mask = 1 << ext_wakeup_pin_1;
        const int ext_wakeup_pin_2 = 4;
        const uint64_t ext_wakeup_pin_2_mask = 1 << ext_wakeup_pin_2;
        ESP_LOGI("External Interrupts","Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_1);
        esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ALL_LOW);
        printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
        esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

        //ESP_LOGI("External Interrupts","Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_2);
        //esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);

        ESP_LOGI("Timer wakeup","Entering deep sleep\n");
        esp_deep_sleep_start();
        //printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
        break;
      }

      case ESP_SLEEP_WAKEUP_UNDEFINED:
      default:
        printf("Not a deep sleep reset\n");

        if(rtc_gpio_is_valid_gpio(2)) {
          ESP_LOGI("Power Up","Pin 2 is valid RTC GPIO");
        } else {
          ESP_LOGI("Power Up","Pin 2 is not valid RTC GPIO");
        }
		if(rtc_gpio_is_valid_gpio(12)) {
          ESP_LOGI("Power Up","Pin 12 is valid RTC GPIO");
        } else {
          ESP_LOGI("Power Up","Pin 12 is not valid RTC GPIO");
        }
        if(rtc_gpio_is_valid_gpio(13)) {
          ESP_LOGI("Power Up","Pin 13 is valid RTC GPIO");
        } else {
          ESP_LOGI("Power Up","Pin 13 is not valid RTC GPIO");
        }
//*****************************Solenoid valve************************************
		if(rtc_gpio_is_valid_gpio(14)) {
          ESP_LOGI("Power Up","Pin 14 is valid RTC GPIO");
        } else {
          ESP_LOGI("Power Up","Pin 14 is not valid RTC GPIO");
        }
        if(rtc_gpio_is_valid_gpio(27)) {
          ESP_LOGI("Power Up","Pin 27 is valid RTC GPIO");
        } else {
          ESP_LOGI("Power Up","Pin 27 is not valid RTC GPIO");
        }
//*********************************On-Board LED**********************************************
        ret = rtc_gpio_init(GPIO_OUTPUT_IO_LED);
        if(ret) {
          ESP_LOGI("GPIO","Could not initialize GPIO 2");
          return;
        }
        ret = rtc_gpio_set_direction(GPIO_OUTPUT_IO_LED, RTC_GPIO_MODE_OUTPUT_ONLY);
        if(ret) {
          ESP_LOGI("GPIO","Could not set direction of GPIO 2");
          return;
        }

        ret = rtc_gpio_pullup_en(GPIO_OUTPUT_IO_LED);
        if(ret) {
          ESP_LOGI("GPIO","Could not enable pull up of GPIO 2");
          return;
        }
     
        ret = rtc_gpio_set_level(GPIO_OUTPUT_IO_LED, 1);
        if(ret) {
          ESP_LOGI("GPIO","Could not set level of GPIO 2");
          return;
        }
        ret = rtc_gpio_hold_en(GPIO_OUTPUT_IO_LED);
        if(ret) {
          ESP_LOGI("GPIO","Could not enable hold of GPIO 2");
          return;
        }
        

//************************************4-Solenoid valves*************************************

		ret = rtc_gpio_init(GPIO_OUTPUT_IO_SOLP1);
        if(ret) {
          ESP_LOGI("GPIO","Could not initialize GPIO 12");
          return;
        }
		ret = rtc_gpio_set_direction(GPIO_OUTPUT_IO_SOLP1, RTC_GPIO_MODE_OUTPUT_ONLY);
        if(ret) {
          ESP_LOGI("GPIO","Could not set direction of GPIO 12");
          return;
        }
		ret = rtc_gpio_pullup_en(GPIO_OUTPUT_IO_SOLP1);
        if(ret) {
          ESP_LOGI("GPIO","Could not enable pull up of GPIO 12");
          return;
        }
		 ret = rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLP1, 0);
        if(ret) {
          ESP_LOGI("GPIO","Could not set level of GPIO 12");
          return;
        }

		/*ret = rtc_gpio_init(GPIO_OUTPUT_IO_SOLN1);
        if(ret) {
          ESP_LOGI("GPIO","Could not initialize GPIO 27");
          return;
        }
		ret = rtc_gpio_set_direction(GPIO_OUTPUT_IO_SOLN1, RTC_GPIO_MODE_OUTPUT_ONLY);
        if(ret) {
          ESP_LOGI("GPIO","Could not set direction of GPIO 27");
          return;
        }
		 ret = rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLN1, 1);
        if(ret) {
          ESP_LOGI("GPIO","Could not set level of GPIO 27");
          return;
        }*/

		ret = rtc_gpio_init(GPIO_OUTPUT_IO_SOLP2);
        if(ret) {
          ESP_LOGI("GPIO","Could not initialize GPIO 13");
          return;
        }
		ret = rtc_gpio_set_direction(GPIO_OUTPUT_IO_SOLP2, RTC_GPIO_MODE_OUTPUT_ONLY);
        if(ret) {
          ESP_LOGI("GPIO","Could not set direction of GPIO 13");
          return;
        }
		ret = rtc_gpio_pullup_en(GPIO_OUTPUT_IO_SOLP2);
        if(ret) {
          ESP_LOGI("GPIO","Could not enable pull up of GPIO 13");
          return;
        }
		 ret = rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLP2, 0);
        if(ret) {
          ESP_LOGI("GPIO","Could not set level of GPIO 13");
          return;
        }

		/*ret = rtc_gpio_init(GPIO_OUTPUT_IO_SOLN2);
        if(ret) {
          ESP_LOGI("GPIO","Could not initialize GPIO 13");
          return;
        }
		ret = rtc_gpio_set_direction(GPIO_OUTPUT_IO_SOLN2, RTC_GPIO_MODE_OUTPUT_ONLY);
        if(ret) {
          ESP_LOGI("GPIO","Could not set direction of GPIO 13");
          return;
        }
		 ret = rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLN2, 1);
        if(ret) {
          ESP_LOGI("GPIO","Could not set level of GPIO 13");
          return;
        }*/

		ret = rtc_gpio_init(GPIO_OUTPUT_IO_SOLP3);
        if(ret) {
          ESP_LOGI("GPIO","Could not initialize GPIO 13");
          return;
        }
		ret = rtc_gpio_set_direction(GPIO_OUTPUT_IO_SOLP3, RTC_GPIO_MODE_OUTPUT_ONLY);
        if(ret) {
          ESP_LOGI("GPIO","Could not set direction of GPIO 13");
          return;
        }
		ret = rtc_gpio_pullup_en(GPIO_OUTPUT_IO_SOLP3);
        if(ret) {
          ESP_LOGI("GPIO","Could not enable pull up of GPIO 13");
          return;
        }
		 ret = rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLP3, 0);
        if(ret) {
          ESP_LOGI("GPIO","Could not set level of GPIO 13");
          return;
        }

		/*ret = rtc_gpio_init(GPIO_OUTPUT_IO_SOLN3);
        if(ret) {
          ESP_LOGI("GPIO","Could not initialize GPIO 26");
          return;
        }
		ret = rtc_gpio_set_direction(GPIO_OUTPUT_IO_SOLN3, RTC_GPIO_MODE_OUTPUT_ONLY);
        if(ret) {
          ESP_LOGI("GPIO","Could not set direction of GPIO 26");
          return;
        }
		 ret = rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLN3, 1);
        if(ret) {
          ESP_LOGI("GPIO","Could not set level of GPIO 26");
          return;
        }*/

		ret = rtc_gpio_init(GPIO_OUTPUT_IO_SOLP4);
        if(ret) {
          ESP_LOGI("GPIO","Could not initialize GPIO 27");
          return;
        }
		ret = rtc_gpio_set_direction(GPIO_OUTPUT_IO_SOLP4, RTC_GPIO_MODE_OUTPUT_ONLY);
        if(ret) {
          ESP_LOGI("GPIO","Could not set direction of GPIO 27");
          return;
        }
		ret = rtc_gpio_pullup_en(GPIO_OUTPUT_IO_SOLP4);
        if(ret) {
          ESP_LOGI("GPIO","Could not enable pull up of GPIO 27");
          return;
        }
		 ret = rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLP4, 0);
        if(ret) {
          ESP_LOGI("GPIO","Could not set level of GPIO 27");
          return;
        }

		/*ret = rtc_gpio_init(GPIO_OUTPUT_IO_SOLN4);
        if(ret) {
          ESP_LOGI("GPIO","Could not initialize GPIO 33");
          return;
        }
		ret = rtc_gpio_set_direction(GPIO_OUTPUT_IO_SOLN4, RTC_GPIO_MODE_OUTPUT_ONLY);
        if(ret) {
          ESP_LOGI("GPIO","Could not set direction of GPIO 33");
          return;
        }
		 ret = rtc_gpio_set_level(GPIO_OUTPUT_IO_SOLN4, 1);
        if(ret) {
          ESP_LOGI("GPIO","Could not set level of GPIO 33");
          return;
        }*/


        const int ext_wakeup_pin_1 = 15;
        const uint64_t ext_wakeup_pin_1_mask = 1 << ext_wakeup_pin_1;
        const int ext_wakeup_pin_2 = 4;
        const uint64_t ext_wakeup_pin_2_mask = 1 << ext_wakeup_pin_2;

        ESP_LOGI("External Interrupts","Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_1);
        esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ALL_LOW);
        printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
        esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

        //ESP_LOGI("External Interrupts","Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_2);
        //esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);

        ESP_LOGI("First boot","Entering deep sleep\n");

        esp_deep_sleep_start();


    }
}
