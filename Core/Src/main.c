/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include "stm32h5xx.h"
#include <inttypes.h>
#include "stm32h5xx_hal.h"
#include "common.h"
#include "fft.h"
#include "memory.h"
#include <stdbool.h>
#include "dma_linkedlist.h"
#include "cbor_format.h"
#include "wisun_frame.h"
#include "wisun_transport.h"
#include "storage_mid.h"
#include <math.h>
#include "storage_cfg.h"
#include "solar_calc.h"
#include "app_x-cube-ai.h"
#include "quant.h"
#include <stdarg.h>
#include "APP/network.h"
#include <assert.h>
#include "adc_user.h"
#include "light_control.h"
#include "rtc_user.h"
#include "debug_log.h"
#include "sensor_measure.h"
#include "snapshot.h"
#include "wisun_app.h"
#include "wisun_router.h"
#include "light_event.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
    float freq;      // Hz
    float amplitude;    // FFT 진폭
} FftData_t;

//typedef struct { uint16_t volt_raw; uint16_t curr_raw; uint32_t t_us; } VIRead;

FftData_t fft_packet[FFT_SIZE / 2];
arm_rfft_fast_instance_f32 fftInstance;

typedef enum {
    WAITING_FOR_STX,
    RECEIVING_PACKET
} PacketParserState;

typedef enum {
    RESP_KIND_NONE = 0,
    RESP_KIND_SNAP,
    RESP_KIND_ACK,      
    RESP_KIND_RAW_BIN,  
    RESP_KIND_LIGHT_ACK,
} resp_kind_t;

typedef struct {
    uint8_t      pending;      
    resp_kind_t  kind;         
    uint16_t     tmid;         
    uint16_t     msg_id;       
    uint32_t     due_tick;     
    uint8_t  cmd;
    uint8_t  result;

    
    uint8_t      buf[128];     
    uint16_t     len;
    uint8_t      has_raw_buf;  
} resp_slot_t;

typedef struct __attribute__((packed)) {
    uint8_t  t;       // 0x10 = generic_ack
    uint8_t  uid[12];

    uint32_t msg_id;  
    uint8_t  ok;      // 1=success, 0=fail
    int8_t   err_code;
} AckBin_t;


typedef struct __attribute__((packed)) {
    uint8_t  t;                  /* SET_SETTING_ACK */
    uint8_t  uid[12];
    uint32_t msg_id;
    uint8_t  ok;
    int8_t   err_code;
    uint8_t  mode;
    uint8_t  apply_coord_type;
    int32_t  applied_lat_e7;
    int32_t  applied_lon_e7;
    uint16_t sunrise_min;
    uint16_t sunset_min;
    uint16_t dawn_min;
    uint16_t dusk_min;
    uint16_t on_time_min;
    uint16_t off_time_min;
} SetSettingAckV2_t;

typedef struct __attribute__((packed)) {
    uint8_t  t;       // 0x10 = power_ctrl_ack
    uint8_t  uid[12];

    uint32_t msg_id;
    uint8_t  ok;
    int8_t   err_code;
    uint8_t  light_on; // 0/1
} PowerCtrlAckBin_t;

typedef struct __attribute__((packed)) {
    uint8_t  t;      // 0x02 = get_status_resp
    uint8_t  uid[12];

    float    volt;
    float    curr;
    float    temp;

    uint8_t  light_on;     // 0/1    
       uint32_t msg_id;   
       uint8_t  ok;       // 1=success
       int8_t   err_code; // 0=OK, 1=에러
} StatusBin_t;

#pragma pack(push, 1)
typedef struct {
  uint8_t t;
  uint8_t uid[12];
  uint16_t msg_id;
  uint8_t ok;
  uint8_t gid;
  uint16_t mid;
  uint8_t dev;
  uint8_t dsp;
  uint8_t rch0, rch1;
  uint8_t txp;
  uint8_t mode;
  uint8_t mac[8];
  uint16_t fw_major, fw_minor;
} NodeInfoBin_t;
#pragma pack(pop)

typedef struct {
  uint8_t gid;
  uint8_t dev;
  uint8_t dsp;
  uint8_t rch0, rch1;
  uint8_t txp;
  uint8_t mode;
  uint8_t mac[8];
  uint16_t fw_major, fw_minor;
  uint8_t valid;
} node_info_cache_t;

#ifndef NODE_INFO_TEXT_MAX
#define NODE_INFO_TEXT_MAX 240u
#endif

typedef struct {
    uint8_t  pending;     
    uint16_t msg_id;      
    uint16_t tmid;        
    uint16_t used;        
    uint8_t  text[NODE_INFO_TEXT_MAX];
    uint32_t deadline_tick;
	uint32_t last_rx_tick;
	uint16_t got_mask;
} nodeinfo_ctx_t;

#pragma pack(push, 1)
typedef struct {
    uint8_t  t;
    uint8_t  uid[12];
    uint16_t msg_id;
    uint8_t  ok;
    int8_t   err_code;
    uint8_t  ch;
} GetChResp_t;
#pragma pack(pop)

#define FFT_DURATION_MS 60000  
#define FFT_DELAY_MS     100

#define AE_ROWS 10
#define AE_COLS 4

#define AI_FEATURE_FREQ_KHZ_IDX 0
#define AI_FEATURE_ADC_PK_IDX   1
#define AI_FEATURE_CURRENT_IDX  2
#define AI_FEATURE_VIN_IDX      3

#define TRIGGER_THRESHOLD  0.1f
#define TRIGGER_TIMEOUT 1000
#define FFT_WINDOW_MS 200
#define NUM_CHANNELS 3
#define ADC_BUFFER_SIZE 256
#define ULTRA_BUF_LEN   4096




#define I_OFFSET_RAW    1990
#define SLOT_JITTER_MS  150

#define LIGHT_Pin        GPIO_PIN_12      
#define LIGHT_GPIO_Port  GPIOA
#define LIGHT_ACTIVE_LOW 0

#ifndef UART6_TO_USART1_BRIDGE
#define UART6_TO_USART1_BRIDGE 0
#endif

#ifndef WISUN_BOOT_APPLY_AT_CFG
#define WISUN_BOOT_APPLY_AT_CFG 0
#endif

#ifndef WISUN_AT_COMMAND_ENABLE
#define WISUN_AT_COMMAND_ENABLE 0
#endif

#ifndef DEBUG_FFT_DATA_EXTRACT_MODE
#define DEBUG_FFT_DATA_EXTRACT_MODE 1
#endif

#ifndef WISUN_IDLE_RESET_ENABLE
#define WISUN_IDLE_RESET_ENABLE 1
#endif

#ifndef WISUN_IDLE_RESET_TIMEOUT_MS
#define WISUN_IDLE_RESET_TIMEOUT_MS 70000u
#endif

#define DEBUG_AT_TO_PC
#define RX_BUFFER_SIZE 100
#define PACKET_MAX_SIZE 256

#define T_NODEINFO_BIN  0x14
#define UID_ADDRESS  ((uint32_t*) 0x08FFF800)
#define __DCACHE_PRESENT 1U
#define __ICACHE_PRESENT 1U

#define GOT_GID   (1u<<0)
#define GOT_MID   (1u<<1)
#define GOT_DEV   (1u<<2)
#define GOT_DSP   (1u<<3)
#define GOT_RCH   (1u<<4)
#define GOT_TXP   (1u<<5)
#define GOT_MODE  (1u<<6)
#define GOT_MAC   (1u<<7)
#define GOT_FWVER (1u<<8)
#define GOT_ALL (GOT_GID|GOT_MID|GOT_DEV|GOT_DSP|GOT_RCH|GOT_TXP|GOT_MODE|GOT_MAC|GOT_FWVER)

#ifndef FW_MAJOR
#define FW_MAJOR 1
#endif
#ifndef FW_MINOR
#define FW_MINOR 0
#endif

#define NOISE_MIN      (0.001f)
#define FFT_SNR_K      5.0f

#define PACKET_STX    0x02
#define PACKET_ETX    0x03
#define LIGHT_ON 0x10
#define LIGHT_OFF 0x11
#define SNAP_REPORT_CMD 0x12
#ifndef LIGHT_STATE_EVENT_CMD
#define LIGHT_STATE_EVENT_CMD 0x15u
#endif
#define GET_STATUS 0x30
#define SET_MID  0x01
#define NODE_CFG 0x20
#define SET_MID_CH  0x21
#define FIRST_BOOT  0x22
#define GETID       0x23
#define SET_SETTING    0x31
#define SET_SETTING_ACK 0x10
#define LIGHT_OFF_ACK 0x32
#define LIGHT_ON_ACK 0x33
#define GET_NODE_INFO 0x40
#define SET_ASTRO_SETTING 0x45
#define GET_CH 0x24
#define SET_CH 0x25
#define T_GET_CH_RESP 0x24
#define CMD_ACK_RELAY 0x7E

#define SNAP_FFT_ADC_RAW_SPAN_MIN   30u
#define SNAP_FFT_VALID_AMP_MIN      0.5f
#define CTRL_DEDUP_CACHE_SIZE       8u
#define CTRL_DEDUP_TTL_MS           120000u

#define RESP_QUEUE_SIZE 4

#ifndef MID_INVALID
#define MID_INVALID ((uint16_t)0xFFFF)
#endif


#ifndef SUP_MIN_HZ
#define SUP_MIN_HZ 20000.0f   // 20 kHz
#endif
#ifndef SUP_MAX_HZ
#define SUP_MAX_HZ 80000.0f   // 80 kHz
#endif


typedef struct {
    uint8_t  valid;
    uint16_t src_mid;
    uint16_t target_mid;
    uint8_t  cmd;
    uint16_t msg_id;
    uint32_t tick;
} ctrl_dedup_entry_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_NodeTypeDef Node_GPDMA1_Channel5;
DMA_QListTypeDef List_GPDMA1_Channel5;
DMA_HandleTypeDef handle_GPDMA1_Channel5;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
typedef char static_assert_AE_COLS_mismatch[
    (AE_COLS == AI_NETWORK_IN_1_SIZE) ? 1 : -1
];
extern ADC_HandleTypeDef hadc2;
PacketParserState packet_state = WAITING_FOR_STX;

node_cfg_t g_node_cfg;
static resp_slot_t g_resp_slot;
static resp_slot_t g_resp_q[RESP_QUEUE_SIZE];
hop_slot_t g_hop_q[HOP_QUEUE_SIZE];
uint32_t g_hop_seen_keys[HOP_SEEN_TABLE_SIZE];
uint8_t g_hop_seen_count;
uint8_t g_hop_seen_pos;

static node_info_cache_t g_node_info = {
    .gid = 0,
    .dev = 1,
    .dsp = 1,
    .txp = 13,
    .rch0 = 0,
    .rch1 = 0xFF,
    .mode = 0,
    .mac = {0},
    .fw_major = 0,
    .fw_minor = 0,
    .valid = 0
};

static char     g_uid_str[40];
static uint32_t uid_ram_local[3];
static volatile bool     wisun_packet_ready = false;
static volatile uint16_t wisun_packet_len   = 0;
static volatile uint32_t wisun_packet_ready_tick = 0;
static uint8_t           wisun_packet_shadow[PACKET_MAX_SIZE];
static uint32_t          g_wisun_activity_tick = 0u;
static volatile uint8_t g_wait_mid_query = 0;
//static uint8_t capturing = 0;

static float ai_mse = 0.0f;
static int ai_pred = 0;
static uint32_t g_last_light_control_tick = 0;

// static bool on_done_today  = false;
// static bool off_done_today = false;

static nodeinfo_ctx_t g_nodeinfo = {0};

//static const float K_VIN     = K_ADC2V * V_DIV_GAIN;                        // raw ??Vin(V)
//static const float K_CURR    = K_ADC2V / (R_SHUNT * I_AMP_GAIN);
//static volatile uint16_t adc_buffer[FFT_TOTAL_SAMPLES];
volatile uint32_t g_monitor_count = 0;
static volatile int       wr_idx           = 0;
static volatile bool     ultra_frame_ready = false;
static volatile bool     ultra_sampling_paused = false;

volatile uint32_t g_frame_c0 = 0;
volatile uint32_t g_frame_c1 = 0;

volatile uint32_t g_dma_done = 0;
volatile uint32_t g_dma_half = 0;

static float32_t g_hann[FFT_SIZE];
static uint8_t g_hann_inited = 0;

static volatile uint8_t  g_ai_sample_ready = 0;
static volatile float    g_ai_sample = 0.0f;

static uint16_t uart_fail_cnt   = 0;
volatile uint32_t uid_ram[3];
volatile uint8_t  ai_pending = 0;
uint32_t          ai_next_run = 0;      
uint32_t          ai_period_ms = 200;   
static uint8_t  g_last_has_msg_id  = 0;
static uint8_t  g_last_has_cmd     = 0;
static ctrl_dedup_entry_t g_ctrl_dedup[CTRL_DEDUP_CACHE_SIZE];
static uint8_t g_ctrl_dedup_pos = 0u;
static volatile uint8_t g_snapshot_suppress_next_tx = 0u;

static uint8_t boot_cfg_started = 0;

static float ai_input[AE_COLS];
static int   ai_index = 0;

static volatile uint8_t  g_at_line_ready = 0;
static uint16_t          g_at_line_len   = 0;
static char              g_at_line[RX_BUFFER_SIZE];

static char ascii_buffer[RX_BUFFER_SIZE];
static uint16_t ascii_index = 0;

uint8_t rxByte, rxByte1;
uint8_t pc_rx_buffer[RX_BUFFER_SIZE];
uint8_t wisun_rx_buffer[RX_BUFFER_SIZE];
uint8_t pc_rx_index = 0;
uint8_t wisun_rx_index = 0;
uint8_t g_region_code;

uint16_t g_sunrise_min;
uint16_t g_sunset_min;
uint16_t g_dawn_min;
uint16_t g_dusk_min;
uint8_t g_light_on;

static uint16_t g_last_sun_year   = 0;
static uint8_t  g_last_sun_month  = 0;
static uint16_t g_last_sun_day    = 0;
static uint8_t  g_last_sun_region = 0xFF;


static uint8_t  buf[PACKET_MAX_SIZE];

uint16_t sunrise, sunset, dawn, dusk;
uint16_t my_mid = 0x0000;
uint16_t target_mid = 0;
uint16_t raw_buffer[FFT_SIZE];

float32_t outputSignal[FFT_SIZE];
float32_t magnitude[FFT_SIZE / 2];

static uint16_t prev_dawn = 0xFFFF;
static uint16_t prev_dusk = 0xFFFF;

volatile uint8_t g_adc_kick = 0;
/*static uint32_t ai_test_start = 0;
static uint8_t  ai_test_done  = 0;

static uint32_t ai_last_tick = 0;
static float ai_v = -1.0f;
static float ai_dv = 0.02f;*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

static void apply_mid(uint16_t new_mid);
static void apply_rch(uint8_t r0, uint8_t r1);
static void apply_mid_chan_from_cfg(void);
static void ai_service(void);
static bool is_uplink_report_cmd(uint8_t cmd);
void boot_poll(void);
//static void build_snapshot_packet(cbor_packet_t* rp, const snapshot_t* s, bool as_resp, const char* topic, uint16_t nonce);
// float Convert_ADC_To_Current(uint32_t adc_value);
void dbg_print_mid_info(const char *tag, uint16_t my_mid, uint16_t target_mid);
static inline void DWT_CYCCNT_Init(void);
void Debug_Print_FFT_Peak(void);
void ExtractFullFFT(const float32_t *in, float fs_hz, FftData_t *dest);
static void ExtractFullFFT_MagOnly(const float32_t *in, float32_t *mag_out);
static float tim6_sample_rate_hz(void);
static void init_uid_string(void);
static bool is_bootstrap_cmd(uint8_t cmd);
void Input_Ai_Model(float v);
static void InitHannWindowOnce(void);
//void StreetLight_ToggleTask(void);
static void handle_binary_cmd(uint8_t cmd, uint8_t flags, uint16_t msg_id, uint16_t tmid, uint16_t target_mid, const uint8_t *data, uint16_t len);
static uint8_t handle_cmd_set_setting(const uint8_t *data, uint16_t len);
static uint8_t ctrl_dedup_check_and_mark(uint16_t src_mid, uint16_t target_mid, uint8_t cmd, uint16_t msg_id);
static void mid_pack_uid12(uint8_t out12[12]);
static uint16_t my_strnlen(const char *s, uint16_t maxn);
bool node_is_provisioned(void);
static int nodeinfo_append_kv_line(const char *line);
static void nodeinfo_start(uint16_t tmid, uint16_t msg_id);
void nodeinfo_collect_line(const char *line);
static void nodeinfo_cache_update_from_kv(const char *kv_line);
static void nodeinfo_finish_and_send(uint8_t ok, int8_t err_code);
void nodeinfo_finish_ok(void);
void nodeinfo_finish_fail(int8_t err);
static void nodeinfo_poll(uint32_t now);
static int norm_min(int t);
void Parse_AT_Response(const char* buffer);
// static inline void PA12_toggle_soft(void);
static int parse_hex8(const char *s, uint8_t out[8]);
static int hexval(char c);
static uint8_t try_handle_uart6_local_cmd(const char *line);
uint16_t build_transport_payload(uint16_t target_mid, uint8_t ttl, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len, uint8_t *out, uint16_t out_cap);
static void log_snap_gateway_tx(const char *tag, uint16_t tmid, const uint8_t *data, uint16_t len);
uint8_t send_transport_direct(uint16_t target_mid, uint8_t ttl, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len);
void Query_MID_From_WiSUN(void);
void Read_UID(void);
// uint32_t Read_Voltage_ADC(void);
static void Read_UID_local(void);
void resp_slot_task_poll(void);
// void readADCData(void);
static void rstrip_inplace(char *s);
static void wisun_activity_mark(uint32_t now);
static void wisun_idle_reset_poll(uint32_t now);
// void Send_UID_UART2(void);
// void Send_Device_ID_UART2(void);
// void Send_Broadcast_Command(uint8_t *request_data, uint8_t request_length);
// void SendFFT_Packet(uint16_t target_mid, FftData_t *fft_data, uint8_t count);
// void SendDataPacket(uint16_t target_mid, uint8_t *data, uint16_t data_length);
void Send_Monitoring_Snapshot_JSON(uint16_t req_msg_id);
void snapshot_suppress_next_tx(void);
static void schedule_resp_with_slot(resp_kind_t kind, uint16_t tmid, uint16_t msg_id, const uint8_t *raw, uint16_t raw_len);
//static bool send_wisun_resp(uint16_t tmid, const uint8_t *cbor, size_t cbor_len);
static bool send_wisun_binary(uint16_t tmid, const uint8_t *data, size_t len);
void Ultra_ResumeNextFrame(void);
void Ultra_StartSampling(void);
void Ultra_StartDmaFrame(void);
void update_sun_times(void);
void wisun_process_rx_mainloop(void);
static inline uint16_t xorshift16(uint16_t x);
static uint16_t wisun_expected_packet_len(const uint8_t *buf, uint16_t have_len);

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/*void Transfer_ADC_To_DAC(void);*/
//void SendReceivedBroadcastPacket(void);
//void Process_WiSun_Command(wisun_rx_buffer, wisun_rx_index);
//void loop_fft_for_duration(uint32_t duration_ms);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int __io_putchar(int ch)
{
#if FOCUS_TIMING_LOG
    (void)ch;
#else
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
#endif
    return ch;
}

uint16_t build_transport_payload(uint16_t target_mid, uint8_t ttl, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len, uint8_t *out, uint16_t out_cap)
{
    uint16_t need = (uint16_t)(body_len + 7u);

    if (out == NULL || need > out_cap) {
        return 0u;
    }

    out[0] = (uint8_t)(target_mid & 0xFFu);
    out[1] = (uint8_t)(target_mid >> 8);
    out[2] = ttl;
    out[3] = cmd;
    out[4] = flags;
    out[5] = (uint8_t)(msg_id >> 8);
    out[6] = (uint8_t)(msg_id & 0xFFu);

    if (body != NULL && body_len > 0u) {
        memcpy(&out[7], body, body_len);
    }

    return need;
}

uint8_t light_event_send_transport(uint16_t target_mid,
                                   uint8_t ttl,
                                   uint8_t cmd,
                                   uint8_t flags,
                                   uint16_t msg_id,
                                   const uint8_t *body,
                                   uint16_t body_len)
{
    return send_transport_direct(target_mid, ttl, cmd, flags, msg_id, body, body_len);
}

uint8_t light_event_node_ready(void)
{
    return node_is_provisioned() ? 1u : 0u;
}

uint8_t light_event_tx_blocked(void)
{
    return ultra_sampling_paused ? 1u : 0u;
}

static void wisun_activity_mark(uint32_t now)
{
    g_wisun_activity_tick = now;
}

static void wisun_idle_reset_poll(uint32_t now)
{
#if WISUN_IDLE_RESET_ENABLE
    if (g_rtc_synced) {
        wisun_activity_mark(now);
        return;
    }

    if (!node_is_provisioned()) {
        wisun_activity_mark(now);
        return;
    }

    if (g_wisun_activity_tick == 0u) {
        wisun_activity_mark(now);
        return;
    }

    if ((uint32_t)(now - g_wisun_activity_tick) < WISUN_IDLE_RESET_TIMEOUT_MS) {
        return;
    }

    {
        static const char cmd[] = "AT+RESET\r\n";
        HAL_StatusTypeDef st = HAL_UART_Transmit(&huart1,
                                                  (uint8_t*)cmd,
                                                  (uint16_t)(sizeof(cmd) - 1u),
                                                  100u);
        uart6_log("[WISUN_IDLE_RESET] t=%lu idle=%lu timeout=%lu tx=%s\r\n",
                  (unsigned long)now,
                  (unsigned long)(now - g_wisun_activity_tick),
                  (unsigned long)WISUN_IDLE_RESET_TIMEOUT_MS,
                  (st == HAL_OK) ? "ok" : "fail");
    }

    wisun_activity_mark(now);
#else
    (void)now;
#endif
}


static void log_snap_gateway_tx(const char *tag, uint16_t tmid, const uint8_t *data, uint16_t len)
{
    uint16_t target_mid;
    uint8_t ttl;
    uint8_t cmd;
    uint8_t flags;
    uint16_t msg_id;

    if (data == NULL || len < 7u) {
        return;
    }

    target_mid = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    ttl = data[2];
    cmd = data[3];
    flags = data[4];
    msg_id = (uint16_t)((uint16_t)data[5] << 8 | ((uint16_t)data[6]));

    if (cmd != SNAP_REPORT_CMD) {
        return;
    }

    uart6_log("%s tmid=0x%04X target_mid=%u ttl=%u flags=0x%02X msg_id=%u len=%u\r\n",
              (tag != NULL) ? tag : "[GW_TX_SNAP]",
              (unsigned)tmid,
              (unsigned)target_mid,
              (unsigned)ttl,
              (unsigned)flags,
              (unsigned)msg_id,
              (unsigned)len);
    dbg_dump_uart6_with_tag((tag != NULL) ? tag : "[GW_TX_SNAP]", data, len);
}

uint8_t send_transport_direct(uint16_t target_mid, uint8_t ttl, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len)
{
    uint8_t payload[HOP_MAX_FRAME];
    uint16_t payload_len = build_transport_payload(target_mid,
                                                   ttl,
                                                   cmd,
                                                   flags,
                                                   msg_id,
                                                   body,
                                                   body_len,
                                                   payload,
                                                   (uint16_t)sizeof(payload));
    if (payload_len == 0u) {
        return 0u;
    }

    /* timing_log("[TLOG_TX_REQ] t=%lu target=0x%04X ttl=%u cmd=0x%02X flags=0x%02X msg=%u body=%u payload=%u\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned)target_mid,
               (unsigned)ttl,
               (unsigned)cmd,
               (unsigned)flags,
               (unsigned)msg_id,
               (unsigned)body_len,
               (unsigned)payload_len); */

    if (cmd == LIGHT_STATE_EVENT_CMD) {
        /* dbg_dump_uart6_with_tag("[LIGHT_EVT_BODY_HEX]", body, body_len); */
        dbg_dump_uart6_with_tag("[LIGHT_EVT_PAYLOAD_HEX]", payload, payload_len);
    }

    return send_wisun_binary(0x0000u, payload, payload_len) ? 1u : 0u;
}

bool node_is_provisioned(void)
{    
    return (my_mid != 0x0000u);
}

static bool is_bootstrap_cmd(uint8_t cmd)
{
   
    switch (cmd) {
    case FIRST_BOOT:
    case SET_MID_CH:   
    case GETID:       
        return true;
    default:
        return false;
    }
}


void boot_poll(void)
{
    if (boot_cfg_started) return;
    
    if (HAL_GetTick() < 1500) return;

    nodeinfo_start(/*tmid=*/0, /*msg_id=*/0);  
    boot_cfg_started = 1;
}

static inline uint16_t xorshift16(uint16_t x){
    if (x==0) x=0xACE1u;
    x ^= x << 7; x ^= x >> 9; x ^= x << 8;
    return x;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART6)
	    {
	        if (pc_rx_index < RX_BUFFER_SIZE - 1)
	        {
	            pc_rx_buffer[pc_rx_index++] = rxByte;

	            if (rxByte == '\n')
	            {
	                uint8_t crlf[] = "\r\n";

	                if (pc_rx_index >= 2 && pc_rx_buffer[pc_rx_index - 2] == '\r') {
	                    pc_rx_index -= 2;
	                } else {
	                    pc_rx_index -= 1;
	                }
	                pc_rx_buffer[pc_rx_index] = '\0';

	                if (!try_handle_uart6_local_cmd((const char *)pc_rx_buffer)) {
#if UART6_TO_USART1_BRIDGE
	                    HAL_UART_Transmit(&huart1, (uint8_t *)pc_rx_buffer, pc_rx_index, HAL_MAX_DELAY);
	                    HAL_UART_Transmit(&huart1, crlf, 2, HAL_MAX_DELAY);
#endif
	                    HAL_UART_Transmit(&huart6, (uint8_t *)pc_rx_buffer, pc_rx_index, HAL_MAX_DELAY);
	                    HAL_UART_Transmit(&huart6, crlf, 2, HAL_MAX_DELAY);
	                }

	                memset(pc_rx_buffer, 0, RX_BUFFER_SIZE);
	                pc_rx_index = 0;
	            }
	        }
	        else
	        {
	            pc_rx_index = 0;
	            memset(pc_rx_buffer, 0, RX_BUFFER_SIZE);
	        }

	        HAL_UART_Receive_IT(&huart6, &rxByte, 1);
	    }
		else if (huart->Instance == USART1) {

		    if (packet_state == WAITING_FOR_STX) {		        
		        if (rxByte1 == PACKET_STX) {
		            wisun_rx_index = 0;
		            wisun_rx_buffer[wisun_rx_index++] = rxByte1;

		            //HAL_UART_Transmit(&huart6, &rxByte1, 1, HAL_MAX_DELAY);
		            packet_state = RECEIVING_PACKET;
		        }		        
		        else {


		            if (ascii_index < RX_BUFFER_SIZE - 1) {
		                ascii_buffer[ascii_index++] = rxByte1;

		                if (rxByte1 == '\r' || rxByte1 == '\n') {		                    
		                    while (ascii_index >= 1 &&
		                          (ascii_buffer[ascii_index - 1] == '\r' || ascii_buffer[ascii_index - 1] == '\n')) {
		                        ascii_index--;
		                    }

		                    if (ascii_index > 0) {
								uint16_t n = ascii_index;
								if (n >= RX_BUFFER_SIZE) n = RX_BUFFER_SIZE - 1;  

								memcpy(g_at_line, ascii_buffer, n);
								g_at_line[n] = '\0';
								g_at_line_len = n;
								g_at_line_ready = 1;
							}

		                    ascii_index = 0;
		                    //memset(ascii_buffer, 0, sizeof(ascii_buffer));
		                }
		            } else {		                
		                ascii_index = 0;
		                //memset(ascii_buffer, 0, sizeof(ascii_buffer));
		            }
		        }
		    }
		    else if (packet_state == RECEIVING_PACKET) {
		        if (wisun_rx_index < RX_BUFFER_SIZE - 1) {
		            wisun_rx_buffer[wisun_rx_index++] = rxByte1;

		            uint16_t expected_len = wisun_expected_packet_len(wisun_rx_buffer, wisun_rx_index);
		            if (expected_len != 0u && wisun_rx_index >= expected_len) {
		                uint16_t packet_len = expected_len;

		                if (packet_len <= PACKET_MAX_SIZE &&
		                    wisun_rx_buffer[packet_len - 1u] == PACKET_ETX) {
		                    memcpy((void*)wisun_packet_shadow, wisun_rx_buffer, packet_len);
		                    wisun_packet_len   = packet_len;
		                    wisun_packet_ready_tick = HAL_GetTick();
		                    wisun_packet_ready = true;
		                }

		                wisun_rx_index = 0;
		                packet_state = WAITING_FOR_STX;
		            }
		        } else {
		            // overflow
		            wisun_rx_index = 0;
		            packet_state = WAITING_FOR_STX;
		        }
		    }		
		    HAL_UART_Receive_IT(&huart1, &rxByte1, 1);
	}
}


static uint8_t ctrl_dedup_check_and_mark(uint16_t src_mid, uint16_t target_mid, uint8_t cmd, uint16_t msg_id)
{
    uint32_t now = HAL_GetTick();

    if (msg_id == 0u) {
        return 0u;
    }

    for (uint8_t i = 0u; i < CTRL_DEDUP_CACHE_SIZE; ++i) {
        ctrl_dedup_entry_t *e = &g_ctrl_dedup[i];

        if (!e->valid) {
            continue;
        }

        if ((uint32_t)(now - e->tick) > CTRL_DEDUP_TTL_MS) {
            e->valid = 0u;
            continue;
        }

        if (e->target_mid == target_mid &&
            e->cmd == cmd &&
            e->msg_id == msg_id) {
            return 1u;
        }
    }

    g_ctrl_dedup[g_ctrl_dedup_pos].valid = 1u;
    g_ctrl_dedup[g_ctrl_dedup_pos].src_mid = src_mid;
    g_ctrl_dedup[g_ctrl_dedup_pos].target_mid = target_mid;
    g_ctrl_dedup[g_ctrl_dedup_pos].cmd = cmd;
    g_ctrl_dedup[g_ctrl_dedup_pos].msg_id = msg_id;
    g_ctrl_dedup[g_ctrl_dedup_pos].tick = now;
    g_ctrl_dedup_pos = (uint8_t)((g_ctrl_dedup_pos + 1u) % CTRL_DEDUP_CACHE_SIZE);

    return 0u;
}

static void handle_binary_cmd(uint8_t cmd, uint8_t flags, uint16_t msg_id, uint16_t tmid, uint16_t target_mid, const uint8_t *data, uint16_t len)
{
    uint8_t uid12[12];
    mid_pack_uid12(uid12);
    switch (cmd)
        {
            case 0x13:
            {
                uart6_log("[SNAP_REQ_IGNORED] cmd=0x13 tmid=0x%04X msg_id=%u periodic_only=1\r\n",
                          (unsigned)tmid,
                          (unsigned)msg_id);
                break;
            }
        
            // LIGHT ON (0x10)            
            case LIGHT_ON:
            {
                uint8_t on = flags & 0x01;
                light_event_set_reason(LIGHT_EVENT_REASON_CMD);
                start_manual_override(on);
                light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
                g_last_light_control_tick = HAL_GetTick();

                PowerCtrlAckBin_t ack = {0};
                ack.t = LIGHT_ON_ACK;
                memcpy(ack.uid, uid12, 12);
                ack.msg_id = msg_id;
                ack.ok = 1;
                ack.err_code = 0;
                ack.light_on = light_is_on_logical();

                schedule_resp_with_slot(
                    RESP_KIND_LIGHT_ACK,
                    tmid,
                    msg_id,
                    (uint8_t*)&ack,
                    sizeof(ack)
                );
                break;
            }
            
            // LIGHT OFF (0x11)
            case LIGHT_OFF:
            {
                light_event_set_reason(LIGHT_EVENT_REASON_CMD);
                start_manual_override(0);
                light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
                g_last_light_control_tick = HAL_GetTick();

                PowerCtrlAckBin_t ack = {0};
                ack.t = LIGHT_OFF_ACK;
                memcpy(ack.uid, uid12, 12);
                ack.msg_id = msg_id;
                ack.ok = 1;
                ack.err_code = 0;
                ack.light_on = light_is_on_logical();

                schedule_resp_with_slot(
                    RESP_KIND_LIGHT_ACK,
                    tmid,
                    msg_id,
                    (uint8_t*)&ack,
                    sizeof(ack)
                );
                break;
            }
            
            // MID 변환(CMD_SET_MID)
            case SET_MID:
            {
                if (len < 2) break;
                uint16_t new_mid = ((uint16_t)data[0] << 8) | data[1];

                apply_mid(new_mid);
                
                /*
                AckBin_t ack = {0};
                ack.t = CMD_SET_MID;
                memcpy(ack.uid, uid12, 12);
                ack.msg_id = msg_id;
                ack.ok = 1;

                schedule_resp_with_slot(
                    RESP_KIND_RAW_BIN,
                    tmid,
                    msg_id,
                    (uint8_t*)&ack,
                    sizeof(ack)
                );
                */
                break;
            }
            case NODE_CFG:
            {
                if (len < 9) {
                    dbg_dump_uart6_with_tag("[NODE_CFG_DROP_SHORT]", data, len);
                    break;
                }

                uint16_t body_target_mid = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

                if (body_target_mid != 0x0000u && body_target_mid != my_mid) {
                    char msg[96];
                    int n = snprintf(msg, sizeof(msg),
                                    "[NODE_CFG_DROP_NOT_MY_MID] my_mid=0x%04X body_target=0x%04X src=0x%04X\r\n",
                                    (unsigned)my_mid,
                                    (unsigned)body_target_mid,
                                    (unsigned)tmid);
                    if (n > 0) {
                        HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 100);
                    }
                    break;
                }

                uint8_t ver = data[2];
                (void)ver;

                g_node_cfg.mode                = data[3];
                g_node_cfg.on_off_mode         = g_node_cfg.mode;
                g_node_cfg.light_on_hour       = data[4];
                g_node_cfg.light_on_min        = data[5];
                g_node_cfg.light_off_hour      = data[6];
                g_node_cfg.light_off_min       = data[7];
                g_node_cfg.manual_duration_min = data[8];

                save_node_cfg_to_flash(&g_node_cfg);

                AckBin_t ack = {0};

                if (g_node_cfg.mode == 3u) {
                    light_event_set_reason(LIGHT_EVENT_REASON_NODE_CFG);
                    start_manual_override(1u);
                    light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
                } else {
                    light_control_clear_manual_override();
                }

                ack.t = NODE_CFG;
                memcpy(ack.uid, uid12, 12);
                ack.msg_id = msg_id;
                ack.ok = 1;
                ack.err_code = 0;

                schedule_resp_with_slot(RESP_KIND_RAW_BIN, tmid, msg_id, (uint8_t*)&ack, sizeof(ack) );

                break;
            }
            case SET_MID_CH:
                    {
                        if (len < 3) break;

                        uint16_t new_mid = ((uint16_t)data[0] << 8) | data[1];
                        uint8_t  new_ch  = data[2];

                        apply_mid(new_mid);

                        g_node_cfg.rch[0] = 0;
                        g_node_cfg.rch[1] = new_ch;
                        (void)save_node_cfg_to_flash(&g_node_cfg);

                        apply_rch(g_node_cfg.rch[0], g_node_cfg.rch[1]);
                        
                        /*
                        AckBin_t ack = {0};
                        ack.t = SET_MID_CH;
                        memcpy(ack.uid, uid12, 12);
                        ack.msg_id = msg_id;
                        ack.ok = 1;
                        ack.err_code = 0;

                        schedule_resp_with_slot(
                            RESP_KIND_RAW_BIN,
                            tmid,
                            msg_id,
                            (uint8_t*)&ack,
                            sizeof(ack)
                        );
                        */
                        break;
			}
            case GET_STATUS:
            {
                StatusBin_t st;
                memset(&st, 0, sizeof(st));

                st.t = 0x02;  // status resp
                memcpy(st.uid, uid12, 12);

                int vi_ok = 0, temp_ok = 0;

                VIRead vi;
                float vin_v = 0.0f, i_adc_v = 0.0f;

                if (AD_DC_Injected_Once(&vi) == HAL_OK) {
                    vin_v   = adc_raw_to_dc_vin(vi.volt_raw);
                    i_adc_v = adc_raw_to_dc_current(vi.curr_raw);
                    vi_ok = 1;
                }

                uint16_t temp_raw = 0;
                float temp_v = 0.0f, temp_c = NTC_INVALID_TEMP_C;

                if (HAL_ADC_Start(&hadc2) == HAL_OK) {
                    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
                        temp_raw = HAL_ADC_GetValue(&hadc2);
                        temp_v   = (float)temp_raw * 3.3f / 4095.0f;                    
                        temp_ok = ntc_voltage_to_temp_c_checked(temp_v, &temp_c);
                    }
                    HAL_ADC_Stop(&hadc2);
                }

                st.volt = vin_v;
                st.curr = i_adc_v;
                st.temp = temp_c;

                st.light_on = light_is_on_logical();
                st.msg_id   = msg_id;

                if (!vi_ok || !temp_ok) {
                    st.ok = 0;
                    st.err_code = -1;
                } else {
                    st.ok = 1;
                    st.err_code = 0;
                }

                schedule_resp_with_slot(
                    RESP_KIND_RAW_BIN,   
                    tmid,
                    msg_id,
                    (uint8_t*)&st,
                    sizeof(st)
                );
                break;
            }
            case GET_CH:   // 0x24
            {
                uint8_t uid12[12];
                mid_pack_uid12(uid12);

                GetChResp_t resp = {0};
                resp.t = T_GET_CH_RESP;
                memcpy(resp.uid, uid12, 12);
                resp.msg_id = msg_id;
                resp.ok = 1;
                resp.err_code = 0;
                resp.ch = g_node_cfg.rch[1];
                char s[96];
				snprintf(s, sizeof(s),
						 "[GET_CH]  tmid =0x%04X msg_id=0x%04X ch=%u\r\n",
						 (unsigned)tmid , (unsigned)msg_id, (unsigned)resp.ch);
				debug6(s);
                schedule_resp_with_slot(
                    RESP_KIND_RAW_BIN,
                    tmid,
                    msg_id,
                    (uint8_t*)&resp,
                    sizeof(resp)
                );
                break;
            }
            case SET_SETTING:
            {
                uint8_t duplicate = ctrl_dedup_check_and_mark(tmid, target_mid, cmd, msg_id);
                uint8_t result = duplicate ? 0u : handle_cmd_set_setting(data, len);

                if (duplicate) {
                    uart6_log("[SET_SETTING_DUP] src=0x%04X target=0x%04X msg_id=%u skip_apply=1\r\n",
                              (unsigned)tmid,
                              (unsigned)target_mid,
                              (unsigned)msg_id);
                }

                SetSettingAckV2_t ack = {0};
                uint8_t uid12[12];
                mid_pack_uid12(uid12);

                ack.t = SET_SETTING_ACK;
                memcpy(ack.uid, uid12, 12);
                ack.msg_id = msg_id;
                ack.ok = (result == 0u) ? 1u : 0u;
                ack.err_code = (int8_t)result;
                ack.mode = g_node_cfg.mode;
                ack.apply_coord_type = g_node_cfg.apply_coord_type;
                ack.applied_lat_e7 = (g_node_cfg.apply_coord_type == 1u)
                    ? g_node_cfg.install_lat_e7 : g_node_cfg.standard_lat_e7;
                ack.applied_lon_e7 = (g_node_cfg.apply_coord_type == 1u)
                    ? g_node_cfg.install_lon_e7 : g_node_cfg.standard_lon_e7;
                ack.sunrise_min = g_sunrise_min;
                ack.sunset_min = g_sunset_min;
                ack.dawn_min = g_dawn_min;
                ack.dusk_min = g_dusk_min;
                (void)light_control_get_effective_schedule(
                    &ack.mode,
                    &ack.on_time_min,
                    &ack.off_time_min
                );

                schedule_resp_with_slot(
                    RESP_KIND_RAW_BIN,
                    tmid,
                    msg_id,
                    (uint8_t *)&ack,
                    sizeof(ack)
                );

                debug6("[ACK_SLOT] extended SET_SETTING ACK scheduled\r\n");
                break;
            }
            case SET_ASTRO_SETTING:
            {
                uart6_log("[SET_ASTRO_DEPRECATED] cmd=0x45 src=0x%04X target=0x%04X msg_id=%u ignored=1 use_cmd=0x31\r\n",
                          (unsigned)tmid,
                          (unsigned)target_mid,
                          (unsigned)msg_id);
                break;
            }
            case SET_RTC_KST:
            {
                char rtc_rx_dbg[128];
                int rtc_rx_dbg_len = snprintf(
                    rtc_rx_dbg, sizeof(rtc_rx_dbg),
                    "[RTC_SYNC_RX] tmid=0x%04X msg_id=0x%04X len=%u\r\n",
                    (unsigned)tmid,
                    (unsigned)msg_id,
                    (unsigned)len
                );

                if (rtc_rx_dbg_len > 0) {
                    HAL_UART_Transmit(&huart6, (uint8_t*)rtc_rx_dbg, (uint16_t)rtc_rx_dbg_len, 100);
                }

                if (data != NULL && len > 0u) {
                    dbg_dump_uart6_with_tag("[RTC_SYNC_PAYLOAD]", data, len);
                }

            
                if (data != NULL && len >= 19u && data[0] == 0x2Bu) {
                    uart6_log("[RTC_SYNC_DROP] reason=rtc_ack_like_payload len=%u data0=0x%02X\r\n",
                            (unsigned)len,
                            (unsigned)data[0]);
                    break;
                }

                uint8_t result = handle_cmd_set_rtc_kst(data, len);

                if (tmid != 0x0000u) {
                    AckBin_t ack = {0};
                    ack.t = SET_RTC_KST;
                    memcpy(ack.uid, uid12, 12);
                    ack.msg_id = msg_id;
                    ack.ok = (result == 0u) ? 1u : 0u;
                    ack.err_code = (int8_t)result;

                    schedule_resp_with_slot(
                        RESP_KIND_RAW_BIN,
                        tmid,
                        msg_id,
                        (uint8_t*)&ack,
                        sizeof(ack)
                    );
                }

                break;
            }
            case GET_NODE_INFO:   // 0x40
            {
                NodeInfoBin_t resp;
                memset(&resp, 0, sizeof(resp));

                resp.t = T_NODEINFO_BIN;

                uint8_t uid12[12];
                mid_pack_uid12(uid12);
                memcpy(resp.uid, uid12, 12);

                resp.msg_id = msg_id;
                resp.ok     = 1;
                
                resp.mid  = g_node_cfg.mid;
                resp.mode = current_control_mode();
                
                resp.rch0 = g_node_cfg.rch[0];
                resp.rch1 = g_node_cfg.rch[1]; 

                resp.gid  = g_node_info.gid;
                resp.dev  = g_node_info.dev;
                resp.dsp  = g_node_info.dsp;
                resp.txp  = g_node_info.txp;
                memcpy(resp.mac, g_node_info.mac, 8);
                
                resp.fw_major = (uint16_t)FW_MAJOR;
                resp.fw_minor = (uint16_t)FW_MINOR;

                schedule_resp_with_slot(
                    RESP_KIND_RAW_BIN,
                    tmid,
                    msg_id,
                    (uint8_t*)&resp,
                    (uint16_t)sizeof(resp)
                );
                break;
            }            
            default:
                break;
        }
}

static int32_t i32_be_at(const uint8_t *p)
{
    uint32_t u = ((uint32_t)p[0] << 24) |
                 ((uint32_t)p[1] << 16) |
                 ((uint32_t)p[2] << 8) |
                 ((uint32_t)p[3]);
    return (int32_t)u;
}

static uint8_t coord_e7_valid(int32_t lat_e7, int32_t lon_e7)
{
    return (lat_e7 >= -900000000L && lat_e7 <= 900000000L &&
            lon_e7 >= -1800000000L && lon_e7 <= 1800000000L) ? 1u : 0u;
}

static uint8_t handle_cmd_set_setting(const uint8_t *data, uint16_t len)
{

	uint8_t result = 0;

    if (len < 13u) {        
    	return 1;
    }

    uint8_t on_off_mode        = data[0];
    uint8_t on_corr_mode       = data[1];
    uint16_t on_corr_time      = data[2];
    uint8_t off_corr_mode      = data[3];
    uint16_t off_corr_time     = data[4];
    uint16_t forced_time       = data[5];
    uint8_t saving_mode        = data[6];
    uint8_t saving_start_hour  = data[7];
    uint8_t saving_start_min   = data[8];
    uint8_t saving_end_hour    = data[9];
    uint8_t saving_end_min     = data[10];
    uint8_t snap_enable        = data[11];
    uint16_t snap_interval_sec = (uint16_t)data[12];
    uint8_t light_on_hour      = g_node_cfg.light_on_hour;
    uint8_t light_on_min       = g_node_cfg.light_on_min;
    uint8_t light_off_hour     = g_node_cfg.light_off_hour;
    uint8_t light_off_min      = g_node_cfg.light_off_min;

    if (on_off_mode > 3u) {
        return 5u;
    }

    if (len >= 18u) {
        forced_time       = (uint16_t)data[5] | ((uint16_t)data[6] << 8);
        saving_mode       = data[7];
        saving_start_hour = data[8];
        saving_start_min  = data[9];
        saving_end_hour   = data[10];
        saving_end_min    = data[11];
        snap_enable       = data[12];
        light_on_hour     = data[14];
        light_on_min      = data[15];
        light_off_hour    = data[16];
        light_off_min     = data[17];

        if (len >= 20u) {
            snap_interval_sec = (uint16_t)data[18] | ((uint16_t)data[19] << 8);
        } else {
            snap_interval_sec = (uint16_t)data[13];
        }
    }
    
    
    if (len >= 30u && data[20] != 0u) {
        uint8_t apply_coord_type = data[21];
        int32_t selected_lat_e7 = i32_be_at(&data[22]);
        int32_t selected_lon_e7 = i32_be_at(&data[26]);

        if (apply_coord_type > 1u) return 3u;
        if (!coord_e7_valid(selected_lat_e7, selected_lon_e7)) return 4u;

        g_node_cfg.coord_enable = 1u;
        g_node_cfg.apply_coord_type = apply_coord_type;
        if (apply_coord_type == 0u) {
            g_node_cfg.standard_lat_e7 = selected_lat_e7;
            g_node_cfg.standard_lon_e7 = selected_lon_e7;
        } else {
            g_node_cfg.install_lat_e7 = selected_lat_e7;
            g_node_cfg.install_lon_e7 = selected_lon_e7;
        }
    }

    g_node_cfg.on_off_mode        = on_off_mode;
    g_node_cfg.on_corr_mode       = on_corr_mode;
    g_node_cfg.on_corr_time_min   = on_corr_time;
    g_node_cfg.off_corr_mode      = off_corr_mode;
    g_node_cfg.off_corr_time_min  = off_corr_time;
    g_node_cfg.forced_time_min    = 0u;

    g_node_cfg.saving_mode        = saving_mode;
    g_node_cfg.saving_start_hour  = saving_start_hour;
    g_node_cfg.saving_start_min   = saving_start_min;
    g_node_cfg.saving_end_hour    = saving_end_hour;
    g_node_cfg.saving_end_min     = saving_end_min;

    g_node_cfg.snap_enable        = snap_enable ? 1u : 0u;
    g_node_cfg.snap_interval_sec  = snap_interval_sec;
    g_node_cfg.light_on_hour      = light_on_hour;
    g_node_cfg.light_on_min       = light_on_min;
    g_node_cfg.light_off_hour     = light_off_hour;
    g_node_cfg.light_off_min      = light_off_min;

    if (on_off_mode <= 3u) {
        g_node_cfg.mode = on_off_mode;
    }

    if (g_node_cfg.mode == 3u) {
        start_forced_time_control(forced_time);
        g_last_light_control_tick = HAL_GetTick();
    } else {
        light_control_clear_manual_override();
    }

    snapshot_reconfigure_timer_from_cfg();

    if (!save_node_cfg_to_flash(&g_node_cfg)) {
        result = 2u;
    } else {
        result = 0u;
        if (len >= 30u && data[20] != 0u) {
            g_last_sun_year = 0u;
            update_sun_times();
            scheduler_poll();
        }
    }
	char msg[160];
	    snprintf(msg, sizeof(msg),
	             "[SET_SETTING] result=%d len=%u mode=%u on=%02u:%02u off=%02u:%02u save=%u %02u:%02u-%02u:%02u forced=%u snap=%u interval=%us\r\n",
	             result,
	             len,
	             on_off_mode,
	             g_node_cfg.light_on_hour,
	             g_node_cfg.light_on_min,
	             g_node_cfg.light_off_hour,
	             g_node_cfg.light_off_min,
	             saving_mode,
	             saving_start_hour,
	             saving_start_min,
	             saving_end_hour,
	             saving_end_min,
	             forced_time,
	             g_node_cfg.snap_enable,
	             (unsigned)g_node_cfg.snap_interval_sec);
	    debug6(msg);
	return result;
}

static uint16_t wisun_expected_packet_len(const uint8_t *buf, uint16_t have_len)
{
    if (buf == NULL || have_len < 6u) {
        return 0u;
    }

    uint8_t sig2 = buf[2];
    uint8_t dl = buf[3];

    if (sig2 == 0xABu) {
        return (uint16_t)(6u + dl + 2u);
    }

    if (sig2 == 0xAAu) {
        uint16_t need_b = (uint16_t)(6u + dl + 2u);
        if (have_len >= need_b && buf[need_b - 1u] == PACKET_ETX) {
            return need_b;
        }

        return (uint16_t)(6u + 10u + dl + 2u);
    }

    return 0u;
}

static int norm_min(int t)
{
    while (t < 0)      t += 1440;
    while (t >= 1440)  t -= 1440;
    return t;
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;
    g_dma_half++;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;

    g_dma_done++;
    g_frame_c1 = DWT->CYCCNT;
    ultra_frame_ready = 1;

    /* static uint32_t last_dma_log_tick = 0u;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_dma_log_tick) >= 1000u) {
        last_dma_log_tick = now;
        uart6_log("[ULTRA_DMA_CPLT] done=%lu half=%lu ready=%u paused=%u tick=%lu\r\n",
                  (unsigned long)g_dma_done,
                  (unsigned long)g_dma_half,
                  (unsigned)(ultra_frame_ready ? 1u : 0u),
                  (unsigned)(ultra_sampling_paused ? 1u : 0u),
                  (unsigned long)now);
    } */
}
static void mid_pack_uid12(uint8_t out12[12]) {
    memcpy(out12, (const void*)uid_ram, 12);
}

static bool send_wisun_binary(uint16_t tmid, const uint8_t *data, size_t len)
{
    bool ok;
    wisun_frame_cfg_t cfg = {
        .sig1 = 0xAA,   
        .sig2 = 0xAB,   
        .tmid = tmid
    };
    /* timing_log("[TLOG_TX_UART_BEGIN] t=%lu frame_tmid=0x%04X len=%u\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned)tmid,
               (unsigned)len); */
    // log_snap_gateway_tx("[GW_TX_SNAP_DIRECT_FINAL]", tmid, data, (uint16_t)len);

    ok = wisun_send_frame(&cfg, data, len, wisun_tx_adapter, NULL);
    /* timing_log("[TLOG_TX_UART_END] t=%lu frame_tmid=0x%04X ok=%u\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned)tmid,
               (unsigned)(ok ? 1u : 0u)); */
    return ok;
}

void dbg_print_mid_info(const char *tag, uint16_t my_mid, uint16_t target_mid)
{
#if FOCUS_TIMING_LOG
    (void)tag;
    (void)my_mid;
    (void)target_mid;
    return;
#else
    char buf[64];
    int len;

    if (tag) {
        len = snprintf(buf, sizeof(buf),
                       "%s my_mid=0x%04X target_mid=0x%04X\r\n",
                       tag, my_mid, target_mid);
    } else {
        len = snprintf(buf, sizeof(buf),
                       "my_mid=0x%04X target_mid=0x%04X\r\n",
                       my_mid, target_mid);
    }

    if (len > 0) {
        if (len > (int)sizeof(buf)) len = sizeof(buf);
        HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
    }
#endif
}

static bool compact_snap_is_own_uid(const uint8_t *data)
{
    uint8_t uid12[12];

    mid_pack_uid12(uid12);
    return memcmp(&data[SNAP_COMPACT_UID_IDX], uid12, sizeof(uid12)) == 0;
}

void Input_Ai_Model(float v)
{
    if (ai_pending) return;  
    
    if (ai_index < AE_COLS) {
        ai_input[ai_index] = v;
        ai_index++;
    }

    if (ai_index >= AE_COLS) {
        ai_index = AE_COLS;   // clamp
        ai_pending = 1;
    }
}

static int hexval(char c){
    if ('0'<=c && c<='9') return c-'0';
    if ('a'<=c && c<='f') return c-'a'+10;
    if ('A'<=c && c<='F') return c-'A'+10;
    return -1;
}

static int parse_hex8(const char *s, uint8_t out[8]){    
    for (int i=0;i<8;i++){
        int hi = hexval(s[i*2]);
        int lo = hexval(s[i*2+1]);
        if (hi<0 || lo<0) return 0;
        out[i] = (uint8_t)((hi<<4)|lo);
    }
    return 1;
}

static uint8_t try_handle_uart6_local_cmd(const char *line)
{
    char tmp[RX_BUFFER_SIZE];
    size_t len;
    char msg[96];

    if (line == NULL) return 0;

    len = strlen(line);
    if (len >= sizeof(tmp)) len = sizeof(tmp) - 1U;
    memcpy(tmp, line, len);
    tmp[len] = '\0';
    rstrip_inplace(tmp);

    if (strcmp(tmp, "SNAP?") == 0) {
        snprintf(msg, sizeof(msg),
                 "[UART SNAP] enable=%u interval=%u sec (%lu ms)\r\n",
                 (unsigned)g_node_cfg.snap_enable,
                 (unsigned)g_node_cfg.snap_interval_sec,
                 (unsigned long)snapshot_interval_ms());
        debug6(msg);
        return 1;
    }

    if (strncmp(tmp, "SNAP=", 5) == 0) {
        return apply_snapshot_uart_cmd(tmp + 5);
    }

    return 0;
}

static void nodeinfo_cache_update_from_kv(const char *kv_line)
{
    const char *p = kv_line;
    if (p[0]=='A' && p[1]=='T' && p[2]=='+') p += 3;

    if (strncmp(p, "DEV=", 4) == 0) {
        g_node_info.dev = (uint8_t)atoi(p+4);
    } else if (strncmp(p, "DSP=", 4) == 0) {
        g_node_info.dsp = (uint8_t)atoi(p+4);
    } else if (strncmp(p, "TXP=", 4) == 0) {
        g_node_info.txp = (uint8_t)atoi(p+4);
    } else if (strncmp(p, "MODE=", 5) == 0) {
        g_node_info.mode = (uint8_t)atoi(p+5);
    } else if (strncmp(p, "RCH=", 4) == 0) {
        int a=-1,b=-1;
        if (sscanf(p+4, "%d,%d", &a, &b) >= 1) {
            g_node_info.rch0 = (uint8_t)a;
            g_node_info.rch1 = (b>=0) ? (uint8_t)b : 0xFF;
        }
    } else if (strncmp(p, "MAC=", 4) == 0) {
        // "00124B002D441B87" -> 8바이??
        parse_hex8(p+4, g_node_info.mac);   
    } else if (strncmp(p, "FWVER=", 6) == 0) {
        int maj=0, min=0;
        if (sscanf(p+6, "%d.%d", &maj, &min) == 2) {
            g_node_info.fw_major = (uint16_t)maj;
            g_node_info.fw_minor = (uint16_t)min;
        }
    }
}

static inline void DWT_CYCCNT_Init(void)
{
	// Trace enable
	    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	    
	#if defined (DWT_LAR)
	    DWT->LAR = 0xC5ACCE55;
	#endif

	    // CYCCNT reset + enable
	    DWT->CYCCNT = 0;
	    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	    
	    __DSB(); __ISB();
}

static float compute_supersonic_rms_from_fftdata(const FftData_t* arr, uint16_t n)
{
    if (!arr || n == 0) return 0.0f;

    float sumsq = 0.0f;
    uint16_t cnt = 0;

    for (uint16_t i = 0; i < n; ++i) {
        float f = arr[i].freq;
        if (f >= SUP_MIN_HZ && f <= SUP_MAX_HZ) {
            float a = arr[i].amplitude; 
            cnt++;
        }
    }
    if (cnt == 0) return 0.0f;
    return sqrtf(sumsq / (float)cnt);  // RMS
}


void update_sun_times(void)
{    
    if (g_rtc_year < 2020 || g_rtc_year > 2100)
        return;
    if (g_rtc_month == 0 || g_rtc_month > 12)
        return;
    if (g_rtc_day == 0 || g_rtc_day > 31)
        return;

    if (g_node_cfg.coord_enable) {
        uint8_t apply_coord_type = (g_node_cfg.apply_coord_type == 1u) ? 1u : 0u;
        int32_t apply_lat_e7 = apply_coord_type ? g_node_cfg.install_lat_e7 : g_node_cfg.standard_lat_e7;
        int32_t apply_lon_e7 = apply_coord_type ? g_node_cfg.install_lon_e7 : g_node_cfg.standard_lon_e7;

        if (!coord_e7_valid(apply_lat_e7, apply_lon_e7)) {
            g_node_cfg.coord_enable = 0u;
        } else {
            if (g_rtc_year   == g_last_sun_year   &&
                g_rtc_month  == g_last_sun_month  &&
                g_rtc_day    == g_last_sun_day    &&
                g_last_sun_region == 0xFEu) {
                return;
            }

            int sr = 0, ss = 0, dawn = 0, dusk = 0;
            double lat = (double)apply_lat_e7 / 10000000.0;
            double lon = (double)apply_lon_e7 / 10000000.0;

            compute_sun_times_latlon(
                g_rtc_year,
                g_rtc_month,
                g_rtc_day,
                lat,
                lon,
                &sr, &ss, &dawn, &dusk
            );

            sr   = norm_min(sr);
            ss   = norm_min(ss);
            dawn = norm_min(dawn);
            dusk = norm_min(dusk);

            g_sunrise_min = (uint16_t)sr;
            g_sunset_min  = (uint16_t)ss;
            g_dawn_min    = (uint16_t)dawn;
            g_dusk_min    = (uint16_t)dusk;

            g_last_sun_year   = g_rtc_year;
            g_last_sun_month  = g_rtc_month;
            g_last_sun_day    = g_rtc_day;
            g_last_sun_region = 0xFEu;

            if (!focus_timing_log_enabled()) {
                char buf[128];
                int len = snprintf(buf, sizeof(buf),
                    "[SUN_COORD] %04u-%02u-%02u coord_type=%u lat=%ld lon=%ld SR=%u SS=%u DAWN=%u DUSK=%u\r\n",
                    g_rtc_year, g_rtc_month, g_rtc_day,
                    (unsigned)apply_coord_type,
                    (long)apply_lat_e7,
                    (long)apply_lon_e7,
                    g_sunrise_min, g_sunset_min, g_dawn_min, g_dusk_min);
                if (len > 0) {
                    HAL_UART_Transmit(&huart6, (uint8_t*)buf, (uint16_t)len, HAL_MAX_DELAY);
                }
            }
            return;
        }
    }

    if (rtc_apply_gateway_sun_times_if_current()) {
        uint8_t sun_gw_should_log =
            (g_rtc_year   != g_last_sun_year) ||
            (g_rtc_month  != g_last_sun_month) ||
            (g_rtc_day    != g_last_sun_day) ||
            (g_region_code != g_last_sun_region);

        g_last_sun_year   = g_rtc_year;
        g_last_sun_month  = g_rtc_month;
        g_last_sun_day    = g_rtc_day;
        g_last_sun_region = g_region_code;

        if (sun_gw_should_log && !focus_timing_log_enabled()) {
            char buf[112];
            int len = snprintf(buf, sizeof(buf),
                "[SUN_GW] %04u-%02u-%02u region=%u SR=%u SS=%u DAWN=%u DUSK=%u\r\n",
                g_rtc_year, g_rtc_month, g_rtc_day, g_region_code,
                g_sunrise_min, g_sunset_min, g_dawn_min, g_dusk_min);
            if (len > 0) {
                HAL_UART_Transmit(&huart6, (uint8_t*)buf, (uint16_t)len, HAL_MAX_DELAY);
            }
        }
        return;
    }
 
    if (g_rtc_year   == g_last_sun_year   &&
        g_rtc_month  == g_last_sun_month  &&
        g_rtc_day    == g_last_sun_day    &&
        g_region_code == g_last_sun_region) {
        return;
    }

    int sr = 0, ss = 0, dawn = 0, dusk = 0;

    compute_sun_times(
        g_rtc_year,
        g_rtc_month,
        g_rtc_day,
        g_region_code,
        &sr, &ss, &dawn, &dusk
    );

    sr   = norm_min(sr);
	ss   = norm_min(ss);
	dawn = norm_min(dawn);
	dusk = norm_min(dusk);

    g_sunrise_min = (uint16_t)sr;
    g_sunset_min  = (uint16_t)ss;
    g_dawn_min    = (uint16_t)dawn;
    g_dusk_min    = (uint16_t)dusk;

    g_last_sun_year   = g_rtc_year;
    g_last_sun_month  = g_rtc_month;
    g_last_sun_day    = g_rtc_day;
    g_last_sun_region = g_region_code;

    if (!focus_timing_log_enabled()) {
        char buf[96];
        int len = snprintf(buf, sizeof(buf),
            "[SUN] %04u-%02u-%02u region=%u SR=%u SS=%u DAWN=%u DUSK=%u\r\n",
            g_rtc_year, g_rtc_month, g_rtc_day, g_region_code,
            g_sunrise_min, g_sunset_min, g_dawn_min, g_dusk_min);
        if (len > 0) {
            HAL_UART_Transmit(&huart6, (uint8_t*)buf, (uint16_t)len, HAL_MAX_DELAY);
        }
    }
}

void wisun_process_rx_mainloop(void)
{
    if (!wisun_packet_ready) return;
    
    uint16_t packet_len = 0;
    uint32_t ready_tick = 0;
    uint8_t  buf[PACKET_MAX_SIZE];

    __disable_irq();
    packet_len = wisun_packet_len;
    ready_tick = wisun_packet_ready_tick;
    if (packet_len > 0 && packet_len <= PACKET_MAX_SIZE) {
        memcpy(buf, (void*)wisun_packet_shadow, packet_len);
    }
    wisun_packet_ready = false;
    __enable_irq();

    if (packet_len == 0 || packet_len > PACKET_MAX_SIZE) return;
    
    // PrintReceivedPacket("Receive Packet : ", buf, packet_len);

    wisun_frame_view_t v;
    if (!wisun_parse_frame(buf, packet_len, &v)) {
        // dbg_dump_uart6_with_tag("[RX_BAD_FRAME]", buf, packet_len);
        return;
    }

    uint16_t src_mid = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

    uint16_t target_mid = 0u;

    uint8_t  ttl   = 0;
    uint8_t  cmd   = 0;
    uint8_t  flags = 0;
    uint16_t msg_id = 0;

    if (is_compact_snap_body(v.data, v.data_len)) {
        uint8_t compact_ttl = v.data[SNAP_COMPACT_TTL_IDX];
        uint16_t compact_snap_count = compact_snap_count16(v.data);
        uint32_t compact_key = compact_snap_seen_key(v.data);

        if (compact_snap_is_own_uid(v.data)) {
            timing_log("[HOP_COMPACT_DROP_OWN] src=0x%04X snap=%u len=%u\r\n",
                       (unsigned)src_mid,
                       (unsigned)compact_snap_count,
                       (unsigned)v.data_len);
            return;
        }

        if (compact_ttl == 0u) {
            dbg_dump_uart6_with_tag("[HOP_COMPACT_DROP_TTL0]", v.data, v.data_len);
            return;
        }

        if (hop_seen_key(compact_key)) {
            dbg_dump_uart6_with_tag("[HOP_COMPACT_DROP_DUP]", v.data, v.data_len);
            return;
        }

        uint32_t now = HAL_GetTick();
        uint32_t base_delay = 5u + (my_mid % 5u);
        uint32_t jitter = xorshift16((uint16_t)(my_mid ^ (uint16_t)now ^ compact_snap_count)) % 40u;
        uint8_t enqueued = 0u;

        for (int qi = 0; qi < HOP_QUEUE_SIZE; ++qi) {
            if (g_hop_q[qi].in_use) continue;

            g_hop_q[qi].tmid = 0x0000u;
            g_hop_q[qi].compact_snap = 1u;
            g_hop_q[qi].data_len = v.data_len;
            memcpy(g_hop_q[qi].data, v.data, v.data_len);
            g_hop_q[qi].due_tick = now + base_delay + jitter;
            g_hop_q[qi].in_use = 1u;
            hop_mark_key(compact_key);
            enqueued = 1u;
            break;
        }

        if (!enqueued) {
            dbg_dump_uart6_with_tag("[HOP_COMPACT_DROP_QFULL]", v.data, v.data_len);
        } else {
            timing_log("[HOP_COMPACT_ENQ] src=0x%04X snap=%u ttl=%u len=%u\r\n",
                       (unsigned)src_mid,
                       (unsigned)compact_snap_count,
                       (unsigned)compact_ttl,
                       (unsigned)v.data_len);
        }
        return;
    }

    if (v.data_len >= 7U) {
        target_mid = (uint16_t)v.data[0] | ((uint16_t)v.data[1] << 8);
        ttl    = v.data[2];
        cmd    = v.data[3];
        flags  = v.data[4];
        msg_id = ((uint16_t)v.data[5] << 8) | v.data[6];
    } else {
        // dbg_dump_uart6_with_tag("[RX_SHORT]", v.data, v.data_len);
        return;
    }
    
    /* if (!focus_timing_log_enabled()) {
        char dbg[96];
        snprintf(dbg, sizeof(dbg),
                 "[MID_DBG] my_mid=0x%04X src_mid=0x%04X tgt_mid=0x%04X ttl=%u cmd=0x%02X\r\n",
                 my_mid, src_mid, target_mid, ttl, cmd);
        HAL_UART_Transmit(&huart6, (uint8_t*)dbg, strlen(dbg), 100);

        dbg_print_mid_info("[MID_CHECK]", my_mid, target_mid);
    } */

    
    if (target_mid == 0x0000 || target_mid == my_mid) {

        if (is_uplink_report_cmd(cmd)) {
            timing_log("[DROP_UPLINK_REPORT_ON_NODE] src=0x%04X target=0x%04X ttl=%u cmd=0x%02X msg=%u len=%u\r\n",
                    (unsigned)src_mid,
                    (unsigned)target_mid,
                    (unsigned)ttl,
                    (unsigned)cmd,
                    (unsigned)msg_id,
                    (unsigned)packet_len);
            return;
        }

        wisun_activity_mark(HAL_GetTick());
        dbg_dump_uart6_with_tag("[RX_FOR_ME]", buf, packet_len);
        timing_log("[TLOG_RX_FOR_ME] t=%lu isr_t=%lu loop_delay=%lu src=0x%04X target=0x%04X ttl=%u cmd=0x%02X flags=0x%02X msg=%u len=%u\r\n",
                (unsigned long)HAL_GetTick(),
                (unsigned long)ready_tick,
                (unsigned long)(HAL_GetTick() - ready_tick),
                (unsigned)src_mid,
                (unsigned)target_mid,
                (unsigned)ttl,
                (unsigned)cmd,
                (unsigned)flags,
                (unsigned)msg_id,
                (unsigned)packet_len);

        handle_binary_cmd(
            cmd,
            flags,
            msg_id,
            src_mid,
            target_mid,
            (v.data_len > 7U) ? &v.data[7] : NULL,
            (v.data_len > 7U) ? (uint16_t)(v.data_len - 7U) : 0U
        );

        return;
    }

    dbg_dump_uart6_with_tag("[HOP_UNI_ENQ]", buf, packet_len);

    uint32_t now        = HAL_GetTick();
    uint32_t base_delay = 5 + (my_mid % 5);
    uint32_t jitter     = xorshift16(my_mid ^ HAL_GetTick()) % 40;

    bool drop_for_forward = false;
    uint16_t tgt_mid = target_mid;
    uint8_t  cmd2 = cmd;
    uint16_t msg_id2 = msg_id;

    if (v.data_len < 7U) {
        dbg_dump_uart6_with_tag("[HOP_DROP_SHORT]", v.data, v.data_len);
        drop_for_forward = true;
    }

    if (!drop_for_forward) {
        if (!node_is_provisioned() && !is_bootstrap_cmd(cmd2)) {
            dbg_dump_uart6_with_tag("[HOP_DROP_UNPROV]", v.data, v.data_len);
            drop_for_forward = true;
        }
    }

    if (!drop_for_forward) {
        uint32_t key = ((uint32_t)cmd2 << 24) ^
               ((uint32_t)tgt_mid << 8) ^
               (uint32_t)msg_id2;
        if (hop_seen_key(key)) {
            dbg_dump_uart6_with_tag("[HOP_DROP_DUP_KEY]", v.data, v.data_len);
            drop_for_forward = true;
        } else {
            hop_mark_key(key);
        }
    }

    if (!drop_for_forward) {
    uint8_t enqueued = 0;
    uint8_t dropped_ttl0 = 0;

    for (int qi = 0; qi < HOP_QUEUE_SIZE; ++qi) {
        if (!g_hop_q[qi].in_use) {
            if (v.data_len > HOP_MAX_FRAME) {
                break; // 너무 길면 버림
            }

            // TTL=0 forward 금지
            if (v.data_len >= 3U) {
                uint8_t ttl0 = v.data[2];
                if (ttl0 == 0U) {
                    dbg_dump_uart6_with_tag("[HOP_DROP_RX_TTL0]", v.data, v.data_len);
                    dropped_ttl0 = 1;
                    break;
                }
            }

            g_hop_q[qi].tmid     = v.tmid;
            g_hop_q[qi].compact_snap = 0u;
            g_hop_q[qi].data_len = v.data_len;
            memcpy(g_hop_q[qi].data, v.data, v.data_len);

            g_hop_q[qi].due_tick = now + base_delay + jitter;
            g_hop_q[qi].in_use   = 1;
            enqueued = 1;
            break;
        }
    }

    if (!enqueued && !dropped_ttl0) {
        dbg_dump_uart6_with_tag("[HOP_DROP_QFULL]", v.data, v.data_len);
    }
}
}

void snapshot_suppress_next_tx(void)
{
    __disable_irq();
    g_snapshot_suppress_next_tx = 1u;
    __enable_irq();
}

void Send_Monitoring_Snapshot_JSON(uint16_t req_msg_id)
{
    uint8_t suppress_tx = 0u;

    __disable_irq();
    suppress_tx = g_snapshot_suppress_next_tx;
    g_snapshot_suppress_next_tx = 0u;
    __enable_irq();

    if (req_msg_id == 0u && !node_is_provisioned()) {
        uart6_log(
            "[SNAP_SKIP] reason=not_provisioned req_msg_id=%u my_mid=%u\r\n",
            (unsigned)req_msg_id,
            (unsigned)my_mid
        );
        return;
    }

    if (!ultra_frame_ready) {
        uart6_log(
            "[SNAP_SKIP] reason=no_ultra_frame req_msg_id=%u paused=%u my_mid=%u\r\n",
            (unsigned)req_msg_id,
            (unsigned)(ultra_sampling_paused ? 1u : 0u),
            (unsigned)my_mid
        );

        if (ultra_sampling_paused) {
            Ultra_StartDmaFrame();
            ultra_sampling_paused = 0;
        }
        return;
    }

    __disable_irq();
    ultra_frame_ready = false;
    __enable_irq();

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_TIM_Base_Stop(&htim6);
    ultra_sampling_paused = 1;

    /* ===== UID ===== */
    uint8_t uid12[12];
    mid_pack_uid12(uid12);

    /* ===== 전압/전류 측정 ===== */
    VIRead vi;
    float vin_v = 0.0f;
    float i_adc_v = 0.0f;

    if (AD_DC_Injected_Once(&vi) == HAL_OK) {
        vin_v = adc_raw_to_dc_vin(vi.volt_raw);
        i_adc_v = adc_raw_to_dc_current(vi.curr_raw);
    }

    /* ===== 온도 측정 ===== */
    uint16_t temp_raw = 0u;
    float temp_v = 0.0f;
    float temp_c = NTC_INVALID_TEMP_C;
    uint8_t temp_ok = 0u;

    if (HAL_ADC_Start(&hadc2) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
            temp_raw = HAL_ADC_GetValue(&hadc2);
            temp_v = (float)temp_raw * 3.3f / 4095.0f;
            temp_ok = ntc_voltage_to_temp_c_checked(temp_v, &temp_c);
        }
        HAL_ADC_Stop(&hadc2);
    }

    uart6_log(
        "[TEMP_READ_SNAP] raw=%u v=%f c=%f ok=%u\r\n",
        (unsigned)temp_raw,
        temp_v,
        temp_c,
        (unsigned)temp_ok
    );

    /* ===== FFT 처리 ===== */
    float fft_freq[SNAP_FFT_PAIRS] = {0};
    float fft_amp[SNAP_FFT_PAIRS] = {0};
    uint8_t fft_cnt = 0u;

    float max_amp = -1.0f;
    float peak_f = 0.0f;
    uint16_t peak_i = 0u;
    uint8_t found = 0u;
    float supersonic_val = 0.0f;
    uint8_t ultra_signal_ok = 0u;
    uint16_t adc_span = 0u;

    float snap_fs_hz = tim6_sample_rate_hz();
    if (snap_fs_hz <= 1.0f || !isfinite(snap_fs_hz)) {
        snap_fs_hz = (float)FSAMPLE;
    }

    {
        static float32_t local_in[FFT_SIZE];
        static float32_t local_mag[FFT_SIZE / 2];

        uint16_t adc_min = 0xFFFFu;
        uint16_t adc_max = 0u;
        uint16_t adc_first = 0u;
        uint16_t adc_mid = 0u;
        uint16_t adc_last = 0u;
        uint32_t adc_sum = 0u;
        uint32_t adc_avg = 0u;

        __disable_irq();

        for (int i = 0; i < FFT_SIZE; ++i) {
            uint16_t raw = raw_buffer[i];

            if (i == 0) {
                adc_first = raw;
            }
            if (i == (FFT_SIZE / 2)) {
                adc_mid = raw;
            }
            if (i == (FFT_SIZE - 1)) {
                adc_last = raw;
            }

            if (raw < adc_min) {
                adc_min = raw;
            }
            if (raw > adc_max) {
                adc_max = raw;
            }

            adc_sum += raw;
            local_in[i] =
                ((float32_t)raw * 3.3f / 4095.0f) - 1.65f;
        }

        __enable_irq();

        adc_avg = adc_sum / (uint32_t)FFT_SIZE;
        adc_span = (uint16_t)(adc_max - adc_min);

        ultra_signal_ok =
            (adc_span >= SNAP_FFT_ADC_RAW_SPAN_MIN) ? 1u : 0u;

        uart6_log(
            "[SNAP_ADC_RAW] min=%u max=%u span=%u avg=%lu "
            "first=%u mid=%u last=%u dma_done=%lu dma_half=%lu\r\n",
            (unsigned)adc_min,
            (unsigned)adc_max,
            (unsigned)adc_span,
            (unsigned long)adc_avg,
            (unsigned)adc_first,
            (unsigned)adc_mid,
            (unsigned)adc_last,
            (unsigned long)g_dma_done,
            (unsigned long)g_dma_half
        );

        uart6_log(
            "[SNAP_ULTRA_SIGNAL] span=%u span_min=%u ok=%u\r\n",
            (unsigned)adc_span,
            (unsigned)SNAP_FFT_ADC_RAW_SPAN_MIN,
            (unsigned)ultra_signal_ok
        );

        ExtractFullFFT_MagOnly(local_in, local_mag);

        for (int i = 0; i < FFT_SIZE / 2; ++i) {
            float mag = local_mag[i];

            if (!isfinite(mag)) {
                mag = 0.0f;
            }

            fft_packet[i].freq =
                (float)i * snap_fs_hz / (float)FFT_SIZE;
            fft_packet[i].amplitude = mag;
        }

        supersonic_val = compute_supersonic_rms_from_fftdata(
            fft_packet,
            FFT_SIZE / 2
        );

        {
            uint16_t nbins = FFT_SIZE / 2;

            for (uint16_t b = 1u; b < nbins; ++b) {
                float freq =
                    (float)b * snap_fs_hz / (float)FFT_SIZE;
                float amp = fft_packet[b].amplitude;

                if (!isfinite(amp)) {
                    amp = 0.0f;
                }

                if (freq >= 80000.0f && freq <= 125000.0f) {
                    if (amp > max_amp) {
                        max_amp = amp;
                        peak_f = freq;
                        peak_i = b;
                        found = 1u;
                    }
                }
            }
        }

        if (found &&
            peak_i >= 2u &&
            peak_i <= (uint16_t)((FFT_SIZE / 2) - 2)) {
            float a0 = fft_packet[peak_i - 1u].amplitude;
            float a1 = fft_packet[peak_i].amplitude;
            float a2 = fft_packet[peak_i + 1u].amplitude;

            max_amp = sqrtf(a0 * a0 + a1 * a1 + a2 * a2);
        }

        {
            const float n = (float)FFT_SIZE;
            const float hann_cg = 0.5f;

            float vpk =
                (found && max_amp > 0.0f)
                    ? ((2.0f / (n * hann_cg)) * max_amp)
                    : 0.0f;

            float mvpk = vpk * 1000.0f;
            float mvpp = 2.0f * mvpk;
            float adc_pk = vpk * (4095.0f / 3.3f);

            uart6_log(
                "[SNAP_FFT_PEAK] mode=magonly found=%u peak_f=%f "
                "bin=%u max_amp=%f mvpk=%f mvpp=%f adc_pk=%f "
                "amp_min=%f span=%u fs=%f\r\n",
                (unsigned)found,
                peak_f,
                (unsigned)peak_i,
                max_amp,
                mvpk,
                mvpp,
                adc_pk,
                SNAP_FFT_VALID_AMP_MIN,
                (unsigned)adc_span,
                snap_fs_hz
            );
        }

        if (found &&
            ultra_signal_ok &&
            max_amp >= SNAP_FFT_VALID_AMP_MIN) {
            fft_freq[0] = peak_f;
            fft_amp[0] = max_amp;
            fft_cnt = 1u;
        } else {
            fft_freq[0] = 0.0f;
            fft_amp[0] = 0.0f;
            fft_cnt = 0u;

            uart6_log(
                "[SNAP_FFT_INVALID] found=%u span=%u span_min=%u "
                "peak_f=%f max_amp=%f amp_min=%f\r\n",
                (unsigned)found,
                (unsigned)adc_span,
                (unsigned)SNAP_FFT_ADC_RAW_SPAN_MIN,
                peak_f,
                max_amp,
                SNAP_FFT_VALID_AMP_MIN
            );
        }
    }

    bool light_on = light_is_on_logical() ? true : false;

    snapshot_push(
        light_on,
        vin_v,
        i_adc_v,
        temp_c,
        supersonic_val
    );

    /* ===== AI 추론 ===== */
    uint8_t snap_ai_valid = 0u;
    float snap_ai_mse = 0.0f;
    uint32_t snap_ai_mse_x1000000 = 0u;
    int snap_ai_pred = 0;

    if (fft_cnt > 0u) {
        float ai_features[AE_COLS] = {0};

        ai_features[AI_FEATURE_FREQ_KHZ_IDX] =
            fft_freq[0] * 0.001f;
        ai_features[AI_FEATURE_ADC_PK_IDX] =
            fft_amp[0];
        ai_features[AI_FEATURE_CURRENT_IDX] =
            i_adc_v;
        ai_features[AI_FEATURE_VIN_IDX] =
            vin_v;

        if (run_inference(
                ai_features,
                &snap_ai_mse,
                &snap_ai_pred
            ) == 0) {
            snap_ai_valid = 1u;
            snap_ai_mse_x1000000 =
                scale_ai_mse_x1000000(snap_ai_mse);

            ai_mse = snap_ai_mse;
            ai_pred = snap_ai_pred;
            ai_pending = 0u;
            ai_index = 0;
            ai_next_run = HAL_GetTick() + ai_period_ms;
        } else {
            uart6_log("[AI] snap inference failed\r\n");
        }
    }

    g_monitor_count++;

    light_sensor_cache_update(
        vin_v,
        i_adc_v,
        temp_c,
        fft_cnt,
        fft_freq,
        fft_amp,
        g_monitor_count,
        light_on ? 1u : 0u
    );

    if (suppress_tx) {
        uart6_log(
            "[SNAP_TX_SUPPRESS] reason=light_event_measure "
            "snap_count=%lu light_on=%u fft_count=%u mid=%u\r\n",
            (unsigned long)g_monitor_count,
            (unsigned)(light_on ? 1u : 0u),
            (unsigned)fft_cnt,
            (unsigned)my_mid
        );

        Ultra_StartDmaFrame();
        g_last_has_msg_id = 0u;
        g_last_has_cmd = 0u;
        return;
    }

#if SNAP_COMPACT_DIRECT_ENABLE
    {
        uint16_t tx_msg_id =
            (req_msg_id != 0u)
                ? req_msg_id
                : (uint16_t)(g_monitor_count & 0xFFFFu);

        uint16_t snap_tmid = 0u;
        uint8_t snap_body[SNAP_COMPACT_BODY_LEN];
        uint16_t snap_body_len = 0u;

        uint8_t local_fft_cnt;
        float local_fft0_freq = 0.0f;
        float local_fft0_amp = 0.0f;
        uint32_t local_monitor_count;

        uint8_t control_mode;
        uint16_t control_on_min;
        uint16_t control_off_min;

        if (tx_msg_id == 0u) {
            tx_msg_id = 1u;
        }

        if (req_msg_id != 0u && g_resp_slot.tmid != 0u) {
            snap_tmid = g_resp_slot.tmid;
        }

        __disable_irq();

        local_fft_cnt = fft_cnt;

        if (local_fft_cnt > 0u) {
            local_fft0_freq = fft_freq[0];
            local_fft0_amp = fft_amp[0];
        }

        local_monitor_count = g_monitor_count;

        __enable_irq();

        uart6_log(
            "[SNAP_FFT_SRC] cnt=%u f0=%f a0=%f "
            "raw_f0=%lu raw_a0=%lu light=%u\r\n",
            (unsigned)local_fft_cnt,
            local_fft0_freq,
            local_fft0_amp,
            (unsigned long)(uint32_t)(local_fft0_freq * 100.0f),
            (unsigned long)(uint32_t)(local_fft0_amp * 1000.0f),
            (unsigned)(light_on ? 1u : 0u)
        );

        (void)light_control_get_effective_schedule(
            &control_mode,
            &control_on_min,
            &control_off_min
        );

        snap_body_len = encode_snap_compact_bin( snap_body, (uint16_t)sizeof(snap_body), uid12, vin_v, i_adc_v, temp_c, light_on ? 1u : 0u, local_fft_cnt > 0u ? 1u : 0u, local_fft0_freq, local_fft0_amp, local_monitor_count, snap_ai_valid, snap_ai_mse_x1000000, (int8_t)snap_ai_pred, 1u, SNAP_COMPACT_TTL_DEFAULT, control_mode, control_on_min, control_off_min );

        if (control_on_min == 0xFFFFu ||
            control_off_min == 0xFFFFu) {
            uart6_log(
                "[SNAP_CTRL] mode=%u on_min=%u off_min=%u "
                "schedule_valid=0\r\n",
                (unsigned)control_mode,
                (unsigned)control_on_min,
                (unsigned)control_off_min
            );
        } else {
            uart6_log(
                "[SNAP_CTRL] mode=%u "
                "on_min=%u on=%02u:%02u "
                "off_min=%u off=%02u:%02u "
                "schedule_valid=1\r\n",
                (unsigned)control_mode,
                (unsigned)control_on_min,
                (unsigned)(control_on_min / 60u),
                (unsigned)(control_on_min % 60u),
                (unsigned)control_off_min,
                (unsigned)(control_off_min / 60u),
                (unsigned)(control_off_min % 60u)
            );
        }

        if (snap_body_len == 0u) {
            uart6_log(
                "[SNAP_SKIP] reason=compact_encode_fail "
                "req_msg_id=%u my_mid=%u\r\n",
                (unsigned)tx_msg_id,
                (unsigned)my_mid
            );

            Ultra_StartDmaFrame();
            g_last_has_msg_id = 0u;
            g_last_has_cmd = 0u;
            return;
        }

        uart6_log(
            "[SNAP_AFTER_BUILD] compact_body_len=%u "
            "tx_msg_id=%u frame_tmid=0x%04X\r\n",
            (unsigned)snap_body_len,
            (unsigned)tx_msg_id,
            (unsigned)snap_tmid
        );

        dbg_dump_uart6_with_tag(
            "[SNAP_COMPACT_BODY]",
            snap_body,
            snap_body_len
        );

        {
            bool snap_sent = send_wisun_binary(
                snap_tmid,
                snap_body,
                snap_body_len
            );

            if (snap_sent) {
                snapshot_mark_tx(
                    (req_msg_id != 0u) ? 1u : 0u
                );
            }
        }

        uart6_log(
            "[SNAP_TX] req_msg_id=%lu snap_count=%lu "
            "light_on=%u fft_count=%u mid=%u "
            "frame_tmid=0x%04X compact=1\r\n",
            (unsigned long)tx_msg_id,
            (unsigned long)g_monitor_count,
            (unsigned)(light_on ? 1u : 0u),
            (unsigned)fft_cnt,
            (unsigned)my_mid,
            (unsigned)snap_tmid
        );

        Ultra_StartDmaFrame();
        g_last_has_msg_id = 0u;
        g_last_has_cmd = 0u;
        return;
    }
#endif

    /* ===== Legacy/non-compact SNAP ===== */
    {
        uint16_t tx_msg_id =
            (req_msg_id != 0u)
                ? req_msg_id
                : (uint16_t)(g_monitor_count & 0xFFFFu);

        uint16_t snap_tmid = 0u;
        uint8_t snap_enqueued = 0u;
        uint8_t use_legacy_struct_once = 0u;

        uint8_t snap_body[SNAP_BIN_BODY_LEN];
        uint16_t snap_body_len = 0u;

        uint8_t snap_tx_dump[HOP_MAX_FRAME];
        uint16_t snap_tx_dump_len = 0u;

        if (tx_msg_id == 0u) {
            tx_msg_id = 1u;
        }

        if (req_msg_id != 0u && g_resp_slot.tmid != 0u) {
            snap_tmid = g_resp_slot.tmid;
        }

        if (SNAP_USE_LEGACY_STRUCT_TEST) {
            SnapBin_t snap;

            memset(&snap, 0, sizeof(snap));

            snap.t = 0x01u;
            memcpy(snap.uid, uid12, sizeof(snap.uid));

            snap.volt = snap_round_4dp(vin_v);
            snap.curr = snap_round_4dp(i_adc_v);
            snap.temp = snap_round_4dp(temp_c);
            snap.light_on = light_on ? 1u : 0u;
            snap.fft_count = fft_cnt;

            for (uint8_t k = 0u; k < SNAP_FFT_PAIRS; ++k) {
                if (k < fft_cnt) {
                    snap.fft[k].freq_x100 =
                        scale_fft_freq_x100(fft_freq[k]);

                    snap.fft[k].amp_x1000 =
                        scale_fft_amp_x1000(fft_amp[k]);
                }
            }

            snap.snap_count = g_monitor_count;
            snap.msg_id = (uint32_t)tx_msg_id;
            snap.ai_valid = snap_ai_valid;
            snap.ai_mse_x1000000 =
                snap_ai_mse_x1000000;
            snap.ai_pred = (int8_t)snap_ai_pred;
            snap.ok = 1u;
            snap.err_code = 0;

            memcpy(snap_body, &snap, sizeof(snap));
            snap_body_len = (uint16_t)sizeof(snap);
            use_legacy_struct_once = 1u;

            uart6_log(
                "[SNAP_SIZE] sizeof(SnapBin_t)=%u body_len=%u\r\n",
                (unsigned)sizeof(SnapBin_t),
                (unsigned)snap_body_len
            );

            uart6_log(
                "[SNAP_SCALED] fft_freq0=%f fft_amp0=%f "
                "snap_freq0_x100=%lu snap_amp0_x1000=%ld\r\n",
                fft_freq[0],
                fft_amp[0],
                (unsigned long)snap.fft[0].freq_x100,
                (long)snap.fft[0].amp_x1000
            );

            dbg_dump_uart6_with_tag(
                "[SNAP_BODY_HEX]",
                snap_body,
                snap_body_len
            );
        } else {
            float local_fft_freq[SNAP_FFT_PAIRS];
            float local_fft_amp[SNAP_FFT_PAIRS];

            uint8_t local_fft_cnt;
            uint32_t local_monitor_count;
            uint32_t local_msg_id;

            memset(
                local_fft_freq,
                0,
                sizeof(local_fft_freq)
            );

            memset(
                local_fft_amp,
                0,
                sizeof(local_fft_amp)
            );

            __disable_irq();

            local_fft_cnt = fft_cnt;

            if (local_fft_cnt > SNAP_FFT_PAIRS) {
                local_fft_cnt = SNAP_FFT_PAIRS;
            }

            for (uint8_t k = 0u; k < local_fft_cnt; ++k) {
                local_fft_freq[k] = fft_freq[k];
                local_fft_amp[k] = fft_amp[k];
            }

            local_monitor_count = g_monitor_count;
            local_msg_id = (uint32_t)tx_msg_id;

            __enable_irq();

            uart6_log(
                "[SNAP_FFT_SRC] cnt=%u f0=%f a0=%f "
                "raw_f0=%lu raw_a0=%lu light=%u\r\n",
                (unsigned)local_fft_cnt,
                local_fft_freq[0],
                local_fft_amp[0],
                (unsigned long)(uint32_t)(
                    local_fft_freq[0] * 100.0f
                ),
                (unsigned long)(uint32_t)(
                    local_fft_amp[0] * 1000.0f
                ),
                (unsigned)(light_on ? 1u : 0u)
            );

            snap_body_len = encode_snap_bin(
                snap_body,
                (uint16_t)sizeof(snap_body),
                uid12,
                vin_v,
                i_adc_v,
                temp_c,
                light_on ? 1u : 0u,
                local_fft_cnt,
                local_fft_freq,
                local_fft_amp,
                local_monitor_count,
                local_msg_id,
                1u,
                0,
                snap_ai_valid,
                snap_ai_mse_x1000000,
                (int8_t)snap_ai_pred
            );
        }

        if (snap_body_len == 0u) {
            uart6_log(
                "[SNAP_SKIP] reason=encode_fail "
                "req_msg_id=%u my_mid=%u\r\n",
                (unsigned)tx_msg_id,
                (unsigned)my_mid
            );

            Ultra_StartDmaFrame();
            g_last_has_msg_id = 0u;
            g_last_has_cmd = 0u;
            return;
        }

        snap_tx_dump_len = build_transport_payload(
            0x0000u,
            0u,
            SNAP_REPORT_CMD,
            0u,
            tx_msg_id,
            snap_body,
            snap_body_len,
            snap_tx_dump,
            (uint16_t)sizeof(snap_tx_dump)
        );

        uart6_log(
            "[SNAP_AFTER_BUILD] body_len=%u tx_len=%u "
            "tx_msg_id=%u\r\n",
            (unsigned)snap_body_len,
            (unsigned)snap_tx_dump_len,
            (unsigned)tx_msg_id
        );

        if (snap_tmid != 0u) {
            snap_enqueued = enqueue_transport_tx(
                snap_tmid,
                SNAP_REPORT_CMD,
                0u,
                tx_msg_id,
                snap_body,
                snap_body_len,
                HOP_TTL_DEFAULT
            );

            if (snap_enqueued) {
                snap_tx_dump_len = build_transport_payload(
                    snap_tmid,
                    HOP_TTL_DEFAULT,
                    SNAP_REPORT_CMD,
                    0u,
                    tx_msg_id,
                    snap_body,
                    snap_body_len,
                    snap_tx_dump,
                    (uint16_t)sizeof(snap_tx_dump)
                );
            }
        }

        uart6_log(
            "[SNAP_TX] req_msg_id=%lu snap_count=%lu "
            "light_on=%u fft_count=%u mid=%u route=0x%04X "
            "via=%s legacy_struct=%u\r\n",
            (unsigned long)tx_msg_id,
            (unsigned long)g_monitor_count,
            (unsigned)(light_on ? 1u : 0u),
            (unsigned)fft_cnt,
            (unsigned)my_mid,
            (unsigned)snap_tmid,
            snap_enqueued ? "hop" : "direct",
            (unsigned)use_legacy_struct_once
        );

        if (!snap_enqueued) {
            if (send_transport_direct(
                    0x0000u,
                    0u,
                    SNAP_REPORT_CMD,
                    0u,
                    tx_msg_id,
                    snap_body,
                    snap_body_len
                )) {
                snapshot_mark_tx(
                    (req_msg_id != 0u) ? 1u : 0u
                );
            }
        } else {
            snapshot_mark_tx(
                (req_msg_id != 0u) ? 1u : 0u
            );
        }

        Ultra_StartDmaFrame();

        g_last_has_msg_id = 0u;
        g_last_has_cmd = 0u;
    }
}

static void schedule_resp_with_slot(resp_kind_t kind, uint16_t tmid, uint16_t msg_id, const uint8_t *raw, uint16_t raw_len)
{
    uint32_t now = HAL_GetTick();

    const uint8_t  SLOT_COUNT    = 8;
    const uint32_t SLOT_LEN_MS   = 150;
    const uint32_t BASE_DELAY_MS = 100;
    const uint32_t JITTER_MS     = 100;

    uint16_t seed = my_mid ^ msg_id;
    if (seed == 0u) seed = my_mid ^ 0xACE1u;

    uint8_t slot_idx = (uint8_t)(my_mid % SLOT_COUNT);
    uint16_t r       = xorshift16(seed);
    uint32_t jitter  = (JITTER_MS > 0u) ? (r % JITTER_MS) : 0u;

    uint32_t delay_ms =
        BASE_DELAY_MS +
        (uint32_t)slot_idx * SLOT_LEN_MS +
        jitter;

    if (kind == RESP_KIND_LIGHT_ACK) {
        delay_ms = 0u;
    }

    uint16_t cap = (uint16_t)sizeof(g_resp_q[0].buf);

    int free_idx = -1;
    int snap_idx = -1;
    uint8_t replacing_snap = 0;

    __disable_irq();

    for (int i = 0; i < RESP_QUEUE_SIZE; ++i) {
        if (!g_resp_q[i].pending) {
            free_idx = i;
            break;
        }
        if (g_resp_q[i].kind == RESP_KIND_SNAP && snap_idx < 0) {
            snap_idx = i;
        }
    }

    if (free_idx < 0) {
        if (kind == RESP_KIND_SNAP) {
            __enable_irq();

            char msg[160];
            int n = snprintf(msg, sizeof(msg),
                             "[SLOT_DROP_BUSY] kind=%u tmid=%u msg_id=%u reason=qfull\r\n",
                             (unsigned)kind,
                             (unsigned)tmid,
                             (unsigned)msg_id);
            if (n > 0) {
                HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 100);
            }
            return;
        }

        if (snap_idx >= 0) {
            free_idx = snap_idx;
            replacing_snap = 1;
        } else {
            __enable_irq();

            char msg[160];
            int n = snprintf(msg, sizeof(msg),
                             "[SLOT_DROP_BUSY] kind=%u tmid=%u msg_id=%u reason=no_free_no_snap\r\n",
                             (unsigned)kind,
                             (unsigned)tmid,
                             (unsigned)msg_id);
            if (n > 0) {
                HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 100);
            }
            return;
        }
    }

    resp_slot_t *slot = &g_resp_q[free_idx];
    memset(slot, 0, sizeof(*slot));

    slot->pending  = 1u;
    slot->kind     = kind;
    slot->tmid     = tmid;
    slot->msg_id   = msg_id;
    slot->due_tick = now + delay_ms;

    if (raw != NULL && raw_len > 0u && raw_len <= sizeof(slot->buf)) {
        memcpy(slot->buf, raw, raw_len);
        slot->len         = raw_len;
        slot->has_raw_buf = 1u;
    } else {
        slot->len         = 0u;
        slot->has_raw_buf = 0u;
    }

    __enable_irq();

    char msg[192];
    int n = snprintf(msg, sizeof(msg),
                     replacing_snap ?
                     "[SLOT_REPLACE_SNAP] q=%d kind=%u raw_len=%u cap=%u tmid=%u msg_id=%u due=%lu\r\n" :
                     "[SLOT_ENQ] q=%d kind=%u raw_len=%u cap=%u tmid=%u msg_id=%u due=%lu\r\n",
                     free_idx,
                     (unsigned)kind,
                     (unsigned)raw_len,
                     (unsigned)cap,
                     (unsigned)tmid,
                     (unsigned)msg_id,
                     (unsigned long)(now + delay_ms));
    if (n > 0) {
        HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 100);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_GPDMA1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  DWT_CYCCNT_Init();
  MX_X_CUBE_AI_Init();
  HAL_UART_Receive_IT(&huart1, &rxByte1, 1);
  HAL_UART_Receive_IT(&huart6, &rxByte, 1);

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  //HAL_ADC_Start_IT(&hadc1);
  //Ultra_StartSampling();
  //HAL_ADC_Stop_IT(&hadc1);
  //HAL_ADC_Stop_DMA(&hadc1);
  //HAL_ADC_Stop(&hadc1);
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
   {
     Error_Handler();
   }
  //HAL_ADC_Start_IT(&hadc2);
  Ultra_StartDmaFrame();
  SCB->SHCSR &= ~(SCB_SHCSR_MEMFAULTENA_Msk);
        __DSB();
        __ISB();

  //if (ICACHE->CR & ICACHE_CR_EN)
	//  {
		  //ICACHE->CR &= ~ICACHE_CR_EN;  // ICACHE 비활
		  //__DSB();  // Data Synchronization Barrier
		  //__ISB();  // Instruction Synchronization Barrier
	  //}

        //Send_UID_UART2();
//        HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

    if (arm_rfft_fast_init_f32(&fftInstance, FFT_SIZE) != ARM_MATH_SUCCESS) {
        char msg[] = "FFT initializing failed\r\n";
        HAL_UART_Transmit(&huart6, (uint8_t*)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
    }
    HAL_TIM_Base_Start(&htim2); 
    init_uid_string();
    g_rtc_synced = 0u;
    uint16_t stored_mid = MID_INVALID;
    bool mid_loaded = load_mid_from_flash(&stored_mid);
    if (mid_loaded && stored_mid != MID_INVALID && stored_mid != 0) {
        my_mid = stored_mid;
        g_node_cfg.mid          = my_mid;
        g_node_cfg.mid_assigned = 1;
    } else {
        Query_MID_From_WiSUN();
    }
    if (!load_node_cfg_from_flash(&g_node_cfg)) {
           node_cfg_init_default(&g_node_cfg);
           (void)save_node_cfg_to_flash(&g_node_cfg);
       }

    if (g_node_cfg.mode > 3u) {
        g_node_cfg.mode = (g_node_cfg.on_off_mode <= 3u) ? g_node_cfg.on_off_mode : 1u;
        g_node_cfg.on_off_mode = g_node_cfg.mode;
        (void)save_node_cfg_to_flash(&g_node_cfg);
    } else if (g_node_cfg.on_off_mode > 3u) {
        g_node_cfg.on_off_mode = g_node_cfg.mode;
        (void)save_node_cfg_to_flash(&g_node_cfg);
    } else if (g_node_cfg.mode != g_node_cfg.on_off_mode) {
        uart6_log("[CFG_MODE_SYNC] mode=%u on_off_mode=%u use_on_off_mode=1\r\n",
                  (unsigned)g_node_cfg.mode,
                  (unsigned)g_node_cfg.on_off_mode);
        g_node_cfg.mode = g_node_cfg.on_off_mode;
        (void)save_node_cfg_to_flash(&g_node_cfg);
    }

#if USER_SCHED_LOCAL_TEST
    {
        RTC_TimeTypeDef test_time = {0};
        RTC_DateTypeDef test_date = {0};

        if (HAL_RTC_GetTime(&hrtc, &test_time, RTC_FORMAT_BIN) == HAL_OK &&
            HAL_RTC_GetDate(&hrtc, &test_date, RTC_FORMAT_BIN) == HAL_OK) {
            uint16_t now_min = (uint16_t)test_time.Hours * 60u + test_time.Minutes;
            uint16_t on_min = (uint16_t)((now_min + 1u) % 1440u);
            uint16_t off_min = (uint16_t)((now_min + 3u) % 1440u);

            g_node_cfg.mode = 2u;
            g_node_cfg.light_on_hour = (uint8_t)(on_min / 60u);
            g_node_cfg.light_on_min = (uint8_t)(on_min % 60u);
            g_node_cfg.light_off_hour = (uint8_t)(off_min / 60u);
            g_node_cfg.light_off_min = (uint8_t)(off_min % 60u);
            g_node_cfg.saving_mode = 0u;
            light_control_clear_manual_override();

            uart6_log("[USER_SCHED_LOCAL_TEST] now=%02u:%02u:%02u on=%02u:%02u off=%02u:%02u flash_save=0\r\n",
                      (unsigned)test_time.Hours,
                      (unsigned)test_time.Minutes,
                      (unsigned)test_time.Seconds,
                      (unsigned)g_node_cfg.light_on_hour,
                      (unsigned)g_node_cfg.light_on_min,
                      (unsigned)g_node_cfg.light_off_hour,
                      (unsigned)g_node_cfg.light_off_min);
        } else {
            uart6_log("[USER_SCHED_LOCAL_TEST] rtc_read_failed\r\n");
        }
    }
#endif

    apply_mid_chan_from_cfg();
    snapshot_reconfigure_timer_from_cfg();
    light_event_init();
    g_light_on = light_is_on_logical();
    printf("[MIDCH_BOOT] mid=0x%04X ch=%u,%u assigned=%u (src=%s)\r\n",
	   my_mid,  g_node_cfg.rch[0], g_node_cfg.rch[1], g_node_cfg.mid_assigned,
	   (mid_loaded && stored_mid != MID_INVALID && stored_mid != 0) ? "flash" : "wisun");

    //static uint32_t ai_last_infer_tick = 0;
    //static int      g_ai_last_result   = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t now = HAL_GetTick();
	  static uint32_t rtc_dbg_tick = 0;
	  /*static uint32_t t0 = 0, t1 = 0;
	  t0 = DWT->CYCCNT;
	  for (volatile int k=0; k<1000; k++) { __NOP(); }
	  t1 = DWT->CYCCNT;

	  char b[80];
	  int n = snprintf(b, sizeof(b), "[DWT] t0=%lu t1=%lu d=%lu\r\n",
	                   (unsigned long)t0, (unsigned long)t1, (unsigned long)(t1-t0));
	  HAL_UART_Transmit(&huart6, (uint8_t*)b, n, 20);*/

	         /* Debug FFT */

	         /*if (g_adc_kick && !ultra_sampling_paused) {
	             g_adc_kick = 0;
	             (void)HAL_ADC_Start_IT(&hadc1);
	         }*/

#if DEBUG_FFT_DATA_EXTRACT_MODE
	         if (ultra_frame_ready) {
	             __disable_irq();
	             ultra_frame_ready = 0;
	             __enable_irq();

	             HAL_ADC_Stop_DMA(&hadc1);
	             HAL_TIM_Base_Stop(&htim6);

	             ultra_sampling_paused = 1;

	             Debug_Print_FFT_Peak();
	         }

	         continue;
#else
			  /* if (ultra_frame_ready) {
				  __disable_irq();
				  ultra_frame_ready = 0;
				  __enable_irq();

				  HAL_ADC_Stop_DMA(&hadc1);
				  HAL_TIM_Base_Stop(&htim6);

				  ultra_sampling_paused = 1;

				  Debug_Print_FFT_Peak();

				  Ultra_StartDmaFrame();
			  } */
#endif

	         //===================== AT RX =====================
	         if (g_at_line_ready)
	         {
	             //HAL_UART_Transmit(&huart6, (uint8_t *)"[AT_LINE_READY]\r\n", 15, 50);
	             char     line_local[RX_BUFFER_SIZE];
	             uint16_t line_len;

	             __disable_irq();

	             line_len = g_at_line_len;
	             if (line_len >= RX_BUFFER_SIZE)
	                 line_len = RX_BUFFER_SIZE - 1;

	             memcpy(line_local, g_at_line, line_len);
	             line_local[line_len] = '\0';   // 직접 NULL 종료
	             g_at_line_ready = 0;

	             __enable_irq();

	             wisun_activity_mark(HAL_GetTick());

	             if (g_nodeinfo.pending)
	             {
	                 nodeinfo_collect_line(line_local);
	             }

	             Parse_AT_Response(line_local);

	        #ifdef DEBUG_AT_TO_PC
	             HAL_UART_Transmit(&huart6, (uint8_t *)line_local, line_len, 50);
	             HAL_UART_Transmit(&huart6, (uint8_t *)"\r\n", 2, 50);
	        #endif
	         }

	         // ===================== Node Info / Wi-SUN App =====================
	         nodeinfo_poll(now);
	         wisun_app_poll(now);
	         rtc_scheduler_poll(now);
	         wisun_idle_reset_poll(HAL_GetTick());
	         snapshot_poll(now,
	                       ultra_frame_ready ? 1u : 0u,
	                       ultra_sampling_paused ? 1u : 0u,
	                       g_last_light_control_tick);

	          //===================== RTC / SUN =====================
	         if ((uint32_t)(now - rtc_dbg_tick) >= 180000u) {
	             char rtc_dbg[256];
	             uint8_t mode_now = current_control_mode();
	             uint16_t now_min_dbg = (uint16_t)g_rtc_hour * 60u + g_rtc_min;
	             uint16_t on_min_dbg = 0u;
	             uint16_t off_min_dbg = 0u;
	             uint16_t saving_start_dbg = (uint16_t)g_node_cfg.saving_start_hour * 60u + g_node_cfg.saving_start_min;
	             uint16_t saving_end_dbg = (uint16_t)g_node_cfg.saving_end_hour * 60u + g_node_cfg.saving_end_min;
	             uint8_t saving_active_dbg = 0u;
	             uint8_t want_on_dbg = 0u;
	             uint8_t managed_dbg = 0u;

	             rtc_dbg_tick = now;

	             switch (mode_now) {
	             case 0:
	                 managed_dbg = 1u;
	                 on_min_dbg = apply_time_correction_min(g_sunset_min, g_node_cfg.on_corr_mode, g_node_cfg.on_corr_time_min);
	                 off_min_dbg = apply_time_correction_min(g_sunrise_min, g_node_cfg.off_corr_mode, g_node_cfg.off_corr_time_min);
	                 want_on_dbg = time_window_contains(now_min_dbg, on_min_dbg, off_min_dbg);
	                 break;
	             case 1:
	                 managed_dbg = 1u;
	                 on_min_dbg = apply_time_correction_min(g_dusk_min, g_node_cfg.on_corr_mode, g_node_cfg.on_corr_time_min);
	                 off_min_dbg = apply_time_correction_min(g_dawn_min, g_node_cfg.off_corr_mode, g_node_cfg.off_corr_time_min);
	                 want_on_dbg = time_window_contains(now_min_dbg, on_min_dbg, off_min_dbg);
	                 break;
	             case 2:
	                 managed_dbg = 1u;
	                 on_min_dbg = (uint16_t)g_node_cfg.light_on_hour * 60u + g_node_cfg.light_on_min;
	                 off_min_dbg = (uint16_t)g_node_cfg.light_off_hour * 60u + g_node_cfg.light_off_min;
	                 on_min_dbg = apply_time_correction_min(on_min_dbg, g_node_cfg.on_corr_mode, g_node_cfg.on_corr_time_min);
	                 off_min_dbg = apply_time_correction_min(off_min_dbg, g_node_cfg.off_corr_mode, g_node_cfg.off_corr_time_min);
	                 if (on_min_dbg != off_min_dbg) {
	                     want_on_dbg = time_window_contains(now_min_dbg, on_min_dbg, off_min_dbg);
	                 }
	                 break;
	             default:
	                 break;
	             }

	             if (g_node_cfg.saving_mode) {
	                 saving_active_dbg = time_window_contains(now_min_dbg, saving_start_dbg, saving_end_dbg);
	                 if (saving_active_dbg) {
	                     want_on_dbg = 0u;
	                 }
	             }

	             int rtc_dbg_len = snprintf(
	                 rtc_dbg, sizeof(rtc_dbg),
	                 "[RTCDBG] %04u-%02u-%02u %02u:%02u:%02u rtc_sync=%u mode=%u cfg_mode=%u now=%u on=%u off=%u save=%u(%u-%u) manual=%u hold=%u want=%u light=%u\r\n",
	                 g_rtc_year, g_rtc_month, g_rtc_day,
	                 g_rtc_hour, g_rtc_min, g_rtc_sec,
	                 (unsigned)g_rtc_synced,
	                 mode_now,
	                 g_node_cfg.mode,
	                 (unsigned)now_min_dbg,
	                 (unsigned)on_min_dbg,
	                 (unsigned)off_min_dbg,
	                 (unsigned)saving_active_dbg,
	                 (unsigned)saving_start_dbg,
	                 (unsigned)saving_end_dbg,
	                 (unsigned)light_control_manual_active(),
	                 (unsigned)manual_override_duration_min(),
	                 (unsigned)(managed_dbg ? want_on_dbg : 0u),
	                 (unsigned)light_is_on_logical()
	             );
	             if (rtc_dbg_len > 0) {
	                 HAL_UART_Transmit(&huart6, (uint8_t*)rtc_dbg, (uint16_t)rtc_dbg_len, 50);
	             }
	         }

	         // int now_min = g_rtc_hour * 60 + g_rtc_min;
	         // int night   = is_night(now_min, g_dusk_min, g_dawn_min);

	         if (g_dawn_min != prev_dawn || g_dusk_min != prev_dusk)
	         {
	             prev_dawn = g_dawn_min;
	             prev_dusk = g_dusk_min;

	             printf(
	                 "[SUN] date=%04u day=%u now=%02u:%02u "
	                 "dawn=%02u:%02u dusk=%02u:%02u\r\n",
	                 g_rtc_year,
	                 (unsigned)g_rtc_day,
	                 g_rtc_hour, g_rtc_min,
	                 g_dawn_min / 60, g_dawn_min % 60,
	                 g_dusk_min / 60, g_dusk_min % 60
	             );
	         }
	         /* ===================== AI ===================== */
	         // static uint32_t hb_t = 0;
	         // static uint32_t ai_feed_tick = 0;
	         now = HAL_GetTick();

	         if (!ai_pending && g_ai_sample_ready)
	         {
	             __disable_irq();
	             float x = g_ai_sample;
	             g_ai_sample_ready = 0;
	             __enable_irq();

	             Input_Ai_Model(x);
	         }
	         /*if (now - hb_t >= 1000) {
	           hb_t = now;
	           //uart6_log("[HB] t=%lu ai_idx=%u pending=%u wisun=%u at=%u\r\n", (unsigned long)now, (unsigned)ai_index, (unsigned)ai_pending,
	           //(unsigned)wisun_packet_ready, (unsigned)g_at_line_ready);
	          }*/
	         if (ai_pending) {
	             ai_service();
	         }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_BACKUPRESET_FORCE();
  __HAL_RCC_BACKUPRESET_RELEASE();

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
  PeriphClkInitStruct.PLL2.PLL2Source = RCC_PLL2_SOURCE_HSE;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 100;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 23;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2_VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2_VCORANGE_WIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL2.PLL2ClockOut = RCC_PLL2_DIVR;
  PeriphClkInitStruct.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_PLL2R;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Disable Injected Queue
  */
  HAL_ADCEx_DisableInjectedQueue(&hadc1);

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  sConfigInjected.InjecOversamplingMode = ENABLE;
  sConfigInjected.InjecOversampling.Ratio = ADC_OVERSAMPLING_RATIO_2;
  sConfigInjected.InjecOversampling.RightBitShift = ADC_RIGHTBITSHIFT_1;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel5_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2u)
  {
    /* Keep RTC untouched. It will be set only by SET_RTC_KST. */
  }
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  /* USER CODE BEGIN RTC_SKIP_FIXED_TIME */
  /* Do not set a fixed RTC value during boot. */
  /* USER CODE END RTC_SKIP_FIXED_TIME */

  /** Enable the TimeStamp
  */
  if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  HAL_NVIC_SetPriority(RTC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 276;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 624;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*static void ai_service(void)
{
    if (!ai_pending) return;

    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - ai_next_run) < 0) return;
    if (wisun_packet_ready || g_at_line_ready) return;


    uint32_t t0 = HAL_GetTick();
    int r = run_inference(ai_input, &ai_mse, &ai_pred);
    uint32_t dt = HAL_GetTick() - t0;
    if (r != 0) {
        char err[64];
        int n = snprintf(err, sizeof(err),
                         "AI run ERR=%d dt=%lu ms\r\n",
                         r, (unsigned long)dt);
        HAL_UART_Transmit(&huart6, (uint8_t*)err, n, 50);

        ai_pending = 0;
        ai_index = 0;
        ai_next_run = now + ai_period_ms;
        return;  
    }
}*/
static void ai_service(void)
{
    if (!ai_pending) return;

    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - ai_next_run) < 0) return;
    if (wisun_packet_ready || g_at_line_ready) return;

    uint32_t t0 = HAL_GetTick();
    int r = run_inference(ai_input, &ai_mse, &ai_pred);
    uint32_t dt = HAL_GetTick() - t0;

    if (r != 0)
    {
        char err[64];
        int n = snprintf(err, sizeof(err),
                         "AI run ERR=%d dt=%lu ms\r\n",
                         r, (unsigned long)dt);
        HAL_UART_Transmit(&huart6, (uint8_t*)err, n, 50);
        
        ai_pending = 0;
        ai_index = 0;
        ai_next_run = now + ai_period_ms;
        return;
    }
    
    {
        char ok[96];
        int n = snprintf(ok, sizeof(ok),
                         "AI OK dt=%lu ms mse=%.6f pred=%d\r\n",
                         (unsigned long)dt, (double)ai_mse, ai_pred);
        HAL_UART_Transmit(&huart6, (uint8_t*)ok, n, 50);

    }

    ai_pending = 0;                
    ai_index = 0;
    uart6_log("[AI] done idx=%u pending=%u\r\n", (unsigned)ai_index, (unsigned)ai_pending);
    ai_next_run = now + ai_period_ms; 
}

void resp_slot_task_poll(void)
{
    uint32_t now = HAL_GetTick();

    int ready_idx = -1;
    uint32_t best_due = 0xFFFFFFFFu;
    uint8_t best_priority = 0xFFu;
    
    __disable_irq();
    for (int i = 0; i < RESP_QUEUE_SIZE; ++i) {
        uint8_t priority;

        if (!g_resp_q[i].pending) continue;
        if ((int32_t)(now - g_resp_q[i].due_tick) < 0) continue;

        if (g_resp_q[i].kind == RESP_KIND_LIGHT_ACK) {
            priority = 0u;
        } else if (g_resp_q[i].kind == RESP_KIND_SNAP) {
            priority = 2u;
        } else {
            priority = 1u;
        }

        if (ready_idx < 0 ||
            priority < best_priority ||
            (priority == best_priority && g_resp_q[i].due_tick < best_due)) {
            ready_idx = i;
            best_due = g_resp_q[i].due_tick;
            best_priority = priority;
        }
    }

    if (ready_idx < 0) {
        __enable_irq();
        return;
    }

    // ?�재 ?�신 ?�롯?�로 복사
    memcpy(&g_resp_slot, &g_resp_q[ready_idx], sizeof(g_resp_slot));
    memset(&g_resp_q[ready_idx], 0, sizeof(g_resp_q[ready_idx]));
    __enable_irq();

    {
        char msg[160];
        int n = snprintf(msg, sizeof(msg),
                         "[SLOT_DEQ] q=%d kind=%u tmid=%u msg_id=%u has_raw=%u len=%u\r\n",
                         ready_idx,
                         (unsigned)g_resp_slot.kind,
                         (unsigned)g_resp_slot.tmid,
                         (unsigned)g_resp_slot.msg_id,
                         (unsigned)g_resp_slot.has_raw_buf,
                         (unsigned)g_resp_slot.len);
        if (n > 0) {
            HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 100);
        }
    }

    // RAW 버퍼 ?�답?�면 그�?�??�송
    if (g_resp_slot.has_raw_buf) {
        char msg[128];
        int n = snprintf(msg, sizeof(msg),
                         "[RAW_TX] kind=%d tmid=%04X len=%u first=%02X\r\n",
                         (int)g_resp_slot.kind,
                         g_resp_slot.tmid,
                         g_resp_slot.len,
                         g_resp_slot.buf[0]);
        if (n > 0) {
            HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 100);
        }

        if (g_resp_slot.tmid != 0u) {
            uint8_t tx_cmd = CMD_ACK_RELAY;

            if (g_resp_slot.len >= 1u) {
                tx_cmd = g_resp_slot.buf[0];

                if (g_resp_slot.buf[0] == 0x10u || g_resp_slot.buf[0] == 0x20u) {
                    tx_cmd = CMD_ACK_RELAY;
                }
            }

            if (g_resp_slot.kind == RESP_KIND_LIGHT_ACK) {
                (void)send_transport_direct(g_resp_slot.tmid, HOP_TTL_DEFAULT, tx_cmd, 0u, g_resp_slot.msg_id, g_resp_slot.buf, g_resp_slot.len);
            } else if (!enqueue_transport_tx(g_resp_slot.tmid, tx_cmd, 0u, g_resp_slot.msg_id, g_resp_slot.buf, g_resp_slot.len, HOP_TTL_DEFAULT)) {
                (void)send_transport_direct(g_resp_slot.tmid, HOP_TTL_DEFAULT, tx_cmd, 0u, g_resp_slot.msg_id, g_resp_slot.buf, g_resp_slot.len);
            }
        } else {
            (void)send_transport_direct(0x0000u, 0u, g_resp_slot.buf[0], 0u, g_resp_slot.msg_id, g_resp_slot.buf, g_resp_slot.len);
        }

        memset(&g_resp_slot, 0, sizeof(g_resp_slot));
        return;
    }

    switch (g_resp_slot.kind) {
    case RESP_KIND_SNAP:
        uart6_log("[SNAP_RESP_SKIP] msg_id=%u periodic_only=1\r\n",
                  (unsigned)g_resp_slot.msg_id);
        break;

    case RESP_KIND_ACK:
    {
        uint8_t  cmd    = g_resp_slot.cmd;
        uint8_t  flags  = 0x80;
        uint16_t msg_id = g_resp_slot.msg_id;
        uint8_t  result = g_resp_slot.result;

        if (g_resp_slot.tmid != 0u) {
            if (!enqueue_transport_tx(g_resp_slot.tmid, cmd, flags, msg_id, &result, 1u, HOP_TTL_DEFAULT)) 
            {
                (void)send_transport_direct(g_resp_slot.tmid, HOP_TTL_DEFAULT, cmd, flags, msg_id, &result, 1u);
            }
        } else {
            (void)send_transport_direct(0x0000u, 0u, cmd, flags, msg_id, &result, 1u);
        }

        debug6("[ACK_TX_DONE]\r\n");
        break;
    }

    case RESP_KIND_RAW_BIN:        
        break;

    case RESP_KIND_NONE:
    default:
        break;
    }

    memset(&g_resp_slot, 0, sizeof(g_resp_slot));
}

static bool is_uplink_report_cmd(uint8_t cmd)
{
    switch (cmd) {
    case SNAP_REPORT_CMD:
    case LIGHT_STATE_EVENT_CMD:
        return true;

    default:
        return false;
    }
}

static void Read_UID_local(void){
    Read_UID(); 
    uid_ram_local[0] = uid_ram[0];
    uid_ram_local[1] = uid_ram[1];
    uid_ram_local[2] = uid_ram[2];
}


void Read_UID(void) {
    uid_ram[0] = UID_ADDRESS[0];
    uid_ram[1] = UID_ADDRESS[1];
    uid_ram[2] = UID_ADDRESS[2];
}

static void init_uid_string(void){
    Read_UID_local();
    snprintf(g_uid_str, sizeof(g_uid_str),
             "%08" PRIX32 "-%08" PRIX32 "-%08" PRIX32,
             uid_ram_local[2], uid_ram_local[1], uid_ram_local[0]);
}
static void apply_mid(uint16_t new_mid)
{
    if (new_mid == 0 || new_mid == MID_INVALID) {
        return;
    }

#if WISUN_AT_COMMAND_ENABLE
    char cmd_buf[32];
    int n = snprintf(cmd_buf, sizeof(cmd_buf), "AT+MID=%u\r\n", new_mid);
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd_buf, (uint16_t)n, HAL_MAX_DELAY);
#endif

    my_mid = new_mid;

    save_mid_to_flash(my_mid);

    g_node_cfg.mid          = new_mid;
    g_node_cfg.mid_assigned = 1;
    (void)save_node_cfg_to_flash(&g_node_cfg);

    //dbg_print_mid_info("[SET_MID]", my_mid, 0);
}

static void apply_rch(uint8_t r0, uint8_t r1)
{
#if WISUN_AT_COMMAND_ENABLE
    char cmd_buf[32];
    int n = snprintf(cmd_buf, sizeof(cmd_buf), "AT+RCH=%u,%u\r\n",
                     (unsigned)r0, (unsigned)r1);
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd_buf, (uint16_t)n, HAL_MAX_DELAY);

    char msg[48];
    int m = snprintf(msg, sizeof(msg), "[SET_RCH] rch=%u,%u\r\n",
                     (unsigned)r0, (unsigned)r1);
    HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)m, HAL_MAX_DELAY);
#else
    (void)r0;
    (void)r1;
#endif
}

static void apply_mid_chan_from_cfg(void)
{
    // 1) MID �?
    if (g_node_cfg.mid_assigned &&
        g_node_cfg.mid != 0 &&
        g_node_cfg.mid != MID_INVALID)
    {
        my_mid = g_node_cfg.mid;

        save_mid_to_flash(my_mid);

#if WISUN_BOOT_APPLY_AT_CFG
        apply_mid(my_mid);
#endif
    }
    else {        
        uint16_t stored_mid = MID_INVALID;
        if (load_mid_from_flash(&stored_mid) &&
            stored_mid != MID_INVALID &&
            stored_mid != 0)
        {
            my_mid = stored_mid;
        }
    }

#if WISUN_BOOT_APPLY_AT_CFG
    apply_rch(g_node_cfg.rch[0], g_node_cfg.rch[1]);
#endif
}

static void nodeinfo_poll(uint32_t now)
{
    if (!g_nodeinfo.pending) return;

    if ((int32_t)(now - g_nodeinfo.deadline_tick) > 0) {
        nodeinfo_finish_fail(-8);
        return;
    }

    if ((g_nodeinfo.got_mask & GOT_ALL) == GOT_ALL) {
        nodeinfo_finish_ok();
        return;
    }
}

static float tim6_sample_rate_hz(void)
{
    RCC_ClkInitTypeDef clk = {0};
    uint32_t flash_latency = 0u;
    uint32_t pclk1_hz;
    uint32_t tim_clk_hz;
    uint32_t prescaler;
    uint32_t period;

    HAL_RCC_GetClockConfig(&clk, &flash_latency);

    pclk1_hz = HAL_RCC_GetPCLK1Freq();
    tim_clk_hz = pclk1_hz;

    if (clk.APB1CLKDivider != RCC_HCLK_DIV1) {
        tim_clk_hz = pclk1_hz * 2u;
    }

    prescaler = htim6.Init.Prescaler + 1u;
    period = htim6.Init.Period + 1u;

    if (prescaler == 0u || period == 0u) {
        return 0.0f;
    }

    return (float)tim_clk_hz / ((float)prescaler * (float)period);
}

static void InitHannWindowOnce(void)
{
    if (g_hann_inited) return;
    for (int i = 0; i < FFT_SIZE; i++) {
        g_hann[i] = 0.5f - 0.5f * arm_cos_f32(2.0f * 3.14159265f * (float)i / (float)(FFT_SIZE - 1));
    }
    g_hann_inited = 1;
}

void ExtractFullFFT(const float32_t *in, float fs_hz, FftData_t *dest) {
	InitHannWindowOnce();	

	static float32_t xw[FFT_SIZE];
	float32_t mean = 0.0f;

	for (int i = 0; i < FFT_SIZE; i++) mean += in[i];
	mean /= (float32_t)FFT_SIZE;

	for (int i = 0; i < FFT_SIZE; i++) {
		xw[i] = (in[i] - mean) * g_hann[i];
	}

	processFFT(xw, outputSignal, magnitude);

	for (int i = 0; i < FFT_SIZE / 2; i++) {
		float freq = ((float)i * fs_hz) / (float)FFT_SIZE;
		dest[i].freq = freq;           
		dest[i].amplitude = magnitude[i];
	}
}

static void ExtractFullFFT_MagOnly(const float32_t *in, float32_t *mag_out)
{
    InitHannWindowOnce();

    static float32_t xw[FFT_SIZE];
    static float32_t out[FFT_SIZE];

    float32_t mean = 0.0f;
    for (int i = 0; i < FFT_SIZE; i++) mean += in[i];
    mean /= (float32_t)FFT_SIZE;

    for (int i = 0; i < FFT_SIZE; i++) {
        xw[i] = (in[i] - mean) * g_hann[i];
    }

    processFFT(xw, out, mag_out);
}

void Query_MID_From_WiSUN(void) {
    HAL_UART_Transmit(&huart6, (uint8_t*)"[MID? TX FUNC ENTER]\r\n", 22, 50);

    const char *cmd = "AT+MID?\r\n";
    g_wait_mid_query = 1;

    char t[64];
    snprintf(t, sizeof(t), "[MID? set wait=1]\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)t, strlen(t), 50);
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
}

void Parse_AT_Response(const char* buffer)
{
    char dbg[128];
    snprintf(dbg, sizeof(dbg), "[AT] wait=%u line=%s\r\n", g_wait_mid_query, buffer);
    //HAL_UART_Transmit(&huart6, (uint8_t*)dbg, strlen(dbg), 50);

    if (!g_wait_mid_query) return;
    
    if (strncmp(buffer, "AT+MID=", 7) == 0) {
        int mid = atoi(buffer + 7);
        if (mid >= 0 && mid <= 65535) {
            my_mid = (uint16_t)mid;

            g_node_cfg.mid          = my_mid;
            g_node_cfg.mid_assigned = 1;

            bool ok1 = save_mid_to_flash(my_mid);
            bool ok2 = save_node_cfg_to_flash(&g_node_cfg);

            char msg[80];
            snprintf(msg, sizeof(msg),
                     "Parsed MID: %u (save_mid=%d, save_cfg=%d)\r\n",
                     my_mid, ok1, ok2);
            HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 50);

            g_wait_mid_query = 0;
        }
    }
}

void Ultra_StartSampling(void) {
    wr_idx = 0;
    ultra_frame_ready = false;
    ultra_sampling_paused = false;
    HAL_ADC_Start_IT(&hadc1);   
}

void Ultra_ResumeNextFrame(void) {
    wr_idx = 0;
    ultra_frame_ready = false;
    ultra_sampling_paused = false;
    HAL_ADC_Start_IT(&hadc1);
}

void Ultra_StartDmaFrame(void)
{
    wr_idx = 0;
    ultra_frame_ready = false;
    ultra_sampling_paused = false;

    (void)HAL_ADCEx_InjectedStop(&hadc1);
    (void)HAL_ADCEx_InjectedStop_IT(&hadc1);

    (void)HAL_ADC_Stop_IT(&hadc1);
    (void)HAL_ADC_Stop_DMA(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
    (void)HAL_TIM_Base_Stop(&htim6);

    HAL_StatusTypeDef adc_ret = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)raw_buffer, FFT_SIZE);

    if (adc_ret != HAL_OK)
    {
        ultra_sampling_paused = true;

        uart6_log("[ULTRA_DMA_START_FAIL] adc_ret=%d ready=%u paused=%u tick=%lu\r\n",
                  (int)adc_ret,
                  (unsigned)(ultra_frame_ready ? 1u : 0u),
                  (unsigned)(ultra_sampling_paused ? 1u : 0u),
                  (unsigned long)HAL_GetTick());

        return;
    }

    g_frame_c0 = DWT->CYCCNT;

    __HAL_TIM_SET_COUNTER(&htim6, 0);

    HAL_StatusTypeDef tim_ret = HAL_TIM_Base_Start(&htim6);

    uart6_log("[ULTRA_DMA_START] adc_ret=%d tim_ret=%d ready=%u paused=%u done=%lu half=%lu tick=%lu\r\n",
              (int)adc_ret,
              (int)tim_ret,
              (unsigned)(ultra_frame_ready ? 1u : 0u),
              (unsigned)(ultra_sampling_paused ? 1u : 0u),
              (unsigned long)g_dma_done,
              (unsigned long)g_dma_half,
              (unsigned long)HAL_GetTick());

    if (tim_ret != HAL_OK)
    {
        ultra_sampling_paused = true;
    }
}

static int nodeinfo_append_kv_line(const char *line)
{
    if (line[0] == 'O' && line[1] == 'K') return 0;
    if (line[0] == 'E' && line[1] == 'R') return 0; 
    
    const char *p = line;
    if (p[0] == 'A' && p[1] == 'T' && p[2] == '+') p += 3;

    uint16_t tmp_len = 0;
    char tmp[96]; 
    while (*p && tmp_len < sizeof(tmp) - 1) {
        if (*p == '\r' || *p == '\n') break;
        tmp[tmp_len++] = *p++;
    }
    tmp[tmp_len] = '\0';
    if (tmp_len == 0) return 0;

    if (strncmp(tmp, "GID=",   4) == 0) g_nodeinfo.got_mask |= GOT_GID;
	if (strncmp(tmp, "MID=",   4) == 0) g_nodeinfo.got_mask |= GOT_MID;
	if (strncmp(tmp, "DEV=",   4) == 0) g_nodeinfo.got_mask |= GOT_DEV;
	if (strncmp(tmp, "DSP=",   4) == 0) g_nodeinfo.got_mask |= GOT_DSP;
	if (strncmp(tmp, "RCH=", 4) == 0) {
	    g_nodeinfo.got_mask |= GOT_RCH;

	    const char *v = tmp + 4;

	    int a = -1, b = -1;
	    if (sscanf(v, " %d , %d", &a, &b) == 2) {
	        // clamp
	        if (a < 0) a = 0; if (a > 255) a = 255;
	        if (b < 0) b = 0; if (b > 255) b = 255;

	        g_node_cfg.rch[0] = (uint8_t)a;
	        g_node_cfg.rch[1] = (uint8_t)b;
	    } else if (sscanf(v, " %d", &b) == 1) {
	        if (b < 0) b = 0; if (b > 255) b = 255;
	        g_node_cfg.rch[0] = 0;
	        g_node_cfg.rch[1] = (uint8_t)b;
	    }
	    
	    (void)save_node_cfg_to_flash(&g_node_cfg);

	    // debug
	    char msg[64];
	    int m = snprintf(msg, sizeof(msg),
	                     "[CFG] RCH saved: %u,%u\r\n",
	                     (unsigned)g_node_cfg.rch[0],
	                     (unsigned)g_node_cfg.rch[1]);
	    HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)m, 50);
	}
	if (strncmp(tmp, "TXP=",   4) == 0) g_nodeinfo.got_mask |= GOT_TXP;
	if (strncmp(tmp, "MODE=",  5) == 0) g_nodeinfo.got_mask |= GOT_MODE;
	if (strncmp(tmp, "MAC=",   4) == 0) g_nodeinfo.got_mask |= GOT_MAC;
	if (strncmp(tmp, "FWVER=", 6) == 0) g_nodeinfo.got_mask |= GOT_FWVER;

    if (g_nodeinfo.used + tmp_len + 2 > NODE_INFO_TEXT_MAX) {
        return -1; // overflow
    }

    memcpy(&g_nodeinfo.text[g_nodeinfo.used], tmp, tmp_len);
    g_nodeinfo.used += tmp_len;
    g_nodeinfo.text[g_nodeinfo.used++] = '\n';
    g_nodeinfo.text[g_nodeinfo.used] = '\0';
    return 1;
}

static void nodeinfo_start(uint16_t tmid, uint16_t msg_id)
{
    char dbg[96];
    int dn = snprintf(dbg, sizeof(dbg),
                      "[NODEINFO_START] tmid=0x%04X msg_id=0x%04X\r\n",
                      (unsigned)tmid,
                      (unsigned)msg_id);
    if (dn > 0) {
        HAL_UART_Transmit(&huart6, (uint8_t*)dbg, (uint16_t)dn, 100);
    }
    g_nodeinfo.pending = 1;
    g_nodeinfo.tmid    = tmid;
    g_nodeinfo.msg_id  = msg_id;
    g_nodeinfo.used    = 0;
    g_nodeinfo.got_mask = 0;
    g_nodeinfo.text[0] = '\0';

    uint32_t now = HAL_GetTick();
    g_nodeinfo.last_rx_tick   = now;
    g_nodeinfo.deadline_tick  = now + 3000;  

    const char *cmd = "AT+CFG?\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, (uint16_t)strlen(cmd), 50);
}

static void rstrip_inplace(char *s)
{
    int n = (int)strlen(s);
    while (n > 0) {
        char c = s[n-1];
        if (c == '\r' || c == '\n' || c == ' ' || c == '\t') {
            s[--n] = '\0';
        } else {
            break;
        }
    }
}

void nodeinfo_collect_line(const char *line)
{
    if (!g_nodeinfo.pending || !line) return;

    char dbg[160];
    int dn = snprintf(dbg, sizeof(dbg),
                      "[NODEINFO_LINE] tmid=0x%04X line=%s\r\n",
                      (unsigned)g_nodeinfo.tmid,
                      line);
    /* if (dn > 0) {
        HAL_UART_Transmit(&huart6, (uint8_t*)dbg, (uint16_t)dn, 100);
    } */

    char tmp[RX_BUFFER_SIZE];
    size_t L = strlen(line);
    if (L >= sizeof(tmp)) L = sizeof(tmp) - 1;
    memcpy(tmp, line, L);
    tmp[L] = '\0';
    rstrip_inplace(tmp);
    if (tmp[0] == '\0') return;

    g_nodeinfo.last_rx_tick = HAL_GetTick();

    if (strncmp(tmp, "ERROR", 5) == 0) {
        nodeinfo_finish_fail(-7);
        return;
    }
    if (strcmp(tmp, "OK") == 0) {
        return;
    }

    if (nodeinfo_append_kv_line(tmp) < 0) {
        nodeinfo_finish_fail(-6);
        return;
    }
    nodeinfo_cache_update_from_kv(tmp);
    
    if (strstr(tmp, "FWVER=") != NULL) {
    	g_node_info.valid = 1;
        nodeinfo_finish_ok();
        return;
    }
}

static uint16_t my_strnlen(const char *s, uint16_t maxn)
{
    uint16_t i = 0;
    while (i < maxn && s[i] != '\0') i++;
    return i;
}

static void nodeinfo_finish_and_send(uint8_t ok, int8_t err_code)
{
    if (!g_nodeinfo.pending) return;

    if (ok) {
        uint16_t maxn = (uint16_t)sizeof(g_nodeinfo.text);
        uint16_t text_len = my_strnlen(g_nodeinfo.text, maxn);
        
        char m[64];
        int n = snprintf(m, sizeof(m), "[NI_SEND] used=%u strnlen=%u\r\n",
                         (unsigned)g_nodeinfo.used, (unsigned)text_len);
        HAL_UART_Transmit(&huart6, (uint8_t*)m, (uint16_t)n, 100);

        schedule_resp_with_slot( RESP_KIND_RAW_BIN, g_nodeinfo.tmid, g_nodeinfo.msg_id, (uint8_t*)g_nodeinfo.text, text_len );
    } else {
        char msg[32];
        int n = snprintf(msg, sizeof(msg), "ERR=%d\n", (int)err_code);
        if (n < 0) n = 0;
        if (n > (int)sizeof(msg)) n = sizeof(msg);

        schedule_resp_with_slot(
            RESP_KIND_RAW_BIN,
            g_nodeinfo.tmid,
            g_nodeinfo.msg_id,
            (uint8_t*)msg,
            (uint16_t)n
        );
    }

    g_nodeinfo.pending = 0;
}

void nodeinfo_finish_ok(void)
{
    char dbg[96];
    int dn = snprintf(dbg, sizeof(dbg),
                      "[NODEINFO_FINISH_OK] tmid=0x%04X msg_id=0x%04X used=%u\r\n",
                      (unsigned)g_nodeinfo.tmid,
                      (unsigned)g_nodeinfo.msg_id,
                      (unsigned)g_nodeinfo.used);
    if (dn > 0) {
        HAL_UART_Transmit(&huart6, (uint8_t*)dbg, (uint16_t)dn, 100);
    }

	if (g_nodeinfo.tmid == 0) {
		g_nodeinfo.pending = 0;
		return;
	}
	nodeinfo_finish_and_send(1, 0);
}

void nodeinfo_finish_fail(int8_t err)
{
	if (g_nodeinfo.tmid == 0) {
		g_nodeinfo.pending = 0;
		return;
	}

    nodeinfo_finish_and_send(0, err);
}

void Debug_Print_FFT_Peak(void)
{
    static uint32_t fft_seq = 0;
    static uint32_t dbg_t = 0;

    InitHannWindowOnce();

    uint32_t my_seq = ++fft_seq;

    float vin_v = 0.0f;
    float i_adc_v = 0.0f;
    {
        VIRead vi;
        if (AD_DC_Injected_Once(&vi) == HAL_OK) {
            vin_v   = adc_raw_to_dc_vin(vi.volt_raw);
            i_adc_v = adc_raw_to_dc_current(vi.curr_raw);
        }
    }

    static float32_t x[FFT_SIZE];
    uint16_t adc_min = 0xFFFFu;
    uint16_t adc_max = 0u;

    for (int i = 0; i < FFT_SIZE; i++) {
        uint16_t raw = raw_buffer[i];

        if (raw < adc_min) adc_min = raw;
        if (raw > adc_max) adc_max = raw;

        x[i] = ((float)raw * 3.3f / 4095.0f) - 1.65f;
    }

    uint16_t adc_span = (uint16_t)(adc_max - adc_min);
    uint8_t ultra_signal_ok =
        (adc_span >= SNAP_FFT_ADC_RAW_SPAN_MIN) ? 1u : 0u;

    /* DWT 기반 프레임 시간은 진단용으로만 사용 */
    uint32_t c0 = g_frame_c0;
    uint32_t c1 = g_frame_c1;
    uint32_t dc = (c1 >= c0)
        ? (c1 - c0)
        : (0xFFFFFFFFu - c0 + c1 + 1u);

    float dt_s = (float)dc / (float)SystemCoreClock;
    float dt_ms = dt_s * 1e3f;
    float fs_eff = (dt_s > 1e-9f)
        ? ((float)FFT_SIZE / dt_s)
        : 0.0f;

    uint32_t now = HAL_GetTick();
#if !DEBUG_FFT_DATA_EXTRACT_MODE
    if ((now - dbg_t) >= 1000u) {
        dbg_t = now;

        float mn = 1e9f;
        float mx = -1e9f;

        for (int i = 0; i < FFT_SIZE; i++) {
            if (x[i] < mn) mn = x[i];
            if (x[i] > mx) mx = x[i];
        }

        char tbuf[160];
        int tn = snprintf(
            tbuf,
            sizeof(tbuf),
            "[IN] min=%.6f max=%.6f p2p=%.6f raw_min=%u raw_max=%u span=%u\r\n",
            (double)mn,
            (double)mx,
            (double)(mx - mn),
            (unsigned)adc_min,
            (unsigned)adc_max,
            (unsigned)adc_span
        );

        if (tn < 0) tn = 0;
        if (tn > (int)sizeof(tbuf)) tn = (int)sizeof(tbuf);

        HAL_StatusTypeDef st2 = HAL_UART_Transmit(
            &huart6,
            (uint8_t*)tbuf,
            (uint16_t)tn,
            20
        );

        if (st2 != HAL_OK) uart_fail_cnt++;
    }
#else
    (void)now;
    (void)dbg_t;
#endif

    /* SNAP과 동일한 FFT 전처리 */
    static float32_t mag[FFT_SIZE / 2];
    ExtractFullFFT_MagOnly(x, mag);

    const float lo_hz = 80000.0f;
    const float hi_hz = 125000.0f;

    /* SNAP과 동일하게 TIM6 설정 기반 샘플링 주파수 사용 */
    float fs = tim6_sample_rate_hz();
    if (fs <= 1.0f || !isfinite(fs)) {
        fs = (float)FSAMPLE;
    }

    float nyq = 0.5f * fs;
    float lo = lo_hz;
    float hi = hi_hz;

    if (hi > nyq) hi = nyq;
    if (lo < 1.0f) lo = 1.0f;

    if (lo >= hi) {
        char buf[180];
        int len = snprintf(
            buf,
            sizeof(buf),
            "[FFT#%lu] fs=%.1fk nyq=%.1fk lo/hi invalid "
            "(lo=%.1fk hi=%.1fk)\r\n",
            (unsigned long)my_seq,
            (double)(fs / 1000.0f),
            (double)(nyq / 1000.0f),
            (double)(lo / 1000.0f),
            (double)(hi / 1000.0f)
        );

        HAL_UART_Transmit(
            &huart6,
            (uint8_t*)buf,
            (uint16_t)len,
            20
        );

        Ultra_StartDmaFrame();
        return;
    }

    float max_amp = -1.0f;
    float peak_f = 0.0f;
    int peak_i = -1;

    float min_amp = 1e30f;
    float min_f = 0.0f;
    int min_i = -1;

    float sum = 0.0f;
    int cnt = 0;
    float top1 = 0.0f;
    float top2 = 0.0f;
    float top3 = 0.0f;

    for (int i = 1; i < (FFT_SIZE / 2); i++) {
        float freq = ((float)i * fs) / (float)FFT_SIZE;
        if (freq < lo || freq > hi) continue;

        float a = mag[i];
        if (!isfinite(a)) a = 0.0f;

        if (a > max_amp) {
            max_amp = a;
            peak_f = freq;
            peak_i = i;
        }

        if (a < min_amp) {
            min_amp = a;
            min_f = freq;
            min_i = i;
        }

        sum += a;
        cnt++;

        if (a > top1) {
            top3 = top2;
            top2 = top1;
            top1 = a;
        } else if (a > top2) {
            top3 = top2;
            top2 = a;
        } else if (a > top3) {
            top3 = a;
        }
    }

    if (cnt == 0 || peak_i < 0) {
        max_amp = 0.0f;
        peak_f = 0.0f;
        peak_i = -1;
        min_amp = 0.0f;
        min_f = 0.0f;
        min_i = -1;
    }

    /* SNAP과 동일한 peak bin ±1의 3-bin RSS */
    if (peak_i >= 2 && peak_i <= (FFT_SIZE / 2 - 2)) {
        float a0 = mag[peak_i - 1];
        float a1 = mag[peak_i];
        float a2 = mag[peak_i + 1];

        max_amp = sqrtf(a0 * a0 + a1 * a1 + a2 * a2);
    }

    /* SNR은 유효 판정이 아닌 참고 로그로만 유지 */
    float noise_floor = 0.0f;
    if (cnt > 0) {
        float trimmed = sum - (top1 + top2 + top3);
        int trimmed_cnt = cnt - 3;

        if (trimmed_cnt < 1) {
            trimmed = sum;
            trimmed_cnt = cnt;
        }

        noise_floor = trimmed / (float)trimmed_cnt;
    }

    if (noise_floor < NOISE_MIN) {
        noise_floor = NOISE_MIN;
    }

    float snr = (noise_floor > 0.0f)
        ? (max_amp / noise_floor)
        : 0.0f;

    /* SNAP과 동일한 최종 유효 조건 */
    bool found =
        (peak_i >= 0) &&
        (cnt > 0) &&
        (ultra_signal_ok != 0u) &&
        (max_amp >= SNAP_FFT_VALID_AMP_MIN);

    const float N = (float)FFT_SIZE;
    const float CG = 0.5f;

    float vpk = found
        ? ((2.0f / (N * CG)) * max_amp)
        : 0.0f;

    float vpp = 2.0f * vpk;
    float vrms = vpk * 0.70710678f;
    float adc_pk = vpk * (4095.0f / 3.3f);
    float adc_pp = vpp * (4095.0f / 3.3f);

    {
        char buf[460];
        int len = snprintf(
            buf,
            sizeof(buf),
            "[FFT#%lu] found=%u peak=%.2fkHz bin=%d "
            "max=%.3g min=%.3g@%.2fkHz "
            "vpk=%.4fV vpp=%.4fV vrms=%.4fV "
            "adc_pk=%.1f adc_pp=%.1f "
            "span=%u span_min=%u amp_min=%.3f "
            "nf=%.3g snr=%.2f cnt=%d "
            "dt=%.2fms fs_eff=%.1fk fs=%.1fk nyq=%.1fk "
            "lo=%.1fk hi=%.1fk vin=%.2fV i=%.2fA\r\n",
            (unsigned long)my_seq,
            (unsigned)(found ? 1u : 0u),
            (double)(peak_f / 1000.0f),
            peak_i,
            (double)max_amp,
            (double)min_amp,
            (double)(min_f / 1000.0f),
            (double)vpk,
            (double)vpp,
            (double)vrms,
            (double)adc_pk,
            (double)adc_pp,
            (unsigned)adc_span,
            (unsigned)SNAP_FFT_ADC_RAW_SPAN_MIN,
            (double)SNAP_FFT_VALID_AMP_MIN,
            (double)noise_floor,
            (double)snr,
            cnt,
            (double)dt_ms,
            (double)(fs_eff / 1000.0f),
            (double)(fs / 1000.0f),
            (double)(nyq / 1000.0f),
            (double)(lo / 1000.0f),
            (double)(hi / 1000.0f),
            (double)vin_v,
            (double)i_adc_v
        );

        if (len < 0) len = 0;
        if (len > (int)sizeof(buf)) len = (int)sizeof(buf);

        uint32_t tmo =
            5u + (uint32_t)((len * 10u * 1000u) / 115200u);

        HAL_StatusTypeDef st = HAL_UART_Transmit(
            &huart6,
            (uint8_t*)buf,
            (uint16_t)len,
            tmo
        );

        if (st != HAL_OK) uart_fail_cnt++;
    }

#if DEBUG_FFT_DATA_EXTRACT_MODE
    {
        char data_buf[220];
        int data_len = snprintf(
            data_buf,
            sizeof(data_buf),
            "[FFT_DATA] seq=%lu found=%u freq_hz=%.2f amp=%.6g vin_v=%.3f curr_a=%.3f "
            "span=%u fs_hz=%.1f vpk=%.6f adc_pk=%.2f\r\n",
            (unsigned long)my_seq,
            (unsigned)(found ? 1u : 0u),
            (double)peak_f,
            (double)max_amp,
            (double)vin_v,
            (double)i_adc_v,
            (unsigned)adc_span,
            (double)fs,
            (double)vpk,
            (double)adc_pk
        );

        if (data_len < 0) data_len = 0;
        if (data_len > (int)sizeof(data_buf)) data_len = (int)sizeof(data_buf);

        HAL_StatusTypeDef data_st = HAL_UART_Transmit(
            &huart6,
            (uint8_t*)data_buf,
            (uint16_t)data_len,
            30
        );

        if (data_st != HAL_OK) uart_fail_cnt++;
    }
#endif

    Ultra_StartDmaFrame();
}


/*void Transfer_ADC_To_DAC(void)
{

    HAL_ADC_Start(&hadc1);


    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);


    uint32_t adc_val = HAL_ADC_GetValue(&hadc1);


    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, adc_val);
}*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_UART_Transmit(&huart6, (uint8_t *)"Error_Handler called\r\n", 23, HAL_MAX_DELAY);
	  HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
