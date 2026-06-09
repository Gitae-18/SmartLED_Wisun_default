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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
    float freq;      // Hz
    float amplitude;    // FFT 진폭
} FftData_t;

typedef struct { uint16_t volt_raw; uint16_t curr_raw; uint32_t t_us; } VIRead;

FftData_t fft_packet[FFT_SIZE / 2];
arm_rfft_fast_instance_f32 fftInstance;

typedef enum {
    WAITING_FOR_STX,
    RECEIVING_PACKET
} PacketParserState;

typedef struct {
    bool     pending;
    uint8_t  which_idx;   
    uint16_t nonce;
    uint32_t due_tick;    
} tx_task_t;

typedef struct {
    bool     light_on;
    float    voltage;
    float    current;
    float    supersonic;
    float    temp;
    uint32_t count;
} snapshot_t;

typedef struct {
    bool     pending;
    uint32_t due_tick;
    uint16_t nonce;
    char     topic[16];
    uint8_t  ok;

    uint8_t  p_on_valid, p_on;   
} resp_task_t;

typedef enum {
    RESP_KIND_NONE = 0,
    RESP_KIND_SNAP,
    RESP_KIND_ACK,      
    RESP_KIND_RAW_BIN,  
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

typedef struct {
    uint32_t start_after_ms;  // p.start_after_ms
    uint32_t slot_len_ms;     // p.slot_len_ms
    uint32_t jitter_ms;       // p.jitter_ms
    uint16_t max_mid;         // p.max_mid
    enum { ORDER_MID_DESC=0 } order;
} resp_slot_cfg_t;

typedef struct {
    uint8_t  in_use;
    uint8_t  compact_snap;

    uint16_t tmid;                 
    uint8_t  data[256];   // Wi-SUN DATA 부분만
    uint16_t data_len;             // DATA 길이
    uint32_t due_tick;             
} hop_slot_t;

#ifndef LIGHT_EVENT_FFT_PAIRS
#define LIGHT_EVENT_FFT_PAIRS 2
#endif

typedef struct {
    uint8_t  pending;
    uint32_t event_id;
    uint8_t  valid_flags;
    uint8_t  light_on;
    uint8_t  mode;
    uint8_t  reason;
    uint32_t tick_ms;
    float    voltage;
    float    current;
    float    temp;
    uint8_t  fft_count;
    float    fft_freq[LIGHT_EVENT_FFT_PAIRS];
    float    fft_amp[LIGHT_EVENT_FFT_PAIRS];
    uint16_t rtc_year;
    uint8_t  rtc_month;
    uint8_t  rtc_day;
    uint8_t  rtc_hour;
    uint8_t  rtc_min;
    uint8_t  rtc_sec;
    uint8_t  rtc_synced;
} light_state_event_t;

typedef struct {
    uint8_t  valid;
    float    voltage;
    float    current;
    float    temp;
    uint8_t  fft_count;
    float    fft_freq[LIGHT_EVENT_FFT_PAIRS];
    float    fft_amp[LIGHT_EVENT_FFT_PAIRS];
    uint32_t tick_ms;
    uint32_t snap_count;
} light_event_sensor_cache_t;

typedef struct __attribute__((packed)) {
    uint8_t  t;       // 0x10 = generic_ack
    uint8_t  uid[12];

    uint32_t msg_id;  
    uint8_t  ok;      // 1=success, 0=fail
    int8_t   err_code;
} AckBin_t;

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

typedef struct __attribute__((packed)) {
    uint8_t  t;        // 0x11 = slot_resp 
    uint8_t  uid[12];  

    uint16_t mid;     
    uint8_t  ok;       // 1=success, 0=fail

    uint8_t  has_on;   // g_resp.p_on_valid
    uint8_t  on;       // g_resp.p_on (0/1)

    uint16_t nonce;    // g_resp.nonce 
} SlotRespBin_t;

#pragma pack(push, 1)
typedef struct {
  uint8_t t;         // T_NODEINFO_BIN
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

static node_info_cache_t g_node_info = {
    .gid = 0,
    .dev = 1,
    .dsp = 1,
    .txp = 13,
    .rch0 = 0,
    .rch1 = 0xFF,
	.mode = 0,
    .mac = {0},
	.fw_major=0, .fw_minor=0,
    .valid = 0
};

#define RESP_QUEUE_SIZE 4
#define LIGHT_STATE_EVENT_QUEUE_SIZE 4
static node_cfg_t g_node_cfg;
static resp_slot_t g_resp_slot;
static resp_slot_t g_resp_q[RESP_QUEUE_SIZE];
static AckBin_t g_nodeinfo_ack;
static uint8_t  g_manual_override_active   = 0;
static uint8_t  g_manual_override_light_on = 0;
static uint32_t g_manual_override_until    = 0;
static uint8_t  g_manual_override_no_timeout = 0;
static uint8_t  g_manual_override_latch_off_on_expire = 0;
static uint8_t  g_rtc_synced               = 0;
static light_state_event_t g_light_event_q[LIGHT_STATE_EVENT_QUEUE_SIZE];
static uint8_t  g_light_event_head = 0;
static uint8_t  g_light_event_tail = 0;
static uint8_t  g_light_event_count = 0;
static uint32_t g_light_event_seq = 0;
static uint8_t  g_light_event_reason_context = 0;
static light_event_sensor_cache_t g_light_sensor_cache = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_AT_TO_PC
#define RX_BUFFER_SIZE 100
#define PACKET_MAX_SIZE 256
#define HOP_QUEUE_SIZE   16   
#define HOP_MAX_FRAME 256

#define HOP_TTL_DEFAULT  20   // TTL 기본크기
#define HOP_SEEN_TABLE_SIZE 16
#define NTC_LUT_SIZE 25

#define AT_LINE_Q_CAP   8
#define AT_LINE_MAX     128

#define T_NODEINFO_BIN  0x14

#define UID_ADDRESS  ((uint32_t*) 0x08FFF800)
#define __DCACHE_PRESENT 1U
#define __ICACHE_PRESENT 1U
//#define VREFINT_CAL_ADDR  ((uint16_t*) (0x08FFF810))
//#define VREFINT_CAL_VALUE  (*VREFINT_CAL_ADDR)
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

#define BASE_ALPHA        (0.0015f)
#define NOISE_ALPHA       (0.01f)
#define TRIG_K            (4.0f)
#define TRIG_MARGIN       (0.005f)
#define TRIG_HOLD_SAMPLES (32u)
#define NOISE_MIN         (0.001f)

#define FFT_SNR_K  5.0f
#define FFT_NOISE_MIN    (1e-6f)

#define SAMPLING_RATE 600000.0f

#define PACKET_STX    0x02
#define PACKET_ETX    0x03
#define SIG1          0xAA
#define SIG2          0xAB

#define LIGHT_ON 0x10
#define LIGHT_OFF 0x11
#define SNAP_REPORT_CMD 0x12
#define LIGHT_STATE_EVENT_CMD 0x15
#define SNAP_USE_LEGACY_STRUCT_TEST 0
#define GET_STATUS 0x30
#define SET_MID  0x01
#define NODE_CFG 0x20
#define SET_MID_CH  0x21
#define FIRST_BOOT  0x22
#define GETID       0x23
#define SET_RTC_KST 0x2B
#define SET_SETTING    0x31
#define GET_NODE_INFO 0x40
#define GET_CH 0x24
#define SET_CH 0x25
#define SET_POWER
#define NODE_CFG_MAGIC      0x4E434647u   // 'N' 'C' 'F' 'G'
#define T_GET_CH_RESP 0x24
#define T_NODE_INFO_TEXT 0x40   
#define NODE_INFO_TEXT_MAX 240
#define LIGHT_TEST_10S_ENABLE    0
#define LIGHT_TEST_10S_PERIOD_MS 10000u
#define CMD_ACK_RELAY 0x7E
#define FOCUS_TIMING_LOG 0
#if FOCUS_TIMING_LOG
#undef DEBUG_AT_TO_PC
#endif

#define LIGHT_EVENT_REASON_UNKNOWN        0u
#define LIGHT_EVENT_REASON_CMD            1u
#define LIGHT_EVENT_REASON_SET_FORCED     2u
#define LIGHT_EVENT_REASON_FORCED_EXPIRE  3u
#define LIGHT_EVENT_REASON_SCHEDULE       4u
#define LIGHT_EVENT_REASON_SAVING         5u
#define LIGHT_EVENT_REASON_RTC_UNSYNCED   6u
#define LIGHT_EVENT_REASON_TEST           7u
#define LIGHT_EVENT_REASON_NODE_CFG       8u

#define LIGHT_EVENT_VALID_LIGHT 0x01u
#define LIGHT_EVENT_VALID_VI    0x02u
#define LIGHT_EVENT_VALID_TEMP  0x04u
#define LIGHT_EVENT_VALID_FFT   0x08u
#define LIGHT_EVENT_VALID_RTC   0x10u
#define LIGHT_EVENT_REQUIRED_SENSOR_FLAGS (LIGHT_EVENT_VALID_LIGHT | LIGHT_EVENT_VALID_VI | LIGHT_EVENT_VALID_TEMP | LIGHT_EVENT_VALID_FFT)

#define LIGHT_STATE_EVENT_BODY_LEN (1u + 12u + 4u + 1u + 1u + 1u + 1u + 4u + 4u + 4u + 4u + 1u + (8u * LIGHT_EVENT_FFT_PAIRS) + 2u + 1u + 1u + 1u + 1u + 1u + 1u)

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

#pragma pack(push, 1)
typedef struct {
    uint8_t  t;          // 0x40
    uint8_t  uid[12];    // uid12
    uint16_t msg_id;     // 요청 msg_id 
    uint8_t  ok;         // 1=성공, 0=실패
    int8_t   err_code;   
    uint16_t text_len;   
    // uint8_t text[text_len];
} NodeInfoTextHdr_t;
#pragma pack(pop)

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

#define ADC_MAX_COUNTS    4095.0f
#define VREF_FIXED        3.3f

#define R_TOP_V           100000.0f   // ?? 100k
#define R_BOT_V           10000.0f    // ?? 10k
#define V_DIV_GAIN        ((R_TOP_V + R_BOT_V) / R_BOT_V)   

#define R_SHUNT           0.05f
#define I_AMP_GAIN        50.0f       // 증폭배수(x)

#define I_OFFSET_RAW    1990
#define SNAP_RING_SIZE  1
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

#ifndef MID_INVALID
#define MID_INVALID ((uint16_t)0xFFFF)
#endif

#ifndef SNAP_FFT_PAIRS
#define SNAP_FFT_PAIRS 2  
#endif

#define FFT_FREQ_SCALE 100.0f
#define FFT_AMP_SCALE  1000.0f
#define AI_MSE_SCALE   1000000.0f

#define SNAP_COMPACT_BODY_LEN 40u
#define SNAP_COMPACT_TTL_DEFAULT 3u
#define SNAP_COMPACT_TTL_IDX 1u
#define SNAP_COMPACT_UID_IDX 2u
#define SNAP_COMPACT_UID_LEN 12u
#define SNAP_COMPACT_SNAP_COUNT_IDX 35u
#define SNAP_AI_RESULT_LEN (1u + 4u + 1u)
#define SNAP_BIN_BODY_LEN (1u + 12u + 4u + 4u + 4u + 1u + 1u + (8u * SNAP_FFT_PAIRS) + 4u + 4u + 1u + 1u + SNAP_AI_RESULT_LEN)

#ifndef SUP_MIN_HZ
#define SUP_MIN_HZ 20000.0f   // 20 kHz
#endif
#ifndef SUP_MAX_HZ
#define SUP_MAX_HZ 80000.0f   // 80 kHz
#endif

typedef struct __attribute__((packed)) {
    uint8_t  t;         
    uint8_t  uid[12];

    float    volt;
    float    curr;
    float    temp;
    uint8_t  light_on;  // 0/1

    uint8_t  fft_count;
    struct {
        uint32_t freq_x100;
        int32_t amp_x1000;
    } fft[SNAP_FFT_PAIRS];
    
       uint32_t snap_count;
       uint32_t msg_id;   
       uint8_t  ai_valid;
       uint32_t ai_mse_x1000000;
       int8_t   ai_pred;
       uint8_t  ok;       // 1=success
       int8_t   err_code; 
} SnapBin_t;

typedef struct {
    uint16_t len;
    char     s[AT_LINE_MAX];
} at_line_t;

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
static snapshot_t g_snap[SNAP_RING_SIZE];
static uint8_t    g_snap_head = 0;
static bool       g_snap_inited = false;
static uint8_t  g_snap_enable      = 0;       // 0: off, 1: on
static uint32_t g_snap_interval_ms = 60000;   // 기본 60s
static uint32_t g_snap_next_tick   = 0;

static char     g_uid_str[40];
static uint32_t uid_ram_local[3];
static uint32_t last_snap_req_tick = 0;
static volatile bool     wisun_packet_ready = false;
static volatile uint16_t wisun_packet_len   = 0;
static volatile uint32_t wisun_packet_ready_tick = 0;
static uint8_t           wisun_packet_shadow[PACKET_MAX_SIZE];
static volatile uint16_t wisun_rx_need  = 0;
static volatile uint8_t g_wait_mid_query = 0;
volatile uint32_t g_ultra_frame_tick = 0;
//static uint8_t capturing = 0;

static float ai_mse = 0.0f;
static int ai_pred = 0;
static char at_accum[256];
static uint16_t at_accum_len = 0;

static bool on_done_today  = false;
static bool off_done_today = false;
static const float ntc_temp_table[NTC_LUT_SIZE] = {
    -40.000000f, -35.000000f, -30.000000f, -25.000000f, -20.000000f, -15.000000f, -10.000000f, -5.000000f,
    0.000000f,   5.000000f,   10.000000f,  15.000000f,  20.000000f,  25.000000f,  30.000000f,  35.000000f,
    40.000000f,  45.000000f,  50.000000f,  55.000000f,  60.000000f,  65.000000f,  70.000000f,  75.000000f,
    80.000000f
};
static const float ntc_voltage_table[NTC_LUT_SIZE] = {
    3.197196f, 3.162500f, 3.117680f, 3.060870f, 2.988679f, 2.902410f, 2.800000f, 2.677358f,
    2.532558f, 2.357143f, 2.200000f, 1.980000f, 1.800000f, 1.650000f, 1.466259f, 1.291357f,
    1.128233f, 0.979325f, 0.845373f, 0.726901f, 0.623167f, 0.533168f, 0.455663f, 0.389177f,
    0.332641f
};
static hop_slot_t g_hop_q[HOP_QUEUE_SIZE];

static uint16_t g_hop_seen_msg_ids[HOP_SEEN_TABLE_SIZE];
static uint8_t  g_hop_seen_count = 0;  
static uint8_t  g_hop_seen_pos   = 0;  

// ?�짜 바�????�래�?리셋??
static uint8_t sched_last_day = 0xFF;

static nodeinfo_ctx_t g_nodeinfo = {0};

static uint8_t g_nodeinfo_txbuf[sizeof(NodeInfoTextHdr_t) + NODE_INFO_TEXT_MAX];

static const float K_ADC2V   = VREF_FIXED / ADC_MAX_COUNTS;                 // raw ??V
//static const float K_VIN     = K_ADC2V * V_DIV_GAIN;                        // raw ??Vin(V)
//static const float K_CURR    = K_ADC2V / (R_SHUNT * I_AMP_GAIN);
//static volatile uint16_t adc_buffer[FFT_TOTAL_SAMPLES];
static volatile int      sample_index = 0;
static volatile resp_task_t g_resp = {0};
volatile uint8_t adc_done = 0;
volatile uint32_t g_monitor_count = 0;
static volatile int       wr_idx           = 0;
static volatile size_t   ultra_idx = 0;
static volatile bool     ultra_frame_ready = false;
static volatile bool     ultra_sampling_paused = false;

volatile uint32_t g_frame_c0 = 0;
volatile uint32_t g_frame_c1 = 0;

volatile uint32_t g_dma_done = 0;
volatile uint32_t g_dma_half = 0;

static float baseline = 0.0f;
static float noise_level = 0.01f;  
static uint8_t trig_on = 0;
static uint16_t trig_hold = 0;
static float32_t g_hann[FFT_SIZE];
static uint8_t g_hann_inited = 0;

static volatile uint8_t  g_ai_sample_ready = 0;
static volatile float    g_ai_sample = 0.0f;

volatile uint32_t g_adc_isr_cnt = 0;
volatile uint32_t g_trig_fire_cnt = 0;
volatile uint32_t g_frame_ready_cnt = 0;
volatile float    g_last_dev = 0.0f, g_last_thr = 0.0f, g_last_noise = 0.0f;
volatile uint16_t g_last_wr = 0;
volatile uint8_t  g_last_cap = 0;

static uint16_t uart_fail_cnt   = 0;
static volatile uint32_t t_frame_start_us = 0;
static uint8_t pa12_state = 0;
static volatile uint32_t t_frame_end_us   = 0;
volatile uint8_t receive_flag = 0;
volatile uint32_t tick_start = 0, tick_end = 0;
volatile uint32_t uid_ram[3];
volatile uint16_t g_last_rx_tmid = 0;
volatile uint8_t ai_ready = 0;
volatile uint8_t  ai_pending = 0;
uint32_t          ai_next_run = 0;      
uint32_t          ai_period_ms = 200;   
uint32_t          ai_budget_ms = 10;
static uint32_t g_last_msg_id      = 0;
static int      g_last_cmd         = -1;
static uint8_t  g_last_has_msg_id  = 0;
static uint8_t  g_last_has_cmd     = 0;
static uint32_t g_hop_seen_keys[HOP_SEEN_TABLE_SIZE];

static uint8_t boot_cfg_started = 0;

static float ai_input[AE_COLS];
static int   ai_index = 0;

static volatile uint8_t  g_at_line_ready = 0;
static uint16_t          g_at_line_len   = 0;
static char              g_at_line[RX_BUFFER_SIZE];

static volatile uint8_t  at_q_w = 0;
static volatile uint8_t  at_q_r = 0;
static at_line_t         at_q[AT_LINE_Q_CAP];

static char ascii_buffer[RX_BUFFER_SIZE];
static uint16_t ascii_index = 0;

bool fft_data_sent = false;

static uint8_t g_nodeinfo_seen_fwver = 0;
static uint8_t g_nodeinfo_wait_final_ok = 0;

uint8_t last_received_packet[RX_BUFFER_SIZE];
uint16_t last_received_len = 0;
uint8_t rxBuffer[100];
uint8_t ADCFlag = 0;
uint8_t rxByte, rxByte1;
uint8_t wisun_rx;
uint8_t pcBuffer[RX_BUFFER_SIZE];
uint8_t pc_rx_buffer[RX_BUFFER_SIZE];
uint8_t wisun_rx_buffer[RX_BUFFER_SIZE];
uint8_t tx_forward_buffer[RX_BUFFER_SIZE];
uint8_t pc_rx_index = 0;
uint8_t wisun_rx_index = 0;
uint8_t packet_buffer[256];
uint8_t packet_index = 0;
uint8_t wisun_packet_index = 0;
uint8_t packet_mode = 0;
uint8_t send_count = 0;
uint8_t g_region_code;
uint8_t g_rtc_hour;  // 0~23
uint8_t g_rtc_min;   // 0~59
uint8_t g_rtc_sec;   // 0~59

uint16_t g_rtc_day;  // ?�수??보통 1~31 (uint8_t??가??
uint8_t g_rtc_month;
uint16_t g_rtc_year;

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

uint32_t last_debug_tick = 0;

uint32_t prev_tick = 0;
uint32_t final_response[100];
uint16_t sunrise, sunset, dawn, dusk;
uint16_t vrefint_adc;
uint16_t adc_dma_buffer[FFT_SIZE];
uint16_t my_mid = 0x0000;
uint16_t target_mid = 0;
uint16_t adc_raw_volt = 0;
uint16_t adc_raw_curr = 0;
uint16_t adc_raw_temp = 0;
uint16_t raw_buffer[FFT_SIZE];
uint16_t loaded;

float32_t inputSignal[FFT_SIZE];
float32_t outputSignal[FFT_SIZE];
float32_t magnitude[FFT_SIZE / 2];
float32_t totalMagnitude = 0;
float32_t energy = 0;
float32_t I_OFFSET_VOLT = 0.0f;
uint32_t last_tick = 0;

static uint16_t prev_dawn = 0xFFFF;
static uint16_t prev_dusk = 0xFFFF;

char debug_msg[64];
char wisun_mid[10] = {0};
char wisun_mac[20] = {0};

volatile uint8_t sendIDFlag = 0;
volatile uint8_t request_step = 0;
volatile uint16_t adc_index = 0;
volatile uint8_t fft_ready;
volatile uint8_t packet_flag = 0;
volatile uint32_t g_sec_tick = 0;
volatile uint8_t started = 0;
volatile uint8_t g_fault_inject = 0;
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

void ADC1_Start_Regular_IN18_IT(void);
static inline float adc_to_vsense(uint16_t raw);
static void apply_mid(uint16_t new_mid);
static void apply_rch(uint8_t r0, uint8_t r1);
static void apply_mid_chan_from_cfg(void);
static void ai_service(void);
static inline uint8_t at_q_next(uint8_t x);
static inline uint8_t at_q_is_full(void);
static inline uint8_t at_q_is_empty(void);
static void at_accum_reset(void);
static bool is_uplink_report_cmd(uint8_t cmd);
void boot_poll(void);
//static void build_snapshot_packet(cbor_packet_t* rp, const snapshot_t* s, bool as_resp, const char* topic, uint16_t nonce);
static uint8_t enqueue_transport_tx(uint16_t tmid, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len, uint8_t ttl);
float Convert_ADC_To_Current(uint32_t adc_value);
float Convert_Voltage_To_Current(float voltage, float offset);
float Convert_Voltage_ADC(uint32_t adc_value);
void dbg_dump_uart6_with_tag(const char *tag, const uint8_t *p, uint16_t n);
void dbg_print_mid_info(const char *tag, uint16_t my_mid, uint16_t target_mid);
void debug_print_boot_info(uint16_t stored_mid);
void debug6(const char *s);
static void timing_log(const char *fmt, ...);
static uint8_t focus_timing_log_enabled(void);
static inline void DWT_CYCCNT_Init(void);
void Debug_Print_FFT_Peak(void);
void ExtractFullFFT(const float32_t *in, float fs_hz, FftData_t *dest);
static void ExtractFullFFT_MagOnly(const float32_t *in, float32_t *mag_out);
void Format_UID(char *msg, size_t size);
static uint16_t find_first_zero(const uint8_t *p, uint16_t n);
static void hop_tx_task_poll(void);
static void init_uid_string(void);
static inline int is_night(int now, int dusk, int dawn);
static bool is_bootstrap_cmd(uint8_t cmd);
void Input_Ai_Model(float v);
void Input_Ai_Model_Features(float freq_khz, float adc_pk, float current, float vin);
static void InitHannWindowOnce(void);
uint32_t Get_Device_ID(void);
float Get_Calibrated_Vref(void);
float Get_Offset_Voltage(void);
//void StreetLight_ToggleTask(void);
static void handle_binary_cmd(uint8_t cmd, uint8_t flags, uint16_t msg_id, uint16_t tmid, const uint8_t *data, uint16_t len);
static bool hop_seen_key(uint32_t key);
static void hop_mark_key(uint32_t key);
static uint8_t handle_cmd_set_setting(const uint8_t *data, uint16_t len);
static uint8_t handle_cmd_set_rtc_kst(const uint8_t *data, uint16_t len);
void light_off(void);
void light_on(void);
static uint8_t current_control_mode(void);
static uint16_t manual_override_duration_min(void);
static void start_manual_override(uint8_t want_on);
static void start_manual_override_for(uint8_t want_on, uint16_t duration_min);
static void start_forced_time_control(uint16_t forced_time_min);
static GPIO_PinState light_pin_state_for(uint8_t want_on);
static uint8_t light_is_on_logical(void);
static void light_event_set_reason(uint8_t reason);
static void light_state_event_note_if_changed(uint8_t before_on, uint8_t after_on);
static uint8_t light_state_event_fill_measurements(light_state_event_t *ev);
static uint8_t encode_light_state_event_bin(uint8_t *out, uint16_t out_cap, const light_state_event_t *ev);
static void light_state_event_poll(void);
static void light_sensor_cache_update(float voltage, float current, float temp, uint8_t fft_count, const float *fft_freq, const float *fft_amp, uint32_t snap_count);
static uint16_t apply_time_correction_min(uint16_t base_min, uint8_t corr_mode, uint16_t corr_delta_min);
static uint8_t time_window_contains(uint16_t now_min, uint16_t start_min, uint16_t end_min);
static void mid_pack_uid12(uint8_t out12[12]);
static uint16_t my_strnlen(const char *s, uint16_t maxn);
static bool node_is_provisioned(void);
float ntc_voltage_to_temp_c(float temp_v);
static int nodeinfo_append_kv_line(const char *line);
static void nodeinfo_start(uint16_t tmid, uint16_t msg_id);
static void nodeinfo_send_err(uint16_t tmid, uint16_t msg_id, int8_t err_code);
void nodeinfo_collect_line(const char *line);
static void nodeinfo_cache_update_from_kv(const char *kv_line);
static void nodeinfo_finish_and_send(uint8_t ok, int8_t err_code);
void nodeinfo_finish_ok(void);
void nodeinfo_finish_fail(int8_t err);
static void nodeinfo_poll(uint32_t now);
static int norm_min(int t);
static uint8_t rtc_weekday_from_ymd(uint16_t year, uint8_t month, uint8_t day);
void Parse_AT_Response(const char* buffer);
void PrintReceivedPacket(const char* prefix, const uint8_t* data, uint16_t length);
void Print_Voltage_Current(void);
static inline void PA12_toggle_soft(void);
static void push_snapshot(bool light_on, float voltage, float current, float temp, float supersonic);
static int parse_hex8(const char *s, uint8_t out[8]);
static int hexval(char c);
static uint8_t try_handle_uart6_local_cmd(const char *line);
static uint8_t apply_snapshot_uart_cmd(const char *arg);
static uint16_t build_transport_payload(uint16_t target_mid, uint8_t ttl, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len, uint8_t *out, uint16_t out_cap);
static uint16_t encode_snap_bin(uint8_t *out, uint16_t out_cap, const uint8_t uid12[12], float volt, float curr, float temp, uint8_t light_on, uint8_t fft_count, const float *fft_freq, const float *fft_amp, uint32_t snap_count, uint32_t msg_id, uint8_t ok, int8_t err_code, uint8_t ai_valid, uint32_t ai_mse_x1000000, int8_t ai_pred_value);
static void log_snap_gateway_tx(const char *tag, uint16_t tmid, const uint8_t *data, uint16_t len);
static uint8_t send_transport_direct(uint16_t target_mid, uint8_t ttl, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len);
void Query_MID_From_WiSUN(void);
void Read_UID(void);
uint32_t Read_Voltage_ADC(void);
uint32_t Read_ADC_Channel(uint32_t channel);
static void Read_UID_local(void);
static void resp_slot_task_poll(void);
void readADCData(void);
void rtc_minute_tick(uint8_t hour, uint8_t min);
static void rstrip_inplace(char *s);
static void scheduler_poll(void);
void Send_UID_UART2(void);
void Send_Device_ID_UART2(void);
void Send_Broadcast_Command(uint8_t *request_data, uint8_t request_length);
void SendFFT_Packet(uint16_t target_mid, FftData_t *fft_data, uint8_t count);
void SendDataPacket(uint16_t target_mid, uint8_t *data, uint16_t data_length);
void Send_Monitoring_Snapshot_JSON(uint16_t req_msg_id);
static void schedule_resp_with_slot(resp_kind_t kind, uint16_t tmid, uint16_t msg_id, const uint8_t *raw, uint16_t raw_len);
//static bool send_wisun_resp(uint16_t tmid, const uint8_t *cbor, size_t cbor_len);
static bool send_wisun_binary(uint16_t tmid, const uint8_t *data, size_t len);
static void send_resp_now_from_task(void);
void tx_task_poll(void);
void Ultra_ResumeNextFrame(void);
void Ultra_StartSampling(void);
void Ultra_StartDmaFrame(void);
static void update_sun_times(void);
static inline float vsense_to_vin(float v_sense);
static inline float vsense_to_current(float v_sense, float offset_v);
void wisun_toss_packet(uint16_t tmid, const uint8_t *rx, uint16_t rx_len);
void wisun_process_rx_mainloop(void);
static inline uint16_t xorshift16(uint16_t x);
static uint16_t wisun_expected_packet_len(const uint8_t *buf, uint16_t have_len);
static bool is_compact_snap_body(const uint8_t *data, uint16_t len);
void reconfigure_snapshot_timer_from_cfg(void);
void rtc_update(void);
bool rtc_ensure_valid_or_set_default(void);
void uart6_log(const char *fmt, ...);

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

static inline float adc_to_vsense(uint16_t raw) {
    return ((float)raw * VREF_FIXED) / ADC_MAX_COUNTS;
}

static inline float vsense_to_vin(float v_sense) {
    return v_sense * V_DIV_GAIN;
}

static inline float vsense_to_current(float v_sense, float offset_v) {
    // v_sense = offset_v + I * R_SHUNT * I_AMP_GAIN
    return (v_sense - offset_v) / (R_SHUNT * I_AMP_GAIN);
}
static inline void PA12_toggle_soft(void){
    pa12_state ^= 1;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,
        pa12_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline uint8_t at_q_next(uint8_t x) { return (uint8_t)((x + 1) % AT_LINE_Q_CAP); }
static inline uint8_t at_q_is_full(void)   { return at_q_next(at_q_w) == at_q_r; }
static inline uint8_t at_q_is_empty(void)  { return at_q_w == at_q_r; }

static uint16_t build_transport_payload(uint16_t target_mid, uint8_t ttl, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len, uint8_t *out, uint16_t out_cap)
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

static uint32_t scale_fft_freq_x100(float freq_hz)
{
    double scaled;

    if (!isfinite(freq_hz) || freq_hz <= 0.0f) {
        return 0u;
    }

    scaled = ((double)freq_hz * (double)FFT_FREQ_SCALE) + 0.5;
    if (scaled >= (double)UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)scaled;
}

static int32_t scale_fft_amp_x1000(float amp)
{
    double scaled;

    if (!isfinite(amp)) {
        return 0;
    }

    scaled = (double)amp * (double)FFT_AMP_SCALE;
    scaled += (scaled >= 0.0) ? 0.5 : -0.5;

    if (scaled > (double)INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled < (double)INT32_MIN) {
        return INT32_MIN;
    }

    return (int32_t)scaled;
}

static uint32_t scale_ai_mse_x1000000(float mse)
{
    double scaled;

    if (!isfinite(mse) || mse <= 0.0f) {
        return 0u;
    }

    scaled = ((double)mse * (double)AI_MSE_SCALE) + 0.5;
    if (scaled >= (double)UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)scaled;
}

static float snap_round_4dp(float v)
{
    if (!isfinite(v)) {
        return 0.0f;
    }

    return roundf(v * 10000.0f) * 0.0001f;
}

static uint16_t encode_snap_bin(uint8_t *out, uint16_t out_cap, const uint8_t uid12[12], float volt, float curr, float temp, uint8_t light_on, uint8_t fft_count, const float *fft_freq, const float *fft_amp, uint32_t snap_count, uint32_t msg_id, uint8_t ok, int8_t err_code, uint8_t ai_valid, uint32_t ai_mse_x1000000, int8_t ai_pred_value)
{
    uint16_t off = 0u;
    float volt_4dp = snap_round_4dp(volt);
    float curr_4dp = snap_round_4dp(curr);
    float temp_4dp = snap_round_4dp(temp);

    if (out == NULL || uid12 == NULL || out_cap < SNAP_BIN_BODY_LEN) {
        return 0u;
    }

    out[off++] = 0x01u;
    memcpy(&out[off], uid12, 12u);
    off += 12u;

    memcpy(&out[off], &volt_4dp, sizeof(volt_4dp));
    off += (uint16_t)sizeof(volt_4dp);

    memcpy(&out[off], &curr_4dp, sizeof(curr_4dp));
    off += (uint16_t)sizeof(curr_4dp);

    memcpy(&out[off], &temp_4dp, sizeof(temp_4dp));
    off += (uint16_t)sizeof(temp_4dp);

    out[off++] = light_on;
    out[off++] = fft_count;

    for (uint8_t k = 0; k < SNAP_FFT_PAIRS; ++k) {
        float freq = 0.0f;
        float amp = 0.0f;
        uint32_t freq_x100;
        int32_t amp_x1000;

        if (k < fft_count) {
            if (fft_freq != NULL) freq = fft_freq[k];
            if (fft_amp  != NULL) amp  = fft_amp[k];
        }

        freq_x100 = scale_fft_freq_x100(freq);
        amp_x1000 = scale_fft_amp_x1000(amp);

        memcpy(&out[off], &freq_x100, sizeof(freq_x100));
        off += (uint16_t)sizeof(freq_x100);
        memcpy(&out[off], &amp_x1000, sizeof(amp_x1000));
        off += (uint16_t)sizeof(amp_x1000);
    }

    memcpy(&out[off], &snap_count, sizeof(snap_count));
    off += (uint16_t)sizeof(snap_count);

    memcpy(&out[off], &msg_id, sizeof(msg_id));
    off += (uint16_t)sizeof(msg_id);

    out[off++] = ai_valid ? 1u : 0u;
    memcpy(&out[off], &ai_mse_x1000000, sizeof(ai_mse_x1000000));
    off += (uint16_t)sizeof(ai_mse_x1000000);
    out[off++] = (uint8_t)ai_pred_value;

    out[off++] = ok;
    out[off++] = (uint8_t)err_code;

    return off;
}

static uint16_t encode_snap_compact_bin(uint8_t *out, uint16_t out_cap, const uint8_t uid12[12], float volt, float curr, float temp, uint8_t light_on, uint8_t has_fft0, float fft0_freq, float fft0_amp, uint32_t snap_count, uint8_t ai_valid, uint32_t ai_mse_x1000000, int8_t ai_pred_value, uint8_t ok, uint8_t ttl)
{
    uint16_t off = 0u;
    float volt_4dp = snap_round_4dp(volt);
    float curr_4dp = snap_round_4dp(curr);
    float temp_4dp = snap_round_4dp(temp);
    uint32_t freq_x100 = has_fft0 ? scale_fft_freq_x100(fft0_freq) : 0u;
    int32_t amp_x1000 = has_fft0 ? scale_fft_amp_x1000(fft0_amp) : 0;
    uint16_t snap_count16 = (uint16_t)(snap_count & 0xFFFFu);
    uint16_t ai_mse16 = (ai_mse_x1000000 > 0xFFFFu) ? 0xFFFFu : (uint16_t)ai_mse_x1000000;
    uint8_t flags = 0u;

    if (out == NULL || uid12 == NULL || out_cap < SNAP_COMPACT_BODY_LEN) {
        return 0u;
    }

    if (ai_valid) flags |= 0x01u;
    if (ai_pred_value != 0) flags |= 0x02u;
    if (ok) flags |= 0x04u;

    out[off++] = 0x01u;
    out[off++] = ttl;
    memcpy(&out[off], uid12, 12u);
    off += 12u;

    memcpy(&out[off], &volt_4dp, sizeof(volt_4dp));
    off += (uint16_t)sizeof(volt_4dp);
    memcpy(&out[off], &curr_4dp, sizeof(curr_4dp));
    off += (uint16_t)sizeof(curr_4dp);
    memcpy(&out[off], &temp_4dp, sizeof(temp_4dp));
    off += (uint16_t)sizeof(temp_4dp);

    out[off++] = light_on ? 1u : 0u;

    memcpy(&out[off], &freq_x100, sizeof(freq_x100));
    off += (uint16_t)sizeof(freq_x100);
    memcpy(&out[off], &amp_x1000, sizeof(amp_x1000));
    off += (uint16_t)sizeof(amp_x1000);

    memcpy(&out[off], &snap_count16, sizeof(snap_count16));
    off += (uint16_t)sizeof(snap_count16);
    memcpy(&out[off], &ai_mse16, sizeof(ai_mse16));
    off += (uint16_t)sizeof(ai_mse16);

    out[off++] = flags;

    return off;
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

static uint8_t send_transport_direct(uint16_t target_mid, uint8_t ttl, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len)
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

static uint8_t encode_light_state_event_bin(uint8_t *out, uint16_t out_cap, const light_state_event_t *ev)
{
    uint16_t off = 0u;

    if (out == NULL || ev == NULL || out_cap < LIGHT_STATE_EVENT_BODY_LEN) {
        return 0u;
    }

    out[off++] = LIGHT_STATE_EVENT_CMD;
    mid_pack_uid12(&out[off]);
    off += 12u;

    memcpy(&out[off], &ev->event_id, sizeof(ev->event_id));
    off += (uint16_t)sizeof(ev->event_id);

    out[off++] = ev->valid_flags;
    out[off++] = ev->light_on;
    out[off++] = ev->mode;
    out[off++] = ev->reason;

    memcpy(&out[off], &ev->tick_ms, sizeof(ev->tick_ms));
    off += (uint16_t)sizeof(ev->tick_ms);

    memcpy(&out[off], &ev->voltage, sizeof(ev->voltage));
    off += (uint16_t)sizeof(ev->voltage);

    memcpy(&out[off], &ev->current, sizeof(ev->current));
    off += (uint16_t)sizeof(ev->current);

    memcpy(&out[off], &ev->temp, sizeof(ev->temp));
    off += (uint16_t)sizeof(ev->temp);

    out[off++] = ev->fft_count;

    for (uint8_t k = 0; k < LIGHT_EVENT_FFT_PAIRS; ++k) {
        float freq = 0.0f;
        float amp = 0.0f;
        uint32_t freq_x100;
        int32_t amp_x1000;

        if (k < ev->fft_count) {
            freq = ev->fft_freq[k];
            amp = ev->fft_amp[k];
        }

        freq_x100 = scale_fft_freq_x100(freq);
        amp_x1000 = scale_fft_amp_x1000(amp);

        memcpy(&out[off], &freq_x100, sizeof(freq_x100));
        off += (uint16_t)sizeof(freq_x100);

        memcpy(&out[off], &amp_x1000, sizeof(amp_x1000));
        off += (uint16_t)sizeof(amp_x1000);
    }

    memcpy(&out[off], &ev->rtc_year, sizeof(ev->rtc_year));
    off += (uint16_t)sizeof(ev->rtc_year);
    out[off++] = ev->rtc_month;
    out[off++] = ev->rtc_day;
    out[off++] = ev->rtc_hour;
    out[off++] = ev->rtc_min;
    out[off++] = ev->rtc_sec;
    out[off++] = ev->rtc_synced;

    return (uint8_t)off;
}

static void light_event_set_reason(uint8_t reason)
{
    g_light_event_reason_context = reason;
}

static void light_sensor_cache_update(float voltage, float current, float temp, uint8_t fft_count, const float *fft_freq, const float *fft_amp, uint32_t snap_count)
{
    light_event_sensor_cache_t cache;

    if (fft_count == 0u || fft_freq == NULL || fft_amp == NULL) {
        /* uart6_log("[LIGHT_CACHE_SKIP] snap=%lu reason=no_fft voltage=%f current=%f temp=%f\r\n",
                  (unsigned long)snap_count,
                  voltage,
                  current,
                  temp); */
        return;
    }

    memset(&cache, 0, sizeof(cache));
    cache.valid = 1u;
    cache.voltage = voltage;
    cache.current = current;
    cache.temp = temp;
    cache.fft_count = (fft_count > LIGHT_EVENT_FFT_PAIRS) ? LIGHT_EVENT_FFT_PAIRS : fft_count;
    for (uint8_t k = 0; k < cache.fft_count; ++k) {
        cache.fft_freq[k] = fft_freq[k];
        cache.fft_amp[k] = fft_amp[k];
    }
    cache.tick_ms = HAL_GetTick();
    cache.snap_count = snap_count;

    __disable_irq();
    g_light_sensor_cache = cache;
    __enable_irq();

    /* uart6_log("[LIGHT_CACHE_SET] snap=%lu temp=%f fft_count=%u fft0=%f/%f\r\n",
              (unsigned long)snap_count,
              temp,
              (unsigned)cache.fft_count,
              cache.fft_freq[0],
              cache.fft_amp[0]); */
}

static uint8_t light_state_event_fill_measurements(light_state_event_t *ev)
{
    light_event_sensor_cache_t cache;

    if (ev == NULL) {
        return 0u;
    }

    ev->valid_flags |= LIGHT_EVENT_VALID_LIGHT;

    memset(&cache, 0, sizeof(cache));
    __disable_irq();
    cache = g_light_sensor_cache;
    __enable_irq();

    if (cache.valid) {
        uint32_t cache_age = HAL_GetTick() - cache.tick_ms;
        ev->voltage = cache.voltage;
        ev->current = cache.current;
        ev->temp = cache.temp;
        ev->fft_count = cache.fft_count;
        for (uint8_t k = 0; k < LIGHT_EVENT_FFT_PAIRS; ++k) {
            ev->fft_freq[k] = cache.fft_freq[k];
            ev->fft_amp[k] = cache.fft_amp[k];
        }
        ev->valid_flags |= LIGHT_EVENT_VALID_VI | LIGHT_EVENT_VALID_TEMP;
        if (ev->fft_count > 0u) {
            ev->valid_flags |= LIGHT_EVENT_VALID_FFT;
        }
        /* uart6_log("[LIGHT_EVT_CACHE_USE] event=%lu age=%lu snap=%lu temp=%f fft_count=%u fft0=%f/%f\r\n",
                  (unsigned long)ev->event_id,
                  (unsigned long)cache_age,
                  (unsigned long)cache.snap_count,
                  ev->temp,
                  (unsigned)ev->fft_count,
                  ev->fft_freq[0],
                  ev->fft_amp[0]); */
    } else {
        static uint32_t last_no_cache_log = 0u;
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_no_cache_log) >= 1000u) {
            last_no_cache_log = now;
            /* uart6_log("[LIGHT_EVT_WAIT_CACHE] event=%lu reason=no_cache\r\n",
                      (unsigned long)ev->event_id); */
        }
    }

    return ((ev->valid_flags & LIGHT_EVENT_REQUIRED_SENSOR_FLAGS) == LIGHT_EVENT_REQUIRED_SENSOR_FLAGS) ? 1u : 0u;
}

static void light_state_event_note_if_changed(uint8_t before_on, uint8_t after_on)
{
    if (before_on == after_on) {
        return;
    }

    light_state_event_t ev;
    RTC_TimeTypeDef rtc_time = {0};
    RTC_DateTypeDef rtc_date = {0};
    memset(&ev, 0, sizeof(ev));
    ev.pending = 1u;
    ev.event_id = ++g_light_event_seq;
    if (ev.event_id == 0u) {
        ev.event_id = ++g_light_event_seq;
    }
    ev.light_on = after_on ? 1u : 0u;
    ev.mode = current_control_mode();
    ev.reason = g_light_event_reason_context;
    ev.tick_ms = HAL_GetTick();
    ev.rtc_synced = g_rtc_synced ? 1u : 0u;
    if (HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) == HAL_OK &&
        HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) == HAL_OK) {
        ev.rtc_year = (uint16_t)(2000u + rtc_date.Year);
        ev.rtc_month = rtc_date.Month;
        ev.rtc_day = rtc_date.Date;
        ev.rtc_hour = rtc_time.Hours;
        ev.rtc_min = rtc_time.Minutes;
        ev.rtc_sec = rtc_time.Seconds;
        ev.valid_flags |= LIGHT_EVENT_VALID_RTC;
    }
    __disable_irq();
    if (g_light_event_count < LIGHT_STATE_EVENT_QUEUE_SIZE) {
        g_light_event_q[g_light_event_tail] = ev;
        g_light_event_tail = (uint8_t)((g_light_event_tail + 1u) % LIGHT_STATE_EVENT_QUEUE_SIZE);
        g_light_event_count++;
    } else {
        uint8_t last = (uint8_t)((g_light_event_tail + LIGHT_STATE_EVENT_QUEUE_SIZE - 1u) % LIGHT_STATE_EVENT_QUEUE_SIZE);
        g_light_event_q[last] = ev;
    }
    __enable_irq();
}

static void light_state_event_poll(void)
{
    static uint32_t last_try_tick = 0u;
    uint32_t now = HAL_GetTick();
    light_state_event_t ev;
    uint8_t body[LIGHT_STATE_EVENT_BODY_LEN];
    uint16_t body_len;
    uint16_t msg_id;

    if (g_light_event_count == 0u || !node_is_provisioned()) {
        return;
    }

    __disable_irq();
    ev = g_light_event_q[g_light_event_head];
    __enable_irq();

    if (!light_state_event_fill_measurements(&ev)) {
        __disable_irq();
        if (g_light_event_count > 0u) {
            g_light_event_q[g_light_event_head] = ev;
        }
        __enable_irq();
        return;
    }

    __disable_irq();
    if (g_light_event_count > 0u) {
        g_light_event_q[g_light_event_head] = ev;
    }
    __enable_irq();

    if (ev.reason == LIGHT_EVENT_REASON_UNKNOWN &&
        (uint32_t)(now - last_try_tick) < 100u) {
        return;
    }
    last_try_tick = now;

    body_len = encode_light_state_event_bin(body, (uint16_t)sizeof(body), &ev);
    if (body_len == 0u) {
        __disable_irq();
        if (g_light_event_count > 0u) {
            g_light_event_head = (uint8_t)((g_light_event_head + 1u) % LIGHT_STATE_EVENT_QUEUE_SIZE);
            g_light_event_count--;
        }
        __enable_irq();
        return;
    }

    msg_id = (uint16_t)(ev.event_id & 0xFFFFu);
    if (msg_id == 0u) {
        msg_id = 1u;
    }

    if (!send_transport_direct(0x0000u,
                               0u,
                               LIGHT_STATE_EVENT_CMD,
                               0u,
                               msg_id,
                               body,
                               body_len)) {
        return;
    }

    timing_log("[TLOG_LIGHT_EVT_TX] t=%lu event=%lu light=%u mode=%u reason=%u msg=%u flags=0x%02X temp=%f fft_count=%u fft0=%f/%f rtc=%04u-%02u-%02u %02u:%02u:%02u sync=%u\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned long)ev.event_id,
               (unsigned)ev.light_on,
               (unsigned)ev.mode,
               (unsigned)ev.reason,
               (unsigned)msg_id,
               (unsigned)ev.valid_flags,
               ev.temp,
               (unsigned)ev.fft_count,
               ev.fft_freq[0],
               ev.fft_amp[0],
               (unsigned)ev.rtc_year,
               (unsigned)ev.rtc_month,
               (unsigned)ev.rtc_day,
               (unsigned)ev.rtc_hour,
               (unsigned)ev.rtc_min,
               (unsigned)ev.rtc_sec,
               (unsigned)ev.rtc_synced);

    __disable_irq();
    if (g_light_event_count > 0u) {
        g_light_event_head = (uint8_t)((g_light_event_head + 1u) % LIGHT_STATE_EVENT_QUEUE_SIZE);
        g_light_event_count--;
    }
    __enable_irq();
}

static void at_accum_reset(void) {
    at_accum_len = 0;
    at_accum[0] = '\0';
}

static bool node_is_provisioned(void)
{
    // my_mid�??��? ?�역?�로 ?�고 ?�으?�까 그거�??�용
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

void light_on(void)
{
    uint8_t before_on = light_is_on_logical();
    uint8_t after_on = 1u;

    HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, light_pin_state_for(1u));
    g_light_on = after_on;

    if (before_on != after_on) {
        timing_log("[TLOG_LIGHT] t=%lu action=ON before=%u after=%u\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned)before_on,
                   (unsigned)after_on);
    }

    light_state_event_note_if_changed(before_on, after_on);
}
void light_off(void)
{
    uint8_t before_on = light_is_on_logical();
    uint8_t after_on = 0u;

    HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, light_pin_state_for(0u));
    g_light_on = after_on;

    if (before_on != after_on) {
        timing_log("[TLOG_LIGHT] t=%lu action=OFF before=%u after=%u\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned)before_on,
                   (unsigned)after_on);
    }

    light_state_event_note_if_changed(before_on, after_on);
}

static GPIO_PinState light_pin_state_for(uint8_t want_on)
{
#if LIGHT_ACTIVE_LOW
    return want_on ? GPIO_PIN_RESET : GPIO_PIN_SET;
#else
    return want_on ? GPIO_PIN_SET : GPIO_PIN_RESET;
#endif
}

static uint8_t light_is_on_logical(void)
{
    GPIO_PinState raw = HAL_GPIO_ReadPin(LIGHT_GPIO_Port, LIGHT_Pin);
#if LIGHT_ACTIVE_LOW
    return (raw == GPIO_PIN_RESET) ? 1u : 0u;
#else
    return (raw == GPIO_PIN_SET) ? 1u : 0u;
#endif
}

static uint16_t apply_time_correction_min(uint16_t base_min, uint8_t corr_mode, uint16_t corr_delta_min)
{
    if (corr_delta_min == 0u) {
        return base_min;
    }

    switch (corr_mode) {
    case 1u:
        return (uint16_t)norm_min((int)base_min + (int)corr_delta_min);
    case 2u:
        return (uint16_t)norm_min((int)base_min - (int)corr_delta_min);
    default:
        return base_min;
    }
}

static uint8_t time_window_contains(uint16_t now_min, uint16_t start_min, uint16_t end_min)
{
    if (start_min == end_min) {
        return 0u;
    }

    if (start_min < end_min) {
        return (now_min >= start_min && now_min < end_min) ? 1u : 0u;
    }

    return (now_min >= start_min || now_min < end_min) ? 1u : 0u;
}

static uint8_t current_control_mode(void)
{
    return g_manual_override_active ? 3u : g_node_cfg.mode;
}

static uint16_t manual_override_duration_min(void)
{
    if (g_node_cfg.manual_duration_min > 0u) {
        return g_node_cfg.manual_duration_min;
    }

    return 30u;
}

static void start_manual_override(uint8_t want_on)
{
    g_manual_override_latch_off_on_expire = 0u;
    start_manual_override_for(want_on, manual_override_duration_min());
}

static void start_manual_override_for(uint8_t want_on, uint16_t duration_min)
{
    uint32_t now = HAL_GetTick();
    uint32_t duration_ms = (uint32_t)duration_min * 60000u;
    uint8_t no_timeout = (duration_min == 0u) ? 1u : 0u;

    if (duration_ms > 0x7fffffffu) {
        duration_ms = 0x7fffffffu;
    }

    /* 이미 같은 방향의 manual override가 활성 상태면 재시작하지 않음 */
    if (g_manual_override_active &&
        g_manual_override_light_on == (want_on ? 1u : 0u) &&
        (g_manual_override_no_timeout || ((int32_t)(g_manual_override_until - now) > 0))) {
        return;
    }

    g_manual_override_active = 1u;
    g_manual_override_light_on = want_on ? 1u : 0u;
    g_manual_override_until = no_timeout ? now : (now + duration_ms);
    g_manual_override_no_timeout = no_timeout;

    if (g_manual_override_light_on) {
        light_on();
    } else {
        light_off();
    }

    if (g_manual_override_no_timeout) {
        timing_log("[TLOG_MANUAL] t=%lu state=%s hold=forever\r\n",
                   (unsigned long)HAL_GetTick(),
                   g_manual_override_light_on ? "ON" : "OFF");
    } else {
        timing_log("[TLOG_MANUAL] t=%lu state=%s hold=%u min\r\n",
                   (unsigned long)HAL_GetTick(),
                   g_manual_override_light_on ? "ON" : "OFF",
                   (unsigned)duration_min);
    }
}

static void start_forced_time_control(uint16_t forced_time_min)
{
    g_manual_override_active = 0u;
    g_manual_override_no_timeout = 0u;

    if (forced_time_min == 0u) {
        g_manual_override_latch_off_on_expire = 0u;
        light_event_set_reason(LIGHT_EVENT_REASON_SET_FORCED);
        start_manual_override_for(0u, 0u);
        light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
        return;
    }

    g_manual_override_latch_off_on_expire = 1u;
    light_event_set_reason(LIGHT_EVENT_REASON_SET_FORCED);
    start_manual_override_for(1u, forced_time_min);
    light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
}

static inline void InvalidateDCacheByAddr(uint32_t *addr, int32_t dsize) {
    if (SCB->CCR & SCB_CCR_DC_Msk) {
        int32_t linesize = 32;  // 보통 Cortex-M33 32-byte cache line
        uint32_t start = (uint32_t)addr & ~(linesize - 1);
        uint32_t end = ((uint32_t)addr + dsize + linesize - 1) & ~(linesize - 1);
        for (uint32_t p = start; p < end; p += linesize) {
            __DSB();
            SCB->DCIMVAC = p;
        }
        __DSB();
        __ISB();
    }
}


static uint16_t find_first_zero(const uint8_t *p, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) if (p[i] == 0x00) return i;
    return 0xFFFF;
}

void boot_poll(void)
{
    if (boot_cfg_started) return;
    
    if (HAL_GetTick() < 1500) return;

    nodeinfo_start(/*tmid=*/0, /*msg_id=*/0);  
    boot_cfg_started = 1;
}

void uart6_log(const char *fmt, ...)
{
#if FOCUS_TIMING_LOG
    (void)fmt;
    return;
#else
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = (int)sizeof(buf);

    HAL_UART_Transmit(&huart6, (uint8_t*)buf, (uint16_t)n, 50);
#endif
}

static uint8_t focus_timing_log_enabled(void)
{
#if FOCUS_TIMING_LOG
    return 1u;
#else
    return 0u;
#endif
}

static void timing_log(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = (int)sizeof(buf);

    HAL_UART_Transmit(&huart6, (uint8_t*)buf, (uint16_t)n, 50);
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


static void hop_tx_task_poll(void)
{
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < HOP_QUEUE_SIZE; ++i) {
        if (!g_hop_q[i].in_use) continue;
        if ((int32_t)(now - g_hop_q[i].due_tick) < 0) continue;

        // ----- TTL 처리 -----
        uint8_t ttl_idx = g_hop_q[i].compact_snap ? SNAP_COMPACT_TTL_IDX : 2u;
        uint8_t min_len = g_hop_q[i].compact_snap ? SNAP_COMPACT_BODY_LEN : 3u;

        if (g_hop_q[i].data_len < min_len) {
            g_hop_q[i].in_use = 0;
            g_hop_q[i].compact_snap = 0u;
            continue;
        }

        uint8_t ttl = g_hop_q[i].data[ttl_idx];
        if (ttl == 0) {
            dbg_dump_uart6_with_tag("[HOP_DROP_TTL0]", g_hop_q[i].data, g_hop_q[i].data_len);
            g_hop_q[i].in_use = 0;
            g_hop_q[i].compact_snap = 0u;
            continue;
        }
        g_hop_q[i].data[ttl_idx] = (uint8_t)(ttl - 1u);
        
        wisun_frame_cfg_t cfg = {
            .sig1 = 0xAA,
            .sig2 = 0xAB,
            .tmid = 0x0000u,
        };

        // log_snap_gateway_tx("[GW_TX_SNAP_HOP_FINAL]", cfg.tmid, g_hop_q[i].data, g_hop_q[i].data_len);
        // dbg_dump_uart6_with_tag("[HOP_TX]", g_hop_q[i].data, g_hop_q[i].data_len);
        (void)wisun_send_frame(&cfg,
                               g_hop_q[i].data,
                               g_hop_q[i].data_len,
                               wisun_tx_adapter,
                               NULL);

        g_hop_q[i].in_use = 0;
        g_hop_q[i].compact_snap = 0u;
    }
}

static uint8_t enqueue_transport_tx(uint16_t tmid, uint8_t cmd, uint8_t flags, uint16_t msg_id, const uint8_t *body, uint16_t body_len, uint8_t ttl)
{
    uint8_t payload[HOP_MAX_FRAME];
    uint16_t payload_len;
    uint32_t now;
    uint32_t base_delay;
    uint32_t jitter;

    if (tmid == 0u) {
        return 0u;
    }

    payload_len = build_transport_payload(tmid, (ttl == 0u) ? HOP_TTL_DEFAULT : ttl, cmd, flags, msg_id, body, body_len, payload, (uint16_t)sizeof(payload));
    if (payload_len == 0u) {
        return 0u;
    }

    now = HAL_GetTick();
    base_delay = 5u + (my_mid % 5u);
    jitter = xorshift16((uint16_t)(my_mid ^ (uint16_t)now ^ msg_id)) % 40u;

    for (int qi = 0; qi < HOP_QUEUE_SIZE; ++qi) {
        if (g_hop_q[qi].in_use) continue;

        g_hop_q[qi].tmid = 0x0000u;
        g_hop_q[qi].compact_snap = 0u;
        g_hop_q[qi].data_len = payload_len;
        memcpy(g_hop_q[qi].data, payload, payload_len);
        g_hop_q[qi].due_tick = now + base_delay + jitter;
        g_hop_q[qi].in_use = 1u;

        return 1u;
    }

    return 0u;
}


static void handle_binary_cmd(uint8_t cmd, uint8_t flags, uint16_t msg_id, uint16_t tmid, const uint8_t *data, uint16_t len)
{
    uint8_t uid12[12];
    mid_pack_uid12(uid12);
    switch (cmd)
        {
            case 0x13:
            {
            	uint32_t now = HAL_GetTick();

            	if ((int32_t)(now - last_snap_req_tick) < 3000) {         
            	        break;
            	}
            	last_snap_req_tick = now;
                
                schedule_resp_with_slot(
                    RESP_KIND_SNAP,   // SNAP 응답
                    tmid,             
                    msg_id,
                    NULL, 0           
                );
                break;
            }
        
            // LIGHT ON (0x10)            
            case LIGHT_ON:
            {
                uint8_t on = flags & 0x01;
                light_event_set_reason(LIGHT_EVENT_REASON_CMD);
                start_manual_override(on);
                light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);

                PowerCtrlAckBin_t ack = {0};
                ack.t = 0x10;
                memcpy(ack.uid, uid12, 12);
                ack.msg_id = msg_id;
                ack.ok = 1;
                ack.err_code = 0;
                ack.light_on = light_is_on_logical();

                schedule_resp_with_slot(
                    RESP_KIND_RAW_BIN,
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

                PowerCtrlAckBin_t ack = {0};
                ack.t = 0x10;
                memcpy(ack.uid, uid12, 12);
                ack.msg_id = msg_id;
                ack.ok = 1;
                ack.err_code = 0;
                ack.light_on = light_is_on_logical();

                schedule_resp_with_slot(
                    RESP_KIND_RAW_BIN,
                    tmid,
                    msg_id,
                    (uint8_t*)&ack,
                    sizeof(ack)
                );
                break;
            }
            
            // MID 변�?(CMD_SET_MID)
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
                    g_manual_override_active = 0u;
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
                    vin_v   = (float)vi.volt_raw * (3.3f / 4095.0f);
                    i_adc_v = (float)vi.curr_raw * K_ADC2V;
                    vi_ok = 1;
                }

                uint16_t temp_raw = 0;
                float temp_v = 0.0f, temp_c = 0.0f;

                if (HAL_ADC_Start(&hadc2) == HAL_OK) {
                    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
                        temp_raw = HAL_ADC_GetValue(&hadc2);
                        temp_v   = (float)temp_raw * 3.3f / 4095.0f;
                        temp_c   = ntc_voltage_to_temp_c(temp_v);
                        temp_ok  = 1;
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
            	uint8_t result = handle_cmd_set_setting(data, len);  
				
				AckBin_t ack = {0};
				ack.t = 0x10;     
				uint8_t uid12[12];
				mid_pack_uid12(uid12);
				memcpy(ack.uid, uid12, 12);

				ack.msg_id   = msg_id;         
				ack.ok       = (result == 0) ? 1 : 0;
				ack.err_code = (int8_t)result; 
			
				schedule_resp_with_slot(
					RESP_KIND_ACK,
					tmid,               
					msg_id,
					(uint8_t*)&ack,
					sizeof(ack)
				);

				debug6("[ACK_SLOT] pending ACK scheduled\r\n");

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

static void nodeinfo_send_err(uint16_t tmid, uint16_t msg_id, int8_t err_code)
{
    char msg[32];
    int n = snprintf(msg, sizeof(msg), "ERR=%d\n", (int)err_code);
    if (n < 0) n = 0;
    if (n > (int)sizeof(msg)) n = sizeof(msg);

    schedule_resp_with_slot(
        RESP_KIND_RAW_BIN,
        tmid,
        msg_id,
        (uint8_t*)msg,
        (uint16_t)n
    );
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
    } else {
        g_manual_override_active = 0u;
        g_manual_override_no_timeout = 0u;
        g_manual_override_latch_off_on_expire = 0u;
    }

    reconfigure_snapshot_timer_from_cfg();

    if (!save_node_cfg_to_flash(&g_node_cfg)) {
		result = 2;
	} else {
		result = 0;
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

static uint8_t rtc_weekday_from_ymd(uint16_t year, uint8_t month, uint8_t day)
{
    static const uint8_t t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    uint16_t y = year;

    if (month < 1u || month > 12u || day < 1u || day > 31u) {
        return RTC_WEEKDAY_MONDAY;
    }

    if (month < 3u) {
        y--;
    }

    {
        uint8_t dow = (uint8_t)((y + y / 4u - y / 100u + y / 400u + t[month - 1u] + day) % 7u);
        return (dow == 0u) ? RTC_WEEKDAY_SUNDAY : dow;
    }
}

static uint8_t handle_cmd_set_rtc_kst(const uint8_t *data, uint16_t len)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    RTC_TimeTypeDef curTime = {0};
    RTC_DateTypeDef curDate = {0};
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint8_t max_day = 31u;
    uint8_t leap = 0u;

    if (data == NULL || len < 7u) {
        debug6("[RTC_SYNC] invalid payload\r\n");
        return 1;
    }

    year = ((uint16_t)data[0] << 8) | data[1];
    month = data[2];
    day = data[3];
    hour = data[4];
    minute = data[5];
    second = data[6];

    {
        char parsed_dbg[128];
        int parsed_dbg_len = snprintf(
            parsed_dbg, sizeof(parsed_dbg),
            "[RTC_SYNC_PARSE] %04u-%02u-%02u %02u:%02u:%02u\r\n",
            (unsigned)year,
            (unsigned)month,
            (unsigned)day,
            (unsigned)hour,
            (unsigned)minute,
            (unsigned)second
        );
        if (parsed_dbg_len > 0) {
            HAL_UART_Transmit(&huart6, (uint8_t*)parsed_dbg, (uint16_t)parsed_dbg_len, 100);
        }
    }

    if (year < 2000u || year > 2099u) return 2;
    if (month < 1u || month > 12u) return 3;
    if (hour > 23u || minute > 59u || second > 59u) return 4;

    leap = ((year % 4u) == 0u) ? 1u : 0u;
    switch (month) {
    case 2:
        max_day = (uint8_t)(leap ? 29u : 28u);
        break;
    case 4:
    case 6:
    case 9:
    case 11:
        max_day = 30u;
        break;
    default:
        max_day = 31u;
        break;
    }

    if (day < 1u || day > max_day) return 5;

    sTime.Hours = hour;
    sTime.Minutes = minute;
    sTime.Seconds = second;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    sDate.Year = (uint8_t)(year - 2000u);
    sDate.Month = month;
    sDate.Date = day;
    sDate.WeekDay = rtc_weekday_from_ymd(year, month, day);

    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) return 6;
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) return 7;

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2u);
    if (HAL_RTC_GetTime(&hrtc, &curTime, RTC_FORMAT_BIN) == HAL_OK &&
        HAL_RTC_GetDate(&hrtc, &curDate, RTC_FORMAT_BIN) == HAL_OK) {
        char applied_dbg[128];
        int applied_dbg_len = snprintf(
            applied_dbg, sizeof(applied_dbg),
            "[RTC_SYNC_APPLIED] %04u-%02u-%02u %02u:%02u:%02u wd=%u\r\n",
            (unsigned)(2000u + curDate.Year),
            (unsigned)curDate.Month,
            (unsigned)curDate.Date,
            (unsigned)curTime.Hours,
            (unsigned)curTime.Minutes,
            (unsigned)curTime.Seconds,
            (unsigned)curDate.WeekDay
        );
        if (applied_dbg_len > 0) {
            HAL_UART_Transmit(&huart6, (uint8_t*)applied_dbg, (uint16_t)applied_dbg_len, 100);
        }
    } else {
        debug6("[RTC_SYNC] readback failed\r\n");
    }

    g_rtc_synced = 1u;
    rtc_update();
    update_sun_times();
    scheduler_poll();
    debug6("[RTC_SYNC] scheduler resumed\r\n");
    return 0;
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


void ADC1_Start_Regular_IN18_IT(void)
{
    // CubeMX에서: Scan Disable, NbrOfConversion=1, Rank1=IN18,
    // Continuous Conversion Enable, EOC Selection=EOC 
    HAL_ADC_Start_IT(&hadc1);  
}


/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;
    uart6_log("[ADC1] DMA Cplt!\r\n");
    // DMA full complete = FFT_SIZE 
    g_frame_c1 = DWT->CYCCNT;
    
    for (int i = 0; i < FFT_SIZE; i++) {
        uint16_t raw = raw_buffer[i];
        inputSignal[i] = ((float)raw * 3.3f / 4095.0f) - 1.65f;
    }

    g_ultra_frame_tick = HAL_GetTick();
    ultra_frame_ready = true;
    ultra_sampling_paused = true;

    HAL_ADC_Stop_DMA(&hadc1);   
    HAL_TIM_Base_Stop(&htim6);
}*/

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
}

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
    	HAL_UART_Transmit(&huart6, (uint8_t*)"Tick\r\n", 6, HAL_MAX_DELAY);
        if (++g_sec_tick >= 10) {   
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
            g_sec_tick = 0;
        }
    }
}
*/

static void mid_pack_uid12(uint8_t out12[12]) {
    memcpy(out12, (const void*)uid_ram, 12);
}

static inline int is_night(int now, int dusk, int dawn)
{
    if (dusk <= dawn) {
        return (now >= dusk) && (now < dawn);
    } else {
        return (now >= dusk) || (now < dawn);
    }
}

static bool send_wisun_binary(uint16_t tmid, const uint8_t *data, size_t len)
{
    bool ok;
    wisun_frame_cfg_t cfg = {
        .sig1 = 0xAA,   // 그�?�??��?
        .sig2 = 0xAB,   // 그�?�??��?
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

void dbg_dump_uart6_with_tag(const char *tag, const uint8_t *p, uint16_t n)
{
#if FOCUS_TIMING_LOG
    (void)tag;
    (void)p;
    (void)n;
    return;
#else
    if (!p || !n) return;
    
    if (tag) {
        HAL_UART_Transmit(&huart6, (uint8_t*)tag, (uint16_t)strlen(tag), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart6, (uint8_t*)" ", 1, HAL_MAX_DELAY);
    }
    
    for (uint16_t i = 0; i < n; ++i) {
        char s[4];
        int m = snprintf(s, sizeof(s), "%02X", p[i]);
        HAL_UART_Transmit(&huart6, (uint8_t*)s, (uint16_t)m, HAL_MAX_DELAY);
        if (i + 1 < n)
            HAL_UART_Transmit(&huart6, (uint8_t*)" ", 1, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
#endif
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

static void send_resp_now_from_task(void)
{
    uint8_t uid12[12];
    mid_pack_uid12(uid12);

    SlotRespBin_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.t = 0x11;                 
    memcpy(pkt.uid, uid12, 12);

    pkt.mid   = my_mid;
    pkt.ok    = g_resp.ok ? 1 : 0;
    pkt.has_on = g_resp.p_on_valid;
    pkt.on     = g_resp.p_on ? 1 : 0;
    pkt.nonce  = g_resp.nonce;    
    
    wisun_frame_cfg_t cfg = {
        .sig1 = 0xAA,
        .sig2 = 0xAB,
        .tmid = 0x0000
    };

    (void)wisun_send_frame(&cfg, (const uint8_t*)&pkt, sizeof(pkt),
                           wisun_tx_adapter, NULL);

    __disable_irq();
    memset((void*)&g_resp, 0, sizeof(g_resp));
    __enable_irq();
}

static bool hop_seen_key(uint32_t key)
{
    uint8_t n = (g_hop_seen_count < HOP_SEEN_TABLE_SIZE) ? g_hop_seen_count : HOP_SEEN_TABLE_SIZE;
    for (uint8_t i = 0; i < n; i++) {
        if (g_hop_seen_keys[i] == key) return true;
    }
    return false;
}

static void hop_mark_key(uint32_t key)
{
    if (g_hop_seen_count < HOP_SEEN_TABLE_SIZE) {
        g_hop_seen_keys[g_hop_seen_count++] = key;
    } else {
        g_hop_seen_keys[g_hop_seen_pos] = key;
        g_hop_seen_pos = (uint8_t)((g_hop_seen_pos + 1U) % HOP_SEEN_TABLE_SIZE);
    }
}

static uint16_t compact_snap_count16(const uint8_t *data)
{
    return (uint16_t)data[SNAP_COMPACT_SNAP_COUNT_IDX] |
           ((uint16_t)data[SNAP_COMPACT_SNAP_COUNT_IDX + 1u] << 8);
}

static uint32_t compact_snap_seen_key(const uint8_t *data)
{
    uint32_t h = 2166136261u;

    for (uint8_t i = 0u; i < SNAP_COMPACT_UID_LEN; ++i) {
        h ^= data[SNAP_COMPACT_UID_IDX + i];
        h *= 16777619u;
    }

    h ^= data[SNAP_COMPACT_SNAP_COUNT_IDX];
    h *= 16777619u;
    h ^= data[SNAP_COMPACT_SNAP_COUNT_IDX + 1u];
    h *= 16777619u;

    return h;
}

static bool compact_snap_is_own_uid(const uint8_t *data)
{
    uint8_t uid12[12];

    mid_pack_uid12(uid12);
    return memcmp(&data[SNAP_COMPACT_UID_IDX], uid12, sizeof(uid12)) == 0;
}

void rtc_minute_tick(uint8_t hour, uint8_t min)
{
    uint16_t now = hour * 60u + min;
    static uint8_t light_state = 0;

    switch (current_control_mode()) {
    case 2: // user_time
    {
        uint16_t on  = g_node_cfg.light_on_hour  * 60u + g_node_cfg.light_on_min;
        uint16_t off = g_node_cfg.light_off_hour * 60u + g_node_cfg.light_off_min;

        if (!light_state && now >= on && now < off) {
            light_on();
            light_state = 1;
        } else if (light_state && (now >= off || now < on)) {
            light_off();
            light_state = 0;
        }
        break;
    }
    case 3: // manual                
        break;

    default:        
        break;
    }
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

void Input_Ai_Model_Features(float freq_khz, float adc_pk, float current, float vin)
{
    if (ai_pending) return;

    ai_input[AI_FEATURE_FREQ_KHZ_IDX] = freq_khz;
    ai_input[AI_FEATURE_ADC_PK_IDX]   = adc_pk;
    ai_input[AI_FEATURE_CURRENT_IDX]  = current;
    ai_input[AI_FEATURE_VIN_IDX]      = vin;
    ai_index = AE_COLS;
    ai_pending = 1;
}

static int hexval(char c){
    if ('0'<=c && c<='9') return c-'0';
    if ('a'<=c && c<='f') return c-'a'+10;
    if ('A'<=c && c<='F') return c-'A'+10;
    return -1;
}

static int parse_hex8(const char *s, uint8_t out[8]){
    // s: "00124B002D441B87" (16 chars)
    for (int i=0;i<8;i++){
        int hi = hexval(s[i*2]);
        int lo = hexval(s[i*2+1]);
        if (hi<0 || lo<0) return 0;
        out[i] = (uint8_t)((hi<<4)|lo);
    }
    return 1;
}

static uint8_t apply_snapshot_uart_cmd(const char *arg)
{
    char *endp = NULL;
    unsigned long minutes_ul;
    uint8_t minutes;
    uint16_t interval_sec;
    char msg[96];

    if (arg == NULL) {
        debug6("[UART SNAP] missing argument\r\n");
        return 0;
    }

    if (strcmp(arg, "OFF") == 0 || strcmp(arg, "0") == 0) {
        g_node_cfg.snap_enable = 0;
        reconfigure_snapshot_timer_from_cfg();

        if (!save_node_cfg_to_flash(&g_node_cfg)) {
            debug6("[UART SNAP] save failed\r\n");
            return 0;
        }

        debug6("[UART SNAP] disabled\r\n");
        return 1;
    }

    minutes_ul = strtoul(arg, &endp, 10);
    if (endp == arg || *endp != '\0') {
        debug6("[UART SNAP] invalid value\r\n");
        return 0;
    }

    if (minutes_ul == 0UL || minutes_ul > 120UL) {
        debug6("[UART SNAP] range is 1..120 min\r\n");
        return 0;
    }

    minutes = (uint8_t)minutes_ul;
    interval_sec = (uint16_t)minutes * 60u;
    g_node_cfg.snap_enable = 1;
    g_node_cfg.snap_interval_sec = interval_sec;
    reconfigure_snapshot_timer_from_cfg();

    if (!save_node_cfg_to_flash(&g_node_cfg)) {
        debug6("[UART SNAP] save failed\r\n");
        return 0;
    }

    snprintf(msg, sizeof(msg),
             "[UART SNAP] enabled interval=%u sec (%lu ms)\r\n",
             (unsigned)g_node_cfg.snap_interval_sec,
             (unsigned long)g_snap_interval_ms);
    debug6(msg);
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
                 (unsigned long)g_snap_interval_ms);
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
        // "00124B002D441B87" -> 8바이트
        parse_hex8(p+4, g_node_info.mac);   
    } else if (strncmp(p, "FWVER=", 6) == 0) {
        int maj=0, min=0;
        if (sscanf(p+6, "%d.%d", &maj, &min) == 2) {
            g_node_info.fw_major = (uint16_t)maj;
            g_node_info.fw_minor = (uint16_t)min;
        }
    }
}

void tx_task_poll(void)
{
    if (!g_resp.pending) return;
    if ((int32_t)(HAL_GetTick() - g_resp.due_tick) < 0) return;

    __disable_irq();
    g_resp.pending = false;
    __enable_irq();

    send_resp_now_from_task();
}

void debug6(const char *s)
{
#if FOCUS_TIMING_LOG
    (void)s;
    return;
#else
    HAL_UART_Transmit(&huart6, (uint8_t*)s, strlen(s), 100);
#endif
}

/*
static void light_force_gpio_out(void)
{
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = LIGHT_Pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LIGHT_GPIO_Port, &gi);
}
*/

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

void reconfigure_snapshot_timer_from_cfg(void)
{
    if (g_node_cfg.snap_enable) {
        g_snap_enable = 1;
        uint32_t interval_sec = g_node_cfg.snap_interval_sec;

        if (interval_sec == 0u) {
            interval_sec = 60u;
            g_node_cfg.snap_interval_sec = (uint16_t)interval_sec;
        }

        if (interval_sec > 65535u) {
            interval_sec = 65535u;
            g_node_cfg.snap_interval_sec = (uint16_t)interval_sec;
        }

        g_snap_interval_ms = interval_sec * 1000u;
        uart6_log("[SNAP_CFG] enable=%u interval_sec=%lu my_mid=%u\r\n",
                  (unsigned)g_snap_enable,
                  (unsigned long)interval_sec,
                  (unsigned)my_mid);
        Send_Monitoring_Snapshot_JSON(0);
        g_snap_next_tick   = HAL_GetTick() + g_snap_interval_ms;
    } else {
        uart6_log("[SNAP_CFG] enable=0 my_mid=%u\r\n", (unsigned)my_mid);
        g_snap_enable = 0;        
        // g_snap_next_tick = 0;
    }
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


void rtc_update(void)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    g_rtc_hour  = sTime.Hours;
    g_rtc_min   = sTime.Minutes;
    g_rtc_sec   = sTime.Seconds;

    g_rtc_year  = 2000 + sDate.Year;   
    g_rtc_month = sDate.Month;
    g_rtc_day   = sDate.Date;
}

static void update_sun_times(void)
{    
    if (g_rtc_year < 2020 || g_rtc_year > 2100)
        return;
    if (g_rtc_month == 0 || g_rtc_month > 12)
        return;
    if (g_rtc_day == 0 || g_rtc_day > 31)
        return;
 
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
                break; // 너무 크면 버림
            }

            // TTL=0이면 forward 금지
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

void Send_Monitoring_Snapshot_JSON(uint16_t req_msg_id)
{
    if (req_msg_id == 0 && !node_is_provisioned()) {
        uart6_log("[SNAP_SKIP] reason=not_provisioned req_msg_id=%u my_mid=%u\r\n",
                  (unsigned)req_msg_id,
                  (unsigned)my_mid);
        return;
    }

    if (!ultra_frame_ready) {
        uart6_log("[SNAP_SKIP] reason=no_ultra_frame req_msg_id=%u paused=%u my_mid=%u\r\n",
                  (unsigned)req_msg_id,
                  (unsigned)(ultra_sampling_paused ? 1u : 0u),
                  (unsigned)my_mid);

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

    /* ===== 2) UID ===== */
    uint8_t uid12[12];
    mid_pack_uid12(uid12);

    /* ===== 3) VI 측정 ===== */
    VIRead vi;
    float vin_v   = 0.0f;
    float i_adc_v = 0.0f;
    if (AD_DC_Injected_Once(&vi) == HAL_OK) {
        vin_v   = (float)vi.volt_raw * (3.3f / 4095.0f);
        i_adc_v = (float)vi.curr_raw * K_ADC2V;
    }

    /* ===== 4) 온도 ===== */
    uint16_t temp_raw = 0;
    float temp_v = 0.0f;
    float temp_c = 0.0f;
    if (HAL_ADC_Start(&hadc2) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
            temp_raw = HAL_ADC_GetValue(&hadc2);
            temp_v   = (float)temp_raw * 3.3f / 4095.0f;
            temp_c   = ntc_voltage_to_temp_c(temp_v);
        }
        HAL_ADC_Stop(&hadc2);
    }

    /* ===== 5) FFT 처리 ===== */
    float   fft_freq[SNAP_FFT_PAIRS] = {0};
    float   fft_amp [SNAP_FFT_PAIRS] = {0};
    uint8_t fft_cnt = 0;

    float max_amp = -1.0f;
    float peak_f  = 0.0f;
    uint8_t found = 0;
    float   supersonic_val = 0.0f;

    {
        static float32_t local_in[FFT_SIZE];
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
            local_in[i] = ((float32_t)raw * 3.3f / 4095.0f) - 1.65f;
        }
        __enable_irq();

        adc_avg = adc_sum / (uint32_t)FFT_SIZE;
        uart6_log("[SNAP_ADC_RAW] min=%u max=%u avg=%lu first=%u mid=%u last=%u dma_done=%lu dma_half=%lu\r\n",
                  (unsigned)adc_min,
                  (unsigned)adc_max,
                  (unsigned long)adc_avg,
                  (unsigned)adc_first,
                  (unsigned)adc_mid,
                  (unsigned)adc_last,
                  (unsigned long)g_dma_done,
                  (unsigned long)g_dma_half);

        arm_rfft_fast_f32(&fftInstance, local_in, outputSignal, 0);

        for (int i = 0; i < FFT_SIZE / 2; ++i) {
            float real = outputSignal[2 * i];
            float imag = outputSignal[2 * i + 1];
            float mag  = sqrtf(real * real + imag * imag);
            if (!isfinite(mag)) mag = 0.0f;

            fft_packet[i].freq      = (float)i * (float)FSAMPLE / (float)FFT_SIZE;
            fft_packet[i].amplitude = mag;
        }

        supersonic_val = compute_supersonic_rms_from_fftdata(fft_packet, FFT_SIZE / 2);

        uint16_t nbins = FFT_SIZE / 2;
        for (uint16_t b = 1; b < nbins; ++b) {
            float freq = (float)b * (float)FSAMPLE / (float)FFT_SIZE;
            float amp  = fft_packet[b].amplitude;
            if (!isfinite(amp)) amp = 0.0f;

            if (freq >= 80000.0f && freq <= 130000.0f) {
                if (amp > max_amp) {
                    max_amp = amp;
                    peak_f  = freq;
                    found   = 1;
                }
            }
        }

        if (found) {
            fft_freq[0] = peak_f;
            fft_amp [0] = max_amp;
            fft_cnt = 1;
        } else {            
            // uart6_log("Target band (80-130kHz) not found.\r\n");
        }
    }

    bool light_on = light_is_on_logical() ? true : false;
    push_snapshot(light_on, vin_v, i_adc_v, temp_c, supersonic_val);

    uint8_t snap_ai_valid = 0u;
    float snap_ai_mse = 0.0f;
    uint32_t snap_ai_mse_x1000000 = 0u;
    int snap_ai_pred = 0;

    if (fft_cnt > 0u) {
        float ai_features[AE_COLS] = {0};

        ai_features[AI_FEATURE_FREQ_KHZ_IDX] = fft_freq[0] * 0.001f;
        ai_features[AI_FEATURE_ADC_PK_IDX]   = fft_amp[0];
        ai_features[AI_FEATURE_CURRENT_IDX]  = i_adc_v;
        ai_features[AI_FEATURE_VIN_IDX]      = vin_v;

        if (run_inference(ai_features, &snap_ai_mse, &snap_ai_pred) == 0) {
            snap_ai_valid = 1u;
            snap_ai_mse_x1000000 = scale_ai_mse_x1000000(snap_ai_mse);
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
    light_sensor_cache_update(vin_v, i_adc_v, temp_c, fft_cnt, fft_freq, fft_amp, g_monitor_count);

    {
        uint16_t tx_msg_id = (req_msg_id != 0u) ? req_msg_id : (uint16_t)(g_monitor_count & 0xFFFFu);
        uint16_t snap_tmid = 0u;
        uint8_t snap_body[SNAP_COMPACT_BODY_LEN];
        uint16_t snap_body_len = 0u;
        uint8_t local_fft_cnt;
        float local_fft0_freq = 0.0f;
        float local_fft0_amp = 0.0f;
        uint32_t local_monitor_count;

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

        uart6_log("[SNAP_FFT_SRC] cnt=%u f0=%f a0=%f raw_f0=%lu raw_a0=%lu light=%u\r\n",
                  (unsigned)local_fft_cnt,
                  local_fft0_freq,
                  local_fft0_amp,
                  (unsigned long)(uint32_t)(local_fft0_freq * 100.0f),
                  (unsigned long)(uint32_t)(local_fft0_amp * 1000.0f),
                  (unsigned)(light_on ? 1u : 0u));

        snap_body_len = encode_snap_compact_bin(snap_body, (uint16_t)sizeof(snap_body), uid12, vin_v, i_adc_v, temp_c, light_on ? 1u : 0u, local_fft_cnt > 0u ? 1u : 0u, local_fft0_freq, local_fft0_amp, local_monitor_count, snap_ai_valid, snap_ai_mse_x1000000, (int8_t)snap_ai_pred, 1u, SNAP_COMPACT_TTL_DEFAULT);

        if (snap_body_len == 0u) {
            uart6_log("[SNAP_SKIP] reason=compact_encode_fail req_msg_id=%u my_mid=%u\r\n",
                      (unsigned)tx_msg_id,
                      (unsigned)my_mid);
            Ultra_StartDmaFrame();
            g_last_has_msg_id = 0;
            g_last_has_cmd = 0;
            return;
        }

        uart6_log("[SNAP_AFTER_BUILD] compact_body_len=%u tx_msg_id=%u frame_tmid=0x%04X\r\n",
                  (unsigned)snap_body_len,
                  (unsigned)tx_msg_id,
                  (unsigned)snap_tmid);
        dbg_dump_uart6_with_tag("[SNAP_COMPACT_BODY]", snap_body, snap_body_len);

        (void)send_wisun_binary(snap_tmid, snap_body, snap_body_len);

        uart6_log("[SNAP_TX] req_msg_id=%lu snap_count=%lu light_on=%u fft_count=%u mid=%u frame_tmid=0x%04X compact=1\r\n",
                  (unsigned long)tx_msg_id,
                  (unsigned long)g_monitor_count,
                  (unsigned)(light_on ? 1u : 0u),
                  (unsigned)fft_cnt,
                  (unsigned)my_mid,
                  (unsigned)snap_tmid);

        Ultra_StartDmaFrame();
        g_last_has_msg_id = 0;
        g_last_has_cmd = 0;
        return;
    }

    /* ===== 7) SnapBin 구성전송 ===== */
    uint16_t tx_msg_id = (req_msg_id != 0u) ? req_msg_id : (uint16_t)(g_monitor_count & 0xFFFFu);
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

    /*
     * Requested SNAP responses must go back to the requester MID that was
     * captured in g_resp_slot.tmid. Periodic/self-scheduled SNAP packets are
     * broadcast, so keep target_mid at 0x0000 in that case.
     *
     * Do not fall back to g_last_rx_tmid here because that value can be mixed
     * with parsed transport fields on other RX paths and may not represent the
     * current requester's MID.
     */
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
        for (uint8_t k = 0; k < SNAP_FFT_PAIRS; ++k) {
            if (k < fft_cnt) {
                snap.fft[k].freq_x100 = scale_fft_freq_x100(fft_freq[k]);
                snap.fft[k].amp_x1000 = scale_fft_amp_x1000(fft_amp[k]);
            }
        }
        snap.snap_count = g_monitor_count;
        snap.msg_id = (uint32_t)tx_msg_id;
        snap.ai_valid = snap_ai_valid;
        snap.ai_mse_x1000000 = snap_ai_mse_x1000000;
        snap.ai_pred = (int8_t)snap_ai_pred;
        snap.ok = 1u;
        snap.err_code = 0;

        memcpy(snap_body, &snap, sizeof(snap));
        snap_body_len = (uint16_t)sizeof(snap);
        use_legacy_struct_once = 1u;
        uart6_log("[SNAP_SIZE] sizeof(SnapBin_t)=%u body_len=%u\r\n",
              (unsigned)sizeof(SnapBin_t),
              (unsigned)snap_body_len);

        uart6_log("[SNAP_SCALED] fft_freq0=%f fft_amp0=%f snap_freq0_x100=%lu snap_amp0_x1000=%ld\r\n",
                fft_freq[0],
                fft_amp[0],
                (unsigned long)snap.fft[0].freq_x100,
                (long)snap.fft[0].amp_x1000);

        dbg_dump_uart6_with_tag("[SNAP_BODY_HEX]", snap_body, snap_body_len);
    } else {
    float    local_fft_freq[SNAP_FFT_PAIRS];
    float    local_fft_amp[SNAP_FFT_PAIRS];
    uint8_t  local_fft_cnt;
    uint32_t local_monitor_count;
    uint32_t local_msg_id;

    memset(local_fft_freq, 0, sizeof(local_fft_freq));
    memset(local_fft_amp,  0, sizeof(local_fft_amp));

    /*
     * fft_cnt, fft_freq, fft_amp, g_monitor_count가
     * ISR 또는 다른 처리 루틴에서 갱신될 수 있다면
     * 여기서 한 번에 복사해서 encode 중 값이 바뀌지 않게 한다.
     */
    __disable_irq();

    local_fft_cnt = fft_cnt;
    if (local_fft_cnt > SNAP_FFT_PAIRS) {
        local_fft_cnt = SNAP_FFT_PAIRS;
    }

    for (uint8_t k = 0; k < local_fft_cnt; ++k) {
        local_fft_freq[k] = fft_freq[k];
        local_fft_amp[k]  = fft_amp[k];
    }

    local_monitor_count = g_monitor_count;
    local_msg_id = (uint32_t)tx_msg_id;

    __enable_irq();

    uart6_log("[SNAP_FFT_SRC] cnt=%u f0=%f a0=%f raw_f0=%lu raw_a0=%lu light=%u\r\n",
              (unsigned)local_fft_cnt,
              local_fft_freq[0],
              local_fft_amp[0],
              (unsigned long)(uint32_t)(local_fft_freq[0] * 100.0f),
              (unsigned long)(uint32_t)(local_fft_amp[0] * 1000.0f),
              (unsigned)(light_on ? 1u : 0u));

    snap_body_len = encode_snap_bin(snap_body, (uint16_t)sizeof(snap_body), uid12, vin_v, i_adc_v, temp_c, light_on ? 1u : 0u, local_fft_cnt, local_fft_freq, local_fft_amp, local_monitor_count, local_msg_id, 1u, 0, snap_ai_valid, snap_ai_mse_x1000000, (int8_t)snap_ai_pred);

    /* uart6_log("[SNAP_ENCODE_RESULT] body_len=%u fft_cnt=%u freq0=%f amp0=%f snap_count=%lu msg_id=%lu\r\n",
              (unsigned)snap_body_len,
              (unsigned)local_fft_cnt,
              local_fft_freq[0],
              local_fft_amp[0],
              (unsigned long)local_monitor_count,
              (unsigned long)local_msg_id);

    dbg_dump_uart6_with_tag("[SNAP_BODY_HEX_ENCODE]", snap_body, snap_body_len); */
    }
    if (snap_body_len == 0u) {
        uart6_log("[SNAP_SKIP] reason=encode_fail req_msg_id=%u my_mid=%u\r\n", (unsigned)tx_msg_id, (unsigned)my_mid);
        Ultra_StartDmaFrame();
        g_last_has_msg_id = 0;
        g_last_has_cmd    = 0;
        return;
    }

    snap_tx_dump_len = build_transport_payload(0x0000u, 0u, SNAP_REPORT_CMD, 0u, tx_msg_id, snap_body, snap_body_len, snap_tx_dump, (uint16_t)sizeof(snap_tx_dump));
    uart6_log("[SNAP_AFTER_BUILD] body_len=%u tx_len=%u tx_msg_id=%u\r\n", (unsigned)snap_body_len, (unsigned)snap_tx_dump_len, (unsigned)tx_msg_id);

    // dbg_dump_uart6_with_tag("[SNAP_AFTER_BUILD_BODY]", snap_body, snap_body_len);
    // dbg_dump_uart6_with_tag("[SNAP_AFTER_BUILD_TX]", snap_tx_dump, snap_tx_dump_len);

    if (snap_tmid != 0u) {
        snap_enqueued = enqueue_transport_tx( snap_tmid, SNAP_REPORT_CMD, 0u, tx_msg_id, snap_body, snap_body_len, HOP_TTL_DEFAULT );

        if (snap_enqueued) {
            snap_tx_dump_len = build_transport_payload(snap_tmid, HOP_TTL_DEFAULT, SNAP_REPORT_CMD, 0u, tx_msg_id, snap_body, snap_body_len, snap_tx_dump, (uint16_t)sizeof(snap_tx_dump));
        }
    }

    uart6_log("[SNAP_TX] req_msg_id=%lu snap_count=%lu light_on=%u fft_count=%u mid=%u route=0x%04X via=%s legacy_struct=%u\r\n",
              (unsigned long)tx_msg_id,
              (unsigned long)g_monitor_count,
              (unsigned)(light_on ? 1u : 0u),
              (unsigned)fft_cnt,
              (unsigned)my_mid,
              (unsigned)snap_tmid,
              snap_enqueued ? "hop" : "direct",
              (unsigned)use_legacy_struct_once);
    // dbg_dump_uart6_with_tag(snap_enqueued ? "[SNAP_TX_HOP_PAYLOAD]" : "[SNAP_TX_DIRECT_PAYLOAD]",
    //                         snap_tx_dump,
    //                         snap_tx_dump_len);
    if (!snap_enqueued) {
        (void)send_transport_direct(0x0000u, 0u, SNAP_REPORT_CMD, 0u, tx_msg_id, snap_body, snap_body_len);
    }
    Ultra_StartDmaFrame();

    g_last_has_msg_id = 0;
    g_last_has_cmd    = 0;
}

float ntc_voltage_to_temp_c(float temp_v)
{

    if (temp_v >= ntc_voltage_table[0]) {
        return ntc_temp_table[0];              // -40도이하
    }
    if (temp_v <= ntc_voltage_table[NTC_LUT_SIZE - 1]) {
        return ntc_temp_table[NTC_LUT_SIZE - 1]; // 80도이상
    }

    for (int i = 1; i < NTC_LUT_SIZE; ++i) {
        float v_hi = ntc_voltage_table[i - 1];
        float v_lo = ntc_voltage_table[i];

        if (temp_v <= v_hi && temp_v >= v_lo) {
            float t_hi = ntc_temp_table[i - 1];
            float t_lo = ntc_temp_table[i];

            float ratio = (v_hi - temp_v) / (v_hi - v_lo); // 0~1
            // 선형 보간
            return t_hi + (t_lo - t_hi) * ratio;
        }
    }

    return ntc_temp_table[NTC_LUT_SIZE - 1];
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
  MX_GPDMA1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
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
 /* HAL_TIM_Base_Start(&htim6);
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, FFT_SIZE) != HAL_OK)
	{
		Error_Handler();
	}*/
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
    rtc_ensure_valid_or_set_default();
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
        g_node_cfg.mode = 1u;
        (void)save_node_cfg_to_flash(&g_node_cfg);
    }

    apply_mid_chan_from_cfg();
    reconfigure_snapshot_timer_from_cfg();
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
	         boot_poll();
	         scheduler_poll();
	         light_state_event_poll();
	         hop_tx_task_poll();
	         resp_slot_task_poll();
	         wisun_process_rx_mainloop();

	         /* Debug FFT */

	         /*if (g_adc_kick && !ultra_sampling_paused) {
	             g_adc_kick = 0;
	             (void)HAL_ADC_Start_IT(&hadc1);
	         }*/

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

	         /* ===================== Wi-SUN RX ===================== */
	         if (wisun_packet_ready)
	         {
	             __disable_irq();

	             uint16_t len = wisun_packet_len;
	             uint8_t  tmp[PACKET_MAX_SIZE];
	             memcpy(tmp, (const void *)wisun_packet_shadow, len);
	             wisun_packet_ready = 0;

	             __enable_irq();

	             dbg_dump_uart6_with_tag("[U1 FRAME]", tmp, len);

	             wisun_frame_view_t v = {0};
	             if (wisun_parse_frame(tmp, len, &v))
	             {
	                 g_last_rx_tmid = v.tmid;
	                 uint16_t src_mid = v.tmid;

	                //최소: target_mid(2) + ttl(1) + cmd(1) + flags(1) + msg_id(2) = 7B
	                 if (v.data_len >= 7)
	                 {
	                     uint16_t target_mid =
	                         (uint16_t)v.data[0] | ((uint16_t)v.data[1] << 8);

	                     uint8_t  ttl    = v.data[2];
	                     uint8_t  cmd    = v.data[3];
	                     uint8_t  flags  = v.data[4];
	                     uint16_t msg_id =
	                         ((uint16_t)v.data[5] << 8) | v.data[6];

	                     const uint8_t *body =
	                         (v.data_len > 7) ? &v.data[7] : NULL;
	                     uint16_t bodylen =
	                         (v.data_len > 7) ? (v.data_len - 7) : 0;

	                     if (target_mid != my_mid && target_mid != 0xFFFF)
	                         continue;

	                     handle_binary_cmd(
	                         cmd, flags, msg_id,
	                         src_mid, body, bodylen
	                     );
	                 }
	             }
	         }

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

	         // ===================== Node Info =====================
	         nodeinfo_poll(now);

	         if (g_snap_enable && node_is_provisioned())
	         {
	             now = HAL_GetTick();
	             if ((int32_t)(now - g_snap_next_tick) >= 0)
	             {
	                 uart6_log("[SNAP_POLL] enable=%u prov=%u ready=%u paused=%u now=%lu next=%lu interval=%lu mid=%u\r\n",
	                           (unsigned)g_snap_enable,
	                           (unsigned)(node_is_provisioned() ? 1u : 0u),
	                           (unsigned)(ultra_frame_ready ? 1u : 0u),
	                           (unsigned)(ultra_sampling_paused ? 1u : 0u),
	                           (unsigned long)now,
	                           (unsigned long)g_snap_next_tick,
	                           (unsigned long)g_snap_interval_ms,
	                           (unsigned)my_mid);
	                 Send_Monitoring_Snapshot_JSON(0);
	                 g_snap_next_tick = now + g_snap_interval_ms;
	             }
	         }
	         else
	         {
	             static uint32_t snap_gate_dbg_tick = 0;
	             now = HAL_GetTick();
	             if ((uint32_t)(now - snap_gate_dbg_tick) >= 10000u) {
	                 snap_gate_dbg_tick = now;
	                 uart6_log("[SNAP_GATE] enable=%u prov=%u my_mid=%u next=%lu interval=%lu\r\n",
	                           (unsigned)g_snap_enable,
	                           (unsigned)(node_is_provisioned() ? 1u : 0u),
	                           (unsigned)my_mid,
	                           (unsigned long)g_snap_next_tick,
	                           (unsigned long)g_snap_interval_ms);
	             }
	         }

	          //===================== RTC / SUN =====================
	         rtc_update();
	         update_sun_times();

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
	                 (unsigned)g_manual_override_active,
	                 (unsigned)manual_override_duration_min(),
	                 (unsigned)(managed_dbg ? want_on_dbg : 0u),
	                 (unsigned)light_is_on_logical()
	             );
	             if (rtc_dbg_len > 0) {
	                 HAL_UART_Transmit(&huart6, (uint8_t*)rtc_dbg, (uint16_t)rtc_dbg_len, 50);
	             }
	         }

	         int now_min = g_rtc_hour * 60 + g_rtc_min;
	         int night   = is_night(now_min, g_dusk_min, g_dawn_min);

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
	         static uint32_t hb_t = 0;
	         static uint32_t ai_feed_tick = 0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

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
    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 0x1;
    sDate.Year = 0x0;
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }
  }
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the TimeStamp
  */
  if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

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

static void push_snapshot(bool light_on, float voltage, float current, float temp, float supersonic)
{
    snapshot_t s = {
        .light_on   = light_on,
        .voltage    = voltage,
        .current    = current,
        .supersonic = supersonic,
        .temp       = temp,
        .count      = g_monitor_count,
    };

    g_snap[g_snap_head] = s;
    g_snap_head = (g_snap_head + 1u) % SNAP_RING_SIZE;
    g_snap_inited = true;
}

static void resp_slot_task_poll(void)
{
    uint32_t now = HAL_GetTick();

    int ready_idx = -1;
    uint32_t best_due = 0xFFFFFFFFu;
    
    __disable_irq();
    for (int i = 0; i < RESP_QUEUE_SIZE; ++i) {
        if (!g_resp_q[i].pending) continue;
        if ((int32_t)(now - g_resp_q[i].due_tick) < 0) continue;

        if (ready_idx < 0 || g_resp_q[i].due_tick < best_due) {
            ready_idx = i;
            best_due = g_resp_q[i].due_tick;
        }
    }

    if (ready_idx < 0) {
        __enable_irq();
        return;
    }

    // 현재 송신 슬롯으로 복사
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

    // RAW 버퍼 응답이면 그대로 전송
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

            if (!enqueue_transport_tx(g_resp_slot.tmid, tx_cmd, 0u, g_resp_slot.msg_id, g_resp_slot.buf, g_resp_slot.len, HOP_TTL_DEFAULT)) 
            {
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
        Send_Monitoring_Snapshot_JSON(g_resp_slot.msg_id);
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

static bool is_compact_snap_body(const uint8_t *data, uint16_t len)
{
    return data != NULL &&
           len == SNAP_COMPACT_BODY_LEN &&
           data[0] == 0x01u;
}

bool rtc_ensure_valid_or_set_default(void)
{
    RTC_TimeTypeDef t = {0};
    RTC_DateTypeDef d = {0};

    if (HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN) != HAL_OK) return false;
    if (HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN) != HAL_OK) return false;

    
    if (d.Year >= 24 && d.Month >= 1 && d.Month <= 12 && d.Date >= 1 && d.Date <= 31 &&
        !(d.Year == 25 && d.Month == 12 && d.Date == 22) &&
        !(d.Year == 26 && d.Month == 3 && d.Date == 24)) {
        return false;
    }

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours   = 11;
    sTime.Minutes = 33;
    sTime.Seconds = 0;

    sDate.Year    = 26;              // 2026
    sDate.Month   = RTC_MONTH_MAY; // 3
    sDate.Date    = 04;
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) return false;
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) return false;
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2u);

    return true;
}

void Vac_Ctrl(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);

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
void Format_UID(char *msg, size_t size) {
    Read_UID();  
    snprintf(msg, size, "%08" PRIX32 "-%08" PRIX32 "-%08" PRIX32 "\r\n",
             uid_ram[2], uid_ram[1], uid_ram[0]);
}

uint32_t Get_Device_ID(void) {
           return DBGMCU->IDCODE;  
}

uint32_t Read_ADC_Channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }

    uint32_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}

float Convert_Voltage_ADC(uint32_t adc_value) {
    float vref_actual = 3.3;
    float voltage = ((float)adc_value / 4095.0f) * vref_actual;
    return voltage;
}

float Convert_Voltage_To_Current(float voltage, float offset) {
    float current = (voltage - offset) / 0.137f;
    return current;
}


static void apply_mid(uint16_t new_mid)
{
    if (new_mid == 0 || new_mid == MID_INVALID) {
        return;
    }

    // 1) Wi-SUN 모듈설정
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

void Print_Voltage_Current(void) {
	ADC_ChannelConfTypeDef sConfig = {0};
	    uint32_t adc_val_current;
	    char log[64];
	    
	    sConfig.Channel = ADC_CHANNEL_3;  // PA6
	    sConfig.Rank = ADC_REGULAR_RANK_1;
	    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	    sConfig.SingleDiff = ADC_SINGLE_ENDED;
	    sConfig.OffsetNumber = ADC_OFFSET_NONE;
	    sConfig.Offset = 0;

	    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
	        Error_Handler();
	    }

	    HAL_ADC_Start(&hadc1);
	    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	    adc_val_current = HAL_ADC_GetValue(&hadc1);
	    HAL_ADC_Stop(&hadc1);
	    
	    float voltage = Convert_Voltage_ADC(adc_val_current);  // 3.3 * adc / 4095
	    float current = Convert_Voltage_To_Current(voltage, 2.50f);  
	    
	    snprintf(log, sizeof(log),
	             "ADC_RAW_CURR: %lu | Voltage: %.2fV | Current: %.2fA\r\n",
	             adc_val_current, voltage, current);

	    HAL_UART_Transmit(&huart6, (uint8_t*)log, strlen(log), HAL_MAX_DELAY);
}

void debug_print_boot_info(uint16_t stored_mid)
{
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
                       "[BOOT] stored_mid=%04X, cfg.mid=%04X, cfg.mid_assigned=%u, my_mid=%04X\r\n",
                       stored_mid, g_node_cfg.mid, g_node_cfg.mid_assigned, my_mid);

    if (len > 0) {
        if (len > sizeof(buf)) {
            len = sizeof(buf); 
        }
        HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
    }
}

void PrintReceivedPacket(const char* prefix, const uint8_t* data, uint16_t length) {
#if FOCUS_TIMING_LOG
    (void)prefix;
    (void)data;
    (void)length;
    return;
#else
    char msg[256];
    uint32_t timestamp = HAL_GetTick();
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    /*HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);*/

    int pos = 0;
    /*pos += snprintf(msg, sizeof(msg), "[%02d:%02d:%02d] %s",
                        sTime.Hours, sTime.Minutes, sTime.Seconds, prefix);*/
    for (int i = 0; i < length && pos < sizeof(msg) - 3; i++) {
        pos += snprintf(&msg[pos], sizeof(msg) - pos, "%02X ", data[i]);
    }
    for (int i = 0; i < length; ++i) {
        if (pos > 240 || i == length-1) {
            msg[pos++] = '\r';
            msg[pos++] = '\n';
            HAL_UART_Transmit(&huart6, (uint8_t*)msg, pos, HAL_MAX_DELAY);
            pos = 0;
        }
    }
    HAL_UART_Transmit(&huart6, (uint8_t*)msg, pos, HAL_MAX_DELAY);
#endif
}


uint8_t calc_checksum(uint8_t *buf, uint16_t len) {
	uint8_t sum = 0;
	for (uint16_t i = 0; i < len; i++) sum += buf[i];
	return sum;
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

static void scheduler_poll(void)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;
    uint8_t want_on = 0;
    uint8_t managed = 0;
    uint32_t now_tick = HAL_GetTick();
    uint8_t control_mode = 0;

    if (g_manual_override_active) {
        if (g_manual_override_no_timeout ||
            ((int32_t)(now_tick - g_manual_override_until) < 0)) {
            light_event_set_reason(g_manual_override_latch_off_on_expire ?
                                   LIGHT_EVENT_REASON_SET_FORCED :
                                   LIGHT_EVENT_REASON_CMD);
            if (g_manual_override_light_on) {
                light_on();
            } else {
                light_off();
            }
            light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
            return;
        }

        if (g_manual_override_latch_off_on_expire) {
            g_manual_override_latch_off_on_expire = 0u;
            light_event_set_reason(LIGHT_EVENT_REASON_FORCED_EXPIRE);
            start_manual_override_for(0u, 0u);
            light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
            return;
        }

        g_manual_override_active = 0u;
        g_manual_override_no_timeout = 0u;
        g_manual_override_latch_off_on_expire = 0u;
    }

#if LIGHT_TEST_10S_ENABLE
    light_event_set_reason(LIGHT_EVENT_REASON_TEST);
    if (((now_tick / LIGHT_TEST_10S_PERIOD_MS) & 0x1u) != 0u) {
        light_on();
    } else {
        light_off();
    }
    light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
    return;
#endif

    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);
    
    uint16_t now_min = (uint16_t)t.Hours * 60u + t.Minutes;
    uint16_t on_min = 0u;
    uint16_t off_min = 0u;
    uint16_t saving_start_min = (uint16_t)g_node_cfg.saving_start_hour * 60u + g_node_cfg.saving_start_min;
    uint16_t saving_end_min = (uint16_t)g_node_cfg.saving_end_hour * 60u + g_node_cfg.saving_end_min;

    if (sched_last_day != d.Date) {
        sched_last_day = d.Date;
        on_done_today  = false;
        off_done_today = false;
    }

    control_mode = current_control_mode();

    switch (control_mode) {
    case 0:
        managed = 1;
        on_min = apply_time_correction_min(g_sunset_min, g_node_cfg.on_corr_mode, g_node_cfg.on_corr_time_min);
        off_min = apply_time_correction_min(g_sunrise_min, g_node_cfg.off_corr_mode, g_node_cfg.off_corr_time_min);
        want_on = time_window_contains(now_min, on_min, off_min);
        break;

    case 1:
        managed = 1;
        on_min = apply_time_correction_min(g_dusk_min, g_node_cfg.on_corr_mode, g_node_cfg.on_corr_time_min);
        off_min = apply_time_correction_min(g_dawn_min, g_node_cfg.off_corr_mode, g_node_cfg.off_corr_time_min);
        want_on = time_window_contains(now_min, on_min, off_min);
        break;

    case 2:
        managed = 1;
        on_min  = (uint16_t)g_node_cfg.light_on_hour  * 60u + g_node_cfg.light_on_min;
        off_min = (uint16_t)g_node_cfg.light_off_hour * 60u + g_node_cfg.light_off_min;
        on_min = apply_time_correction_min(on_min, g_node_cfg.on_corr_mode, g_node_cfg.on_corr_time_min);
        off_min = apply_time_correction_min(off_min, g_node_cfg.off_corr_mode, g_node_cfg.off_corr_time_min);
        if (on_min != off_min) {
            want_on = time_window_contains(now_min, on_min, off_min);
        }
        break;

    case 3:
        managed = 1;
        want_on = 0u;
        break;

    default:
        break;
    }

    if (!managed) return;

    if (g_node_cfg.saving_mode &&
        time_window_contains(now_min, saving_start_min, saving_end_min)) {
        want_on = 0u;
    }

    light_event_set_reason((g_node_cfg.saving_mode && time_window_contains(now_min, saving_start_min, saving_end_min)) ? LIGHT_EVENT_REASON_SAVING : LIGHT_EVENT_REASON_SCHEDULE);

    if (want_on) {
        light_on();
    } else {
        light_off();
    }
    light_event_set_reason(LIGHT_EVENT_REASON_UNKNOWN);
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

void wisun_toss_packet(uint16_t tmid, const uint8_t *rx, uint16_t rx_len)
{
    wisun_frame_view_t v;
    if (!wisun_parse_frame(rx, rx_len, &v)) {
        return; 
    }

    wisun_frame_cfg_t cfg = {
        .sig1 = 0xAA,
        .sig2 = 0xAB,  
        .tmid = tmid,
    };
    
    wisun_send_frame(&cfg, v.data, v.data_len, wisun_tx_adapter, NULL);
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

    HAL_StatusTypeDef st = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)raw_buffer, FFT_SIZE);
    if (st != HAL_OK) {
        ultra_sampling_paused = true;
		char b[64];
		int n = snprintf(b, sizeof(b), "[DMA] start fail st=%d\r\n", (int)st);
		HAL_UART_Transmit(&huart6, (uint8_t*)b, n, 20);
	}
    g_frame_c0 = DWT->CYCCNT;
    
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    (void)HAL_TIM_Base_Start(&htim6);
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

    /*if (!ultra_frame_ready) return;   
        ultra_frame_ready = 0;*/
        static uint32_t fft_seq = 0;
		static uint32_t dbg_t = 0;
        
        InitHannWindowOnce();

        uint32_t my_seq = ++fft_seq;
        
        float vin_v = 0.0f;
        float i_adc_v = 0.0f;
        {
            VIRead vi;
            if (AD_DC_Injected_Once(&vi) == HAL_OK) {
                vin_v   = (float)vi.volt_raw * (3.3f / 4095.0f);
                i_adc_v = (float)vi.curr_raw * K_ADC2V;
            }
        }

        static float32_t x[FFT_SIZE];        

        /*__disable_irq();
        ultra_frame_ready = false;
        __enable_irq();*/

        for (int i = 0; i < FFT_SIZE; i++) {
		   uint16_t raw = raw_buffer[i];
		   x[i] = ((float)raw * 3.3f / 4095.0f) - 1.65f;
	   }
        uint32_t c0 = g_frame_c0;
        uint32_t c1 = g_frame_c1;
        uint32_t dc = (c1 >= c0) ? (c1 - c0) : (0xFFFFFFFFu - c0 + c1 + 1u);

        float dt_s  = (float)dc / (float)SystemCoreClock;
        float dt_us = dt_s * 1e6f;
        float dt_ms = dt_s * 1e3f;
        float fs_eff = (dt_s > 1e-9f) ? ((float)FFT_SIZE / dt_s) : 0.0f;
        /*uart6_log("[DTDBG] c0=%lu c1=%lu dc=%lu SystemCoreClock=%lu FFT_SIZE=%u dt_ms=%.3f fs_eff=%.1f\r\n",
                  (unsigned long)c0, (unsigned long)c1, (unsigned long)dc,
                  (unsigned long)SystemCoreClock, (unsigned)FFT_SIZE, dt_ms, fs_eff);*/
        // ---- Zero-crossing 주파수 추정 ----
        /*int zc = 0;
        for (int i = 1; i < FFT_SIZE; i++) {
            float a = x[i-1];
            float b = x[i];
            if ((a <= 0.0f && b > 0.0f) || (a >= 0.0f && b < 0.0f)) zc++;
        }
        float f_zc = 0.0f;
        if (fs_eff > 1.0f) {
            // crossing 2??= 1주기 가??
            f_zc = ((float)zc * 0.5f) * (fs_eff / (float)FFT_SIZE);
        }

        char b[160];
        int n = snprintf(b, sizeof(b),
            "[RAWCHK] dt=%.2fms fs_eff=%.1fHz zc=%d f_zc=%.1fkHz\r\n",
            (double)dt_ms, (double)fs_eff, zc, (double)(f_zc/1000.0f));
        HAL_UART_Transmit(&huart6, (uint8_t*)b, (uint16_t)n, 20);*/

        
        uint32_t now = HAL_GetTick();
        if ((now - dbg_t) >= 1000u) {
            dbg_t = now;
            float mn = 1e9f, mx = -1e9f;
            for (int i = 0; i < FFT_SIZE; i++) {
                if (x[i] < mn) mn = x[i];
                if (x[i] > mx) mx = x[i];
            }
            char tbuf[120];
			int tn = snprintf(tbuf, sizeof(tbuf),
				"[IN] min=%.6f max=%.6f p2p=%.6f\r\n",
				(double)mn, (double)mx, (double)(mx - mn)
			);

			if (tn < 0) tn = 0;
			if (tn > (int)sizeof(tbuf)) tn = (int)sizeof(tbuf);

			HAL_StatusTypeDef st2 = HAL_UART_Transmit(&huart6, (uint8_t*)tbuf, (uint16_t)tn, 20);
			if (st2 != HAL_OK) uart_fail_cnt++;
        }

        /* ---- FFT -> magnitude (ExtractFullFFT) ---- */
        static float32_t mag[FFT_SIZE/2];
        ExtractFullFFT_MagOnly(x, mag);
    
        const float lo_hz = 80000.0f;
        const float hi_hz = 130000.0f;

        float fs = (fs_eff > 1.0f) ? fs_eff : (float)FSAMPLE;
        float nyq = 0.5f * fs;

        float lo = lo_hz;
        float hi = hi_hz;
        
        const float guard = 3000.0f;         
        if (hi > nyq - guard) hi = nyq - guard;
        if (lo < 1.0f) lo = 1.0f;

        if (lo >= hi) {
            char buf[160];
            int len = snprintf(buf, sizeof(buf),
                "[FFT#%lu] fs=%.1fk nyq=%.1fk lo/hi invalid (lo=%.1fk hi=%.1fk)\r\n",
                (unsigned long)my_seq,
                (double)(fs/1000.0f), (double)(nyq/1000.0f),
                (double)(lo/1000.0f), (double)(hi/1000.0f));
            HAL_UART_Transmit(&huart6, (uint8_t*)buf, (uint16_t)len, 20);
            //Ultra_ResumeNextFrame();
            Ultra_StartDmaFrame();
            return;
        }
        
        float max_amp = 0.0f;
        float peak_f  = 0.0f;
        int peak_i  = -1;

        float min_amp = 1e30f;
        float min_f   = 0.0f;
        int   min_i   = -1;

        float sum = 0.0f;
        int   cnt = 0;
        float top1 = 0.0f, top2 = 0.0f, top3 = 0.0f;

        for (int i = 1; i < (FFT_SIZE/2); i++) {
        	float freq = ((float)i * fs) / (float)FFT_SIZE;
        	if (freq < lo || freq > hi) continue;

            float a = mag[i];

            if (a > max_amp) {
                max_amp = a;
                peak_f  = freq;
                peak_i  = i;
            }

            if (a < min_amp) {
			   min_amp = a;
			   min_f   = freq;
			   min_i   = i;
		   }

            sum += a;
            cnt++;

            if (a > top1) { top3 = top2; top2 = top1; top1 = a; }
            else if (a > top2) { top3 = top2; top2 = a; }
            else if (a > top3) { top3 = a; }

            if (a < min_amp) { min_amp = a; min_f = freq; min_i = i; }
            if (min_i < 0) {
              min_amp = 0.0f;
              min_f   = 0.0f;
            }
        }
        if (cnt == 0) {
            min_amp = 0.0f;
            min_f = 0.0f;
            min_i   = -1;
        }
        if (peak_i >= 2 && peak_i <= (FFT_SIZE/2 - 2)) {
            float a0 = mag[peak_i - 1];
            float a1 = mag[peak_i];
            float a2 = mag[peak_i + 1];
            
            float a_rss = sqrtf(a0*a0 + a1*a1 + a2*a2);

            max_amp = a_rss;
        }

        float noise_floor = 0.0f;
        if (cnt > 0) {
            float trimmed = sum - (top1 + top2 + top3);
            int trimmed_cnt = cnt - 3;
            if (trimmed_cnt < 1) { trimmed = sum; trimmed_cnt = cnt; }
            noise_floor = trimmed / (float)trimmed_cnt;
        }
        if (noise_floor < NOISE_MIN) noise_floor = NOISE_MIN;

        const float N  = (float)FFT_SIZE;
        const float CG = 0.5f;                 // Hann coherent gain(근사)
        
        float vpk  = (2.0f / (N * CG)) * max_amp;   // ??(4/N)*max_amp
        float mvpk = vpk * 1000.0f;

        float vpp  = 2.0f * vpk;
        float mvpp = vpp * 1000.0f;

        float vrms = vpk * 0.70710678f;         // = vpk/sqrt(2)

        float adc_pk  = vpk * (4095.0f / 3.3f);
        float adc_pp  = vpp * (4095.0f / 3.3f);

        float snr = (noise_floor > 0.0f) ? (max_amp / noise_floor) : 0.0f;
        bool found = (cnt > 0) && (snr > FFT_SNR_K);
        //bool found = (cnt > 0) && (max_amp > (noise_floor * FFT_SNR_K));
        
        {
            char buf[420];
            int len;

            if (found) {
                len = snprintf(buf, sizeof(buf),
                    "[FFT#%lu] found=1 peak=%.2fkHz bin=%d "
                    "max=%.3g min=%.3g@%.2fkHz "
                    "vpk=%.4fV vpp=%.4fV vrms=%.4fV "
                    "adc_pk=%.1f adc_pp=%.1f "
                    "nf=%.3g snr=%.2f cnt=%d "
                    "dt=%.2fms fs_eff=%.1fk fs=%.1fk nyq=%.1fk lo=%.1fk hi=%.1fk "
                    "vin=%.2fV i=%.2fA\r\n",
                    (unsigned long)my_seq,
                    (double)(peak_f/1000.0f),
                    peak_i,
                    (double)max_amp,
                    (double)min_amp,
                    (double)(min_f/1000.0f),
                    (double)vpk,
                    (double)vpp,
                    (double)vrms,
                    (double)adc_pk,
                    (double)adc_pp,
                    (double)noise_floor,
                    (double)snr,
                    cnt,
                    (double)dt_ms,
                    (double)(fs_eff/1000.0f),
                    (double)(fs/1000.0f),
                    (double)(nyq/1000.0f),
                    (double)(lo/1000.0f),
                    (double)(hi/1000.0f),
                    (double)vin_v,
                    (double)i_adc_v
                );
            } else {
                len = snprintf(buf, sizeof(buf),
                    "[FFT#%lu] found=0 peak=%.2fkHz bin=%d "
                    "max=%.3g min=%.3g@%.2fkHz "
                    "nf=%.3g snr=%.2f cnt=%d "
                    "dt=%.2fms fs_eff=%.1fk fs=%.1fk nyq=%.1fk lo=%.1fk hi=%.1fk "
                    "vin=%.2fV i=%.2fA\r\n",
                    (unsigned long)my_seq,
                    (double)(peak_f/1000.0f),
                    peak_i,
                    (double)max_amp,
                    (double)min_amp,
                    (double)(min_f/1000.0f),
                    (double)noise_floor,
                    (double)snr,
                    cnt,
                    (double)dt_ms,
                    (double)(fs_eff/1000.0f),
                    (double)(fs/1000.0f),
                    (double)(nyq/1000.0f),
                    (double)(lo/1000.0f),
                    (double)(hi/1000.0f),
                    (double)vin_v,
                    (double)i_adc_v
                );
            }
            uint32_t tmo = 5 + (uint32_t)((len * 10u * 1000u) / 115200u);
            HAL_StatusTypeDef st = HAL_UART_Transmit(&huart6, (uint8_t*)buf, (uint16_t)len, tmo);
            if (st != HAL_OK) uart_fail_cnt++;
        }
        
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
