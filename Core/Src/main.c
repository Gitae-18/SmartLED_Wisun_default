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
#include <string.h>
#include <stdint.h>
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
    uint8_t  which_idx;   // 링버퍼 인덱스 (여기선 1개만 쓰므로 0)
    uint16_t nonce;
    uint32_t due_tick;    // 전송 예정 시각 (ms)
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

    uint8_t  p_on_valid, p_on;   // light_on 결과 회신용
} resp_task_t;

typedef enum {
    RESP_KIND_NONE = 0,
    RESP_KIND_SNAP,
    RESP_KIND_ACK,      // 간단한 ACK/에러 등
    RESP_KIND_RAW_BIN,  // 이미 완성된 바이너리 버퍼를 그냥 보내기
} resp_kind_t;

typedef struct {
    uint8_t      pending;      // 1이면 예약 있음
    resp_kind_t  kind;         // 어떤 타입의 응답인지
    uint16_t     tmid;         // 응답 TMID (보통 BR MID)
    uint16_t     msg_id;       // 요청의 msg_id (슬롯 seed용)
    uint32_t     due_tick;     // 언제 보낼지
    uint8_t  cmd;
    uint8_t  result;

    // 이미 완성된 버퍼로 보내고 싶을 때 사용
    uint8_t      buf[128];     // 필요 크기에 맞게 조정
    uint16_t     len;
    uint8_t      has_raw_buf;  // buf/len을 사용할지 여부
} resp_slot_t;

typedef struct {
    uint32_t start_after_ms;  // p.start_after_ms
    uint32_t slot_len_ms;     // p.slot_len_ms
    uint32_t jitter_ms;       // p.jitter_ms
    uint16_t max_mid;         // p.max_mid
    enum { ORDER_MID_DESC=0 } order;
} resp_slot_cfg_t;

typedef struct {
    uint8_t  in_use;               // 0 = 비어 있음, 1 = 사용 중

    uint16_t tmid;                 // 이 프레임의 TMID (어디로 보낼지)
    uint8_t  data[256];   // Wi-SUN DATA 부분만
    uint16_t data_len;             // DATA 길이

    uint32_t due_tick;             // 언제 보낼지 (ms)
} hop_slot_t;

/*typedef struct {
    uint8_t  mode;              // 0:일출/일몰(reserve), 1:시민박명(reserve)
                                // 2:user_time, 3:manual

    uint8_t  light_on_hour;     // mode == 2 일 때 의미 있음
    uint8_t  light_on_min;
    uint8_t  light_off_hour;
    uint8_t  light_off_min;

    uint8_t  manual_duration_min; // mode == 3 일 때 의미(분 단위, 0~255분 정도)

    uint8_t  snap_enable;       // 0/1
    uint8_t  snap_period_min;   // 스냅 주기(분) – 게이트웨이에서 sec → min 변환
} node_cfg_t;*/

typedef struct __attribute__((packed)) {
    uint8_t  t;       // 0x10 = generic_ack
    uint8_t  uid[12];

    uint32_t msg_id;  // PC에서 보낸 msg_id 그대로 반사
    uint8_t  ok;      // 1=success, 0=fail
    int8_t   err_code;
} AckBin_t;

typedef struct __attribute__((packed)) {
    uint8_t  t;      // 0x02 = get_status_resp
    uint8_t  uid[12];

    float    volt;
    float    curr;
    float    temp;

    uint8_t  light_on;     // 0/1
    // ---- 공통 응답 메타데이터 ----
       uint32_t msg_id;   // 요청이 있을 때는 그 msg_id, 주기 스냅이면 0
       uint8_t  ok;       // 1=success
       int8_t   err_code; // 0=OK, 그 외 에러
} StatusBin_t;

typedef struct __attribute__((packed)) {
    uint8_t  t;        // 0x11 = slot_resp (예: light_on 브로드캐스트 응답)
    uint8_t  uid[12];  // 노드 UID

    uint16_t mid;      // 내 MID (게이트웨이에서 누가 응답했는지 알 수 있게)
    uint8_t  ok;       // 1=success, 0=fail

    uint8_t  has_on;   // g_resp.p_on_valid
    uint8_t  on;       // g_resp.p_on (0/1)

    uint16_t nonce;    // g_resp.nonce (없으면 0 그대로)
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
  uint8_t gid;        // 네 프로젝트에서 필요하면
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

static node_cfg_t g_node_cfg;
static resp_slot_t g_resp_slot;
static AckBin_t g_nodeinfo_ack;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 100
#define PACKET_MAX_SIZE 256
#define HOP_QUEUE_SIZE   16   // 필요하면 4~8 정도로
#define HOP_MAX_FRAME 256

#define HOP_TTL_DEFAULT  20   // TTL 기본값
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

#define FFT_SNR_K  8.0f
#define FFT_NOISE_MIN    (1e-6f)

#define SAMPLING_RATE 600000.0f

#define PACKET_STX    0x02
#define PACKET_ETX    0x03
#define SIG1          0xAA
#define SIG2          0xAB

#define LIGHT_ON 0x10
#define LIGHT_OFF 0x11
#define GET_STATUS 0x30
#define SET_MID  0x01
#define NODE_CFG 0x20
#define SET_MID_CH  0x21
#define FIRST_BOOT  0x22
#define GETID       0x23
#define SET_SETTING    0x31
#define GET_NODE_INFO 0x40
#define GET_CH 0x24
#define SET_CH 0x25
#define SET_POWER
#define NODE_CFG_MAGIC      0x4E434647u   // 'N' 'C' 'F' 'G'
#define T_GET_CH_RESP 0x24
#define T_NODE_INFO_TEXT 0x40   // 응답 body 타입(예시)
#define NODE_INFO_TEXT_MAX 240

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
    uint16_t msg_id;     // 요청 msg_id 그대로
    uint8_t  ok;         // 1=성공, 0=실패
    int8_t   err_code;   // 0=OK, <0 error
    uint16_t text_len;   // 뒤 텍스트 길이
    // uint8_t text[text_len];
} NodeInfoTextHdr_t;
#pragma pack(pop)

typedef struct {
    uint8_t  pending;     // 1이면 수집 중
    uint16_t msg_id;      // 요청 msg_id
    uint16_t tmid;        // 응답 보낼 대상(mid)
    uint16_t used;        // text 사용량
    uint8_t  text[NODE_INFO_TEXT_MAX];
    uint32_t deadline_tick;
	uint32_t last_rx_tick;
	uint16_t got_mask;
} nodeinfo_ctx_t;

#define FFT_DURATION_MS 60000  // 측정 시간 1분 (1000 * 60)
#define FFT_DELAY_MS     100

#define AE_ROWS 10
#define AE_COLS 130

#define TRIGGER_THRESHOLD  0.1f
#define TRIGGER_TIMEOUT 1000
#define FFT_WINDOW_MS 200
#define NUM_CHANNELS 3
#define ADC_BUFFER_SIZE 256
#define ULTRA_BUF_LEN   4096

#define ADC_MAX_COUNTS    4095.0f
#define VREF_FIXED        3.3f

// 전압 분배비 (없으면 1.0으로)
#define R_TOP_V           100000.0f   // 예: 100k
#define R_BOT_V           10000.0f    // 예: 10k
#define V_DIV_GAIN        ((R_TOP_V + R_BOT_V) / R_BOT_V)   // 분배 없으면 R_TOP_V=0, R_BOT_V=1

// 전류측정 체인
#define R_SHUNT           0.05f
#define I_AMP_GAIN        50.0f       // 증폭배수(x)

#define I_OFFSET_RAW    1990
#define SNAP_RING_SIZE  1
#define SLOT_JITTER_MS  150

#define LIGHT_Pin        GPIO_PIN_12       // 원하는 핀 번호
#define LIGHT_GPIO_Port  GPIOA
#ifndef MID_INVALID
#define MID_INVALID ((uint16_t)0xFFFF)
#endif

#ifndef SNAP_FFT_PAIRS
#define SNAP_FFT_PAIRS 2  // 기본 안전값
#endif

#ifndef SUP_MIN_HZ
#define SUP_MIN_HZ 20000.0f   // 20 kHz
#endif
#ifndef SUP_MAX_HZ
#define SUP_MAX_HZ 80000.0f   // 80 kHz
#endif

typedef struct __attribute__((packed)) {
    uint8_t  t;         // 타입 예: 0x01 = snap
    uint8_t  uid[12];

    float    volt;
    float    curr;
    float    temp;

    uint8_t  fft_count;
    struct {
        float freq;
        float amp;
    } fft[SNAP_FFT_PAIRS];

    // ---- 공통 응답 메타데이터 ----
       uint32_t msg_id;   // 요청이 있을 때는 그 msg_id, 주기 스냅이면 0
       uint8_t  ok;       // 1=success
       int8_t   err_code; // 0=OK, 그 외 에러
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

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

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
static uint32_t g_snap_interval_ms = 60000;   // 기본 60초
static uint32_t g_snap_next_tick   = 0;

static char     g_uid_str[40];
static uint32_t uid_ram_local[3];
static uint32_t last_snap_req_tick = 0;
static volatile bool     wisun_packet_ready = false;
static volatile uint16_t wisun_packet_len   = 0;
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
static uint8_t  g_hop_seen_count = 0;  // 현재 사용 중인 엔트리 수 (<= HOP_SEEN_TABLE_SIZE)
static uint8_t  g_hop_seen_pos   = 0;  // 링버퍼 인덱스

// 날짜 바뀔 때 플래그 리셋용
static uint8_t sched_last_day = 0xFF;

static nodeinfo_ctx_t g_nodeinfo = {0};

static uint8_t g_nodeinfo_txbuf[sizeof(NodeInfoTextHdr_t) + NODE_INFO_TEXT_MAX];

static const float K_ADC2V   = VREF_FIXED / ADC_MAX_COUNTS;                 // raw → V
//static const float K_VIN     = K_ADC2V * V_DIV_GAIN;                        // raw → Vin(V)
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

static float baseline = 0.0f;
static float noise_level = 0.01f;  // 초기값 적당히
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
uint32_t          ai_next_run = 0;      // 다음 실행 가능 시각
uint32_t          ai_period_ms = 200;   // 추론 최소 간격(처음엔 넉넉히)
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

uint16_t g_rtc_day;  // 일수는 보통 1~31 (uint8_t도 가능)
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

static uint32_t ai_test_start = 0;
static uint8_t  ai_test_done  = 0;

static uint32_t ai_last_tick = 0;
static float ai_v = -1.0f;
static float ai_dv = 0.02f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC2_Init(void);
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
void boot_poll(void);
//static void build_snapshot_packet(cbor_packet_t* rp, const snapshot_t* s, bool as_resp, const char* topic, uint16_t nonce);
float Convert_ADC_To_Current(uint32_t adc_value);
float Convert_Voltage_To_Current(float voltage, float offset);
float Convert_Voltage_ADC(uint32_t adc_value);
void dbg_dump_uart6_with_tag(const char *tag, const uint8_t *p, uint16_t n);
void dbg_print_mid_info(const char *tag, uint16_t my_mid, uint16_t target_mid);
void debug_print_boot_info(uint16_t stored_mid);
void debug6(const char *s);
static inline void DWT_CYCCNT_Init(void);
static void dump_hex_uart(const uint8_t *p, uint16_t n);
void Debug_Print_FFT_Peak(void);
void ExtractFullFFT(const float32_t *in, FftData_t *dest);
static void ExtractFullFFT_MagOnly(const float32_t *in, float32_t *mag_out);
void Format_UID(char *msg, size_t size);
static uint16_t find_first_zero(const uint8_t *p, uint16_t n);
static void hop_tx_task_poll(void);
static void init_uid_string(void);
static inline int is_night(int now, int dusk, int dawn);
static bool is_bootstrap_cmd(uint8_t cmd);
void Input_Ai_Model(float v);
static void InitHannWindowOnce(void);
uint32_t Get_Device_ID(void);
float Get_Calibrated_Vref(void);
float Get_Offset_Voltage(void);
//void StreetLight_ToggleTask(void);
static void handle_binary_cmd(uint8_t cmd, uint8_t flags, uint16_t msg_id, uint16_t tmid, const uint8_t *data, uint16_t len);
static bool hop_seen_key(uint32_t key);
static void hop_mark_key(uint32_t key);
static uint8_t handle_cmd_set_setting(const uint8_t *data, uint16_t len);
void light_off(void);
void light_on(void);
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
void Parse_AT_Response(const char* buffer);
void PrintReceivedPacket(const char* prefix, const uint8_t* data, uint16_t length);
void Print_Voltage_Current(void);
void Print_FFT_Summary(uint16_t *raw_buf);
static inline void PA12_toggle_soft(void);
static void push_snapshot(bool light_on, float voltage, float current, float temp, float supersonic);
static int parse_hex8(const char *s, uint8_t out[8]);
static int hexval(char c);
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
void startADCInterrupt(void);
void Send_UID_UART2(void);
void Send_Device_ID_UART2(void);
void Send_Broadcast_Command(uint8_t *request_data, uint8_t request_length);
void SendFFT_Packet(uint16_t target_mid, FftData_t *fft_data, uint8_t count);
void SendDataPacket(uint16_t target_mid, uint8_t *data, uint16_t data_length);
void Send_Monitoring_Snapshot_JSON(uint16_t req_msg_id);
static void schedule_resp_with_slot(resp_kind_t kind, uint16_t tmid, uint16_t msg_id, const uint8_t *raw, uint16_t raw_len);
static bool send_wisun_resp(uint16_t tmid, const uint8_t *cbor, size_t cbor_len);
static bool send_wisun_binary(uint16_t tmid, const uint8_t *data, size_t len);
static void send_resp_now_from_task(void);
void tx_task_poll(void);
void Ultra_ResumeNextFrame(void);
void Ultra_StartSampling(void);
static void update_sun_times(void);
static inline float vsense_to_vin(float v_sense);
static inline float vsense_to_current(float v_sense, float offset_v);
void wisun_toss_packet(uint16_t tmid, const uint8_t *rx, uint16_t rx_len);
void wisun_process_rx_mainloop(void);
static inline uint16_t xorshift16(uint16_t x);
void reconfigure_snapshot_timer_from_cfg(void);
void rtc_update(void);
bool rtc_ensure_valid_or_set_default(void);
static void uart6_log(const char *fmt, ...);

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
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
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

static void at_q_push_isr(const char *s, uint16_t len)
{
    if (len >= AT_LINE_MAX) len = AT_LINE_MAX - 1;

    uint8_t next = at_q_next(at_q_w);
    if (next == at_q_r) {
        // full: 정책 선택
        // 1) drop newest (그냥 리턴)
        // 2) drop oldest (r 한칸 전진)  -> 추천
        at_q_r = at_q_next(at_q_r); // drop oldest
    }

    at_q[at_q_w].len = len;
    memcpy(at_q[at_q_w].s, s, len);
    at_q[at_q_w].s[len] = '\0';
    at_q_w = next;
}

static void at_accum_reset(void) {
    at_accum_len = 0;
    at_accum[0] = '\0';
}

// main에서 호출 (pop)
static uint8_t at_q_pop(char *out, uint16_t out_sz)
{
    if (at_q_is_empty()) return 0;

    __disable_irq();
    uint8_t r = at_q_r;
    uint16_t len = at_q[r].len;
    if (len >= out_sz) len = out_sz - 1;
    memcpy(out, at_q[r].s, len);
    out[len] = '\0';
    at_q_r = at_q_next(at_q_r);
    __enable_irq();

    return 1;
}

static bool node_is_provisioned(void)
{
    // my_mid를 이미 전역으로 쓰고 있으니까 그거만 이용
    return (my_mid != 0x0000u);
}

static bool is_bootstrap_cmd(uint8_t cmd)
{
    // 실제 enum/define 값에 맞춰서 넣으면 됨
    switch (cmd) {
    case FIRST_BOOT:
    case SET_MID_CH:   // 게이트웨이 → 노드 프로비저닝용
    case GETID:        // uid 질의용 있으면 같이
        return true;
    default:
        return false;
    }
}

void light_on(void)
{
    HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);
}

void light_off(void)
{
    HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);
}

static inline void InvalidateDCacheByAddr(uint32_t *addr, int32_t dsize) {
    if (SCB->CCR & SCB_CCR_DC_Msk) {
        int32_t linesize = 32;  // 보통 Cortex-M33은 32-byte cache line
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

static void dump_hex_uart(const uint8_t *p, uint16_t n)
{
    // 너무 길면 120바이트만
    uint16_t m = (n > 120) ? 120 : n;
    for (uint16_t i = 0; i < m; i++) {
        char b[8];
        int k = snprintf(b, sizeof(b), "%02X ", p[i]);
        HAL_UART_Transmit(&huart6, (uint8_t*)b, (uint16_t)k, 100);
    }
    HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, 100);
}

void boot_poll(void)
{
    if (boot_cfg_started) return;

    // Wi-SUN 모듈이 준비된 타이밍 조건이 있으면 여기 넣고,
    // 없으면 부팅 후 1~2초 딜레이로도 충분함.
    if (HAL_GetTick() < 1500) return;

    nodeinfo_start(/*tmid=*/0, /*msg_id=*/0);  // 부팅용이라 tmid/msg_id 의미없으면 0
    boot_cfg_started = 1;
}

static void uart6_log(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
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

	            // \n 수신되면 처리
	            if (rxByte == '\n')
	            {
	                // \r\n 제거
	                if (pc_rx_index >= 2 && pc_rx_buffer[pc_rx_index - 2] == '\r') {
	                    pc_rx_index -= 2;
	                } else {
	                    pc_rx_index -= 1;
	                }
	                pc_rx_buffer[pc_rx_index] = '\0';  // 문자열 종료

	                // UART1로 전송 (\r\n 붙여서)
	                HAL_UART_Transmit(&huart1, (uint8_t *)pc_rx_buffer, pc_rx_index, HAL_MAX_DELAY);
	                uint8_t crlf[] = "\r\n";
	                HAL_UART_Transmit(&huart1, crlf, 2, HAL_MAX_DELAY);

	                // PC에도 다시 echo 출력
	                HAL_UART_Transmit(&huart6, (uint8_t *)pc_rx_buffer, pc_rx_index, HAL_MAX_DELAY);
	                HAL_UART_Transmit(&huart6, crlf, 2, HAL_MAX_DELAY);

	                // 버퍼 초기화
	                memset(pc_rx_buffer, 0, RX_BUFFER_SIZE);
	                pc_rx_index = 0;
	            }
	        }
	        else
	        {
	            // 오버플로우 시 초기화
	            pc_rx_index = 0;
	            memset(pc_rx_buffer, 0, RX_BUFFER_SIZE);
	        }

	        // 다음 바이트 수신 준비
	        HAL_UART_Receive_IT(&huart6, &rxByte, 1);
	    }

		else if (huart->Instance == USART1) {

		    if (packet_state == WAITING_FOR_STX) {
		        // 바이너리 패킷의 시작 여부 검사
		        if (rxByte1 == PACKET_STX) {
		            wisun_rx_index = 0;
		            wisun_rx_buffer[wisun_rx_index++] = rxByte1;

		            //HAL_UART_Transmit(&huart6, &rxByte1, 1, HAL_MAX_DELAY);
		            packet_state = RECEIVING_PACKET;
		        }
		        // 그 외는 ASCII 응답으로 취급
		        else {


		            if (ascii_index < RX_BUFFER_SIZE - 1) {
		                ascii_buffer[ascii_index++] = rxByte1;

		                if (rxByte1 == '\r' || rxByte1 == '\n') {
		                    // 마지막에 \r 또는 \n만 남기지 않도록 제거
		                    while (ascii_index >= 1 &&
		                          (ascii_buffer[ascii_index - 1] == '\r' || ascii_buffer[ascii_index - 1] == '\n')) {
		                        ascii_index--;
		                    }

		                    if (ascii_index > 0) {
								uint16_t n = ascii_index;
								if (n >= RX_BUFFER_SIZE) n = RX_BUFFER_SIZE - 1;  // 안전

								memcpy(g_at_line, ascii_buffer, n);
								g_at_line[n] = '\0';
								g_at_line_len = n;
								g_at_line_ready = 1;   // "새 라인 도착"
							}

		                    ascii_index = 0;
		                    //memset(ascii_buffer, 0, sizeof(ascii_buffer));
		                }
		            } else {
		                // 오버플로우 시 초기화
		                ascii_index = 0;
		                //memset(ascii_buffer, 0, sizeof(ascii_buffer));
		            }
		        }
		    }
		    else if (packet_state == RECEIVING_PACKET) {
		        if (wisun_rx_index < RX_BUFFER_SIZE - 1) {
		            wisun_rx_buffer[wisun_rx_index++] = rxByte1;

		            if (rxByte1 == PACKET_ETX && wisun_rx_index >= 7) {
		                uint16_t packet_len = wisun_rx_index;

		                // ✅ ISR에서는 "완성 프레임"만 shadow로 넘기고 끝
		                if (packet_len <= PACKET_MAX_SIZE) {
		                    memcpy((void*)wisun_packet_shadow, wisun_rx_buffer, packet_len);
		                    wisun_packet_len   = packet_len;
		                    wisun_packet_ready = true;
		                }

		                // ✅ 상태 리셋 (memset은 굳이 필요 없음)
		                wisun_rx_index = 0;
		                packet_state = WAITING_FOR_STX;
		            }
		        } else {
		            // overflow
		            wisun_rx_index = 0;
		            packet_state = WAITING_FOR_STX;
		        }
		    }
		    // 마지막에 항상 재-arm
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
        if (g_hop_q[i].data_len < 1) {
            g_hop_q[i].in_use = 0;
            continue;
        }

        uint8_t ttl = g_hop_q[i].data[0];
        if (ttl == 0) {
            dbg_dump_uart6_with_tag("[HOP_DROP_TTL0]", g_hop_q[i].data, g_hop_q[i].data_len);
            g_hop_q[i].in_use = 0;
            continue;
        }
        g_hop_q[i].data[0] = ttl - 1;

        // ----- 실제 전송: wisun_send_frame로 새 프레임 구성 -----
        wisun_frame_cfg_t cfg = {
            .sig1 = 0xAA,
            .sig2 = 0xAB,
            .tmid = g_hop_q[i].tmid,
        };

        dbg_dump_uart6_with_tag("[HOP_TX]", g_hop_q[i].data, g_hop_q[i].data_len);
        (void)wisun_send_frame(&cfg,
                               g_hop_q[i].data,
                               g_hop_q[i].data_len,
                               wisun_tx_adapter,
                               NULL);

        g_hop_q[i].in_use = 0;
    }
}


static void handle_binary_cmd(uint8_t cmd,
                              uint8_t flags,
                              uint16_t msg_id,
                              uint16_t tmid,
                              const uint8_t *data,
                              uint16_t len)
{
    uint8_t uid12[12];
    mid_pack_uid12(uid12);
    uint8_t  ttl      = data[0];
    uint16_t src_mid  = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    uint16_t dst_mid  = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
    switch (cmd)
        {
            // ───────────────────────────────
            //  SNAP 요청 (0x13)
            // ───────────────────────────────
            case 0x13:
            {
            	uint32_t now = HAL_GetTick();

            	if ((int32_t)(now - last_snap_req_tick) < 3000) {
            	        // 너무 빠르면 요청 무시(또는 에러 응답)
            	        break;
            	}
            	last_snap_req_tick = now;

                // 지금 즉시 SNAP 전송 → X (충돌의 원인)
                // 슬롯 기반 SNAP 응답 예약
                schedule_resp_with_slot(
                    RESP_KIND_SNAP,   // SNAP 응답
                    tmid,             // 요청 원본에게 유니캐스트 하고 싶으면 tmid
                    msg_id,
                    NULL, 0           // SNAP은 raw 버퍼 없음. poll에서 생성.
                );
                break;
            }

            // ───────────────────────────────
            // LIGHT ON (0x10)
            // ───────────────────────────────
            case LIGHT_ON:
            {
                uint8_t on = flags & 0x01;
                HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin,
                                  on ? GPIO_PIN_SET : GPIO_PIN_RESET);

                // ACK 생성 (바이너리)
                AckBin_t ack = {0};
                ack.t = 0x10;
                memcpy(ack.uid, uid12, 12);
                ack.msg_id = msg_id;
                ack.ok = 1;
                ack.err_code = 0;

                // 슬롯 기반 ACK 전송 예약 (raw packet)
                schedule_resp_with_slot(
                    RESP_KIND_RAW_BIN,    // 이미 완성된 바이너리
                    src_mid,
                    msg_id,
                    (uint8_t*)&ack,
                    sizeof(ack)
                );
                break;
            }

            // ───────────────────────────────
            // LIGHT OFF (0x11)
            // ───────────────────────────────
            case LIGHT_OFF:
            {
                HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);

                AckBin_t ack = {0};
                ack.t = 0x10;
                memcpy(ack.uid, uid12, 12);
                ack.msg_id = msg_id;
                ack.ok = 1;
                ack.err_code = 0;

                schedule_resp_with_slot(
                    RESP_KIND_RAW_BIN,
                    src_mid,
                    msg_id,
                    (uint8_t*)&ack,
                    sizeof(ack)
                );
                break;
            }

            // ───────────────────────────────
            // MID 변경 (CMD_SET_MID)
            // ───────────────────────────────
            case SET_MID:
            {
                if (len < 2) break;
                uint16_t new_mid = ((uint16_t)data[0] << 8) | data[1];

                apply_mid(new_mid);

                // ACK 보내고 싶으면 아래처럼 예약
                // (선택)
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
                if (len < 9) break;

                uint8_t ver = data[0];
                (void)ver; // 나중에 버전 따라 분기 하고 싶으면 사용

                g_node_cfg.mode              = data[1];
                g_node_cfg.light_on_hour     = data[2];
                g_node_cfg.light_on_min      = data[3];
                g_node_cfg.light_off_hour    = data[4];
                g_node_cfg.light_off_min     = data[5];
                g_node_cfg.manual_duration_min = data[6];
                g_node_cfg.snap_enable       = data[7];
                g_node_cfg.snap_period_min   = data[8];

                save_node_cfg_to_flash(&g_node_cfg);          // 필요 시
                reconfigure_snapshot_timer_from_cfg();        // snap_enable/period_min 기준으로 타이머 생성

                // ACK 보내고 싶으면
                AckBin_t ack = {0};
                ack.t = NODE_CFG;
                memcpy(ack.uid, uid12, 12);
                ack.msg_id = msg_id;
                ack.ok = 1;
                ack.err_code = 0;
                schedule_resp_with_slot(
                    RESP_KIND_RAW_BIN,
                    0x0000,
                    msg_id,
                    (uint8_t*)&ack,
                    sizeof(ack)
                );
                break;
            }
            case SET_MID_CH:
                    {
                        if (len < 3) break;

                        uint16_t new_mid = ((uint16_t)data[0] << 8) | data[1];
                        uint8_t  new_ch  = data[2];

                        // 1) MID 적용 (node_cfg + flash까지 포함)
                        apply_mid(new_mid);

                        // 2) 채널 cfg + flash 반영
                        g_node_cfg.rch[0] = 0;
                        g_node_cfg.rch[1] = new_ch;
                        (void)save_node_cfg_to_flash(&g_node_cfg);

                        // 3) 채널 AT 설정
                        apply_rch(g_node_cfg.rch[0], g_node_cfg.rch[1]);

                        // 4) (선택) ACK 전송
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

                st.light_on = (HAL_GPIO_ReadPin(LIGHT_GPIO_Port, LIGHT_Pin) == GPIO_PIN_SET) ? 1 : 0;
                st.msg_id   = msg_id;

                if (!vi_ok || !temp_ok) {
                    st.ok = 0;
                    st.err_code = -1;
                } else {
                    st.ok = 1;
                    st.err_code = 0;
                }

                schedule_resp_with_slot(
                    RESP_KIND_RAW_BIN,   // ✅ 중요
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
                resp.ch = g_node_cfg.rch[1];   // 저장된 값
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
            	uint8_t result = handle_cmd_set_setting(data, len);  // 0=OK, 그 외 에러

				// 2) AckBin_t로 ACK 패킷 구성
				AckBin_t ack = {0};
				ack.t = 0x10;              // generic ACK 타입 (약속한 값 사용)
				uint8_t uid12[12];
				mid_pack_uid12(uid12);
				memcpy(ack.uid, uid12, 12);

				ack.msg_id   = msg_id;         // 요청의 msg_id 그대로
				ack.ok       = (result == 0) ? 1 : 0;
				ack.err_code = (int8_t)result; // 0이면 OK, 나머지는 에러코드

				// 3) 슬롯 기반 RAW BIN 응답 예약
				schedule_resp_with_slot(
					RESP_KIND_ACK,
					tmid,                 // 응답 TMID (보통 게이트웨이 MID)
					msg_id,
					(uint8_t*)&ack,
					sizeof(ack)
				);

				debug6("[ACK_SLOT] pending ACK scheduled\r\n");

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

                // ✅ Flash/설정(cfg)에서 오는 값
                resp.mid  = g_node_cfg.mid;
                resp.mode = g_node_cfg.mode;

                // 채널은 cfg 우선(정책), 필요하면 node_info 우선으로 바꿔도 됨
                resp.rch0 = g_node_cfg.rch[0];
                resp.rch1 = g_node_cfg.rch[1]; // N/A (cfg가 1채널이라면)

                // ✅ 장치/모듈 정보(info cache)에서 오는 값
                resp.gid  = g_node_info.gid;
                resp.dev  = g_node_info.dev;
                resp.dsp  = g_node_info.dsp;
                resp.txp  = g_node_info.txp;
                memcpy(resp.mac, g_node_info.mac, 8);

                // ✅ FW 버전
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
            // ───────────────────────────────
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
        // 길이 부족 → 무시
    	return 1;
    }

    uint8_t on_off_mode        = data[0];
    uint8_t on_corr_mode       = data[1];
    uint8_t on_corr_time       = data[2];
    uint8_t off_corr_mode      = data[3];
    uint8_t off_corr_time      = data[4];
    uint8_t forced_time        = data[5];
    uint8_t saving_mode        = data[6];
    uint8_t saving_start_hour  = data[7];
    uint8_t saving_start_min   = data[8];
    uint8_t saving_end_hour    = data[9];
    uint8_t saving_end_min     = data[10];
    uint8_t snap_enable        = data[11];
    uint8_t snap_period_min    = data[12];

    // node_cfg에 반영 (필요한 것만 우선)
    g_node_cfg.on_off_mode        = on_off_mode;
    g_node_cfg.on_corr_mode       = on_corr_mode;
    g_node_cfg.on_corr_time_min   = on_corr_time;
    g_node_cfg.off_corr_mode      = off_corr_mode;
    g_node_cfg.off_corr_time_min  = off_corr_time;
    g_node_cfg.forced_time_min    = forced_time;

    g_node_cfg.saving_mode        = saving_mode;
    g_node_cfg.saving_start_hour  = saving_start_hour;
    g_node_cfg.saving_start_min   = saving_start_min;
    g_node_cfg.saving_end_hour    = saving_end_hour;
    g_node_cfg.saving_end_min     = saving_end_min;

    g_node_cfg.snap_enable        = snap_enable ? 1u : 0u;
    g_node_cfg.snap_period_min    = snap_period_min;

    // 스냅 관련 타이머 재설정
    reconfigure_snapshot_timer_from_cfg();

    // 설정 저장 (이미 쓰고 있는 함수 있으면 그거 사용)
    if (!save_node_cfg_to_flash(&g_node_cfg)) {
		result = 2;
	} else {
		result = 0;
	}
	char msg[80];
	    snprintf(msg, sizeof(msg), "[SET_SETTING] result=%d, len=%d\r\n", result, len);
	    debug6(msg);
	return result;
}

static int norm_min(int t)
{
    // 0 ~ 1439 범위로 정규화
    while (t < 0)      t += 1440;
    while (t >= 1440)  t -= 1440;
    return t;
}


void ADC1_Start_Regular_IN18_IT(void)
{
    // CubeMX에서: Scan Disable, NbrOfConversion=1, Rank1=IN18,
    // Continuous Conversion Enable, EOC Selection=EOC 로 설정
    HAL_ADC_Start_IT(&hadc1);   // 한 번만 호출
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;

    /* ✅ 프레임 시작 타임스탬프 */
    if (wr_idx == 0) {
        g_frame_c0 = DWT->CYCCNT;
    }

    uint16_t raw = HAL_ADC_GetValue(hadc);
    raw_buffer[wr_idx]  = raw;
    inputSignal[wr_idx] = ((float)raw * 3.3f / 4095.0f) - 1.65f;

    wr_idx++;

    if (wr_idx >= FFT_SIZE) {
        /* ✅ 프레임 끝 타임스탬프 */
        g_frame_c1 = DWT->CYCCNT;

        wr_idx = 0;
        g_ultra_frame_tick = HAL_GetTick();
        ultra_frame_ready = true;
        ultra_sampling_paused = true;

        (void)HAL_ADC_Stop_IT(&hadc1);
        return;
    }

    if (!ultra_sampling_paused) {
        (void)HAL_ADC_Start_IT(&hadc1);
    }
}

/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;
    g_adc_isr_cnt++;

    uint16_t raw = HAL_ADC_GetValue(hadc);
    float x = ((float)raw * 3.3f / 4095.0f) - 1.65f;
    if (wr_idx == 0) g_frame_t0 = HAL_GetTick();
    g_ai_sample = x;
    g_ai_sample_ready = 1;
    // 1) 캡처 중이 아니면 baseline/noise 갱신 + 트리거 검사
    if (!ultra_sampling_paused) {

            inputSignal[wr_idx++] = x;

            // 디버그(원하면 유지)
            g_last_wr  = wr_idx;

            if (wr_idx >= FFT_SIZE) {
            	g_frame_t1 = HAL_GetTick();
                wr_idx = 0;
                ultra_frame_ready = true;
                ultra_sampling_paused = true;
                g_frame_ready_cnt++;

                // 프레임 다 찼으니 ADC stop (다음 재시작은 Resume에서)
                (void)HAL_ADC_Stop_IT(&hadc1);
                return;
            }
    }
    // 기존 방식 유지: single conversion을 계속 걸어야 한다면
    if (!ultra_sampling_paused) {
        HAL_ADC_Start_IT(&hadc1);
    }
}
*/

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
    	HAL_UART_Transmit(&huart6, (uint8_t*)"Tick\r\n", 6, HAL_MAX_DELAY);
        if (++g_sec_tick >= 10) {   // 1800초 = 30분
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
    // dusk(저녁) ~ 24:00 또는 00:00 ~ dawn(아침) 구간이 야간
    if (dusk <= dawn) {
        // (특이 케이스) 같은 날 안에서 끝나는 경우
        return (now >= dusk) && (now < dawn);
    } else {
        // 일반 케이스: 자정 넘어감
        return (now >= dusk) || (now < dawn);
    }
}

static bool send_wisun_resp(uint16_t tmid, const uint8_t *cbor, size_t cbor_len)
{
    wisun_frame_cfg_t cfg = {
        .sig1 = 0xAA,     // 응답/요청 공통 SIG1
        .sig2 = 0xAB,     // 응답 채널(또는 프로젝트 규칙에 맞게 조정)
        .tmid = tmid      // 요청의 TMID를 그대로 반사하는 게 일반적
    };
    // DATA 길이 255 바이트 제한 내에서 호출해야 합니다.
    return wisun_send_frame(&cfg, cbor, cbor_len, wisun_tx_adapter, NULL);
}

static bool send_wisun_binary(uint16_t tmid, const uint8_t *data, size_t len)
{
    wisun_frame_cfg_t cfg = {
        .sig1 = 0xAA,   // 그대로 유지
        .sig2 = 0xAB,   // 그대로 유지
        .tmid = tmid
    };
    // data 안에 0x00 이 있어도 상관 없음. len 기준으로 전송됨.
    return wisun_send_frame(&cfg, data, len, wisun_tx_adapter, NULL);
}

void dbg_dump_uart6_with_tag(const char *tag, const uint8_t *p, uint16_t n)
{
    if (!p || !n) return;

    // 태그 먼저 출력
    if (tag) {
        HAL_UART_Transmit(&huart6, (uint8_t*)tag, (uint16_t)strlen(tag), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart6, (uint8_t*)" ", 1, HAL_MAX_DELAY);
    }

    // 바이트들을 HEX로 출력
    for (uint16_t i = 0; i < n; ++i) {
        char s[4];
        int m = snprintf(s, sizeof(s), "%02X", p[i]);
        HAL_UART_Transmit(&huart6, (uint8_t*)s, (uint16_t)m, HAL_MAX_DELAY);
        if (i + 1 < n)
            HAL_UART_Transmit(&huart6, (uint8_t*)" ", 1, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

void dbg_print_mid_info(const char *tag, uint16_t my_mid, uint16_t target_mid)
{
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
}

static void send_resp_now_from_task(void)
{
    uint8_t uid12[12];
    mid_pack_uid12(uid12);

    SlotRespBin_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.t = 0x11;                 // slot_resp 타입 (게이트웨이랑 약속)
    memcpy(pkt.uid, uid12, 12);

    pkt.mid   = my_mid;
    pkt.ok    = g_resp.ok ? 1 : 0;
    pkt.has_on = g_resp.p_on_valid;
    pkt.on     = g_resp.p_on ? 1 : 0;
    pkt.nonce  = g_resp.nonce;    // 0이면 “없음” 취급

    // TMID는 브로드캐스트용으로 0x0000 유지
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

void rtc_minute_tick(uint8_t hour, uint8_t min)
{
    uint16_t now = hour * 60u + min;
    static uint8_t light_on = 0;

    switch (g_node_cfg.mode) {
    case 2: // user_time
    {
        uint16_t on  = g_node_cfg.light_on_hour  * 60u + g_node_cfg.light_on_min;
        uint16_t off = g_node_cfg.light_off_hour * 60u + g_node_cfg.light_off_min;

        if (!light_on && now >= on && now < off) {
            HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);
            light_on = 1;
        } else if (light_on && (now >= off || now < on)) {
            HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);
            light_on = 0;
        }
        break;
    }
    case 3: // manual
        // CMD_NODE_CFG 수신 시점에 ON 해두고,
        // g_node_cfg.manual_duration_min 만큼 지난 후 OFF 하는 쪽이 자연스러움
        break;

    default:
        // 0,1 (일출/시민박명)는 나중에 확장용
        break;
    }
}

void Input_Ai_Model(float v)
{
    if (ai_pending) return;  // 추론 대기 중이면 더 안 채움

    // 1) 아직 공간 있으면 채움
    if (ai_index < AE_COLS) {
        ai_input[ai_index] = v;
        ai_index++;
    }

    // 2) 다 찼으면 pending 세팅
    if (ai_index >= AE_COLS) {
        ai_index = AE_COLS;   // clamp
        ai_pending = 1;
    }
}
/*
bool save_node_cfg_to_flash(const node_cfg_t *cfg)
{
    HAL_StatusTypeDef st;
    uint32_t page_error = 0;

    node_cfg_flash_t image;
    image.magic = NODE_CFG_MAGIC;
    image.cfg   = *cfg;
    image.crc   = node_cfg_calc_crc(cfg);

    // 1. FLASH unlock
    HAL_FLASH_Unlock();

    // 2. 해당 페이지 erase
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;   // H5: PAGE erase
    erase.Banks     = NODE_CFG_FLASH_BANK;     // FLASH_BANK_1
    erase.Page      = NODE_CFG_FLASH_PAGE;     // 예시: 255
    erase.NbPages   = 1;

    st = HAL_FLASHEx_Erase(&erase, &page_error);
    if (st != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    // 3. 64비트 단위로 image를 써 넣기
    const uint64_t *src = (const uint64_t *)&image;
    uint32_t addr = NODE_CFG_FLASH_ADDR;
    size_t   n_qwords = (sizeof(image) + 7u) / 8u;   // 8바이트 단위 개수

    for (size_t i = 0; i < n_qwords; ++i) {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]);
        if (st != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        addr += 8u;
    }

    // 4. FLASH lock
    HAL_FLASH_Lock();
    return true;
}
*/

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

static void nodeinfo_cache_update_from_kv(const char *kv_line)
{
    // kv_line은 "DEV=1" 처럼 AT+ 제거된 형태가 nodeinfo_append_kv_line에서 만들어지지만,
    // 여기서는 tmp가 "AT+DEV=1" 일 수도 있으니 둘 다 처리
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
        parse_hex8(p+4, g_node_info.mac);   // 네가 가진 hex parser 쓰면 됨
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

void Print_FFT_Summary(uint16_t *raw_buf)
{
    char msg[96];
    const int step = 16; // ✅ 출력 줄 수 줄이기(테스트 후 조절)

    for (int i = 1; i < FFT_SIZE/2; i += step) {
    	int len = snprintf(msg, sizeof(msg),
    	                           "%.1f, %.5f, %.u\r\n",
    	                           fft_packet[i].freq,
    	                           fft_packet[i].amplitude,
    	                           raw_buf[i]);
		HAL_UART_Transmit(&huart6, (uint8_t*)msg, len, 10);
    }
}

void debug6(const char *s)
{
    HAL_UART_Transmit(&huart6, (uint8_t*)s, strlen(s), 100);
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
    /* DWT/ITM access enable */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Unlock (필요한 코어에서만) */
    #if defined (DWT_LAR)
    DWT->LAR = 0xC5ACCE55;
    #endif

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void reconfigure_snapshot_timer_from_cfg(void)
{
    if (g_node_cfg.snap_enable) {
        g_snap_enable = 1;

        uint8_t min = g_node_cfg.snap_period_min;

        // 0분 또는 이상 값 방지 (1~120분만 허용)
        if (min == 0 || min > 120) {
            min = 1;
            g_node_cfg.snap_period_min = 1;
        }

        g_snap_interval_ms = (uint32_t)min * 60000u;
        Send_Monitoring_Snapshot_JSON(0);
        g_snap_next_tick   = HAL_GetTick() + g_snap_interval_ms;
    } else {
        g_snap_enable = 0;
        // 필요하면 g_snap_next_tick 은 그대로 두거나 0으로 리셋
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
            float a = arr[i].amplitude; // 이미 magnitude라고 가정
            sumsq += a * a;
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

    // 현재 시간 읽기
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    // 날짜를 읽어줘야 shadow register가 갱신됨 (HAL 규칙)
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // 전역 변수에 저장
    g_rtc_hour  = sTime.Hours;
    g_rtc_min   = sTime.Minutes;
    g_rtc_sec   = sTime.Seconds;

    // 날짜는 BCD에서 BIN으로 이미 읽힘
    g_rtc_year  = 2000 + sDate.Year;   // HAL RTC는 Year=25 → 2025
    g_rtc_month = sDate.Month;
    g_rtc_day   = sDate.Date;
}

static void update_sun_times(void)
{
    // RTC가 아직 안 맞았거나 말도 안 되는 값이면 그냥 건너뜀
    if (g_rtc_year < 2020 || g_rtc_year > 2100)
        return;
    if (g_rtc_month == 0 || g_rtc_month > 12)
        return;
    if (g_rtc_day == 0 || g_rtc_day > 31)
        return;

    // 날짜/지역이 이전과 같으면 재계산 안 함
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

    char buf[96];
	int len = snprintf(buf, sizeof(buf),
		"[SUN] %04u-%02u-%02u region=%u SR=%u SS=%u DAWN=%u DUSK=%u\r\n",
		g_rtc_year, g_rtc_month, g_rtc_day, g_region_code,
		g_sunrise_min, g_sunset_min, g_dawn_min, g_dusk_min);
	if (len > 0) {
		HAL_UART_Transmit(&huart6, (uint8_t*)buf, (uint16_t)len, HAL_MAX_DELAY);
	}
}

void wisun_process_rx_mainloop(void)
{
    if (!wisun_packet_ready) return;

    // 0) ISR shadow → local copy (원자적으로)
    uint16_t packet_len = 0;
    uint8_t  buf[PACKET_MAX_SIZE];

    __disable_irq();
    packet_len = wisun_packet_len;
    if (packet_len > 0 && packet_len <= PACKET_MAX_SIZE) {
        memcpy(buf, (void*)wisun_packet_shadow, packet_len);
    }
    wisun_packet_ready = false;
    __enable_irq();

    if (packet_len == 0 || packet_len > PACKET_MAX_SIZE) return;

    // 1) 수신 로그 출력 (이제 main이니까 마음껏)
    PrintReceivedPacket("Receive Packet : ", buf, packet_len);

    // 2) 프레임 파싱
    wisun_frame_view_t v;
    if (!wisun_parse_frame(buf, packet_len, &v)) {
        // 필요하면 켜기
        // dbg_dump_uart6_with_tag("[RX_BAD_FRAME]", buf, packet_len);
        return;
    }

    // 3) 헤더에서 src_mid (프레임 4,5 바이트가 TMID라고 했던 너 코드 유지)
    uint16_t src_mid = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

    // 너가 쓰는 규칙: v.tmid = "target_mid"
    uint16_t target_mid = v.tmid;

    // 4) 최소 transport 헤더(너가 바꾼 포맷 기준: ttl/cmd/flags/msg_id = 5바이트)
    uint8_t  ttl   = 0;
    uint8_t  cmd   = 0;
    uint8_t  flags = 0;
    uint16_t msg_id = 0;

    if (v.data_len >= 5U) {
        ttl    = v.data[0];
        cmd    = v.data[1];
        flags  = v.data[2];
        msg_id = ((uint16_t)v.data[3] << 8) | v.data[4];
    } else {
        // 너무 짧으면 버림
        // dbg_dump_uart6_with_tag("[RX_SHORT]", v.data, v.data_len);
        return;
    }

    // 5) 디버그 출력
    {
        char dbg[96];
        snprintf(dbg, sizeof(dbg),
                 "[MID_DBG] my_mid=0x%04X src_mid=0x%04X tgt_mid=0x%04X ttl=%u cmd=0x%02X\r\n",
                 my_mid, src_mid, target_mid, ttl, cmd);
        HAL_UART_Transmit(&huart6, (uint8_t*)dbg, strlen(dbg), 100);

        dbg_print_mid_info("[MID_CHECK]", my_mid, target_mid);
    }

    // 6) 목적지 판정
    if (target_mid == 0x0000 || target_mid == my_mid) {
        // ───── 나(or 브로드캐스트)용 ─────
        dbg_dump_uart6_with_tag("[RX_FOR_ME]", buf, packet_len);

        // ✅ 여기서 "기존 for-me 처리"를 실행하면 됨.
        // 가장 중요한 포인트: 응답은 src_mid(보낸놈)으로 해야 함.
        //
        // handle_binary_cmd()가 "tmid=응답대상"을 받는 형태라면 src_mid를 넘겨.
        //
        // v.data는 ttl부터 시작하는 구조 그대로이므로 v.data/v.data_len 그대로 넘기면 됨.
        handle_binary_cmd(
            cmd,
            flags,
            msg_id,
            src_mid,       // ✅ 응답 대상은 src_mid (게이트웨이/직전 홉)
            v.data,
            v.data_len
        );

        return;
    }

    // 7) ───── 나는 목적지가 아니면 홉 후보 enqueue ─────
    dbg_dump_uart6_with_tag("[HOP_UNI_ENQ]", buf, packet_len);

    uint32_t now        = HAL_GetTick();
    uint32_t base_delay = 5 + (my_mid % 5);
    uint32_t jitter     = xorshift16(my_mid ^ HAL_GetTick()) % 40;

    bool drop_for_forward = false;
    uint16_t tgt_mid = v.tmid;   // = target_mid
    uint8_t  cmd2 = cmd;
    uint16_t msg_id2 = msg_id;

    // (이미 v.data_len>=5 검사했으니 여기서는 보통 생략 가능)
    // 그래도 안전하게 유지하려면:
    if (v.data_len < 5U) {
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
        uint32_t key = ((uint32_t)tgt_mid << 16) | (uint32_t)msg_id2;
        if (hop_seen_key(key)) {
            dbg_dump_uart6_with_tag("[HOP_DROP_DUP_KEY]", v.data, v.data_len);
            drop_for_forward = true;
        } else {
            hop_mark_key(key);
        }
    }

    if (!drop_for_forward) {
        for (int qi = 0; qi < HOP_QUEUE_SIZE; ++qi) {
            if (!g_hop_q[qi].in_use) {
                if (v.data_len > HOP_MAX_FRAME) {
                    break; // 너무 크면 버림
                }

                g_hop_q[qi].tmid     = v.tmid;     // ★ target_mid 유지
                g_hop_q[qi].data_len = v.data_len;
                memcpy(g_hop_q[qi].data, v.data, v.data_len);

                // TTL 보정
                if (g_hop_q[qi].data_len >= 1U) {
                    uint8_t ttl0 = g_hop_q[qi].data[0];
                    if (ttl0 == 0U) g_hop_q[qi].data[0] = HOP_TTL_DEFAULT;
                }

                g_hop_q[qi].due_tick = now + base_delay + jitter;
                g_hop_q[qi].in_use   = 1;
                break;
            }
        }
    }
}

void Send_Monitoring_Snapshot_JSON(uint16_t req_msg_id)
{
    if (req_msg_id == 0 && !node_is_provisioned()) {
        return;
    }

    bool fft_ok = (ultra_frame_ready && ultra_sampling_paused);

    // 요청 스냅(req_msg_id!=0)은 FFT 준비 필수(원래 정책 유지)
    if (req_msg_id != 0 && !fft_ok) {
        return;
    }

    // 2) UID
    uint8_t uid12[12];
    mid_pack_uid12(uid12);

    // 4) VI 측정
    VIRead vi;
    float vin_v   = 0.0f;
    float i_adc_v = 0.0f;
    if (AD_DC_Injected_Once(&vi) == HAL_OK) {
        vin_v   = (float)vi.volt_raw * (3.3f / 4095.0f);
        i_adc_v = (float)vi.curr_raw * K_ADC2V;
    }

    // 5) 온도 측정
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

    // SnapBin에 넣을 요약 FFT (기본값: FFT 없으면 0개)
    float   fft_freq[SNAP_FFT_PAIRS] = {0};
    float   fft_amp [SNAP_FFT_PAIRS] = {0};
    uint8_t fft_cnt = 0;
    float max_amp = -1.0f;
	float peak_f  = 0.0f;
	uint8_t found = 0;
    float   supersonic_val = 0.0f;

    if (fft_ok) {
        static float local_in[FFT_SIZE];

        __disable_irq();
        ultra_frame_ready = false;
        for (int i = 0; i < FFT_SIZE; ++i) {
            local_in[i] = inputSignal[i];
        }
        __enable_irq();

        // FFT 입력
        for (int i = 0; i < FFT_SIZE; ++i) {
            inputSignal[i] = local_in[i];
        }

        arm_rfft_fast_f32(&fftInstance, inputSignal, outputSignal, 0);


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
        for (uint16_t b = 1; b < nbins && fft_cnt < SNAP_FFT_PAIRS; ++b) {
            float freq = (float)b * (float)FSAMPLE / (float)FFT_SIZE;
            float amp  = fft_packet[b].amplitude;
            if (!isfinite(amp)) amp = 0.0f;

            if (freq >= 80000.0f && freq <= 130000.0f) {
                    if (amp > max_amp) {
                        max_amp = amp;
                        peak_f  = freq;
                        found = 1;
				}
			}

            //fft_freq[fft_cnt] = freq;
            //fft_amp [fft_cnt] = amp;
            //fft_cnt++;
        }

        fft_cnt = 0;
        if (found) {
            fft_freq[0] = peak_f;
            fft_amp [0] = max_amp;
            fft_cnt = 1;
        }
        Ultra_ResumeNextFrame();
    } else {
        if (req_msg_id == 0 && ultra_sampling_paused) {
            Ultra_ResumeNextFrame();
        }
        char* msg = "Target band (80-130kHz) not found.\r\n";
		HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 100);
    }

    // 7) 스냅샷 히스토리 저장
    bool light_on = (HAL_GPIO_ReadPin(LIGHT_GPIO_Port, LIGHT_Pin) == GPIO_PIN_SET);
    push_snapshot(light_on, vin_v, i_adc_v, temp_c, supersonic_val);

    g_monitor_count++;

    // 9) SnapBin 구성
    SnapBin_t snap;
    memset(&snap, 0, sizeof(snap));

    snap.t = 0x01;
    memcpy(snap.uid, uid12, 12);

    snap.volt = vin_v;
    snap.curr = i_adc_v;
    snap.temp = temp_c;

    snap.fft_count = fft_cnt;
    for (uint8_t k = 0; k < fft_cnt && k < SNAP_FFT_PAIRS; ++k) {
        snap.fft[k].freq = fft_freq[k];
        snap.fft[k].amp  = fft_amp[k];
    }

    snap.msg_id   = (uint32_t)req_msg_id;
    snap.ok       = 1;
    snap.err_code = 0;

    (void)send_wisun_binary(0x0000, (uint8_t*)&snap, sizeof(SnapBin_t));

    g_last_has_msg_id = 0;
    g_last_has_cmd    = 0;
}

float ntc_voltage_to_temp_c(float temp_v)
{

    if (temp_v >= ntc_voltage_table[0]) {
        return ntc_temp_table[0];              // -40℃ 이하
    }
    if (temp_v <= ntc_voltage_table[NTC_LUT_SIZE - 1]) {
        return ntc_temp_table[NTC_LUT_SIZE - 1]; // 80℃ 이상
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

    // seed: 노드마다 & 요청마다 조금씩 다르게
    uint16_t seed = my_mid ^ msg_id;
    if (seed == 0) seed = my_mid ^ 0xACE1u;

    uint8_t slot_idx = (uint8_t)(my_mid % SLOT_COUNT);
    uint16_t r       = xorshift16(seed);
    uint32_t jitter  = (JITTER_MS > 0) ? (r % JITTER_MS) : 0;

    uint32_t delay_ms =
        BASE_DELAY_MS +
        (uint32_t)slot_idx * SLOT_LEN_MS +
        jitter;
    uint16_t cap = (uint16_t)sizeof(g_resp_slot.buf);

	// IRQ 구간에서는 최소 작업만
	uint16_t saved_len = 0;
	uint8_t  saved_ok  = 0;
    __disable_irq();
    g_resp_slot.pending = 1;
    g_resp_slot.kind    = kind;
    g_resp_slot.tmid    = tmid;
    g_resp_slot.msg_id  = msg_id;
    g_resp_slot.due_tick= now + delay_ms;

    if (raw && raw_len > 0 && raw_len <= sizeof(g_resp_slot.buf)) {
        memcpy(g_resp_slot.buf, raw, raw_len);
        g_resp_slot.len        = raw_len;
        g_resp_slot.has_raw_buf= 1;
    } else {
        g_resp_slot.has_raw_buf= 0;
        g_resp_slot.len        = 0;
    }
    __enable_irq();

    char msg[128];
    int n = snprintf(msg, sizeof(msg),
        "[SLOT] kind=%u raw=%p raw_len=%u cap=%u ok=%u saved_len=%u tmid=%u msg_id=%u\r\n",
        (unsigned)kind, raw, (unsigned)raw_len, (unsigned)cap,
        (unsigned)saved_ok, (unsigned)saved_len, (unsigned)tmid, (unsigned)msg_id);
	if (n > 0) {
		debug6(msg);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_ADC2_Init();
  MX_X_CUBE_AI_Init();
  DWT_CYCCNT_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rxByte1, 1);
  HAL_UART_Receive_IT(&huart6, &rxByte, 1);

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_ADC_Start_IT(&hadc1);
  Ultra_StartSampling();

  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
   {
     Error_Handler();
   }
  //HAL_ADC_Start_IT(&hadc2);
  /*if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, FFT_SIZE) != HAL_OK)
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
        char msg[] = "FFT 초기화 실패\r\n";
        HAL_UART_Transmit(&huart6, (uint8_t*)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
    }
    HAL_TIM_Base_Start(&htim2);   // now_us()용 타이머 시작
    init_uid_string();
    rtc_ensure_valid_or_set_default();
    uint16_t stored_mid = MID_INVALID;
    bool mid_loaded = load_mid_from_flash(&stored_mid);
    if (mid_loaded && stored_mid != MID_INVALID && stored_mid != 0) {
        // 플래시에 유효 MID 있음 → 그대로 사용
        my_mid = stored_mid;
        g_node_cfg.mid          = my_mid;
        g_node_cfg.mid_assigned = 1;
    } else {
        // 플래시에 유효 MID 없음 → Wi-SUN 모듈에 물어봄
        Query_MID_From_WiSUN();
    }
    if (!load_node_cfg_from_flash(&g_node_cfg)) {
           // 플래시에 유효한 설정이 없으면 기본값 사용
           node_cfg_init_default(&g_node_cfg);
           (void)save_node_cfg_to_flash(&g_node_cfg);
       }

    apply_mid_chan_from_cfg();
    reconfigure_snapshot_timer_from_cfg();
    printf("[MIDCH_BOOT] mid=0x%04X ch=%u,%u assigned=%u (src=%s)\r\n",
	   my_mid,  g_node_cfg.rch[0], g_node_cfg.rch[1], g_node_cfg.mid_assigned,
	   (mid_loaded && stored_mid != MID_INVALID && stored_mid != 0) ? "flash" : "wisun");

    //static uint32_t ai_last_infer_tick = 0;
    //static int      g_ai_last_result   = 0;
    light_on();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        boot_poll();
        /*hop_tx_task_poll();
        resp_slot_task_poll();
        scheduler_poll();
        wisun_process_rx_mainloop();*/

        /* Debug FFT */

        /*if (g_adc_kick && !ultra_sampling_paused) {
            g_adc_kick = 0;
            (void)HAL_ADC_Start_IT(&hadc1);
        }*/

        if (ultra_sampling_paused) {
            Debug_Print_FFT_Peak(); // ready 여부는 함수가 판단
        }


        /* ===================== Wi-SUN RX ===================== */
        /*if (wisun_packet_ready)
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

                 최소: target_mid(2) + ttl(1) + cmd(1) + flags(1) + msg_id(2) = 7B
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

         ===================== AT RX =====================
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

         ===================== Node Info =====================
        nodeinfo_poll(now);

        if (g_snap_enable && node_is_provisioned())
        {
            now = HAL_GetTick();
            if ((int32_t)(now - g_snap_next_tick) >= 0)
            {
                Send_Monitoring_Snapshot_JSON(0);
                g_snap_next_tick = now + g_snap_interval_ms;
            }
        }

         ===================== RTC / SUN =====================
        rtc_update();
        update_sun_times();

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
        }*/

        /* ===================== AI ===================== */
        //static uint8_t forced = 0;
       /* if (!forced && HAL_GetTick() > 3000)
        {
            forced = 1;
            ai_pending = 1;
            ai_index = AE_COLS;
            uart6_log("[AI] forced pending=1\r\n");
        }*/

        /*static uint32_t hb_t = 0;
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
        if (now - hb_t >= 1000) {
          hb_t = now;
          uart6_log("[HB] t=%lu ai_idx=%u pending=%u wisun=%u at=%u\r\n", (unsigned long)now, (unsigned)ai_index, (unsigned)ai_pending,
          // ai_pending을 쓰는 경우
          (unsigned)wisun_packet_ready, (unsigned)g_at_line_ready);
         }
        if (ai_pending) {
            ai_service();
        }*/
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
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
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

    // (예) 최소 주기 제한 / 바쁠 때 미루기
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
        return;   // ⭐ 여기서 반드시 탈출
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

        // ❗ 에러일 때도 다음 샘플링이 다시 시작되도록 리셋
        ai_pending = 0;
        ai_index = 0;
        ai_next_run = now + ai_period_ms;
        return;
    }

    // ✅ 성공 케이스 처리 (이게 없어서 pending이 계속 1이었던 것)
    {
        char ok[96];
        int n = snprintf(ok, sizeof(ok),
                         "AI OK dt=%lu ms mse=%.6f pred=%d\r\n",
                         (unsigned long)dt, (double)ai_mse, ai_pred);
        HAL_UART_Transmit(&huart6, (uint8_t*)ok, n, 50);

    }

    ai_pending = 0;                 // 다음 샘플 채우기 허용
    ai_index = 0;
    uart6_log("[AI] done idx=%u pending=%u\r\n",
                      (unsigned)ai_index, (unsigned)ai_pending);
    ai_next_run = now + ai_period_ms; // 주기 제한
}

static void push_snapshot(bool light_on,
                          float voltage,
                          float current,
                          float temp,
                          float supersonic)
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
    if (!g_resp_slot.pending)
        return;

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - g_resp_slot.due_tick) < 0)
        return;

    // 한 번만 보낸다
    g_resp_slot.pending = 0;

    if (g_resp_slot.has_raw_buf) {
        // 이미 완성된 버퍼를 그대로 전송
    	char msg[128];
    	int n = snprintf(msg, sizeof(msg),
    	                 "[RAW_TX] kind=%d tmid=%04X len=%u first=%02X\r\n",
    	                 (int)g_resp_slot.kind,
    	                 g_resp_slot.tmid,
    	                 g_resp_slot.len,
    	                 g_resp_slot.buf[0]);

    	HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 100);
        (void)send_wisun_binary(g_resp_slot.tmid,
                                g_resp_slot.buf,
                                g_resp_slot.len);
        return;
    }

    // 여기서 kind별로 실제 응답 패킷을 만들어서 전송
    switch (g_resp_slot.kind) {
    case RESP_KIND_SNAP:
        Send_Monitoring_Snapshot_JSON(g_resp_slot.msg_id);
        break;

    case RESP_KIND_ACK:
    	uint8_t payload[8];
		uint8_t idx = 0;

		uint8_t  cmd    = g_resp_slot.cmd;     // 예: CMD_SET_SETTING (0x31)
		uint8_t  flags  = 0x80;                // bit7=1 → response
		uint16_t msg_id = g_resp_slot.msg_id;  // 요청에서 받은 msg_id
		uint8_t  result = g_resp_slot.result;  // 0=OK, 그 외 에러코드

		payload[idx++] = cmd;
		payload[idx++] = flags;
		payload[idx++] = (uint8_t)(msg_id >> 8);
		payload[idx++] = (uint8_t)(msg_id & 0xFF);
		payload[idx++] = result;

		// tmid = 요청 보낸 쪽 MID (주로 게이트웨이 MID)
		(void)send_wisun_binary(g_resp_slot.tmid, payload, idx);

		debug6("[ACK_TX_DONE]\r\n");

        break;

    case RESP_KIND_RAW_BIN:

        break;

    default:
        break;
    }
}

bool rtc_ensure_valid_or_set_default(void)
{
    RTC_TimeTypeDef t = {0};
    RTC_DateTypeDef d = {0};

    // HAL 규칙: time → date 순으로 읽어야 shadow 갱신됨
    if (HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN) != HAL_OK) return false;
    if (HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN) != HAL_OK) return false;

    // 유효성 판단 (예: 2024년 이전이면 "미설정"으로 간주)
    // HAL RTC는 Year가 0~99 (2000~2099로 가정)
    if (d.Year >= 24 && d.Month >= 1 && d.Month <= 12 && d.Date >= 1 && d.Date <= 31) {
        return false; // 이미 유효
    }

    // 기본값(테스트용): 2025-12-22 12:34:00
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours   = 12;
    sTime.Minutes = 34;
    sTime.Seconds = 0;

    sDate.Year    = 25;                 // 2025
    sDate.Month   = RTC_MONTH_DECEMBER; // 12
    sDate.Date    = 22;
    sDate.WeekDay = RTC_WEEKDAY_MONDAY; // 실제 요일과 맞추는 게 좋음(테스트면 큰 영향은 적음)

    // 주의: Date 먼저/Time 먼저는 STM32 시리즈에 따라 차이 이슈가 있어
    // 일반적으로 HAL 예제는 Time->Date 순을 많이 쓰지만, 이 방식도 동작함.
    // 문제 생기면 SetDate→SetTime 순으로 바꿔보면 됨.
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) return false;
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) return false;

    printf("[RTC] default time set: 2025-12-22 12:34:00\r\n");
    return true;
}

void Vac_Ctrl(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);

}

static void Read_UID_local(void){
    Read_UID(); // 기존 함수로 UID를 uid_ram[]에 읽음
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
    Read_UID();  // Unique ID ?   ??
    snprintf(msg, size, "%08" PRIX32 "-%08" PRIX32 "-%08" PRIX32 "\r\n",
             uid_ram[2], uid_ram[1], uid_ram[0]);
}

uint32_t Get_Device_ID(void) {
           return DBGMCU->IDCODE;  // Device ID ?   ??
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

    // 1) Wi-SUN 모듈에 설정 (AT)
    char cmd_buf[32];
    int n = snprintf(cmd_buf, sizeof(cmd_buf), "AT+MID=%u\r\n", new_mid);
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd_buf, (uint16_t)n, HAL_MAX_DELAY);

    // 2) 로컬 변수 업데이트
    my_mid = new_mid;

    // 3) legacy MID 저장
    save_mid_to_flash(my_mid);

    // ★ 4) node_cfg에도 반영 + 플래시 저장
    g_node_cfg.mid          = new_mid;
    g_node_cfg.mid_assigned = 1;
    (void)save_node_cfg_to_flash(&g_node_cfg);

    // 5) 디버깅용 출력
    dbg_print_mid_info("[SET_MID]", my_mid, 0);
}

static void apply_rch(uint8_t r0, uint8_t r1)
{
    char cmd_buf[32];
    int n = snprintf(cmd_buf, sizeof(cmd_buf), "AT+RCH=%u,%u\r\n",
                     (unsigned)r0, (unsigned)r1);
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd_buf, (uint16_t)n, HAL_MAX_DELAY);

    char msg[48];
    int m = snprintf(msg, sizeof(msg), "[SET_RCH] rch=%u,%u\r\n",
                     (unsigned)r0, (unsigned)r1);
    HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)m, HAL_MAX_DELAY);
}

static void apply_mid_chan_from_cfg(void)
{
    // 1) MID 쪽
    if (g_node_cfg.mid_assigned &&
        g_node_cfg.mid != 0 &&
        g_node_cfg.mid != MID_INVALID)
    {
        my_mid = g_node_cfg.mid;

        // 기존 MID flash에도 맞춰주고
        save_mid_to_flash(my_mid);

        // Wi-SUN 모듈에도 다시 적용 (보드 교체 등 대비)
        apply_mid(my_mid);
    }
    else {
        // 예전 방식 플래시 MID가 있으면 fallback
        uint16_t stored_mid = MID_INVALID;
        if (load_mid_from_flash(&stored_mid) &&
            stored_mid != MID_INVALID &&
            stored_mid != 0)
        {
            my_mid = stored_mid;
        }
    }

    // 2) 채널 쪽 (0도 유효 채널이라면 그대로 사용)
    apply_rch(g_node_cfg.rch[0], g_node_cfg.rch[1]);
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

	    // === 전류 입력 채널 설정: PA6 (ADC_CHANNEL_3) ===
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

	    // === 변환: ADC → 전압 → 전류 변환 (옵션) ===
	    float voltage = Convert_Voltage_ADC(adc_val_current);  // 예: 3.3 * adc / 4095
	    float current = Convert_Voltage_To_Current(voltage, 2.50f);  // 기준 전압은 상황에 따라 조정

	    // === 출력 ===
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
            len = sizeof(buf);   // 혹시나 오버 나오면 방어
        }
        HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
    }
}

void PrintReceivedPacket(const char* prefix, const uint8_t* data, uint16_t length) {
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
}


uint8_t calc_checksum(uint8_t *buf, uint16_t len) {
	uint8_t sum = 0;
	for (uint16_t i = 0; i < len; i++) sum += buf[i];
	return sum;
}

void startADCInterrupt(void) {
    adc_index = 0;
    adc_done = 0;
    HAL_ADC_Start_IT(&hadc1);
}

static void InitHannWindowOnce(void)
{
    if (g_hann_inited) return;
    for (int i = 0; i < FFT_SIZE; i++) {
        g_hann[i] = 0.5f - 0.5f * arm_cos_f32(2.0f * 3.14159265f * (float)i / (float)(FFT_SIZE - 1));
    }
    g_hann_inited = 1;
}

void ExtractFullFFT(const float32_t *in, FftData_t *dest) {
	InitHannWindowOnce();

	// 1) local 작업 버퍼
	static float32_t xw[FFT_SIZE];

	// 2) DC 제거용 평균
	float32_t mean = 0.0f;
	for (int i = 0; i < FFT_SIZE; i++) mean += in[i];
	mean /= (float32_t)FFT_SIZE;

	// 3) DC 제거 + 윈도우 적용
	for (int i = 0; i < FFT_SIZE; i++) {
		xw[i] = (in[i] - mean) * g_hann[i];
	}

	// 4) FFT 수행 (너의 processFFT가 xw를 입력으로 받도록)
	processFFT(xw, outputSignal, magnitude);

	// 5) 결과 저장 (0~Nyquist: FFT_SIZE/2)
	for (int i = 0; i < FFT_SIZE / 2; i++) {
		float freq = ((float)i * (float)FSAMPLE) / (float)FFT_SIZE;
		dest[i].freq = freq;            // ⚠️ FftData_t.freq는 float이어야 함
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

    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);

    // 현재 시각을 "자정 기준 분 단위"로 변환
    uint16_t now_min = (uint16_t)t.Hours * 60u + t.Minutes;

    uint16_t on_min  = (uint16_t)g_node_cfg.light_on_hour  * 60u + g_node_cfg.light_on_min;
    uint16_t off_min = (uint16_t)g_node_cfg.light_off_hour * 60u + g_node_cfg.light_off_min;

    // 날짜가 바뀌면 하루 플래그를 리셋
    if (sched_last_day != d.Date) {
        sched_last_day = d.Date;
        on_done_today  = false;
        off_done_today = false;
    }

    // ON 스케줄
    /*if (!on_done_today && now_min >= on_min) {
        // 실제 조명 ON 제어
        HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);
        on_done_today = true;
    }

    // OFF 스케줄
    if (!off_done_today && now_min >= off_min) {
        // 실제 조명 OFF 제어
        HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);
        off_done_today = true;
    }*/
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

    // ✅ MID?를 보낸 경우에만 MID 라인 처리
    if (!g_wait_mid_query) return;

    // ✅ 라인 시작이 AT+MID= 인 경우만
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
        return; // 프레임이 이상하면 그냥 버림
    }

    wisun_frame_cfg_t cfg = {
        .sig1 = 0xAA,
        .sig2 = 0xAB,   // 네가 원하는 TX용 SIG 규칙
        .tmid = tmid,
    };

    // v.data / v.data_len 은 레거시든 새 포맷이든 알아서 맞게 설정됨
    wisun_send_frame(&cfg, v.data, v.data_len, wisun_tx_adapter, NULL);
}

void Ultra_StartSampling(void) {
    wr_idx = 0;
    ultra_frame_ready = false;
    ultra_sampling_paused = false;
    HAL_ADC_Start_IT(&hadc1);   // IN18 연속변환 시작(Continuous=Enable 가정)
}

void Ultra_ResumeNextFrame(void) {
    wr_idx = 0;
    ultra_frame_ready = false;
    ultra_sampling_paused = false;
    HAL_ADC_Start_IT(&hadc1);
}

static int nodeinfo_append_kv_line(const char *line)
{
    // 1) OK / ERROR 라인 제외
    if (line[0] == 'O' && line[1] == 'K') return 0;
    if (line[0] == 'E' && line[1] == 'R') return 0; // "ERROR..."도 일단 제외(원하면 실패 처리로 바꿔도 됨)

    // 2) "AT+" 제거
    const char *p = line;
    if (p[0] == 'A' && p[1] == 'T' && p[2] == '+') p += 3;

    // 3) \r \n 제거하면서 복사 (KEY=VALUE만 남기기)
    //    빈 줄이면 무시
    uint16_t tmp_len = 0;
    char tmp[96]; // 한 줄 최대 길이(여유 있게)
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

	    // tmp 예: "RCH=0,20" 또는 "RCH=20" (둘 다 방어)
	    const char *v = tmp + 4;

	    int a = -1, b = -1;
	    if (sscanf(v, " %d , %d", &a, &b) == 2) {
	        // clamp
	        if (a < 0) a = 0; if (a > 255) a = 255;
	        if (b < 0) b = 0; if (b > 255) b = 255;

	        g_node_cfg.rch[0] = (uint8_t)a;
	        g_node_cfg.rch[1] = (uint8_t)b;
	    } else if (sscanf(v, " %d", &b) == 1) {
	        // 만약 모듈이 "RCH=20"처럼 1개만 주는 경우
	        if (b < 0) b = 0; if (b > 255) b = 255;
	        g_node_cfg.rch[0] = 0;
	        g_node_cfg.rch[1] = (uint8_t)b;
	    }

	    // (선택) flash 저장: 매 부팅마다 쓰기 싫으면 "값 변경 시"로 조건 걸기 추천
	    (void)save_node_cfg_to_flash(&g_node_cfg);

	    // 디버그
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
    // 4) 버퍼에 "tmp + '\n'" 형태로 누적
    //    (세미콜론 구분 원하면 '\n' 대신 ';'로 바꿔도 됨)
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
    g_nodeinfo.pending = 1;
    g_nodeinfo.tmid    = tmid;
    g_nodeinfo.msg_id  = msg_id;
    g_nodeinfo.used    = 0;
    g_nodeinfo.got_mask = 0;
    g_nodeinfo.text[0] = '\0';

    uint32_t now = HAL_GetTick();
    g_nodeinfo.last_rx_tick   = now;
    g_nodeinfo.deadline_tick  = now + 3000;   // 3초 정도로 넉넉히

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
    // ✅ FWVER 라인 들어오면 여기서 “완료”로 끝내기
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

        // 디버그: 실제 길이 확인
        char m[64];
        int n = snprintf(m, sizeof(m), "[NI_SEND] used=%u strnlen=%u\r\n",
                         (unsigned)g_nodeinfo.used, (unsigned)text_len);
        HAL_UART_Transmit(&huart6, (uint8_t*)m, (uint16_t)n, 100);

        schedule_resp_with_slot(
            RESP_KIND_RAW_BIN,
            g_nodeinfo.tmid,
            g_nodeinfo.msg_id,
            (uint8_t*)g_nodeinfo.text,
            text_len
        );
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

    /* 상태 로그 (매 호출 출력) */
    /*{
        char b[180];
        int n = snprintf(b, sizeof(b),
            "[DBG] isr=%lu trig=%lu frame=%lu dev=%.4f thr=%.4f noise=%.4f paused=%d ready=%d\r\n",
            (unsigned long)g_adc_isr_cnt,
            (unsigned long)g_trig_fire_cnt,
            (unsigned long)g_frame_ready_cnt,
            (double)g_last_dev, (double)g_last_thr, (double)g_last_noise,
            ultra_sampling_paused ? 1 : 0,
            ultra_frame_ready ? 1 : 0
        );

        HAL_StatusTypeDef st = HAL_UART_Transmit(&huart6, (uint8_t*)b, (uint16_t)n, 20);
        if (st != HAL_OK) uart_fail_cnt++;
    }*/

    /* 프레임 준비되었고 샘플링 멈춘 상태면 FFT 수행 */
    if (ultra_frame_ready && ultra_sampling_paused) {

        /* ✅ Hann 테이블 보장 초기화 */
        InitHannWindowOnce();

        uint32_t my_seq = ++fft_seq;

        /* ---- 전압/전류 ---- */
        float vin_v = 0.0f;
        float i_adc_v = 0.0f;
        {
            VIRead vi;
            if (AD_DC_Injected_Once(&vi) == HAL_OK) {
                vin_v   = (float)vi.volt_raw * (3.3f / 4095.0f);
                i_adc_v = (float)vi.curr_raw * K_ADC2V;
            }
        }

        /* ---- 입력 안전 복사 ---- */
        static float32_t x[FFT_SIZE];
        __disable_irq();
        for (int i = 0; i < FFT_SIZE; i++) {
            x[i] = inputSignal[i];
        }
        ultra_frame_ready = false;
        __enable_irq();

        uint32_t c0 = g_frame_c0;
        uint32_t c1 = g_frame_c1;
        uint32_t dc = (c1 >= c0) ? (c1 - c0) : (0xFFFFFFFFu - c0 + c1 + 1u);

        float dt_s  = (float)dc / (float)SystemCoreClock;
        float dt_ms = dt_s * 1000.0f;
        float fs_eff = (dt_s > 1e-9f) ? ((float)FFT_SIZE / dt_s) : 0.0f;

        // ---- Zero-crossing 주파수 추정 ----
        /*int zc = 0;
        for (int i = 1; i < FFT_SIZE; i++) {
            float a = x[i-1];
            float b = x[i];
            if ((a <= 0.0f && b > 0.0f) || (a >= 0.0f && b < 0.0f)) zc++;
        }
        float f_zc = 0.0f;
        if (fs_eff > 1.0f) {
            // crossing 2회 = 1주기 가정
            f_zc = ((float)zc * 0.5f) * (fs_eff / (float)FFT_SIZE);
        }

        char b[160];
        int n = snprintf(b, sizeof(b),
            "[RAWCHK] dt=%.2fms fs_eff=%.1fHz zc=%d f_zc=%.1fkHz\r\n",
            (double)dt_ms, (double)fs_eff, zc, (double)(f_zc/1000.0f));
        HAL_UART_Transmit(&huart6, (uint8_t*)b, (uint16_t)n, 20);*/


        /* ---- 1초에 1번 입력 상태 체크 ---- */
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

        /* ---- FFT -> magnitude (✅ ExtractFullFFT 적용) ---- */
        static float32_t mag[FFT_SIZE/2];
        ExtractFullFFT_MagOnly(x, mag);

        /* ---- 대역 (원하는 값으로) ---- */
        const float lo_hz = 80000.0f;
        const float hi_hz = 115000.0f;

        float fs  = (fs_eff > 1.0f) ? fs_eff : (float)FSAMPLE;
        float nyq = 0.5f * fs;

        float lo = lo_hz;
        float hi = hi_hz;

        /* nyquist 넘어가면 잘라내기 + 끝 bin 가드 */
        const float guard = 3000.0f;          // 3kHz (원하면 5k)
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
            Ultra_ResumeNextFrame();
            return;
        }

        /* ---- 대역 스캔 + noise_floor(trimmed mean) ---- */
        float max_amp = 0.0f;
        float peak_f  = 0.0f;

        float sum = 0.0f;
        int   cnt = 0;
        float top1 = 0.0f, top2 = 0.0f, top3 = 0.0f;
        int peak_i = -1;
        for (int i = 1; i < (FFT_SIZE/2); i++) {
        	float freq = ((float)i * fs) / (float)FFT_SIZE;
        	if (freq < lo || freq > hi) continue;

            float a = mag[i];

            if (a > max_amp) {
                max_amp = a;
                peak_f  = freq;
                peak_i  = i;
            }

            sum += a;
            cnt++;

            if (a > top1) { top3 = top2; top2 = top1; top1 = a; }
            else if (a > top2) { top3 = top2; top2 = a; }
            else if (a > top3) { top3 = a; }
        }

        if (peak_i >= 2 && peak_i <= (FFT_SIZE/2 - 2)) {
            float a0 = mag[peak_i - 1];
            float a1 = mag[peak_i];
            float a2 = mag[peak_i + 1];

            /* 에너지 합산(루트-제곱합) */
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

        /* FFT magnitude -> 전압 진폭(기본파) */
        float vpk  = (2.0f / (N * CG)) * max_amp;   // ≈ (4/N)*max_amp
        float mvpk = vpk * 1000.0f;

        float vpp  = 2.0f * vpk;
        float mvpp = vpp * 1000.0f;

        float vrms = vpk * 0.70710678f;         // = vpk/sqrt(2)

        /* ---- (옵션) ADC counts 환산 (FFT 기반) ---- */
        float adc_pk  = vpk * (4095.0f / 3.3f);
        float adc_pp  = vpp * (4095.0f / 3.3f);

        float snr = (noise_floor > 0.0f) ? (max_amp / noise_floor) : 0.0f;
        bool found = (cnt > 0) && (snr > FFT_SNR_K);
        //bool found = (cnt > 0) && (max_amp > (noise_floor * FFT_SNR_K));

        /* ---- 결과 출력(예전 스타일) ---- */
        {
            char buf[220];
            int len;

            if (found) {
            	len = snprintf(buf, sizeof(buf),
            	  "[FFT#%lu] found=%d peak=%.2fkHz "
            	  "vpk=%.4fV mvpk=%.1f "
            	  "vpp=%.4fV mvpp=%.1f "
            	  "vrms=%.4fV "
            	  "adc_pk=%.1f adc_pp=%.1f "
            	  "dt=%.2fms fs_eff=%.1fk "
				  "fs=%.1fk nyq=%.1fk lo=%.1fk hi=%.1fk snr=%.2f "
				  "vin=%.2fV i=%.2fA\r\n",
            	  (unsigned long)my_seq,
            	  found ? 1 : 0,
            	  (double)(peak_f/1000.0f),
            	  (double)vpk, (double)mvpk,
            	  (double)vpp, (double)mvpp,
            	  (double)vrms,
            	  (double)adc_pk, (double)adc_pp,
				  (double)dt_ms,  (double)(fs_eff/1000.0f),
				  (double)(fs/1000.0f),
				  (double)(nyq/1000.0f),
			      (double)(lo/1000.0f),
			      (double)(hi/1000.0f),
			      (double)snr,
            	  (double)vin_v, (double)i_adc_v
            	);
            } else {
            	len = snprintf(buf, sizeof(buf),
            	    "[FFT#%lu] found=0 peak=%.2fkHz "
            	    "max=%.3g nf=%.3g snr=%.2f cnt=%d "
            	    "dt=%.2fms fs_eff=%.1fk fs=%.1fk nyq=%.1fk "
            	    "lo=%.1fk hi=%.1fk "
            	    "vin=%.2fV i=%.2fA\r\n",
            	    (unsigned long)my_seq,
            	    (double)(peak_f/1000.0f),
            	    (double)max_amp,
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

            HAL_StatusTypeDef st = HAL_UART_Transmit(&huart6, (uint8_t*)buf, (uint16_t)len, 20);
            if (st != HAL_OK) uart_fail_cnt++;
        }

        /* ✅ 네가 원래 쓰던 구조 그대로 */
        Ultra_ResumeNextFrame();
    }
    else if (!ultra_frame_ready && ultra_sampling_paused) {
        /* 멈췄는데 frame_ready가 아닌 이상 상태면 그냥 재개 */
        Ultra_ResumeNextFrame();
    }
}




/* void Process_Ultra_Frame_Then_VI(void)
{
    if (!ultra_frame_ready || !ultra_sampling_paused) return;
    static uint16_t local_raw[FFT_SIZE];
    static float    local_in [FFT_SIZE];
    __disable_irq();
    ultra_frame_ready = false;
    for (int i=0; i<FFT_SIZE; ++i) { local_raw[i] = raw_buffer[i]; local_in[i] = inputSignal[i]; }
    __enable_irq();

    VIRead vi;
    if (AD_DC_Injected_Once(&vi) == HAL_OK) {
        // --- 변환: 가장 단순 버전 ---
        float vin_v     = (float)vi.volt_raw * (3.3f / 4095.0f);
        float i_adc = (float)vi.curr_raw * K_ADC2V;

        // 간단 출력 (짧은 포맷)
        char m[96];
        int n = snprintf(m, sizeof(m),
                         "VI: %u,%u => %.3f V, %.3f A @ %luus\r\n",
                         vi.volt_raw, vi.curr_raw, vin_v, i_adc, vi.t_us);
        HAL_UART_Transmit(&huart6, (uint8_t*)m, n, 20);
    } else {
        char m[] = "VI read failed\r\n";
        HAL_UART_Transmit(&huart6, (uint8_t*)m, sizeof(m)-1, 10);
    }

    ExtractFullFFT(fft_packet);
    Print_FFT_Summary(local_raw);
    Ultra_ResumeNextFrame();
} */

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
