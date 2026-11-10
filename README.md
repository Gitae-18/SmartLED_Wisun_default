# Smart Street Light Project

스마트가로등 노드와 게이트웨이를 이용하여 **조명 제어, 센서 데이터 수집, Wi-SUN 통신, MQTT 서버 연동, 이상 상태 감지**를 수행하는 프로젝트입니다.

본 프로젝트는 크게 다음 두 영역으로 구성됩니다.

- **STM32H5 기반 스마트가로등 Node**
- **Raspberry Pi CM4 기반 Wi-SUN Gateway**

---

## 1. 프로젝트 개요

스마트가로등 단말은 조명의 상태와 센서 데이터를 수집하고, Wi-SUN 네트워크를 통해 게이트웨이로 전송합니다.

게이트웨이는 단말 데이터를 수신하여 MQTT를 통해 서버/관제 시스템으로 전달하고, 서버에서 수신한 제어 명령을 다시 단말로 전달합니다.

주요 기능은 다음과 같습니다.

- 조명 ON / OFF 제어
- 시간 기반 자동 점·소등
- 일출·일몰 시간 기반 조명 제어
- 조명 상태 이벤트 전송
- 전압 / 전류 / 온도 측정
- 초음파 센서 FFT 분석
- SMPS 이상 상태 감지
- Wi-SUN Mesh 통신
- MQTT 서버 연동
- 단말 설정 변경
- 통신 주기 변경
- 단말 상태 조회
- 연결 끊김 감지
- 설정 명령 ACK 처리
- 노드 정보 및 채널 조회

---

## 2. 시스템 구성

```text
+----------------------+
|      Monitoring      |
|      / Server        |
+----------+-----------+
           |
           | MQTT
           |
+----------v-----------+
| Raspberry Pi CM4     |
| Wi-SUN Gateway       |
|                      |
| - MQTT Client        |
| - Command Router     |
| - Node Store         |
| - Connection Watchdog|
+----------+-----------+
           |
           | Wi-SUN
           |
+----------v-----------+
| STM32H5 Node         |
|                      |
| - Light Control      |
| - RTC / Scheduler    |
| - ADC Sensor         |
| - FFT Analysis       |
| - Wi-SUN Protocol    |
+----------+-----------+
           |
           +---- SMPS / LED Light
           +---- Voltage Sensor
           +---- Current Sensor
           +---- NTC Temperature
           +---- Ultrasonic Sensor
```

---

## 3. Hardware

### Node

- MCU: STM32H562 / STM32H573
- Wi-SUN Module
- USART1: Wi-SUN 통신
- USART6: Debug UART
- ADC1: 초음파 센서
- ADC2: NTC 온도 센서
- RTC: 시간 기반 조명 제어
- TIM6 + DMA: FFT 샘플링
- LIGHT GPIO: 조명 제어

### Gateway

- Raspberry Pi CM4
- Wi-SUN 통신 모듈
- Ethernet / Wi-Fi / LTE
- MQTT Client

---

## 4. Node 주요 기능

### 4.1 조명 제어

지원 제어 방식:

- 수동 ON
- 수동 OFF
- 지정 시간 점·소등
- 일출 / 일몰 시간 기반 자동 제어
- 보정 시간 적용
- 강제 점등 / 소등 시간 적용

기본 자동 제어 예:

```text
ON  : 19:00
OFF : 06:00
```

---

## 5. RTC / 태양 시간 계산

노드는 RTC 시간을 기준으로 자동 점·소등을 수행합니다.

필요 시 위도/경도 및 날짜 정보를 기반으로 일출·일몰 시간을 계산하여 조명 제어에 반영할 수 있습니다.

태양 시간 계산은 매 Main Loop마다 수행하지 않고, 필요 시 하루 1~2회 또는 날짜 변경 시 수행하도록 구성할 수 있습니다.

---

## 6. 센서 데이터

노드에서 수집하는 주요 데이터는 다음과 같습니다.

| 항목 | 설명 |
|---|---|
| Voltage | 입력 전압 |
| Current | 조명 소비 전류 |
| Temperature | NTC 기반 온도 |
| Light State | 조명 ON / OFF 상태 |
| FFT Frequency | 초음파 주요 주파수 |
| FFT Amplitude | FFT 진폭 |
| Wi-SUN Status | 통신 상태 |
| AI Prediction | 이상 상태 추론 결과 |
| RUL | 잔여 수명 관련 값 |

---

## 7. FFT / 이상 감지

초음파 센서 신호를 ADC DMA 방식으로 수집한 후 CMSIS-DSP FFT를 수행합니다.

주요 설정 예:

```text
FFT_SIZE       = 4096
Sampling Rate  = 400 kHz
Analysis Band  = 80 ~ 130 kHz
```

FFT 처리:

```c
arm_rfft_fast_f32()
arm_cmplx_mag_f32()
```

주요 분석값:

- Peak Frequency
- Peak Amplitude
- Signal Level
- Noise Level
- Trigger 상태

SMPS 동작 시 발생하는 초음파 특성을 이용하여 정상 / 비정상 상태 분석에 활용합니다.

---

## 8. Wi-SUN 프로토콜

### 기본 Frame

```text
STX | TYPE | LEN | TMID | DATA | XOR | ETX
```

주요 값:

```text
STX = 0x02
ETX = 0x03

Uplink   = 0xAA 0xAA
Downlink = 0xAA 0xAB
```

Downlink DATA 예:

```text
[tgt_mid][ttl][cmd][flags][msg_id][extra...]
```

TTL 기본값:

```text
10
```

---

## 9. Node Command

| Command | Value | 설명 |
|---|---:|---|
| LIGHT ON | `0x10` | 조명 ON |
| LIGHT OFF | `0x11` | 조명 OFF |
| SNAP | `0x13` | 센서 데이터 요청 |
| SET MID / CH | `0x21` | MID 및 채널 설정 |
| FIRST BOOT | `0x22` | 초기 등록 |
| GET ID | `0x23` | 단말 ID 요청 |
| GET CHANNEL | `0x24` | 채널 정보 요청 |
| GET STATUS | `0x30` | 단말 상태 요청 |
| SET SETTING | `0x31` | 설정 변경 |
| GET NODE INFO | `0x40` | 노드 정보 요청 |

---

## 10. 주요 Uplink

### SNAP

주기적으로 센서 및 조명 상태를 게이트웨이에 전송합니다.

예시 데이터:

```text
voltage
current
temperature
light_on
fft_count
frequency
amplitude
snap_count
msg_id
result
error
```

### ACK

게이트웨이에서 전달된 설정 및 제어 명령에 대한 처리 결과를 반환합니다.

예:

```json
{
  "cmd": "set_setting_ack",
  "mid": 25,
  "result": "success",
  "err_code": 0
}
```

### Light State Event

조명의 상태가 변경되었을 때 별도의 이벤트를 발생시킵니다.

예:

```text
OFF -> ON
ON  -> OFF
```

조명 상태 이벤트는 일반 SNAP 패킷과 구분하여 처리해야 합니다.

---

## 11. Gateway 주요 기능

Gateway는 다음 기능을 담당합니다.

- Wi-SUN 수신
- Wi-SUN Downlink 전송
- MQTT Publish / Subscribe
- 명령 라우팅
- Node 상태 관리
- Node 정보 저장
- SNAP 변환
- ACK 변환
- 연결 상태 감시
- 서버 명령 전달

---

## 12. MQTT Topic

### SNAP

```text
node/{gwid}/snap
```

예:

```text
node/gw001/snap
```

### 명령 응답

```text
node/{gwid}/response/{cmd}
```

예:

```text
node/gw001/response/set_setting
```

### Raw Data

```text
gw/{gwid}/raw
```

### Gateway Command

```text
gw/{gwid}/#
```

---

## 13. Node Store

Gateway는 단말별 최신 상태를 `nodes_store.bin`에 관리합니다.

주요 저장 정보:

```text
uid
mid
mac
last_ts
voltage
current
temperature
fft
online
```

단말 상태 관리 및 연결 끊김 판별에 사용됩니다.

---

## 14. Connection Watchdog

단말의 마지막 Uplink 수신 시간을 기준으로 연결 상태를 판단합니다.

판별 기준 예:

```text
현재 시간 - 마지막 수신 시간
>
SNAP 주기 + 60초
```

조건을 초과하면 단말을 `disconnected` 상태로 판단합니다.

예:

```json
{
  "mid": 25,
  "status": "disconnected",
  "reason": "no_recent_uplink"
}
```

---

## 15. 설정 변경

서버에서 Gateway를 통해 Node의 동작 설정을 변경할 수 있습니다.

예:

- SNAP 주기
- ON/OFF 시간
- 자동 제어 모드
- 보정 시간
- 강제 제어 시간
- 채널 정보
- MID 정보

설정 완료 후 Node ACK를 Gateway가 수신하여 MQTT로 전달합니다.

---

## 16. Provisioning

Node 최초 부팅 시 Wi-SUN Module에서 MID 정보를 확인합니다.

예:

```text
AT+MID?
```

MID가 정상적으로 할당되면 Provisioning 완료 상태로 판단합니다.

예:

```c
cfg.mid_assigned = 1;
```

---

## 17. Flash Configuration

Node 설정값은 내부 Flash에 저장합니다.

예시 Flash 영역:

```text
APP : 0x08000000
CFG : 0x081FE000
SIZE: 8 KB
```

설정 데이터에는 CRC32를 적용하여 데이터 손상을 확인할 수 있습니다.

Flash 수명을 고려하여 불필요하게 짧은 주기의 저장은 피하는 것이 좋습니다.

---

## 18. 프로젝트 디렉토리 예시

```text
SmartLED_Wisun/
│
├─ Core/
│  ├─ Inc/
│  ├─ Src/
│  └─ services/
│
├─ Drivers/
│
├─ Middlewares/
│
├─ Gateway/
│  ├─ cmd_router.py
│  ├─ mqtt_client.py
│  ├─ store.py
│  ├─ provisioning.py
│  └─ gateway.py
│
├─ docs/
│  ├─ protocol.md
│  └─ test_result.md
│
└─ README.md
```

---

## 19. 개발 환경

### Node

- STM32CubeIDE
- STM32CubeMX
- STM32CubeProgrammer
- ARM GCC
- CMSIS-DSP

### Gateway

- Raspberry Pi OS
- Python 3
- paho-mqtt
- systemd
- Wi-SUN Serial Interface

---

## 20. Debug

Node Debug UART를 통해 주요 상태를 확인할 수 있습니다.

예:

```text
[SNAP]
[FFT_DATA]
[LIGHT EVENT]
[SET_SETTING]
[ACK]
[RTC]
[WISUN TX]
[WISUN RX]
```

Gateway에서는 systemd 또는 로그 파일을 통해 상태를 확인할 수 있습니다.

예:

```bash
sudo systemctl status gateway.service
```

```bash
journalctl -u gateway.service -f
```

---

## 23. 프로젝트 목적

본 프로젝트의 최종 목표는 스마트가로등의 상태를 원격에서 모니터링하고, 조명 제어 및 설비 이상을 조기에 감지할 수 있는 시스템을 구축하는 것입니다.

이를 통해 다음과 같은 효과를 기대할 수 있습니다.

- 조명 원격 제어
- 유지보수 효율 향상
- 통신 장애 조기 감지
- SMPS 고장 사전 예측
- 센서 기반 상태 진단
- 관제 시스템 통합
