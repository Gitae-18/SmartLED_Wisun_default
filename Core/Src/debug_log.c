#include "debug_log.h"

#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart6;

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

uint8_t focus_timing_log_enabled(void)
{
#if FOCUS_TIMING_LOG
    return 1u;
#else
    return 0u;
#endif
}

void timing_log(const char *fmt, ...)
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

void debug6(const char *s)
{
#if FOCUS_TIMING_LOG
    (void)s;
    return;
#else
    HAL_UART_Transmit(&huart6, (uint8_t*)s, strlen(s), 100);
#endif
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
    (void)prefix;
    (void)timestamp;
    (void)sTime;
    (void)sDate;

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
