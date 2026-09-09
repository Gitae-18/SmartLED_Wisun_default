// Wi-SUN UART transport
#include "protocol/wisun/wisun_transport.h"

#include "stm32h5xx_hal.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

#define WISUN_UART_TX_TIMEOUT_MS 100u

static volatile wisun_tx_status_t g_wisun_tx_last_status = WISUN_TX_STATUS_OK;

void dbg_dump_uart6(const uint8_t *p, uint16_t n) {
    if (!p || !n) return;

    const char *prefix = "[TX] ";
    HAL_UART_Transmit(&huart6, (uint8_t*)prefix, strlen(prefix), HAL_MAX_DELAY);

    char bytebuf[4];

    for (uint16_t i = 0; i < n; ++i) {
        int m = snprintf(bytebuf, sizeof(bytebuf), "%02X", p[i]);
        HAL_UART_Transmit(&huart6, (uint8_t*)bytebuf, m, HAL_MAX_DELAY);
        if (i + 1 < n) {
            char sp = ' ';
            HAL_UART_Transmit(&huart6, (uint8_t*)&sp, 1, HAL_MAX_DELAY);
        }
    }

    const char *crlf = "\r\n";
    HAL_UART_Transmit(&huart6, (uint8_t*)crlf, strlen(crlf), HAL_MAX_DELAY);
}

bool wisun_transport_send_blocking(const uint8_t *data, uint16_t len) {
    HAL_StatusTypeDef hal_status;

    if (!data || !len) {
        g_wisun_tx_last_status = WISUN_TX_STATUS_INVALID;
        return false;
    }

    hal_status = HAL_UART_Transmit(&huart1, (uint8_t*)data, len, WISUN_UART_TX_TIMEOUT_MS);
    switch (hal_status) {
    case HAL_OK:
        g_wisun_tx_last_status = WISUN_TX_STATUS_OK;
        return true;
    case HAL_BUSY:
        g_wisun_tx_last_status = WISUN_TX_STATUS_BUSY;
        break;
    case HAL_TIMEOUT:
        g_wisun_tx_last_status = WISUN_TX_STATUS_TIMEOUT;
        break;
    case HAL_ERROR:
    default:
        g_wisun_tx_last_status = WISUN_TX_STATUS_ERROR;
        break;
    }

    return false;
}

bool wisun_tx_adapter(const uint8_t *data, uint16_t len, void *user) {
    (void)user;
    if (!data || !len) return false;
    
    return wisun_transport_send_blocking(data, len);
}

wisun_tx_status_t wisun_transport_get_last_status(void) {
    return g_wisun_tx_last_status;
}

void wisun_transport_abort_tx(void) {
    (void)HAL_UART_AbortTransmit(&huart1);
}
