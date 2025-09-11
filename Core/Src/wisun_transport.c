// Core/Src/wisun_transport.c
#include "wisun_transport.h"

#include "stm32h5xx_hal.h"   // MCU 맞게 수정

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

void dbg_dump_uart6(const uint8_t *p, uint16_t n) {
    //HAL_UART_Transmit(&huart6, (uint8_t*)"[TX] ", 5, 50);
    for (uint16_t i = 0; i < n; ++i) {
        char s[4];
        int m = snprintf(s, sizeof(s), "%02X", p[i]);
        //HAL_UART_Transmit(&huart6, (uint8_t*)s, (uint16_t)m, 50);
        //if (i + 1 < n) HAL_UART_Transmit(&huart6, (uint8_t*)" ", 1, 50);
    }
    //HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, 50);
}

bool wisun_transport_send_blocking(const uint8_t *data, uint16_t len) {
    if (!data || !len) return false;
    char line[48];
	int n = snprintf(line, sizeof(line),
					 "[Wi-SUN] TX->UART1 len=%u\r\n", (unsigned)len);
	//HAL_UART_Transmit(&huart6, (uint8_t*)line, (uint16_t)n, 100);
    return HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 1000) == HAL_OK;
}
bool wisun_tx_adapter(const uint8_t *data, uint16_t len, void *user) {
    // 여기서 UART6으로 "len" 한 줄만 찍고
    {
    	if (!data || !len) return false;
		dbg_dump_uart6(data, len);                 // ★ 여기서 한 번만 찍음
		return wisun_transport_send_blocking(data, len);
    }
    // 실제 전송은 기존 함수로
    return wisun_transport_send_blocking(data, len);
}
