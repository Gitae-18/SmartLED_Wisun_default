// Core/Inc/wisun_transport.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 블로킹 전송 (HAL_UART_Transmit 기반)
 * @param data  전송할 바이트 배열 포인터
 * @param len   전송 길이(바이트)
 * @return true  전송 성공
 * @return false 전송 실패(포인터/길이 오류 또는 HAL 에러)
 *
 * @note 구현은 wisun_transport.c에서 extern UART_HandleTypeDef huart1; 를 사용합니다.
 *       (필요 시 huart1 → huart6 등으로 변경)
 */
bool wisun_transport_send_blocking(const uint8_t *data, uint16_t len);
bool wisun_tx_adapter(const uint8_t *data, uint16_t len, void *user);
void dbg_dump_uart6(const uint8_t *p, uint16_t n);
#ifdef __cplusplus
}
#endif
