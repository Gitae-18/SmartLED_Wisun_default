/*
 * memory.c
 *
 *  Created on: May 16, 2025
 *      Author: nicek
 */


#include "memory.h"
#include <stdio.h>
#include <stdint.h>
#include "cmsis_gcc.h"  // __get_MSP() 선언을 위해 (혹은 core_cm*.h)

/// 링커 스크립트에서 정의된 심볼들
extern uint32_t _sdata;   // 데이터 섹션 시작
extern uint32_t _edata;   // 데이터 섹션 끝
extern uint32_t _sbss;    // BSS 섹션 시작
extern uint32_t _ebss;    // BSS 섹션 끝
extern uint32_t _estack;  // 스택 최상위 주소

void Print_MemoryUsage(void)
{
    // .data 크기
    uint32_t data_sz  = (uint8_t*)&_edata - (uint8_t*)&_sdata;
    // .bss  크기
    uint32_t bss_sz   = (uint8_t*)&_ebss - (uint8_t*)&_sbss;
    // 스택 사용량 = 스택 시작(_estack)에서 현재 MSP 위치까지 차이
    uint32_t stack_sz = (uint8_t*)&_estack - (uint8_t*) __get_MSP();

    uint32_t total = data_sz + bss_sz + stack_sz;
    /*printf("RAM Usage — .data: %lu, .bss: %lu, stack: %lu, total: %lu bytes\r\n",
           data_sz, bss_sz, stack_sz, total);*/
}
