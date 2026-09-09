#ifndef PLATFORM_RUL_FLASH_H
#define PLATFORM_RUL_FLASH_H

#include <stdbool.h>
#include <stdint.h>

/* Bank 2 sectors 124/125; keep synchronized with both linker scripts. */
#define RUL_FLASH_BASE        0x081F8000u
#define RUL_FLASH_SECTOR_SIZE 0x2000u
#define RUL_FLASH_SECTORS     2u
#define RUL_FLASH_RECORD_SIZE 32u

bool RUL_Flash_Read(uint32_t address, void *record);
bool RUL_Flash_Erase(uint32_t sector_index);
bool RUL_Flash_Program(uint32_t address, const void *record);
/* Called first by NMI_Handler; handles only an armed RUL data read fault. */
bool RUL_Flash_HandleNMI(void);

#endif
