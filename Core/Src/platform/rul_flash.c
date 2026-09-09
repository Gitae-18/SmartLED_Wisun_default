#include "platform/rul_flash.h"
#include "stm32h5xx_hal.h"

#include <stddef.h>
#include <string.h>

#define RUL_FIRST_SECTOR 124u

static volatile uint32_t g_read_address;
static volatile bool g_read_active;
static volatile bool g_read_fault;

_Static_assert(RUL_FLASH_SECTOR_SIZE == FLASH_SECTOR_SIZE, "RUL flash sector size");

static bool address_valid(uint32_t address)
{
    return address >= RUL_FLASH_BASE &&
        address <= RUL_FLASH_BASE + RUL_FLASH_SECTORS * RUL_FLASH_SECTOR_SIZE -
                   RUL_FLASH_RECORD_SIZE &&
        (address % RUL_FLASH_RECORD_SIZE) == 0u;
}

static bool bank_mapping_valid(void)
{
    /* This project's configuration storage also assumes unswapped banks. */
    return (FLASH->OPTSR_CUR & FLASH_OPTSR_SWAP_BANK) == 0u;
}

static void invalidate_cache(void)
{
    if ((ICACHE->CR & ICACHE_CR_EN) != 0u) {
        while ((ICACHE->SR & ICACHE_SR_BUSYF) != 0u) {}
        SET_BIT(ICACHE->CR, ICACHE_CR_CACHEINV);
        while ((ICACHE->SR & ICACHE_SR_BUSYF) != 0u) {}
        __DSB();
        __ISB();
    }
}

bool RUL_Flash_HandleNMI(void)
{
    uint32_t ecc = FLASH->ECCDETR;
    if (!g_read_active || (ecc & FLASH_ECCR_ECCD) == 0u) return false;
    uint32_t area = ecc & ~(FLASH_ECCR_ADDR_ECC | FLASH_ECCR_ECCD);
    uint32_t address = FLASH_BASE + FLASH_BANK_SIZE +
                       ((ecc & FLASH_ECCR_ADDR_ECC) << 4u);
    if (area != FLASH_ECC_AREA_USER_BANK2 ||
        address != (g_read_address & ~15u)) return false;

    /* Only a bad data word being probed by the RUL journal is recoverable.
     * Unrelated NMI sources continue to the existing fatal handler. */
    g_read_fault = true;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);
    invalidate_cache();
    __DSB();
    return true;
}

bool RUL_Flash_Read(uint32_t address, void *record)
{
    if (record == NULL || !address_valid(address) || !bank_mapping_valid())
        return false;
    uint32_t words[RUL_FLASH_RECORD_SIZE / sizeof(uint32_t)];
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    g_read_fault = false;
    for (size_t i = 0; i < sizeof(words) / sizeof(words[0]); ++i) {
        g_read_address = address + (uint32_t)i * 4u;
        g_read_active = true;
        __DSB();
        words[i] = *(volatile const uint32_t *)(uintptr_t)g_read_address;
        __DSB();
        g_read_active = false;
        if (g_read_fault) break;
    }
    bool ok = !g_read_fault;
    __set_PRIMASK(irq_mask);
    if (ok) memcpy(record, words, sizeof(words));
    return ok;
}

bool RUL_Flash_Erase(uint32_t sector_index)
{
    if (sector_index >= RUL_FLASH_SECTORS || !bank_mapping_valid()) return false;
    if (HAL_FLASH_Unlock() != HAL_OK) return false;
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error = 0u;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_2;
    erase.Sector = RUL_FIRST_SECTOR + sector_index;
    erase.NbSectors = 1u;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &error);
    HAL_FLASH_Lock();
    invalidate_cache();
    return status == HAL_OK;
}

bool RUL_Flash_Program(uint32_t address, const void *record)
{
    if (record == NULL || !address_valid(address) || !bank_mapping_valid())
        return false;
    /* HAL reads four aligned words per 128-bit program operation. */
    uint32_t words[RUL_FLASH_RECORD_SIZE / sizeof(uint32_t)];
    memcpy(words, record, sizeof(words));
    if (HAL_FLASH_Unlock() != HAL_OK) return false;
    HAL_StatusTypeDef status = HAL_OK;
    for (uint32_t offset = 0u; offset < sizeof(words); offset += 16u) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address + offset,
                                  (uint32_t)(uintptr_t)((uint8_t *)words + offset));
        if (status != HAL_OK) break;
    }
    HAL_FLASH_Lock();
    invalidate_cache();
    return status == HAL_OK;
}
