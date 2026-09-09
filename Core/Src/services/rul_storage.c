#include "services/rul_storage.h"
#include "platform/rul_flash.h"

#include <stddef.h>
#include <string.h>

#define RUL_RECORD_MAGIC   0x52554C31u
/* Bump version if the persisted state layout or RUL interpretation changes. */
#define RUL_RECORD_VERSION 1u
#define RECORDS_PER_SECTOR (RUL_FLASH_SECTOR_SIZE / RUL_FLASH_RECORD_SIZE)

typedef struct {
    RUL_State state;
    uint32_t sequence;
    uint32_t magic;
    uint32_t version;
    uint32_t reserved;
    uint32_t crc;
} RUL_Record;

_Static_assert(sizeof(RUL_Record) == RUL_FLASH_RECORD_SIZE, "RUL record size");
_Static_assert(offsetof(RUL_Record, magic) == 16u, "Commit quadword must be last");

typedef struct {
    RUL_Record latest;
    uint32_t latest_sector;
    uint32_t next_slot[RUL_FLASH_SECTORS];
    bool found;
    bool occupied;
} RUL_Scan;

static uint32_t crc32(const void *data, size_t size)
{
    const uint8_t *bytes = data;
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < size; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
    return crc ^ UINT32_MAX;
}

static bool erased(const RUL_Record *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    for (size_t i = 0; i < sizeof(*record); ++i)
        if (bytes[i] != 0xFFu) return false;
    return true;
}

static bool valid(const RUL_Record *record)
{
    return record->magic == RUL_RECORD_MAGIC &&
        record->version == RUL_RECORD_VERSION && record->reserved == 0u &&
        record->crc == crc32(record, offsetof(RUL_Record, crc)) &&
        RUL_StateValid(&record->state);
}

static RUL_Scan scan(void)
{
    RUL_Scan result = {0};
    for (uint32_t sector = 0; sector < RUL_FLASH_SECTORS; ++sector) {
        for (uint32_t slot = 0; slot < RECORDS_PER_SECTOR; ++slot) {
            RUL_Record record;
            uint32_t address = RUL_FLASH_BASE + sector * RUL_FLASH_SECTOR_SIZE +
                               slot * RUL_FLASH_RECORD_SIZE;
            bool readable = RUL_Flash_Read(address, &record);
            if (readable && erased(&record)) continue;
            /* A torn write/ECC fault consumes the slot; never program it again. */
            result.occupied = true;
            result.next_slot[sector] = slot + 1u;
            if (readable && valid(&record) &&
                (!result.found ||
                 (int32_t)(record.sequence - result.latest.sequence) > 0)) {
                result.latest = record;
                result.latest_sector = sector;
                result.found = true;
            }
        }
    }
    return result;
}

RUL_StorageStatus RUL_Storage_Load(RUL_State *state)
{
    if (state == NULL) return RUL_STORAGE_INVALID;
    RUL_Scan result = scan();
    if (result.found) {
        *state = result.latest.state;
        return RUL_STORAGE_LOADED;
    }
    return result.occupied ? RUL_STORAGE_INVALID : RUL_STORAGE_EMPTY;
}

bool RUL_Storage_Save(const RUL_State *state)
{
    if (!RUL_StateValid(state)) return false;
    RUL_Scan result = scan();
    uint32_t sector = result.found ? result.latest_sector : 0u;
    uint32_t slot = result.next_slot[sector];
    /* Caller supplies trusted RAM state. Runtime blocks saves after an invalid
     * boot recovery, but may retry its first-ever torn write without a reboot. */
    if (slot == RECORDS_PER_SECTOR) {
        sector ^= 1u;
        /* The latest valid record stays intact throughout erase/program. */
        if (!RUL_Flash_Erase(sector)) return false;
        slot = 0u;
    }
    RUL_Record record = {
        .state = *state,
        .sequence = result.found ? result.latest.sequence + 1u : 1u,
        .magic = RUL_RECORD_MAGIC,
        .version = RUL_RECORD_VERSION,
        .reserved = 0u
    };
    record.crc = crc32(&record, offsetof(RUL_Record, crc));
    uint32_t address = RUL_FLASH_BASE + sector * RUL_FLASH_SECTOR_SIZE +
                       slot * RUL_FLASH_RECORD_SIZE;
    if (!RUL_Flash_Program(address, &record)) return false;
    RUL_Record verify;
    return RUL_Flash_Read(address, &verify) && valid(&verify) &&
           memcmp(&verify, &record, sizeof(record)) == 0;
}
