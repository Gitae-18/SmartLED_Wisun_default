#ifndef SERVICES_RUL_STORAGE_H
#define SERVICES_RUL_STORAGE_H

#include <stdbool.h>
#include "signal/rul.h"

typedef enum {
    RUL_STORAGE_EMPTY = 0,
    RUL_STORAGE_LOADED = 1,
    RUL_STORAGE_INVALID = -1
} RUL_StorageStatus;

RUL_StorageStatus RUL_Storage_Load(RUL_State *state);
bool RUL_Storage_Save(const RUL_State *state);

#endif
