/*
 * commin.h
 *
 *  Created on: Mar 27, 2025
 *      Author: nicek
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#define BOOT_ADDRESS   0x08000000
#define APP1_ADDRESS  0x08010000
#define APP2_ADDRESS  0x08080000
#define OTA_FLAG_ADDR  0x080FF000

typedef enum {
    OTA_IDLE      = 0xFFFFFFFF,
    OTA_READY     = 0xA5A5A5A5,
    OTA_FAILED    = 0x00000000,
    OTA_COMPLETED = 0x5A5A5A5A
} ota_flag_t;

#endif /* INC_COMMON_H_ */
