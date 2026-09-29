#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "flight_sensors.h"

#define FLIGHT_LOG_SYNC     0xAA   /* must match FLASH_SYNC_WORD in the flash driver */
#define FLIGHT_LOG_VERSION  1      /* bump if the layout changes */

/* flags bits */
#define FLOG_FLAG_AB_LIMIT    (1U << 0)  /* airbrake max limit switch triggered */
#define FLOG_FLAG_ROLL_LIMIT  (1U << 1)  /* roll limit switch triggered */
#define FLOG_FLAG_AB_ACTIVE   (1U << 2)  /* airbrake controller running (COAST) */

/* 64-byte record, fields naturally aligned, little-endian.
 * Byte 0 must be the sync word: flash_recover_write_pointer() relies on it. */
typedef struct __attribute__((packed)) {
    uint8_t  sync;            /*  0 */
    uint8_t  version;         /*  1 */
    uint8_t  flight_state;    /*  2 */
    uint8_t  flags;           /*  3 */
    uint32_t tick_ms;         /*  4 */
    float    baro_alt;        /*  8  raw baro altitude */
    float    kf_alt;          /* 12  kalman altitude */
    float    kf_vel;          /* 16  kalman velocity */
    float    ax_mg;           /* 20 */
    float    ay_mg;           /* 24 */
    float    az_mg;           /* 28 */
    uint16_t airbrake_us;     /* 32  actual pulse on TIM3 CH4 */
    uint16_t roll_us;         /* 34  actual pulse on TIM3 CH3 */
    float    ab_pred_apogee;  /* 36  controller internal, 0 if not exposed */
    uint8_t  reserved[22];    /* 40  room for more fields without a size change */
    uint16_t crc;             /* 62  CRC-16/CCITT over bytes 0..61 */
} FlightLogRecord;

_Static_assert(sizeof(FlightLogRecord) == 64, "FlightLogRecord must be 64 bytes");

HAL_StatusTypeDef flight_log_record(const FlightSensorData *d);
HAL_StatusTypeDef flash_dump_serial(void);

#endif /* FLIGHT_LOG_H */