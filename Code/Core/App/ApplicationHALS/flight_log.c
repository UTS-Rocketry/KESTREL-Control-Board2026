#include "flight_log.h"
#include "W25Q128_HAL.h"
#include "flight_state.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stddef.h>

extern UART_HandleTypeDef huart5;

static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

HAL_StatusTypeDef flight_log_record(const FlightSensorData *d)
{
    FlightLogRecord rec;
    memset(&rec, 0, sizeof(rec));

    rec.sync         = FLIGHT_LOG_SYNC;
    rec.version      = FLIGHT_LOG_VERSION;
    rec.flight_state = (uint8_t)FSM_get_state();
    rec.tick_ms      = HAL_GetTick();

    rec.baro_alt = d->altitude;
    rec.kf_alt   = d->kalman_altitude;
    rec.kf_vel   = d->kalman_velocity;
    rec.ax_mg    = d->x_mg_IMU;   /* Kestrel's up axis */
    rec.ay_mg    = d->y_mg_IMU;
    rec.az_mg    = d->z_mg_IMU;

    /* Log what the servos are actually being driven with, not what was requested:
     * TIM3 ticks at 1 us, so CCR is the pulse width directly. */
    rec.airbrake_us = (uint16_t)TIM3->CCR4;
    rec.roll_us     = (uint16_t)TIM3->CCR3;

    rec.ab_pred_apogee = 0.0f;   /* TODO: airbrake_get_predicted_apogee() once exposed */

    if (HAL_GPIO_ReadPin(LImitSwitchAirbrakes_GPIO_Port, LImitSwitchAirbrakes_Pin) == GPIO_PIN_SET)
        rec.flags |= FLOG_FLAG_AB_LIMIT;
    if (HAL_GPIO_ReadPin(LimitSwitchRoll_GPIO_Port, LimitSwitchRoll_Pin) == GPIO_PIN_SET)
        rec.flags |= FLOG_FLAG_ROLL_LIMIT;
    if (FSM_get_state() == STATE_COAST)
        rec.flags |= FLOG_FLAG_AB_ACTIVE;

    rec.crc = crc16_ccitt((const uint8_t *)&rec, offsetof(FlightLogRecord, crc));

    return flash_log_packet((uint8_t *)&rec, sizeof(rec));
}

/* Prints every record as CSV on UART5. Uses snprintf + HAL_UART_Transmit directly
 * because printf is compiled out in non-DEBUG builds (see main.h). */
HAL_StatusTypeDef flash_dump_serial(void)
{
    char line[200];
    int n;
    uint32_t count = flash_get_record_count();

    n = snprintf(line, sizeof(line),
        "idx,tick_ms,state,flags,baro_alt,kf_alt,kf_vel,ax_mg,ay_mg,az_mg,"
        "airbrake_us,roll_us,ab_pred_apogee,crc_ok\r\n");
    HAL_UART_Transmit(&huart5, (uint8_t *)line, (uint16_t)n, HAL_MAX_DELAY);

    for (uint32_t i = 0; i < count; i++) {
        FlightLogRecord rec;
        if (flash_read_record(i, (uint8_t *)&rec, sizeof(rec)) != HAL_OK) return HAL_ERROR;

        uint8_t crc_ok = (rec.sync == FLIGHT_LOG_SYNC) &&
            (rec.crc == crc16_ccitt((const uint8_t *)&rec, offsetof(FlightLogRecord, crc)));

        n = snprintf(line, sizeof(line),
            "%lu,%lu,%u,%u,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%u,%u,%.1f,%u\r\n",
            (unsigned long)i, (unsigned long)rec.tick_ms, rec.flight_state, rec.flags,
            rec.baro_alt, rec.kf_alt, rec.kf_vel, rec.ax_mg, rec.ay_mg, rec.az_mg,
            rec.airbrake_us, rec.roll_us, rec.ab_pred_apogee, crc_ok);
        HAL_UART_Transmit(&huart5, (uint8_t *)line, (uint16_t)n, HAL_MAX_DELAY);
    }

    n = snprintf(line, sizeof(line), "# END %lu records\r\n", (unsigned long)count);
    HAL_UART_Transmit(&huart5, (uint8_t *)line, (uint16_t)n, HAL_MAX_DELAY);
    return HAL_OK;
}