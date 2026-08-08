#include "sport_telemetry.h"
#include "common.h"
#include "dshot.h"
#include "eeprom.h"

#ifdef USE_SPORT_TELEMETRY

#define SPORT_START_STOP 0x7EU
#define SPORT_CAPTURED_START_STOP 0x7FU
#define SPORT_DATA_FRAME 0x10U
#define SPORT_CONFIG_REQUEST_FRAME 0x30U
#define SPORT_CONFIG_REQUEST_FRAME_FPORT 0x31U
#define SPORT_CONFIG_REPLY_FRAME 0x32U
#define SPORT_STUFF_BYTE 0x7DU
#define SPORT_STUFF_MASK 0x20U
#define SPORT_SENSOR_ID_MASK 0x1FU
#define SPORT_CONFIG_SENSOR_ID 0x0DU

#define SPORT_CONFIG_REQUEST_MAGIC 0x5100U
#define SPORT_CONFIG_REPLY_MAGIC 0x5200U
#define SPORT_CONFIG_VALUE_MASK 0xFFFFU

#define SPORT_ID_CURRENT 0x0200U
#define SPORT_ID_VFAS 0x0210U
#define SPORT_ID_TEMP2 0x0400U
#define SPORT_ID_RPM 0x0500U
#define SPORT_ID_FUEL 0x0B30U
#define SPORT_ID_TEMP1 0x0B70U

#define SPORT_SLOT_COUNT 6U
#define SPORT_SLOT_REPEATS 1U

#define SPORT_CONFIG_CMD_PING 0x01U
#define SPORT_CONFIG_CMD_GET 0x02U
#define SPORT_CONFIG_CMD_SET 0x03U
#define SPORT_CONFIG_CMD_SAVE 0x04U

#define SPORT_CONFIG_STATUS_OK 0x00U
#define SPORT_CONFIG_STATUS_UNKNOWN 0x01U
#define SPORT_CONFIG_STATUS_BAD_PARAM 0x02U
#define SPORT_CONFIG_STATUS_RANGE 0x03U
#define SPORT_CONFIG_STATUS_BUSY 0x04U

uint8_t aTxBuffer[SPORT_TELEMETRY_MAX_PACKET_SIZE] __attribute__((aligned(4)));
uint8_t nbDataToTransmit = sizeof(aTxBuffer);

static uint8_t telemetry_field_index;
static uint8_t telemetry_repeat_count;

static uint8_t rx_after_start;
static uint8_t rx_config_active;
static uint8_t rx_config_stuff_byte;
static uint8_t rx_config_count;
static uint8_t rx_config_payload[7];
static uint16_t rx_config_crc;

static uint8_t config_reply_pending;
static uint16_t config_reply_data_id;
static uint32_t config_reply_value;

typedef struct sport_config_parameter {
    uint8_t id;
    uint8_t offset;
    uint16_t min;
    uint16_t max;
} sport_config_parameter_t;

static const sport_config_parameter_t sport_config_parameters[] = {
    { 1U,  27U, 2U,   64U  }, // motor_poles
    { 2U,  17U, 0U,   1U   }, // dir_reversed
    { 3U,  18U, 0U,   1U   }, // bi_direction
    { 4U,  30U, 0U,   11U  }, // beep_volume
    { 5U,  24U, 8U,   144U }, // pwm_frequency
    { 6U,  5U,  1U,   200U }, // max_ramp
    { 7U,  6U,  0U,   50U  }, // minimum_duty_cycle
    { 8U,  19U, 0U,   1U   }, // use_sine_start
    { 9U,  20U, 0U,   1U   }, // comp_pwm
    { 10U, 22U, 0U,   1U   }, // stuck_rotor_protection
    { 11U, 25U, 50U,  150U }, // startup_power
    { 12U, 41U, 1U,   10U  }, // drag_brake_strength
    { 13U, 42U, 1U,   10U  }, // driving_brake_strength
    { 14U, 21U, 0U,   2U   }, // variable_pwm
    { 15U, 23U, 0U,   30U  }, // advance_level (user 0-30, stored as +10 in eeprom)
    { 16U, 26U, 20U,  10220U }, // motor_kv (actual kV; stored as (kV-20)/40)
    { 17U, 28U, 0U,   1U   }, // brake_on_stop
    { 18U, 29U, 0U,   1U   }, // stall_protection
    { 19U, 36U, 0U,   1U   }, // low_voltage_cut_off
    { 20U, 37U, 0U,   100U }, // low_cell_volt_cutoff (raw; mV/cell = value+250)
    { 21U, 38U, 0U,   1U   }, // rc_car_reverse
    { 22U, 39U, 0U,   1U   }, // use_hall_sensors
    { 23U, 40U, 5U,   25U  }, // sine_mode_changeover_throttle_level
    { 24U, 43U, 70U,  255U }, // limits.temperature (255 = disabled)
    { 25U, 44U, 0U,   100U }, // limits.current (0 = disabled)
    { 26U, 45U, 1U,   10U  }, // sine_mode_power
    { 27U, 46U, 0U,   5U   }, // input_type
    { 28U, 47U, 0U,   1U   }, // auto_advance
};

static void sport_crc_add(uint16_t* crc, uint8_t value)
{
    *crc += value;
    *crc += *crc >> 8;
    *crc &= 0x00FFU;
}

static uint8_t sport_append_byte(uint8_t* buffer, uint8_t offset, uint8_t value)
{
    if ((value == SPORT_START_STOP) || (value == SPORT_STUFF_BYTE)) {
        buffer[offset++] = SPORT_STUFF_BYTE;
        buffer[offset++] = value ^ SPORT_STUFF_MASK;
    } else {
        buffer[offset++] = value;
    }
    return offset;
}

static uint8_t sport_append_frame(uint8_t* buffer, uint8_t frame_id, uint16_t data_id, uint32_t value)
{
    uint8_t frame[8];
    uint16_t crc = 0;
    uint8_t offset = 0;

    frame[0] = frame_id;
    frame[1] = data_id & 0xFFU;
    frame[2] = data_id >> 8;
    frame[3] = value & 0xFFU;
    frame[4] = (value >> 8) & 0xFFU;
    frame[5] = (value >> 16) & 0xFFU;
    frame[6] = (value >> 24) & 0xFFU;

    for (uint8_t i = 0; i < 7U; i++) {
        sport_crc_add(&crc, frame[i]);
    }
    frame[7] = 0xFFU - crc;

    for (uint8_t i = 0; i < sizeof(frame); i++) {
        offset = sport_append_byte(buffer, offset, frame[i]);
    }

    nbDataToTransmit = offset;
    return offset;
}

static uint8_t sport_append_response(uint8_t* buffer, uint16_t data_id, uint32_t value)
{
    return sport_append_frame(buffer, SPORT_DATA_FRAME, data_id, value);
}

static void sport_queue_config_reply(uint8_t status, uint8_t command, uint8_t parameter_id, uint16_t value)
{
    config_reply_data_id = SPORT_CONFIG_REPLY_MAGIC | status;
    config_reply_value = ((uint32_t)command << 24) | ((uint32_t)value << 8) | parameter_id;
    config_reply_pending = 1;
}

static const sport_config_parameter_t* sport_find_parameter(uint8_t parameter_id)
{
    for (uint8_t i = 0; i < (sizeof(sport_config_parameters) / sizeof(sport_config_parameters[0])); i++) {
        if (sport_config_parameters[i].id == parameter_id) {
            return &sport_config_parameters[i];
        }
    }
    return 0;
}

static uint8_t sport_config_safe_to_write(void)
{
    return (!running || (newinput == 0U)) ? 1U : 0U;
}

static uint16_t sport_config_read_parameter(const sport_config_parameter_t* parameter)
{
    uint8_t raw = eepromBuffer.buffer[parameter->offset];
    if (parameter->offset == 23U && raw >= 10U) {
        return raw - 10U;
    }
    if (parameter->offset == 26U) {
        return (uint16_t)raw * 40U + 20U;
    }
    return raw;
}

static void sport_config_write_parameter(const sport_config_parameter_t* parameter, uint16_t value)
{
    const uint8_t last_dir_reversed = eepromBuffer.dir_reversed;
    const uint8_t last_bi_direction = eepromBuffer.bi_direction;

    uint8_t stored = (uint8_t)value;
    if (parameter->offset == 23U) {
        stored = (uint8_t)(value + 10U);
    } else if (parameter->offset == 26U) {
        stored = (uint8_t)((value - 20U) / 40U);
    }
    eepromBuffer.buffer[parameter->offset] = stored;

    if ((last_dir_reversed != eepromBuffer.dir_reversed) || (last_bi_direction != eepromBuffer.bi_direction)) {
        forward = 1 - eepromBuffer.dir_reversed;
        running = 0;
        armed = 0;
    }
}

static void sport_config_handle_request(uint8_t command, uint32_t value)
{
    const uint8_t parameter_id = value & 0xFFU;
    const uint16_t parameter_value = (value >> 8) & SPORT_CONFIG_VALUE_MASK;
    const sport_config_parameter_t* parameter;

    if (command == SPORT_CONFIG_CMD_PING) {
        config_reply_data_id = SPORT_CONFIG_REPLY_MAGIC | SPORT_CONFIG_STATUS_OK;
        config_reply_value = 0x32335053U; // "SP32" little-endian
        config_reply_pending = 1;
        return;
    }

    if (command == SPORT_CONFIG_CMD_SAVE) {
        if (!sport_config_safe_to_write()) {
            sport_queue_config_reply(SPORT_CONFIG_STATUS_BUSY, command, 0U, 0U);
            return;
        }
        saveEEpromSettings();
        sport_queue_config_reply(SPORT_CONFIG_STATUS_OK, command, 0U, 0U);
        return;
    }

    parameter = sport_find_parameter(parameter_id);
    if (parameter == 0) {
        sport_queue_config_reply(SPORT_CONFIG_STATUS_BAD_PARAM, command, parameter_id, 0U);
        return;
    }

    if (command == SPORT_CONFIG_CMD_GET) {
        sport_queue_config_reply(SPORT_CONFIG_STATUS_OK, command, parameter_id, sport_config_read_parameter(parameter));
        return;
    }

    if (command == SPORT_CONFIG_CMD_SET) {
        if (!sport_config_safe_to_write()) {
            sport_queue_config_reply(SPORT_CONFIG_STATUS_BUSY, command, parameter_id, sport_config_read_parameter(parameter));
            return;
        }
        if ((parameter_value < parameter->min) || (parameter_value > parameter->max)) {
            sport_queue_config_reply(SPORT_CONFIG_STATUS_RANGE, command, parameter_id, sport_config_read_parameter(parameter));
            return;
        }
        sport_config_write_parameter(parameter, parameter_value);
        sport_queue_config_reply(SPORT_CONFIG_STATUS_OK, command, parameter_id, sport_config_read_parameter(parameter));
        return;
    }

    sport_queue_config_reply(SPORT_CONFIG_STATUS_UNKNOWN, command, parameter_id, 0U);
}

static void sport_config_handle_frame(void)
{
    uint16_t data_id;
    uint32_t value;

    if ((rx_config_payload[0] != SPORT_CONFIG_REQUEST_FRAME) &&
        (rx_config_payload[0] != SPORT_CONFIG_REQUEST_FRAME_FPORT)) {
        return;
    }

    data_id = (uint16_t)rx_config_payload[1] | ((uint16_t)rx_config_payload[2] << 8);
    if ((data_id & 0xFF00U) != SPORT_CONFIG_REQUEST_MAGIC) {
        return;
    }

    value = (uint32_t)rx_config_payload[3] |
            ((uint32_t)rx_config_payload[4] << 8) |
            ((uint32_t)rx_config_payload[5] << 16) |
            ((uint32_t)rx_config_payload[6] << 24);

    sport_config_handle_request(data_id & 0xFFU, value);
}

uint8_t sport_prepare_response(uint8_t* buffer)
{
    uint16_t data_id;
    uint32_t value;

    if (config_reply_pending) {
        config_reply_pending = 0;
        return sport_append_frame(buffer, SPORT_CONFIG_REPLY_FRAME, config_reply_data_id, config_reply_value);
    }

    switch (telemetry_field_index) {
    default:
    case 0:
        data_id = SPORT_ID_TEMP1;
        value = (degrees_celsius > 0) ? (uint32_t)degrees_celsius : 0U;
        break;
    case 1:
        data_id = SPORT_ID_TEMP2;
        value = 0U;
        break;
    case 2:
        data_id = SPORT_ID_VFAS;
        value = battery_voltage;
        break;
    case 3:
        data_id = SPORT_ID_CURRENT;
        value = (actual_current > 0) ? ((uint16_t)actual_current / 10U) : 0U;
        break;
    case 4:
        data_id = SPORT_ID_FUEL;
        value = 0U;
        break;
    case 5:
        data_id = SPORT_ID_RPM;
        value = (eepromBuffer.motor_poles == 0U) ? 0U : ((uint32_t)e_rpm * 200U) / eepromBuffer.motor_poles;
        break;
    }

    telemetry_repeat_count++;
    if (telemetry_repeat_count >= SPORT_SLOT_REPEATS) {
        telemetry_repeat_count = 0;
        telemetry_field_index++;
        if (telemetry_field_index >= SPORT_SLOT_COUNT) {
            telemetry_field_index = 0;
        }
    }

    return sport_append_response(buffer, data_id, value);
}

uint8_t sport_rx_byte(uint8_t byte)
{
    if ((byte == SPORT_START_STOP) || (byte == SPORT_CAPTURED_START_STOP)) {
        rx_after_start = 1;
        rx_config_active = 0;
        rx_config_stuff_byte = 0;
        rx_config_count = 0;
        return 0;
    }

    if (rx_after_start) {
        rx_after_start = 0;

        if ((byte & SPORT_SENSOR_ID_MASK) == SPORT_SENSOR_ID) {
            return 1;
        }

        if (byte == SPORT_CONFIG_SENSOR_ID) {
            rx_config_active = 1;
            rx_config_count = 0;
            rx_config_crc = 0;
        }
        return 0;
    }

    if (rx_config_active) {
        if (byte == SPORT_STUFF_BYTE) {
            rx_config_stuff_byte = 1;
            return 0;
        }

        if (rx_config_stuff_byte) {
            byte ^= SPORT_STUFF_MASK;
            rx_config_stuff_byte = 0;
        }

        if (rx_config_count < sizeof(rx_config_payload)) {
            rx_config_payload[rx_config_count++] = byte;
            sport_crc_add(&rx_config_crc, byte);
            return 0;
        }

        if (byte == (uint8_t)(0xFFU - rx_config_crc)) {
            sport_config_handle_frame();
        }
        rx_config_active = 0;
        rx_config_count = 0;
    }

    return 0;
}

#endif
