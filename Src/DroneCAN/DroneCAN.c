/*
 * DroneCAN.c - support for DroneCAN protocol for ESC control and telemetry
 */

#include "targets.h"

//#pragma GCC optimize("O0")

#if DRONECAN_SUPPORT

#include "peripherals.h"
#include "serial_telemetry.h"
#include <common.h>
#include <signal.h>
#include <version.h>
#include <eeprom.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "sys_can.h"
#include <canard.h>
#include "phaseouts.h"
#include "functions.h"
#include "filter.h"

// include the headers for the generated DroneCAN messages from the
// dronecan_dsdlc compiler
#include "dsdl_generated/dronecan_msgs.h"

#ifndef PREFERRED_NODE_ID
#define PREFERRED_NODE_ID 0
#endif

#ifndef CANARD_POOL_SIZE
#define CANARD_POOL_SIZE 4096
#endif

// use set input at 1kHz
#define TARGET_PERIOD_US 1000U

#define EEPROM_MOTOR_KV_INDEX 26
#define EEPROM_TUNE_INDEX 48
#define EEPROM_TUNE_MAX_LEN 128

static CanardInstance canard;
static uint8_t canard_memory_pool[CANARD_POOL_SIZE];

struct CANStats canstats;

static bool dronecan_armed;
static bool done_startup;

#define APP_SIGNATURE_MAGIC1 0x68f058e6
#define APP_SIGNATURE_MAGIC2 0xafcee5a0

/*
  application signature, filled in by set_app_signature.py
 */
const struct {
    uint32_t magic1;
    uint32_t magic2;
    uint32_t fwlen; // total fw length in bytes
    uint32_t crc1; // crc32 up to start of app_signature
    uint32_t crc2; // crc32 from end of app_signature to end of fw
    char mcu[16];
    uint32_t unused[2];
} app_signature __attribute__((section(".app_signature"))) = {
        .magic1 = APP_SIGNATURE_MAGIC1,
        .magic2 = APP_SIGNATURE_MAGIC2,
        .fwlen = 0,
        .crc1 = 0,
        .crc2 = 0,
        .mcu = AM32_MCU,
};

/*
  DroneCAN uses a chunk of eeprom storage starting at offset 176
  for DroneCAN specific settings
 */
enum eeprom_offset {
        EEPROM_CAN_NODE = 176,
        EEPROM_ESC_INDEX = 177,
        EEPROM_REQUIRE_ARMING = 178,
        EEPROM_TELEM_RATE = 179,
        EEPROM_REQUIRE_ZERO_THROTTLE = 180,
        EEPROM_FILTER_HZ = 181,
};

/*
  state of user settings. This will be saved in settings.dat. On a
  real device a better storage system will be needed
  For simplicity we store all parameters as floats in this example
*/
static struct
{
    uint8_t can_node;
    uint8_t esc_index;
    bool require_arming;
    bool require_zero_throttle;
    uint8_t telem_rate;
    uint8_t filter_hz;
} settings;

enum VarType {
    T_BOOL = 0,
    T_UINT8,
    T_UINT16,
    T_STRING,
};

static void can_printf(const char *fmt, ...);

// some convenience macros
#define MIN(a,b) ((a)<(b)?(a):(b))
#define C_TO_KELVIN(temp) (temp + 273.15f)
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

/*
  access to settings from main.c
 */
extern char dir_reversed;
extern char bi_direction;
extern char advance_level;
extern char motor_poles;
extern uint16_t motor_kv;
extern char VARIABLE_PWM;
extern char use_sin_start;
extern char comp_pwm;
extern char stuck_rotor_protection;
extern char armed;
extern char drag_brake_strength;
extern uint8_t driving_brake_strength;
extern char brake_on_stop;
extern char auto_advance;
static uint16_t last_can_input;
static struct {
    uint32_t sum;
    uint32_t count;
} current;

extern void saveEEpromSettings(void);
extern void loadEEpromSettings(void);
static void set_input(uint16_t input);

/*
  the set of parameters to present to the user over DroneCAN
*/
static struct parameter {
    char *name;
    enum VarType vtype;
    uint16_t min_value;
    uint16_t max_value;
    uint16_t default_value;
    void *ptr;
    uint8_t eeprom_index;
} parameters[] = {
        // list of settable parameters
        { "CAN_NODE",               T_UINT8, 0, 127, 0, &settings.can_node, EEPROM_CAN_NODE},
        { "ESC_INDEX",              T_UINT8, 0, 32,  0, &settings.esc_index, EEPROM_ESC_INDEX},
        { "DIR_REVERSED",           T_BOOL,  0, 1,   0, &dir_reversed, 0 },
        { "MOTOR_KV",               T_UINT16,0, 10220, 2000, &motor_kv, EEPROM_MOTOR_KV_INDEX},
        { "BI_DIRECTIONAL",         T_BOOL,  0, 1,   0, &bi_direction, 0 },
        { "MOTOR_POLES",            T_UINT8, 0, 64,  14, &motor_poles, 27 },
        { "REQUIRE_ARMING",         T_BOOL,  0, 1,   1, &settings.require_arming, EEPROM_REQUIRE_ARMING},
        { "TELEM_RATE",             T_UINT8, 0, 200, 25, &settings.telem_rate, EEPROM_TELEM_RATE },
        { "REQUIRE_ZERO_THROTTLE",  T_BOOL,  0, 1,   1, &settings.require_zero_throttle, EEPROM_REQUIRE_ZERO_THROTTLE },
        { "VARIABLE_PWM",           T_BOOL,  0, 1,   1, &VARIABLE_PWM, 0},
        { "USE_SIN_START",          T_BOOL,  0, 1,   0, &use_sin_start, 0},
        { "COMP_PWM",               T_BOOL,  0, 1,   1, &comp_pwm, 0},
        { "STUCK_ROTOR_PROTECTION", T_BOOL,  0, 1,   1, &stuck_rotor_protection, 0},
        { "ADVANCE_LEVEL",          T_UINT8, 0, 4,   2, &advance_level, 0},
        { "AUTO_ADVANCE",           T_BOOL,  0, 1,   1, &auto_advance, 47},
        { "BRAKE_ON_STOP",          T_BOOL,  0, 1,   1, &brake_on_stop, 28},
        { "DRIVING_BRAKE_STRENGTH", T_UINT8, 1, 10,  10, &driving_brake_strength, 42},
        { "DRAG_BRAKE_STRENGTH",    T_UINT8, 1, 10,  10, &drag_brake_strength, 41},
        { "INPUT_FILTER_HZ",        T_UINT8, 0, 100, 0, &settings.filter_hz, EEPROM_FILTER_HZ},
        { "STARTUP_TUNE",           T_STRING,0, 4,   0, NULL, EEPROM_TUNE_INDEX},
};

/*
  get settings from eeprom
*/
static void load_settings(void)
{
    if (eepromBuffer[EEPROM_CAN_NODE] <= 127) {
        settings.can_node = eepromBuffer[EEPROM_CAN_NODE];
    }
    if (eepromBuffer[EEPROM_ESC_INDEX] <= 32) {
        settings.esc_index = eepromBuffer[EEPROM_ESC_INDEX];
    }

    settings.require_arming = eepromBuffer[EEPROM_REQUIRE_ARMING] == 0?false:true;
    settings.require_zero_throttle = eepromBuffer[EEPROM_REQUIRE_ZERO_THROTTLE] == 0?false:true;

    if (eepromBuffer[EEPROM_TELEM_RATE] <= 200 && eepromBuffer[EEPROM_TELEM_RATE] > 0) {
        settings.telem_rate = eepromBuffer[EEPROM_TELEM_RATE];
    } else {
        settings.telem_rate = 25;
    }
    if (eepromBuffer[EEPROM_FILTER_HZ] <= 100) {
        settings.filter_hz = eepromBuffer[EEPROM_FILTER_HZ];
    }
}

/*
  save settings to flash
 */
static void save_settings(void)
{
    eepromBuffer[EEPROM_CAN_NODE] = settings.can_node;
    eepromBuffer[EEPROM_ESC_INDEX] = settings.esc_index;
    eepromBuffer[EEPROM_REQUIRE_ARMING] = settings.require_arming;
    eepromBuffer[EEPROM_TELEM_RATE] = settings.telem_rate;
    eepromBuffer[EEPROM_REQUIRE_ZERO_THROTTLE] = settings.require_zero_throttle;
    eepromBuffer[EEPROM_FILTER_HZ] = settings.filter_hz;
    saveEEpromSettings();
    can_printf("saved settings");
}

/*
  hold our node status as a static variable. It will be updated on any errors
*/
static struct uavcan_protocol_NodeStatus node_status;

static bool safe_to_write_settings(void)
{
    return !running || newinput == 0;
}

/*
  simple 16 bit random number generator
*/
static uint16_t get_random16(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 0xFFFFu) + (m_z >> 16);
    m_w = 18000 * (m_w & 0xFFFFu) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}

/*
  get a 64 bit monotonic timestamp in microseconds since start. This
  is platform specific

  NOTE: this should be in functions.c
*/
static uint64_t micros64(void)
{
    static uint64_t base_us;
    static uint16_t last_cnt;
#ifdef ARTERY
    uint16_t cnt = UTILITY_TIMER->cval;
#else
    uint16_t cnt = UTILITY_TIMER->CNT;
#endif
    if (cnt < last_cnt) {
	base_us += 0x10000;
    }
    last_cnt = cnt;
    return base_us + cnt;
}

/*
  get monotonic time in milliseconds since startup
*/
static uint32_t millis32(void)
{
    return micros64() / 1000ULL;
}

/*
  default settings, based on public/assets/eeprom_default.bin in AM32 configurator
 */
static const uint8_t default_settings[] = {
    0x01, 0x02, 0x01, 0x01, 0x23, 0x4e, 0x45, 0x4f, 0x45, 0x53, 0x43, 0x20, 0x66, 0x30, 0x35, 0x31,
    0x20, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x02, 0x18, 0x64, 0x37, 0x0e, 0x00, 0x00, 0x05, 0x00,
    0x80, 0x80, 0x80, 0x32, 0x00, 0x32, 0x00, 0x00, 0x0f, 0x0a, 0x0a, 0x8d, 0x66, 0x06, 0x00, 0x00
};

// printf to CAN LogMessage for debugging
static void can_printf(const char *fmt, ...)
{
    struct uavcan_protocol_debug_LogMessage pkt;
    memset(&pkt, 0, sizeof(pkt));

    uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
    va_list ap;
    va_start(ap, fmt);
    uint32_t n = vsnprintf((char*)pkt.text.data, sizeof(pkt.text.data), fmt, ap);
    va_end(ap);
    pkt.text.len = MIN(n, sizeof(pkt.text.data));

    uint32_t len = uavcan_protocol_debug_LogMessage_encode(&pkt, buffer);
    static uint8_t logmsg_transfer_id;

    canardBroadcast(&canard,
		    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
		    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
		    &logmsg_transfer_id,
		    CANARD_TRANSFER_PRIORITY_LOW,
		    buffer, len);
}

/*
  handle parameter GetSet request
*/
static void handle_param_GetSet(CanardInstance* ins, CanardRxTransfer* transfer)
{
    struct uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req)) {
        return;
    }

    struct parameter *p = NULL;
    if (req.name.len != 0) {
        for (uint16_t i=0; i<ARRAY_SIZE(parameters); i++) {
            if (req.name.len == strlen(parameters[i].name) &&
                strncmp((const char *)req.name.data, parameters[i].name, req.name.len) == 0) {
                p = &parameters[i];
                break;
            }
        }
    } else if (req.index < ARRAY_SIZE(parameters)) {
        p = &parameters[req.index];
    }
    if (p != NULL && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
        const char last_dir_reversed = dir_reversed;

        /*
	  a parameter set command
	*/
	switch (p->vtype) {
	case T_UINT8:
	    *(uint8_t *)p->ptr = req.value.integer_value;
	    if (p->eeprom_index != 0) {
		eepromBuffer[p->eeprom_index] = *(uint8_t *)p->ptr;
	    }
            break;
	case T_UINT16:
	    *(uint16_t *)p->ptr = req.value.integer_value;
	    if (p->eeprom_index == EEPROM_MOTOR_KV_INDEX) {
	        eepromBuffer[EEPROM_MOTOR_KV_INDEX] = (uint8_t)((*(uint16_t *)p->ptr - 20) / 40);
	    }
            break;
	case T_BOOL:
	    *(uint8_t *)p->ptr = req.value.boolean_value?1:0;
	    if (p->eeprom_index != 0) {
		eepromBuffer[p->eeprom_index] = *(uint8_t *)p->ptr;
	    }
            break;
	case T_STRING:
	    if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE) {
	        if (p->eeprom_index == EEPROM_TUNE_INDEX) {
	            for (size_t i = 0; i < EEPROM_TUNE_MAX_LEN; i++) {
	                if (i < req.value.string_value.len) {
	                    eepromBuffer[EEPROM_TUNE_INDEX + i] = req.value.string_value.data[i];
	                }  else {
	                    eepromBuffer[EEPROM_TUNE_INDEX + i] = 0xFF;
	                }
	            }
	        }
	    }
            break;
	default:
            return;
	}

        if (last_dir_reversed != dir_reversed) {
            // make dir_reversed change work without reboot
            forward = 1 - dir_reversed;
            running = 0;
            set_input(0);
        }
    }

    /*
      for both set and get we reply with the current value
    */
    struct uavcan_protocol_param_GetSetResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    if (p != NULL) {
	switch (p->vtype) {
	case T_UINT8:
	    pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            pkt.value.integer_value = *(uint8_t *)p->ptr;
            pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            pkt.default_value.integer_value = p->default_value;
            pkt.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
            pkt.max_value.integer_value = p->max_value;
            pkt.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
            pkt.min_value.integer_value = p->min_value;
            break;
	case T_UINT16:
	    pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
	    pkt.value.integer_value = *(uint16_t *)p->ptr;
            pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            pkt.default_value.integer_value = p->default_value;
            pkt.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
            pkt.max_value.integer_value = p->max_value;
            pkt.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
            pkt.min_value.integer_value = p->min_value;
            break;
	case T_STRING:
	    pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;
	    if (p->eeprom_index == EEPROM_TUNE_INDEX) {
	        pkt.value.string_value.len = EEPROM_TUNE_MAX_LEN;
	        for (size_t i=0; i < EEPROM_TUNE_MAX_LEN; i++) {
	            pkt.value.string_value.data[i] = eepromBuffer[EEPROM_TUNE_INDEX + i];
	        }
	    }
            break;
	case T_BOOL:
	    pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
	    pkt.value.boolean_value = (*(uint8_t *)p->ptr)?true:false;
            pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
            pkt.default_value.boolean_value = p->default_value;
            break;
	default:
            return;
        }
        pkt.name.len = strlen(p->name);
        strcpy((char *)pkt.name.data, p->name);
    }

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle parameter executeopcode request
*/
static void handle_param_ExecuteOpcode(CanardInstance* ins, CanardRxTransfer* transfer)
{
    struct uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req)) {
        return;
    }
    struct uavcan_protocol_param_ExecuteOpcodeResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.ok = false;

    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        if (!safe_to_write_settings()) {
	    can_printf("No erase while running");
	} else {
	    can_printf("resetting to defaults");
	    memset(eepromBuffer, 0xff, sizeof(eepromBuffer));
	    memcpy(eepromBuffer, default_settings, sizeof(default_settings));
	    save_flash_nolib(eepromBuffer, sizeof(eepromBuffer), eeprom_address);
            loadEEpromSettings();
            load_settings();
	    pkt.ok = true;
	}
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
        if (!safe_to_write_settings()) {
	    can_printf("No save while running");
	} else {
	    save_settings();
	    pkt.ok = true;
	}
    }


    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle RestartNode request
*/
static void handle_RestartNode(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // reboot the ESC
    allOff();
    set_rtc_backup_register(0, 0);
    NVIC_SystemReset();
}

/*
  handle a GetNodeInfo request
*/
static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;

    memset(&pkt, 0, sizeof(pkt));

    node_status.uptime_sec = micros64() / 1000000ULL;
    pkt.status = node_status;

    // fill in your major and minor firmware version
    pkt.software_version.major = VERSION_MAJOR;
    pkt.software_version.minor = VERSION_MINOR;
    pkt.software_version.optional_field_flags = 0;
    pkt.software_version.vcs_commit = 0; // should put git hash in here

    // should fill in hardware version
    pkt.hardware_version.major = 2;
    pkt.hardware_version.minor = 3;

    sys_can_getUniqueID(pkt.hardware_version.unique_id);

    strncpy((char*)pkt.name.data, FIRMWARE_NAME, sizeof(pkt.name.data));
    pkt.name.len = strnlen((char*)pkt.name.data, sizeof(pkt.name.data));

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

extern void transfercomplete();
extern void setInput();


/*
  process throttle input from DroneCAN
 */
static void set_input(uint16_t input)
{
    if (!armed && input != 0 && settings.require_arming &&
        dronecan_armed && !settings.require_zero_throttle) {
        // allow restart if unexpected ESC reboot in flight
        armed = 1;
    }

    const uint16_t unfiltered_input = (dronecan_armed || !settings.require_arming)? input : 0;
    const uint16_t filtered_input = Filter2P_apply(unfiltered_input, settings.filter_hz, 1000);

    newinput = filtered_input;
    last_can_input = unfiltered_input;
    inputSet = 1;
    transfercomplete();
    setInput();

    canstats.num_input++;
}

/*
  handle a ESC RawCommand request
*/
static void handle_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer)
{
    struct uavcan_equipment_esc_RawCommand cmd;
    if (uavcan_equipment_esc_RawCommand_decode(transfer, &cmd)) {
        return;
    }
    // see if it is for us
    if (cmd.cmd.len <= settings.esc_index) {
        return;
    }

    // throttle demand is a value from -8191 to 8191. Negative values
    // are for reverse throttle
    const int16_t input_can = cmd.cmd.data[(unsigned)settings.esc_index];

    /*
      we need to map onto the AM32 expected range, which is a 11 bit number, where:
      0: off
      1-46: special codes
      47-2047: throttle
    */
    uint16_t this_input = 0;
    if (input_can == 0) {
        this_input = 0;
    } else if (bi_direction) {
        const float scaled_value = input_can * (1000.0 / 8192);
        if (scaled_value >= 0) {
            this_input = (uint16_t)(47 + scaled_value);
        } else {
            this_input = (uint16_t)(47 + (1000 - scaled_value));
        }
    } else if (input_can > 0) {
        const float scaled_value = input_can * (2000.0 / 8192);
        this_input = (uint16_t)(47 + scaled_value);
    }

    const uint64_t ts = micros64();
    canstats.num_commands++;
    canstats.total_commands++;
    canstats.last_raw_command_us = ts;

    set_input(this_input);
}

/*
  handle ArmingStatus messages
*/
static void handle_ArmingStatus(CanardInstance *ins, CanardRxTransfer *transfer)
{
    struct uavcan_equipment_safety_ArmingStatus cmd;
    if (uavcan_equipment_safety_ArmingStatus_decode(transfer, &cmd)) {
        return;
    }

    dronecan_armed = (cmd.status == UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_FULLY_ARMED);
    if (!dronecan_armed && settings.require_arming && canstats.last_raw_command_us != 0) {
        set_input(0);
    }
}

/*
  handle a BeginFirmwareUpdate request from a management tool like
  DroneCAN GUI tool or MissionPlanner
 */
static void handle_begin_firmware_update(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if (!safe_to_write_settings()) {
        can_printf("No update while running");
        return;
    }

    struct uavcan_protocol_file_BeginFirmwareUpdateRequest req;
    if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req)) {
        return;
    }

    sys_can_disable_IRQ();

    uint32_t reg[2] = {0,0};
    if (req.image_file_remote_path.path.len <= 8) {
        // path is normally hashed and fits in 8 bytes, put in rtc backup registers 1 and 2
        memcpy((uint8_t *)reg, req.image_file_remote_path.path.data, req.image_file_remote_path.path.len);

        uint8_t buffer[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
        struct uavcan_protocol_file_BeginFirmwareUpdateResponse reply;
        memset(&reply, 0, sizeof(reply));
        reply.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;

        uint32_t total_size = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&reply, buffer);

        canardRequestOrRespond(ins,
            transfer->source_node_id,
            UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
            UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
            &transfer->transfer_id,
            transfer->priority,
            CanardResponse,
            &buffer[0],
            total_size);

        while (canardPeekTxQueue(&canard) != NULL) {
            DroneCAN_processTxQueue();
        }

        // time to transmit
        delayMillis(2);
    }

    set_rtc_backup_register(1, reg[0]);
    set_rtc_backup_register(2, reg[1]);

    // tell the bootloader we are doing fw update
    set_rtc_backup_register(0,
        (canardGetLocalNodeID(&canard)<<24) |
        (transfer->source_node_id<<16) |
        RTC_BKUP0_FWUPDATE);

    // reboot and let bootloader handle the request, this means the
    // first request doesn't get a reply, and the client re-sends. We
    // need this to get the path to the client. We could instead
    // define a memory block which is not reset on boot and put the
    // path there, but this is simpler
    NVIC_SystemReset();
}

/*
  data for dynamic node allocation process
*/
static struct {
    uint32_t send_next_node_id_allocation_request_at_ms;
    uint32_t node_id_allocation_unique_id_offset;
} DNA;

/*
  handle a DNA allocation packet
*/
static void handle_DNA_Allocation(CanardInstance *ins, CanardRxTransfer *transfer)
{
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) {
        // already allocated
        return;
    }

    // Rule C - updating the randomized time interval
    DNA.send_next_node_id_allocation_request_at_ms =
        millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
	(get_random16() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
	DNA.node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Copying the unique ID from the message
    struct uavcan_protocol_dynamic_node_id_Allocation msg;

    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg)) {
	/* bad packet */
	return;
    }

    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    sys_can_getUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
	DNA.node_id_allocation_unique_id_offset = 0;
        // No match, return
        return;
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        // The allocator has confirmed part of unique ID, switching to
        // the next stage and updating the timeout.
        DNA.node_id_allocation_unique_id_offset = msg.unique_id.len;
        DNA.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

    } else {
        // Allocation complete - copying the allocated node ID from the message
        canardSetLocalNodeID(ins, msg.node_id);
    }
}

/*
  ask for a dynamic node allocation
*/
static void request_DNA()
{
    const uint32_t now = millis32();
    static uint8_t node_id_allocation_transfer_id = 0;

    DNA.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
	(get_random16() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(PREFERRED_NODE_ID << 1U);

    if (DNA.node_id_allocation_unique_id_offset == 0) {
        allocation_request[0] |= 1;     // First part of unique ID
    }

    uint8_t my_unique_id[16];
    sys_can_getUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(16 - DNA.node_id_allocation_unique_id_offset);
    
    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request[1], &my_unique_id[DNA.node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    canardBroadcast(&canard,
		    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
		    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
		    &node_id_allocation_transfer_id,
		    CANARD_TRANSFER_PRIORITY_LOW,
		    &allocation_request[0],
		    (uint16_t) (uid_size + 1));

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    DNA.node_id_allocation_unique_id_offset = 0;
}

/*
  This callback is invoked by the library when a new message or request or response is received.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    // tell main loop we have had signal so we don't reset
    signaltimeout = 0;

    canstats.on_receive++;
    // switch on data type ID to pass to the right handler function
    if (transfer->transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (transfer->data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
            handle_GetNodeInfo(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
            handle_param_GetSet(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
            handle_param_ExecuteOpcode(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
            handle_RestartNode(ins, transfer);
            break;
        }
	case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID: {
	    handle_begin_firmware_update(ins, transfer);
	    break;
	}
	}
    }
    if (transfer->transfer_type == CanardTransferTypeBroadcast) {
        // check if we want to handle a specific broadcast message
        switch (transfer->data_type_id) {
        case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID: {
            handle_RawCommand(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
            handle_DNA_Allocation(ins, transfer);
            break;
        }
	case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID: {
	    handle_ArmingStatus(ins, transfer);
            break;
        }
        }
    }
}


/*
  This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
  by the local node.
  If the callback returns true, the library will receive the transfer.
  If the callback returns false, the library will ignore the transfer.
  All transfers that are addressed to other nodes are always ignored.

  This function must fill in the out_data_type_signature to be the signature of the message.
*/
static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    canstats.should_accept++;
    if (transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
            return true;
        }
	case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID: {
	    *out_data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
	    return true;
	}
	}
    }
    if (transfer_type == CanardTransferTypeBroadcast) {
        // see if we want to handle a specific broadcast packet
        switch (data_type_id) {
        case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID: {
            *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }
	case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID: {
	    *out_data_type_signature = UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE;
            return true;
        }
        }
    }
    // we don't want any other messages
    return false;
}

/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
*/
static void send_NodeStatus(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    node_status.uptime_sec = micros64() / 1000000ULL;
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;

    // put number of commands we have received in vendor status since the last NodeStatus
    // this means vendor status gives us approximate command rate in commands/second
    node_status.vendor_specific_status_code = canstats.num_commands;
    canstats.num_commands = 0;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

/*
  This function is called at 1 Hz rate from the main loop.
*/
static void process1HzTasks(uint64_t timestamp_usec)
{
    /*
      Purge transfers that are no longer transmitted. This can free up some memory
    */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
      Transmit the node status message
    */
    send_NodeStatus();
}

/*
  send ESC status at TELEM_RATE Hz
*/
static void send_ESCStatus(void)
{
    struct uavcan_equipment_esc_Status pkt;
    memset(&pkt, 0, sizeof(pkt));
    uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];

    // make up some synthetic status data
    pkt.error_count = 0;
    pkt.voltage = battery_voltage * 0.01;

    pkt.current = (current.sum/(float)current.count) * 0.01;
    current.sum = 0;
    current.count = 0;

    pkt.temperature = C_TO_KELVIN(degrees_celsius);
    pkt.rpm = (e_rpm * 100) / ((uint8_t)motor_poles);
    pkt.power_rating_pct = 0; // how do we get this?
    pkt.esc_index = settings.esc_index;

    uint32_t len = uavcan_equipment_esc_Status_encode(&pkt, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}


/*
  receive one frame, only called from interrupt context
*/
void DroneCAN_receiveFrame(void)
{
    CanardCANFrame rx_frame = {0};
    while (sys_can_receive(&rx_frame) > 0) {
	canstats.num_receive++;
	int ecode = canardHandleRxFrame(&canard, &rx_frame, micros64());
	if (ecode != CANARD_OK) {
	    canstats.rx_ecode = ecode;
	    canstats.rxframe_error++;
	}
    }
}

/*
  Transmits all frames from the TX queue
*/
void DroneCAN_processTxQueue(void)
{
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        const int16_t tx_res = sys_can_transmit(txf);
        if (tx_res == 0) {
            // no space, stop trying
            break;
        }
        // success or error, remove frame
        canardPopTxQueue(&canard);
    }
}

static void DroneCAN_Startup(void)
{
    load_settings();

    canardInit(&canard,
	       canard_memory_pool,              // Raw memory chunk used for dynamic allocation
               sizeof(canard_memory_pool),
	       onTransferReceived,                // Callback, see CanardOnTransferReception
	       shouldAcceptTransfer,              // Callback, see CanardShouldAcceptTransfer
	       NULL);

    if (settings.can_node != 0) {
        canardSetLocalNodeID(&canard, settings.can_node);
    }

    // initialise low level CAN peripheral hardware
    sys_can_init();
}

void DroneCAN_update()
{
    sys_can_disable_IRQ();

    static uint64_t next_1hz_service_at;
    static uint64_t next_telem_service_at;
    if (!done_startup) {
        DroneCAN_Startup();
        done_startup = true;
        set_rtc_backup_register(0, RTC_BKUP0_BOOTED);
    }

    if (canstats.on_receive == 5) {
        // indicate to bootloader that we were fully operational
        set_rtc_backup_register(0, RTC_BKUP0_SIGNAL);
    }

    DroneCAN_processTxQueue();

    // see if we are still doing DNA
    if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID) {
	// we're still waiting for a DNA allocation of our node ID
	if (millis32() > DNA.send_next_node_id_allocation_request_at_ms) {
	    request_DNA();
	}
        sys_can_enable_IRQ();
	return;
    }

    const uint64_t ts = micros64();

    if (ts >= next_1hz_service_at) {
	next_1hz_service_at += 1000000ULL;
	process1HzTasks(ts);
    }
    if (ts >= next_telem_service_at) {
        next_telem_service_at += 1000000ULL/settings.telem_rate;
	send_ESCStatus();
    }

    DroneCAN_processTxQueue();

    if (canstats.last_raw_command_us != 0 && ts - canstats.last_raw_command_us > 250000ULL) {
        /*
          we have stopped getting CAN RawCommand, zero input
         */
        canstats.last_raw_command_us = 0;
        set_input(0);
    }
    if (ts - canstats.last_raw_command_us > TARGET_PERIOD_US) {
        // ensure at least 1kHz signal is seen by main code
        set_input(last_can_input);
        canstats.last_raw_command_us = ts;
    }

    sys_can_enable_IRQ();

    // keep summed current for averaging
    current.sum += actual_current;
    current.count++;
}

bool DroneCAN_active(void)
{
    return canstats.total_commands != 0;
}

#endif // DRONECAN_SUPPORT
