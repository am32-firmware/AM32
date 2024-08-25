/*
 * DroneCAN.c - support for DroneCAN protocol for ESC control and telemetry
 */

#include "targets.h"

//#pragma GCC optimize("O0")

#if DRONECAN_SUPPORT

#include "peripherals.h"
#include "serial_telemetry.h"
#include <canard_stm32.h>
#include <_internal_bxcan.h>
#include <common.h>
#include <signal.h>
#include <version.h>
#include <eeprom.h>
#include <stdarg.h>
#include <stdio.h>

// include the headers for the generated DroneCAN messages from the
// dronecan_dsdlc compiler
#include "dsdl_generated/dronecan_msgs.h"

#ifndef PREFERRED_NODE_ID
#define PREFERRED_NODE_ID 0
#endif

#ifndef CANARD_POOL_SIZE
#define CANARD_POOL_SIZE 4096
#endif

#define BXCAN CANARD_STM32_CAN1

static CanardInstance canard;
static uint8_t canard_memory_pool[CANARD_POOL_SIZE];

#define NUM_TX_MAILBOXES 3

struct {
    uint32_t num_commands;
    uint32_t num_receive;
    uint32_t num_tx_interrupts;
    uint32_t num_rx_interrupts;
    uint32_t rx_errors;
    uint32_t esr;
    uint32_t rxframe_error;
    int32_t rx_ecode;
    uint32_t should_accept;
    uint32_t on_receive;
} canstats, last_canstats;

/*
  keep the state for firmware update
*/
static struct {
    char path[256];
    uint8_t node_id;
    uint8_t transfer_id;
    uint32_t last_read_ms;
    uint32_t offset;
} fwupdate;

static bool dronecan_armed;

// _flash_update points to the flash space after eeprom, usually 0x08010000
extern uint8_t _flash_update[1];

/*
  state of user settings. This will be saved in settings.dat. On a
  real device a better storage system will be needed
  For simplicity we store all parameters as floats in this example
*/
static struct
{
    uint8_t can_node;
    uint8_t esc_index;
} settings;

enum VarType {
    T_BOOL = 0,
    T_UINT8,
};

static void can_printf(const char *fmt, ...);
static void processTxQueue(void);

// some convenience macros
#define MIN(a,b) ((a)<(b)?(a):(b))
#define C_TO_KELVIN(temp) (temp + 273.15f)
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

/*
  macros to prevent corruption of canard state from processing an IRQ while in main update code
 */
#define DISABLE_CAN_IRQ() do { NVIC_DisableIRQ(CAN1_RX0_IRQn); NVIC_DisableIRQ(CAN1_RX1_IRQn); NVIC_DisableIRQ(CAN1_TX_IRQn); } while(0)
#define RESTORE_CAN_IRQ() do { NVIC_EnableIRQ(CAN1_RX0_IRQn); NVIC_EnableIRQ(CAN1_RX1_IRQn); NVIC_EnableIRQ(CAN1_TX_IRQn); } while(0)

/*
  access to settings from main.c
 */
extern char dir_reversed;
extern char motor_poles;
extern char VARIABLE_PWM;
extern char use_sin_start;
extern char comp_pwm;
extern char stuck_rotor_protection;

extern void saveEEpromSettings(void);
extern void loadEEpromSettings(void);


/*
  the set of parameters to present to the user over DroneCAN
*/
static struct parameter {
    char *name;
    enum VarType vtype;
    uint16_t min_value;
    uint16_t max_value;
    void *ptr;
    uint8_t eeprom_index;
} parameters[] = {
    // list of settable parameters
    { "CAN_NODE",               T_UINT8, 0, 127, &settings.can_node, 176},
    { "ESC_INDEX",              T_UINT8, 0, 32,  &settings.esc_index, 177},
    { "DIR_REVERSED",           T_BOOL,  0, 1,   &dir_reversed},
    { "MOTOR_POLES",            T_UINT8, 0, 64,  &motor_poles, 27 },
    { "VARIABLE_PWM",           T_BOOL,  0, 1,   &VARIABLE_PWM},
    { "USE_SIN_START",          T_BOOL,  0, 1,   &use_sin_start},
    { "COMP_PWM",               T_BOOL,  0, 1,   &comp_pwm},
    { "STUCK_ROTOR_PROTECTION", T_BOOL,  0, 1,   &stuck_rotor_protection },
};

/*
  get settings from eeprom
*/
static void load_settings(void)
{
    if (eepromBuffer[176] <= 127) {
	settings.can_node = eepromBuffer[176];
    }
    if (eepromBuffer[177] <= 32) {
	settings.esc_index = eepromBuffer[177];
    }
}

/*
  save settings to flash
 */
static void save_settings(void)
{
    eepromBuffer[176] = settings.can_node;
    eepromBuffer[177] = settings.esc_index;
    saveEEpromSettings();
    can_printf("saved settings");
}

/*
  hold our node status as a static variable. It will be updated on any errors
*/
static struct uavcan_protocol_NodeStatus node_status;

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
    uint16_t cnt = UTILITY_TIMER->CNT;
    if (cnt < last_cnt) {
	base_us += 0x10000;
    }
    last_cnt = cnt;
    return base_us + cnt;
}

void usleep(uint32_t usec)
{
    const uint64_t t0 = micros64();
    while (micros64() - t0 < usec) ;
}


/*
  get monotonic time in milliseconds since startup
*/
static uint32_t millis32(void)
{
    return micros64() / 1000ULL;
}

/*
  get a 16 byte unique ID for this node, this should be based on the CPU unique ID or other unique ID
*/
static void getUniqueID(uint8_t id[16])
{
    const uint8_t *uidbase = (const uint8_t *)UID_BASE;
    memcpy(id, uidbase, 12);

    // put CPU ID in last 4 bytes, handy for knowing the exact MCU we are on
    const uint32_t cpuid = SCB->CPUID;
    memcpy(&id[12], &cpuid, 4);
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
	case T_BOOL:
	    *(uint8_t *)p->ptr = req.value.boolean_value?1:0;
	    if (p->eeprom_index != 0) {
		eepromBuffer[p->eeprom_index] = *(uint8_t *)p->ptr;
	    }
            break;
	default:
            return;
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
            break;
	case T_BOOL:
	    pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
	    pkt.value.boolean_value = (*(uint8_t *)p->ptr)?true:false;
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
	if (running) {
	    can_printf("No erase while running");
	} else {
	    can_printf("resetting to defaults");
	    memset(eepromBuffer, 0xff, sizeof(eepromBuffer));
	    memcpy(eepromBuffer, default_settings, sizeof(default_settings));
	    save_flash_nolib(eepromBuffer, sizeof(eepromBuffer), eeprom_address);
	    loadEEpromSettings();
	    pkt.ok = true;
	}
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
	if (running) {
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
    // the ESC should reboot now!
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

    getUniqueID(pkt.hardware_version.unique_id);

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
    newinput = dronecan_armed? input : 0;
    inputSet = 1;
    transfercomplete();
    setInput();
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

      we will ignore reverse throttle for now. The main code expects
      throttle demands in newinput global
    */
    uint16_t this_input;
    if (input_can <= 0) {
	this_input = 0;
    } else {
	this_input = (uint16_t)(47 + input_can * (2000.0 / 8192));
    }

    set_input(this_input);

    canstats.num_commands++;
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
    if (!dronecan_armed) {
	set_input(0);
    }
}

/*
  handle a BeginFirmwareUpdate request from a management tool like
  DroneCAN GUI tool or MissionPlanner
 */
static void handle_begin_firmware_update(CanardInstance* ins, CanardRxTransfer* transfer)
{
    /*
      decode the request
     */
    struct uavcan_protocol_file_BeginFirmwareUpdateRequest req;
    if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req)) {
        return;
    }

    /*
      check for a repeated BeginFirmwareUpdateRequest
     */
    if (fwupdate.node_id == transfer->source_node_id &&
	memcmp(fwupdate.path, req.image_file_remote_path.path.data, req.image_file_remote_path.path.len) == 0) {
	/* ignore duplicate request */
	return;
    }

    if (running) {
	can_printf("No firmware update while running");
	return;
    }


    fwupdate.offset = 0;
    fwupdate.node_id = transfer->source_node_id;
    strncpy(fwupdate.path, (char*)req.image_file_remote_path.path.data, req.image_file_remote_path.path.len);

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

    can_printf("Started firmware update\n");
}

/*
  send a read for a firmware update. This asks the client (firmware
  server) for a piece of the new firmware
 */
static void send_firmware_read(void)
{
    uint32_t now = millis32();
    if (fwupdate.last_read_ms != 0 && now - fwupdate.last_read_ms < 750) {
        // the server may still be responding
        return;
    }
    fwupdate.last_read_ms = now;

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_READ_REQUEST_MAX_SIZE];

    struct uavcan_protocol_file_ReadRequest pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.path.path.len = strlen((const char *)fwupdate.path);
    pkt.offset = fwupdate.offset;
    memcpy(pkt.path.path.data, fwupdate.path, pkt.path.path.len);

    uint16_t total_size = uavcan_protocol_file_ReadRequest_encode(&pkt, buffer);

    canardRequestOrRespond(&canard,
			   fwupdate.node_id,
                           UAVCAN_PROTOCOL_FILE_READ_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_READ_ID,
			   &fwupdate.transfer_id,
                           CANARD_TRANSFER_PRIORITY_HIGH,
                           CanardRequest,
                           &buffer[0],
                           total_size);
}

/*
  handle response to send_firmware_read()
 */
static void handle_file_read_response(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if (running) {
	// cancel if user starts a motor
	fwupdate.node_id = 0;
	return;
    }

    if ((transfer->transfer_id+1)%32 != fwupdate.transfer_id ||
	transfer->source_node_id != fwupdate.node_id) {
	/* not for us */
	can_printf("Firmware update: not for us id=%u/%u\n", (unsigned)transfer->transfer_id, (unsigned)fwupdate.transfer_id);
	return;
    }
    struct uavcan_protocol_file_ReadResponse pkt;
    if (uavcan_protocol_file_ReadResponse_decode(transfer, &pkt)) {
	/* bad packet */
	can_printf("Firmware update: bad packet\n");
	return;
    }
    if (pkt.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK) {
	/* read failed */
	fwupdate.node_id = 0;
	can_printf("Firmware update read failure\n");
	return;
    }

    uint32_t len = pkt.data.len;
    len = (len+7U) & ~7U;
    save_flash_nolib(pkt.data.data, len, (uint32_t)&_flash_update[fwupdate.offset]);

    fwupdate.offset += pkt.data.len;

    if (pkt.data.len < 256) {
	/* firmware updare done */
	can_printf("Firmwate update complete\n");
	fwupdate.node_id = 0;
	flash_upgrade(&_flash_update[0], fwupdate.offset); // this jumps to new firmware
	return;
    }

    /* trigger a new read */
    fwupdate.last_read_ms = 0;
    send_firmware_read();
    processTxQueue();
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
    getUniqueID(my_unique_id);

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
    getUniqueID(my_unique_id);

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
    if (transfer->transfer_type == CanardTransferTypeResponse) {
	switch (transfer->data_type_id) {
	case UAVCAN_PROTOCOL_FILE_READ_ID:
	    handle_file_read_response(ins, transfer);
	    break;
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
    if (transfer_type == CanardTransferTypeResponse) {
        // check if we want to handle a specific service request
        switch (data_type_id) {
	case UAVCAN_PROTOCOL_FILE_READ_ID:
	    *out_data_type_signature = UAVCAN_PROTOCOL_FILE_READ_SIGNATURE;
	    return true;
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
    last_canstats = canstats;
    node_status.vendor_specific_status_code = canstats.num_commands;
    memset(&canstats, 0, sizeof(canstats));

    /*
      when doing a firmware update put the size in kbytes in VSSC so
      the user can see how far it has reached
    */
    if (fwupdate.node_id != 0) {
	node_status.vendor_specific_status_code = fwupdate.offset / 1024;
	node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE;
    }

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
  send ESC status at 50Hz
*/
static void send_ESCStatus(void)
{
    struct uavcan_equipment_esc_Status pkt;
    memset(&pkt, 0, sizeof(pkt));
    uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];

    // make up some synthetic status data
    pkt.error_count = 0;
    pkt.voltage = battery_voltage * 0.01;
    pkt.current = actual_current * 0.02;
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
static void receiveFrame(void)
{
    CanardCANFrame rx_frame = {0};
    while (canardSTM32Receive(&rx_frame) > 0) {
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
static void processTxQueue(void)
{
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
	const int16_t tx_res = canardSTM32Transmit(txf);
	if (tx_res != 0) {  // no timeout,  drop the frame
	    canardPopTxQueue(&canard);
	}
    }
}

/* CAN init function */
void DroneCAN_Init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = 9; // AF9==CAN
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void DroneCAN_Startup(void)
{
    load_settings();

    LL_RCC_ClocksTypeDef Clocks;
    LL_RCC_GetSystemClocksFreq(&Clocks);

    CanardSTM32CANTimings timings;
    canardSTM32ComputeCANTimings(Clocks.PCLK1_Frequency, 1000000, &timings);

    canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);

    canardInit(&canard,                         // Uninitialized library instance
	       canard_memory_pool,              // Raw memory chunk used for dynamic allocation
	       sizeof(canard_memory_pool),      // Size of the above, in bytes
	       onTransferReceived,                // Callback, see CanardOnTransferReception
	       shouldAcceptTransfer,              // Callback, see CanardShouldAcceptTransfer
	       NULL);

    canardSetLocalNodeID(&canard, settings.can_node);

    /*
      enable interrupt for CAN receive and transmit
    */
    NVIC_SetPriority(CAN1_RX0_IRQn, 5);
    NVIC_SetPriority(CAN1_RX1_IRQn, 5);
    NVIC_SetPriority(CAN1_TX_IRQn, 5);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_RX1_IRQn);
    NVIC_EnableIRQ(CAN1_TX_IRQn);
    BXCAN->IER = CANARD_STM32_CAN_IER_FMPIE0 | CANARD_STM32_CAN_IER_FMPIE1 | CANARD_STM32_CAN_IER_TMEIE;
}

void DroneCAN_update()
{
    static uint64_t next_1hz_service_at;
    static uint64_t next_50hz_service_at;
    static bool done_startup;
    if (!done_startup) {
	done_startup = true;
	DroneCAN_Startup();
    }

    DISABLE_CAN_IRQ();

    processTxQueue();

    // see if we are still doing DNA
    if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID) {
	// we're still waiting for a DNA allocation of our node ID
	if (millis32() > DNA.send_next_node_id_allocation_request_at_ms) {
	    request_DNA();
	}
	RESTORE_CAN_IRQ();
	return;
    }

    const uint64_t ts = micros64();

    if (ts >= next_1hz_service_at) {
	next_1hz_service_at += 1000000ULL;
	process1HzTasks(ts);
    }
    if (ts >= next_50hz_service_at && fwupdate.node_id == 0) {
	next_50hz_service_at += 1000000ULL/50U;
	send_ESCStatus();
    }

    if (fwupdate.node_id != 0) {
	send_firmware_read();
    }

    processTxQueue();

    RESTORE_CAN_IRQ();
}

static void handleTxMailboxInterrupt(uint8_t mbox, bool txok)
{
    processTxQueue();
}

static void pollErrorFlagsFromISR()
{
    const uint8_t lec = (uint8_t)((BXCAN->ESR & CANARD_STM32_CAN_ESR_LEC_MASK) >> CANARD_STM32_CAN_ESR_LEC_SHIFT);
    if (lec != 0) {
	canstats.esr = BXCAN->ESR; // Record error status
	BXCAN->ESR = 0;
    }
}

static void handleTxInterrupt(void)
{
    canstats.num_tx_interrupts++;

    // TXOK == false means that there was a hardware failure
    if (BXCAN->TSR & CANARD_STM32_CAN_TSR_RQCP0) {
        const bool txok = BXCAN->TSR & CANARD_STM32_CAN_TSR_TXOK0;
        BXCAN->TSR = CANARD_STM32_CAN_TSR_RQCP0;
	handleTxMailboxInterrupt(0, txok);
    }
    if (BXCAN->TSR & CANARD_STM32_CAN_TSR_RQCP1) {
        const bool txok = BXCAN->TSR & CANARD_STM32_CAN_TSR_TXOK1;
        BXCAN->TSR = CANARD_STM32_CAN_TSR_RQCP1;
	handleTxMailboxInterrupt(1, txok);
    }
    if (BXCAN->TSR & CANARD_STM32_CAN_TSR_RQCP2) {
        const bool txok = BXCAN->TSR & CANARD_STM32_CAN_TSR_TXOK2;
        BXCAN->TSR = CANARD_STM32_CAN_TSR_RQCP2;
	handleTxMailboxInterrupt(2, txok);
    }

    pollErrorFlagsFromISR();
}

static void handleRxInterrupt(uint8_t fifo_index)
{
    canstats.num_rx_interrupts++;

    volatile uint32_t* const rfr_reg = (fifo_index == 0) ? &BXCAN->RF0R : &BXCAN->RF1R;
    if ((*rfr_reg & CANARD_STM32_CAN_RFR_FMP_MASK) == 0) {
        return;
    }

    /*
     * Register overflow as a hardware error
     */
    if ((*rfr_reg & CANARD_STM32_CAN_RFR_FOVR) != 0) {
	canstats.rx_errors++;
    }

    receiveFrame();

    pollErrorFlagsFromISR();
}

/*
  interrupt handlers for CAN1
*/
void CAN1_RX0_IRQHandler(void)
{
    handleRxInterrupt(0);
}

void CAN1_RX1_IRQHandler(void)
{
    handleRxInterrupt(1);
}

void CAN1_TX_IRQHandler(void)
{
    handleTxInterrupt();
}

#endif // DRONECAN_SUPPORT
