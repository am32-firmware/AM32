/*
 * DroneCAN.c - support for DroneCAN protocol for ESC control and telemetry
 */

#if DRONECAN_SUPPORT

#include "peripherals.h"
#include "targets.h"
#include "serial_telemetry.h"
#include <canard_stm32.h>
#include <stm32l4xx_hal.h>
#include <common.h>

// include the headers for the generated DroneCAN messages from the
// dronecan_dsdlc compiler
#include <dronecan_msgs.h>

static CAN_HandleTypeDef hcan;

#ifndef PREFERRED_NODE_ID
#define PREFERRED_NODE_ID 77
#endif

#ifndef CANARD_POOL_SIZE
#define CANARD_POOL_SIZE 4096
#endif

static CanardInstance canard;
static uint8_t canard_memory_pool[CANARD_POOL_SIZE];

static struct {
    uint32_t num_commands;
} stats;

/*
  keep the state for firmware update
 */
static struct {
    char path[256];
    uint8_t node_id;
    uint8_t transfer_id;
    uint32_t last_read_ms;
    int fd;
    uint32_t offset;
} fwupdate;

/*
  state of user settings. This will be saved in settings.dat. On a
  real device a better storage system will be needed
  For simplicity we store all parameters as floats in this example
 */
static struct
{
    float can_node;
    float esc_index;
    float direction;
} settings;

/*
  a set of parameters to present to the user. In this example we don't
  actually save parameters, this is just to show how to handle the
  parameter protocol
 */
static struct parameter {
    char *name;
    enum uavcan_protocol_param_Value_type_t type;
    float *value;
    float min_value;
    float max_value;
} parameters[] = {
    // add any parameters you want users to be able to set
    { "CAN_NODE", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &settings.can_node, 0, 127 }, // CAN node ID
    { "ESC_INDEX", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &settings.esc_index, 0, 32 }, // index in RawCommand
    { "DIRECTION", UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE, &settings.direction, 0, 1 }, // spin direction
};

// some convenience macros
#define MIN(a,b) ((a)<(b)?(a):(b))
#define C_TO_KELVIN(temp) (temp + 273.15f)
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

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
  save all settings
 */
static void save_settings(void)
{
    // NOP for now
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
          this is a parameter set command. The implementation can
          either choose to store the value in a persistent manner
          immediately or can instead store it in memory and save to permanent storage on a
         */
        switch (p->type) {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            *p->value = req.value.integer_value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE:
            *p->value = req.value.boolean_value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
            *p->value = req.value.real_value;
            break;
        default:
            return;
        }
	save_settings();
    }

    /*
      for both set and get we reply with the current value
     */
    struct uavcan_protocol_param_GetSetResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    if (p != NULL) {
        pkt.value.union_tag = p->type;
        switch (p->type) {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            pkt.value.integer_value = *p->value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE:
            pkt.value.integer_value = *p->value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
            pkt.value.real_value = *p->value;
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
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        // here is where you would reset all parameters to defaults
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
        // here is where you would save all the changed parameters to permanent storage
    }

    struct uavcan_protocol_param_ExecuteOpcodeResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.ok = true;

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

    strncpy((char*)pkt.name.data, "AM32 ESC", sizeof(pkt.name.data));
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

    // tell main loop we have had signal so we don't reset
    signaltimeout = 0;
    
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
    if (input_can <= 0) {
	newinput = 0;
    } else {
	newinput = (uint16_t)(47 + input_can * (2000.0 / 8192));
    }

    stats.num_commands++;
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
    node_status.vendor_specific_status_code = stats.num_commands;
    stats.num_commands = 0;

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
    pkt.rpm = e_rpm * 100;
    pkt.power_rating_pct = 0; // how do we get this?

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
	CanardCANFrame rx_frame;
	while (canardSTM32Receive(&rx_frame) > 0) {
		canardHandleRxFrame(&canard, &rx_frame, micros64());
	}
}

/*
  Transmits all frames from the TX queue
*/
static void processTxQueue(void)
{
	// Transmitting, disable interrupts to prevent being called from IRQ and main
	HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);

	for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
		const int16_t tx_res = canardSTM32Transmit(txf);
		if (tx_res != 0) {  // no timeout,  drop the frame
			canardPopTxQueue(&canard);
		}
	}

	// re-enable interrupts
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (canHandle->Instance==CAN) {
		__HAL_RCC_CAN1_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		// CAN1 on PB8/PB9
		GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = 9; // AF9
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
}

/* CAN init function */
void DroneCAN_Init(void)
{
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 16;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	HAL_CAN_Init(&hcan);
}

static void DroneCAN_Startup(void)
{
	CanardSTM32CANTimings timings;
	int result = canardSTM32ComputeCANTimings(HAL_RCC_GetPCLK1Freq(), 1000000, &timings);
	if (result) {
		__ASM volatile("BKPT #01");
	}
	canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);
	result = canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);
	if (result) {
		__ASM volatile("BKPT #01");
	}

	canardInit(&canard,                         // Uninitialized library instance
		   canard_memory_pool,              // Raw memory chunk used for dynamic allocation
		   sizeof(canard_memory_pool),      // Size of the above, in bytes
		   onTransferReceived,                // Callback, see CanardOnTransferReception
		   shouldAcceptTransfer,              // Callback, see CanardShouldAcceptTransfer
		   NULL);

	canardSetLocalNodeID(&canard, PREFERRED_NODE_ID);

	/*
	  enable interrupt for CAN receive
	 */
        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
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

	processTxQueue();

	// see if we are still doing DNA
	if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID) {
		// we're still waiting for a DNA allocation of our node ID
		if (millis32() > DNA.send_next_node_id_allocation_request_at_ms) {
			request_DNA();
		}
		return;
	}

	const uint64_t ts = micros64();

        if (ts >= next_1hz_service_at) {
            next_1hz_service_at += 1000000ULL;
            process1HzTasks(ts);
        }
        if (ts >= next_50hz_service_at) {
            next_50hz_service_at += 1000000ULL/50U;
            send_ESCStatus();
	}
}

/*
  handler for message pending
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanptr)
{
    receiveFrame();
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    processTxQueue();
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    processTxQueue();
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    processTxQueue();
}

/*
  interrupt handler for CAN1
 */
void CAN1_RX0_IRQHandler(void)
{
    // call the HAL CAN IRQ handler, which will dispatch to the right
    // function
    HAL_CAN_IRQHandler(&hcan);
}

#endif // DRONECAN_SUPPORT

