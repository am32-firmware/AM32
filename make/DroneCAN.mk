# extra makefile elements for DroneCAN support

SRC_DIR_DRONECAN := \
	modules/DroneCAN/libcanard \
	modules/DroneCAN/libcanard/drivers/stm32 \
	Src/DroneCAN

CFLAGS_DRONECAN := \
	-Imodules/DroneCAN/libcanard \
	-Imodules/DroneCAN/libcanard/drivers/stm32 \
	-Igen/dsdl_generated/include \
	-ISrc/DroneCAN \
	-DDRONECAN_SUPPORT=1

# add in generated code
SRC_DRONECAN := $(foreach dir,$(SRC_DIR_DRONECAN),$(wildcard $(dir)/*.c))
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.protocol.NodeStatus.c
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.protocol.GetNodeInfo_res.c
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.equipment.esc.RawCommand.c
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.equipment.esc.Status.c
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.protocol.dynamic_node_id.Allocation.c
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.protocol.param.GetSet_req.c
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.protocol.param.GetSet_res.c
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.protocol.param.ExecuteOpcode_req.c
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.protocol.param.ExecuteOpcode_res.c
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.protocol.file.BeginFirmwareUpdate_req.c
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.protocol.file.BeginFirmwareUpdate_res.c
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.protocol.file.Read_req.c
SRC_DRONECAN += gen/dsdl_generated/src/uavcan.protocol.file.Read_res.c

EXTRA_DEPS := dsdl_generate
