#ifndef _CANOPEN_H_
#define _CANOPEN_H_

static const unsigned short CAN_COB_BITS       = 11;
static const unsigned short CAN_NODE_ID_BITS   = 4;
static const unsigned short CAN_FUNC_CODE_BITS = CAN_COB_BITS - CAN_NODE_ID_BITS;
static const unsigned int   CAN_NODE_ID_MASK   = (1 << CAN_NODE_ID_BITS)-1;
static const unsigned int   CAN_FUNC_CODE_MASK = ~CAN_NODE_ID_MASK;

static const unsigned int   CANOPEN_HEARTBEAT_PERIOD           = 100; // Heartbeat time in [ms]
static const unsigned int   CANOPEN_HEARTBEAT_OVERTIME_ALLOWED = 150; // Heartbeat overtime allowed in [%]

#include <Golem/Device/Katana450/direction.h>

#include <Golem/Device/Katana450/enum_nmt.h>
#include <Golem/Device/Katana450/types.h>

// Get this configuration header file from default include path:
// - this is a specific CANopen configuration (EDS file info)
//   (e.g. axis_controller oder sensor_controller)
#include <Golem/Device/Katana450/canopen_config.h>

#include <Golem/Device/Katana450/emcy.h>

#endif // _CANOPEN_H_
