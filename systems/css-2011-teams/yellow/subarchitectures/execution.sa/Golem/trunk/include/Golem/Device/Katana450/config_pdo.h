#ifndef _CANOPEN_PDO_CONFIG_H_
#define _CANOPEN_PDO_CONFIG_H_

#include <Golem/Device/Katana450/direction.h>
#include <Golem/Device/Katana450/enum_pdo_data.h>
#include <Golem/Device/Katana450/enum_pdo.h>

enum ECANopenPdoTriggerMethod
{
	NO_TRIGGER,
	EVENT_DRIVEN,
	TIME_DRIVEN,
	INDIVIDUAL_POLLING,
	SYNCRONIZED
};

struct CANopenPdoDesc {
	ECANopenPdo              pdo;
	Uint16                   functionCode;
	Direction::EDir          direction;
	ECANopenPdoTriggerMethod trigger;
};

struct CANopenPdoDataDesc {
	ECANopenPdoData name;
	ECANopenPdo     pdo;
	ECanOpenType    type;
	Uint8           offset;
	Uint8           size;
};

// global PDO configuration table
static const CANopenPdoDesc URANUS_PDO_LIST[] = {
	// {PDO Name, func code, Direction, TX trigger type}
	// Axis
	{TPDO_DYNAMICS,                   0x1A0, Direction::TX, TIME_DRIVEN,      },
	{TPDO_POWER,                      0x1B0, Direction::TX, EVENT_DRIVEN,},
	{TPDO_VERSION,                    0x1C0, Direction::TX, INDIVIDUAL_POLLING},

	{RPDO_CONTROL_PARAMETERS_1,       0x1D0, Direction::RX, NO_TRIGGER,       },
	{RPDO_CONTROL_PARAMETERS_2,       0x1E0, Direction::RX, NO_TRIGGER,       },
	{RPDO_CONTROL_PARAMETERS_3,       0x4A0, Direction::RX, NO_TRIGGER,       },
//	{RPDO_CONTROL_PARAMETERS_4,       0x4B0, Direction::RX, NO_TRIGGER,       },
	{RPDO_COLLISION_DETECTION,        0x220, Direction::RX, NO_TRIGGER,       },

	// Axis FSM
	{TPDO_AXIS_FSM_STATE,             0x180, Direction::TX, TIME_DRIVEN,      },
	{RPDO_AXIS_FSM_COMMAND,           0x190, Direction::RX, NO_TRIGGER,       },

	// Axis FSM parameters
	{RPDO_P2P_SETUP,                  0x1F0, Direction::RX, NO_TRIGGER,       },
 	{RPDO_KNI_RNI_ENC_FACTORS,        0x200, Direction::RX, NO_TRIGGER,       },
	{RPDO_ENCODER_SETUP,              0x210, Direction::RX, NO_TRIGGER,       },

	// MoveBuffer
	{RPDO_MOVE_BUFFER_FSM_COMMAND,    0x230, Direction::RX, NO_TRIGGER,       },
	{RPDO_POLY_PARAMETERS_1,          0x240, Direction::RX, NO_TRIGGER,       },
	{RPDO_POLY_PARAMETERS_2,          0x250, Direction::RX, NO_TRIGGER,       },

	// Debug
	{RPDO_DEBUG,                      0x260, Direction::RX, NO_TRIGGER,       },
	{TPDO_DEVIATION,                  0x270, Direction::TX, EVENT_DRIVEN,     },
	{TPDO_DEBUG_1,                    0x280, Direction::TX, EVENT_DRIVEN,     },
	{TPDO_DEBUG_2,                    0x290, Direction::TX, EVENT_DRIVEN,     },
	
	// Analysis
	{TPDO_ANALYSIS,                   0x300, Direction::TX, EVENT_DRIVEN,     },
	{RPDO_ANALYSIS_COMMAND,           0x310, Direction::RX, NO_TRIGGER,       },

	// Step
	{RPDO_STEP_DETAILS,               0x320, Direction::RX, NO_TRIGGER,       }
};

// global Axis Parameter configuration table
static const CANopenPdoDataDesc CANOPEN_PDO_DATA_LIST[] = {
	// {Parameter, located in PDO, offset, length}
	// -------------------------------------------------------------------------
	// Axis
	// -------------------------------------------------------------------------
	// monitor data
	{TX_DATA_POSITION,                  TPDO_DYNAMICS,                     CANopen_uint32, 0, 4},
	{TX_DATA_SPEED,                     TPDO_DYNAMICS,                     CANopen_int16 , 4, 2},
	{TX_DATA_DRIVE,                     TPDO_DYNAMICS,                     CANopen_int16 , 6, 2},

	{TX_DATA_CURRENT,                   TPDO_POWER,                        CANopen_int16 , 0, 2},
	{TX_DATA_VOLTAGE,                   TPDO_POWER,                        CANopen_int16 , 2, 2},

	// version information
	{TX_DATA_VERSION_HARDWARE,          TPDO_VERSION,                      CANopen_uint16, 0, 2},
	{TX_DATA_VERSION_FW_MAJOR,          TPDO_VERSION,                      CANopen_uint8,  2, 1},
	{TX_DATA_VERSION_FW_MINOR,          TPDO_VERSION,                      CANopen_uint8,  3, 1},
	{TX_DATA_VERSION_FW_RELEASE,        TPDO_VERSION,                      CANopen_uint8,  4, 1},
	{TX_DATA_VERSION_BOOT_MAJOR,        TPDO_VERSION,                      CANopen_uint8,  5, 1},
	{TX_DATA_VERSION_BOOT_MINOR,        TPDO_VERSION,                      CANopen_uint8,  6, 1},
	{TX_DATA_VERSION_BOOT_RELEASE,      TPDO_VERSION,                      CANopen_uint8,  7, 1},

	// controller parameters 1
	{RX_DATA_CONTROLLER_KP_POS,         RPDO_CONTROL_PARAMETERS_1,         CANopen_uint16, 0, 2},
	{RX_DATA_CONTROLLER_KI_POS,         RPDO_CONTROL_PARAMETERS_1,         CANopen_uint16, 2, 2},
	{RX_DATA_CONTROLLER_KD_POS,         RPDO_CONTROL_PARAMETERS_1,         CANopen_uint16, 4, 2},
	{RX_DATA_CONTROLLER_KI_LEGACY,      RPDO_CONTROL_PARAMETERS_1,         CANopen_uint16, 6, 2},
	
	// controller parameters 2
	{RX_DATA_CONTROLLER_KP_CURRENT,     RPDO_CONTROL_PARAMETERS_2,         CANopen_uint16, 0, 2},
	{RX_DATA_CONTROLLER_KI_CURRENT,     RPDO_CONTROL_PARAMETERS_2,         CANopen_uint16, 2, 2},
	{RX_DATA_MAX_DRIVE,                 RPDO_CONTROL_PARAMETERS_2,         CANopen_uint16, 4, 2},
	{RX_DATA_MAX_CURRENT,               RPDO_CONTROL_PARAMETERS_2,         CANopen_uint16, 6, 2},

	// controller parameters 3
	{RX_DATA_CONTROLLER_TYPE,           RPDO_CONTROL_PARAMETERS_3,         CANopen_uint8,  0, 1},
	{RX_DATA_CONTROLLER_KE,             RPDO_CONTROL_PARAMETERS_3,         CANopen_uint16, 1, 2},
	{RX_DATA_CONTROLLER_RA,             RPDO_CONTROL_PARAMETERS_3,         CANopen_uint16, 3, 2},
	{RX_DATA_CONTROLLER_OFFSET_E1,      RPDO_CONTROL_PARAMETERS_3,         CANopen_int16,  5, 2},
	
	// collision detection
	{RX_DATA_COLLISION_ACTIVE,          RPDO_COLLISION_DETECTION,          CANopen_uint8,  0, 1},
	{RX_DATA_COLLISION_LIMIT_ERR_POS,   RPDO_COLLISION_DETECTION,          CANopen_uint16, 1, 2},
	{RX_DATA_COLLISION_LIMIT_ERR_SPEED, RPDO_COLLISION_DETECTION,          CANopen_uint16, 3, 2},

	// -------------------------------------------------------------------------
	// Axis FSM
	// -------------------------------------------------------------------------
	{TX_DATA_AXIS_STATE,                TPDO_AXIS_FSM_STATE,               CANopen_uint8,  0, 1},
	{TX_DATA_MOVE_BUFFER_SIZE,          TPDO_AXIS_FSM_STATE,               CANopen_uint8,  1, 1},
	{TX_DATA_MOVE_BUFFER_COUNTER,       TPDO_AXIS_FSM_STATE,               CANopen_uint8,  2, 1},
	
	{RX_DATA_AXIS_COMMAND,              RPDO_AXIS_FSM_COMMAND,             CANopen_uint8,  0, 1},

	// -------------------------------------------------------------------------
	// Axis FSM parameters
	// -------------------------------------------------------------------------
	// move P2P param
	{RX_DATA_P2P_TARGET_POS,            RPDO_P2P_SETUP,                    CANopen_uint32, 0, 4},
	{RX_DATA_P2P_MAX_SPEED,             RPDO_P2P_SETUP,                    CANopen_uint16, 4, 2},
	{RX_DATA_P2P_MAX_ACCEL,             RPDO_P2P_SETUP,                    CANopen_uint8,  6, 1},
	{RX_DATA_P2P_TOLERANCE,             RPDO_P2P_SETUP,                    CANopen_uint8,  7, 1},
	
	// config poly move
	{RX_DATA_KNI_RNI_ENC_OFFSET,        RPDO_KNI_RNI_ENC_FACTORS,          CANopen_int32,  0, 4},
	{RX_DATA_KNI_RNI_ENC_SLOPE,         RPDO_KNI_RNI_ENC_FACTORS,          CANopen_int16,  4, 2},

 	// encoder setup
	{RX_DATA_ENCODER_POS,               RPDO_ENCODER_SETUP,                CANopen_uint32, 0, 4},

	// -------------------------------------------------------------------------
	// MoveBuffer
	// -------------------------------------------------------------------------
	// move buffer cmd
	{RX_DATA_MOVE_BUFFER_COMMAND,       RPDO_MOVE_BUFFER_FSM_COMMAND,      CANopen_uint8,  0, 1},

	// move poly param 1
	{RX_DATA_POLY_TARGET_POS,           RPDO_POLY_PARAMETERS_1,            CANopen_uint32, 0, 4},
	{RX_DATA_POLY_TIME,                 RPDO_POLY_PARAMETERS_1,            CANopen_uint16, 4, 2},
	{RX_DATA_POLY_TOLERANCE,            RPDO_POLY_PARAMETERS_1,            CANopen_uint8,  6, 1},
	{RX_DATA_POLY_NEXT,                 RPDO_POLY_PARAMETERS_1,            CANopen_uint8,  7, 1},

	// move poly param 2
	{RX_DATA_POLY_PARAM_P0,             RPDO_POLY_PARAMETERS_2,            CANopen_int16,  0, 2},
	{RX_DATA_POLY_PARAM_P1,             RPDO_POLY_PARAMETERS_2,            CANopen_int16,  2, 2},
	{RX_DATA_POLY_PARAM_P2,             RPDO_POLY_PARAMETERS_2,            CANopen_int16,  4, 2},
	{RX_DATA_POLY_PARAM_P3,             RPDO_POLY_PARAMETERS_2,            CANopen_int16,  6, 2},

	// -------------------------------------------------------------------------
	// Debug
	// -------------------------------------------------------------------------
	{RX_DATA_DEBUG,                     RPDO_DEBUG,                        CANopen_uint8,  0, 1},

	{TX_DATA_POSITION_ERROR,            TPDO_DEVIATION,                    CANopen_int32,  0, 4},
	{TX_DATA_SPEED_ERROR,               TPDO_DEVIATION,                    CANopen_int16,  4, 2},
	{TX_DATA_SETPOINT_ACCEL,            TPDO_DEVIATION,                    CANopen_int16,  6, 2},

	{TX_DATA_DEBUG_1_INT32,             TPDO_DEBUG_1,                      CANopen_int32,  0, 4},
	{TX_DATA_DEBUG_2_INT32,             TPDO_DEBUG_1,                      CANopen_int32,  4, 4},

	{TX_DATA_DEBUG_3_UINT32,            TPDO_DEBUG_2,                      CANopen_uint32, 0, 4},
	{TX_DATA_DEBUG_4_INT16,             TPDO_DEBUG_2,                      CANopen_int16,  4, 2},
	{TX_DATA_DEBUG_5_UINT16,            TPDO_DEBUG_2,                      CANopen_uint16, 6, 2},
	
	// -------------------------------------------------------------------------
	// Step
	// -------------------------------------------------------------------------
	{RX_DATA_STEP_TYPE,                 RPDO_STEP_DETAILS,                 CANopen_uint8, 0, 1},
	{RX_DATA_STEP_VALUE,                RPDO_STEP_DETAILS,                 CANopen_int16, 1, 2},

	// -------------------------------------------------------------------------
	// Analysis
	// -------------------------------------------------------------------------
	{TX_DATA_ANALYSIS_DESCRIPTOR,       TPDO_ANALYSIS,                     CANopen_int16,  0, 2},
	{TX_DATA_ANALYSIS_DATA,             TPDO_ANALYSIS,                     CANopen_int32,  2, 4},
		
	{RX_DATA_ANALYSIS_COMMAND,          RPDO_ANALYSIS_COMMAND,             CANopen_int16,  0, 2},
	{RX_DATA_ANALYSIS_COMMAND_DATA,     RPDO_ANALYSIS_COMMAND,             CANopen_int32,  2, 4}
};

#endif // _CANOPEN_PDO_CONFIG_H_
