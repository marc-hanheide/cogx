#ifndef _CANOPEN_PDO_DATA_H_
#define _CANOPEN_PDO_DATA_H_

enum ECANopenPdoData
{
	// -------------------------------------------------------------------------
	// Axis
	// -------------------------------------------------------------------------
	// monitor data
	TX_DATA_POSITION,
	TX_DATA_SPEED,
	TX_DATA_DRIVE,

	TX_DATA_CURRENT,
	TX_DATA_VOLTAGE,

	// controller parameters 1
	RX_DATA_CONTROLLER_KP_POS,
	RX_DATA_CONTROLLER_KI_POS,
	RX_DATA_CONTROLLER_KD_POS,
	RX_DATA_CONTROLLER_KI_LEGACY,

	// controller parameters 2
	RX_DATA_CONTROLLER_KP_CURRENT,
	RX_DATA_CONTROLLER_KI_CURRENT,
	RX_DATA_MAX_DRIVE,
	RX_DATA_MAX_CURRENT,
	
	// controller parameters 3
	RX_DATA_CONTROLLER_TYPE,
	RX_DATA_CONTROLLER_KE,
	RX_DATA_CONTROLLER_RA,
	RX_DATA_CONTROLLER_OFFSET_E1,
	
	// collision detection
	RX_DATA_COLLISION_ACTIVE,
	RX_DATA_COLLISION_LIMIT_ERR_POS,
	RX_DATA_COLLISION_LIMIT_ERR_SPEED,
 
	// config poly move
	RX_DATA_KNI_RNI_ENC_OFFSET,
	RX_DATA_KNI_RNI_ENC_SLOPE,

	// version information
	TX_DATA_VERSION_HARDWARE,
	TX_DATA_VERSION_FW_MAJOR,
	TX_DATA_VERSION_FW_MINOR,
	TX_DATA_VERSION_FW_RELEASE,
	TX_DATA_VERSION_BOOT_MAJOR,
	TX_DATA_VERSION_BOOT_MINOR,
	TX_DATA_VERSION_BOOT_RELEASE,

	// -------------------------------------------------------------------------
	// Axis FSM
	// -------------------------------------------------------------------------
	TX_DATA_AXIS_STATE,
	RX_DATA_AXIS_COMMAND,

	// -------------------------------------------------------------------------
	// Axis FSM parameters
	// -------------------------------------------------------------------------
	// move P2P param
	RX_DATA_P2P_TARGET_POS,
	RX_DATA_P2P_MAX_SPEED,
	RX_DATA_P2P_MAX_ACCEL,
	RX_DATA_P2P_TOLERANCE,

 	// encoder setup
	RX_DATA_ENCODER_POS,

	// -------------------------------------------------------------------------
	// MoveBuffer
	// -------------------------------------------------------------------------
	// move buffer cmd
	RX_DATA_MOVE_BUFFER_COMMAND,

	// move buffer state
	TX_DATA_MOVE_BUFFER_SIZE,
	TX_DATA_MOVE_BUFFER_COUNTER,

	// move poly param 1
	RX_DATA_POLY_TARGET_POS,
	RX_DATA_POLY_TIME,
	RX_DATA_POLY_TOLERANCE,
	RX_DATA_POLY_NEXT,

	// move poly param 2
	RX_DATA_POLY_PARAM_P0,
	RX_DATA_POLY_PARAM_P1,
	RX_DATA_POLY_PARAM_P2,
	RX_DATA_POLY_PARAM_P3,
	
	// -------------------------------------------------------------------------
	// Debug
	// -------------------------------------------------------------------------
	RX_DATA_DEBUG,

	TX_DATA_POSITION_ERROR,
	TX_DATA_SPEED_ERROR,
	TX_DATA_SETPOINT_ACCEL,

	TX_DATA_DEBUG_1_INT32,
	TX_DATA_DEBUG_2_INT32,

	TX_DATA_DEBUG_3_UINT32,
	TX_DATA_DEBUG_4_INT16,
	TX_DATA_DEBUG_5_UINT16,
	
	// -------------------------------------------------------------------------
	// Step
	// -------------------------------------------------------------------------
	RX_DATA_STEP_TYPE,
	RX_DATA_STEP_VALUE,

	// -------------------------------------------------------------------------
	// Analysis
	// -------------------------------------------------------------------------
	TX_DATA_ANALYSIS_DESCRIPTOR,
	TX_DATA_ANALYSIS_DATA,
	
	RX_DATA_ANALYSIS_COMMAND,
	RX_DATA_ANALYSIS_COMMAND_DATA
};

#endif // _CANOPEN_PDO_DATA_H_
