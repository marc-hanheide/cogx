#ifndef _CANOPEN_EMCY_ENUM_H_
#define _CANOPEN_EMCY_ENUM_H_

enum ECanOpenEmcyCode {
	EMCY_NOP           = 0x0000, // no EMCY
	EMCY_RCV_MSG_LOST  = 0xFF00, // receive message lost
	EMCY_SND_MSG_LOST  = 0xFF01, // receive message lost
	EMCY_NMT_STATE_ERR = 0xFF02, // unknown CANopen NMT state
	EMCY_CAN_BUS_ERR   = 0xFF03, // CAN bus error
	EMCY_SCHEDULE_ERR  = 0xFF04  // scheduling over time
};

#endif // _CANOPEN_EMCY_ENUM_H_
