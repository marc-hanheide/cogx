#ifndef _CANOPEN_NMT_ENUM_H_
#define _CANOPEN_NMT_ENUM_H_

enum ECanOpenNmtCob {
	NMT_MasterMessage = 0x000,
	NMT_Emergency     = 0x080,
	NMT_HeartBeat     = 0x700
};

enum ECanOpenNmtState {
	NMT_State_Bootup         =   0, // 0x00
	NMT_State_Stopped        =   4, // 0x04
	NMT_State_Operational    =   5, // 0x05
	NMT_State_PreOperational = 127,  // 0x7F
	NMT_State_Nop            = 255  // Nop state does not exist in CANopen spec. But we need something like this here
};

enum ECanOpenNmtCmd {
	NMT_Cmd_Nop                =   0, // 0x00
	NMT_Cmd_Operational        =   1, // 0x01
	NMT_Cmd_Stopped            =   2, // 0x02
	NMT_Cmd_PreOperational     = 128, // 0x80
	NMT_Cmd_ResetNode          = 129, // 0x81
	NMT_Cmd_ResetCommunication = 130  // 0x82
};

#endif
