/**************************************************************************
 * bootloaderfsm.h -
 * Command enumerations for Katana 1.2 Axis Controller Bootloader
 * Copyright (C) 2006 Neuronics AG
 **************************************************************************/

#ifndef _BOOTLOADER_FSM_H__
#define _BOOTLOADER_FSM_H__

struct BootloaderFsm
{
	// (mixed old & new bootloader version)
	//BootloaderState
	enum EBootloaderState
	{
		BOOTLOADER_STATE_INIT     =  0,
		BOOTLOADER_STATE_WAIT      =  1,
		BOOTLOADER_STATE_BOOT      =  2,
		BOOTLOADER_STATE_COMMAND   =  3,
		BOOTLOADER_STATE_ERASE_A   =  4,
		BOOTLOADER_STATE_ERASE_B   =  5,
		BOOTLOADER_STATE_ERASE_C   =  6,
		BOOTLOADER_STATE_ERASE_D   =  7,
		BOOTLOADER_STATE_VERSION   =  8,
		BOOTLOADER_STATE_WRITEWORD =  9,
		BOOTLOADER_STATE_ERROR     = 11,
		BOOTLOADER_STATE_ERASE_UMBRIEL	=  14,
		BOOTLOADER_STATE_ERASE_URANUS	=  15,
		BOOTLOADER_STATE_ERROR_UMBRIEL  =  30,
		BOOTLOADER_STATE_ERROR_URANUS	=  31,
		BOOTLOADER_STATE_RESET          =  255	
	};
	
	//BootloaderCommand
	enum EBootloaderCommand
	{
		BOOTLOADER_CMD_NOP       =  0,
		BOOTLOADER_CMD_ESCAPE    =  1,
		BOOTLOADER_CMD_VERSION   =  2,
		BOOTLOADER_CMD_RESET     =  3,
		BOOTLOADER_CMD_ERASE_A   =  4,
		BOOTLOADER_CMD_ERASE_B   =  5,
		BOOTLOADER_CMD_ERASE_C   =  6,
		BOOTLOADER_CMD_ERASE_D   =  7,
		BOOTLOADER_CMD_WRITEWORD =  8,
		BOOTLOADER_CMD_ERROR_ACK = 10,
		BOOTLOADER_CMD_ERROR     = 11,
		BOOTLOADER_CMD_ERASE_UMBRIEL	=  14,
		BOOTLOADER_CMD_ERASE_URANUS		=  15
	};
	
};

#endif//_BOOTLOADER_FSM_H__

