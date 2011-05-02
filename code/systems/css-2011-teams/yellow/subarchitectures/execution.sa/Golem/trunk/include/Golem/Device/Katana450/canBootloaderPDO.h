/**************************************************************************
 * canBootloaderPDO.h -
 * CAN protocol PDO enumerations for Katana 1.2 Axis Controller Bootloader
 * Copyright (C) 2006 Neuronics AG
 **************************************************************************/

#ifndef _CAN_BOOTLOADER_PDO_H__
#define _CAN_BOOTLOADER_PDO_H__

struct CanBootloaderPDO
{
	//!	THe PDO numbers from/to the axes
	enum EAxisControllerPdo
	{
		Bootloaderstate          = 0x500, // ( 80 << 4)
		BootloaderVersion        = 0x510, // ( 81 << 4)
		BootloaderError          = 0x520, // ( 82 << 4)
		BootloaderFlashStatus    = 0x530, // ( 83 << 4)
		BootloaderCommand        = 0x540, // ( 84 << 4)
		BootloaderWriteData      = 0x550, // ( 85 << 4)
		BootloaderWriteAck       = 0x560  // ( 86 << 4)
	};
};

#endif //_CAN_BOOTLOADER_PDO_H__
