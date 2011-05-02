#ifndef _CAN_BOOTLOADER_MASTER_H_
#define _CAN_BOOTLOADER_MASTER_H_

#include <Golem/Device/Katana450/bootloaderfsm.h>
#include <Golem/Device/Katana450/canBootloaderPDO.h>

#include <Golem/Device/Katana450/canframe.h>
#include <Golem/Device/Katana450/Timer.h>

class CanBootloaderMaster
{
public:

	struct FlashLib
	{
		unsigned short Status;
		unsigned int   FirstFailAddr;
		unsigned short ExpectedData;
		unsigned short ActualData;

		FlashLib()
		{
			Status        = 0;
			FirstFailAddr = 0;
			ExpectedData  = 0;
			ActualData    = 0;
		}
	};

private:

	BootloaderFsm::EBootloaderState mState;

	unsigned short mBootloaderErrorCode;
	
	FlashLib mFlashLib;

	//! timer used to monitor the heartbeat period
	KNI::Timer       mTimer;

public:

	//! constructor
	CanBootloaderMaster ();
	
	bool isAlive () const;

	void pushCanFrame (CanFrame const &frame);

	BootloaderFsm::EBootloaderState getState () const;

	unsigned short getLastErrorCode () const;
	
	FlashLib getFlashLibStatus () const;
	
private:
};

#endif // _CAN_BOOTLOADER_MASTER_H_
