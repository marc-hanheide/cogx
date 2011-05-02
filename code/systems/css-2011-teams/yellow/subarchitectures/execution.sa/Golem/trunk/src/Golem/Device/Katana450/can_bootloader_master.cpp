#include <Golem/Device/Katana450/can_bootloader_master.h>

//#include <iostream>

// -----------------------------------------------------------------------------
CanBootloaderMaster::CanBootloaderMaster()
{
	mState = BootloaderFsm::BOOTLOADER_STATE_INIT;

	mBootloaderErrorCode = 0;
	
	mTimer.Set_And_Start(1);
	while ( ! mTimer.Elapsed() ) {};
}

// -----------------------------------------------------------------------------
void CanBootloaderMaster::pushCanFrame (CanFrame const &frame)
{
	unsigned short funcCode = frame.id & 0x7F0;

//	std::cout << std::hex << frame.id << ": " << frame.getByte(0) << ", funcCode = "<< funcCode << std::endl;
	
	switch (funcCode)
	{
	case CanBootloaderPDO::Bootloaderstate:
		mState = static_cast<BootloaderFsm::EBootloaderState>(frame.getByte(0));
		mTimer.Set_And_Start(500); // TODO: define a const (CAN bootloader state has period of ~100ms)
		break;
	case CanBootloaderPDO::BootloaderError:
		mBootloaderErrorCode = static_cast<unsigned short>(frame.getByte(0)) | ( static_cast<unsigned short>(frame.getByte(1)) << 8 );
		mFlashLib.Status     = static_cast<unsigned short>(frame.getByte(2)) | ( static_cast<unsigned short>(frame.getByte(3)) << 8 );
		break;
	case CanBootloaderPDO::BootloaderFlashStatus:
		mFlashLib.FirstFailAddr =   static_cast<unsigned int>(frame.getByte(0)) |
		                          ( static_cast<unsigned int>(frame.getByte(1)) <<  8 ) | 
		                          ( static_cast<unsigned int>(frame.getByte(2)) << 16 ) | 
		                          ( static_cast<unsigned int>(frame.getByte(3)) << 24 );
		mFlashLib.ExpectedData  = static_cast<unsigned short>(frame.getByte(4)) | ( static_cast<unsigned short>(frame.getByte(5)) << 8 );
		mFlashLib.ActualData    = static_cast<unsigned short>(frame.getByte(6)) | ( static_cast<unsigned short>(frame.getByte(7)) << 8 );	
		break;
	default:
		break;
	}
}

// ----------------------------------------------------------------------------
bool CanBootloaderMaster::isAlive () const
{
	return (! mTimer.Elapsed());
}

// -----------------------------------------------------------------------------
BootloaderFsm::EBootloaderState CanBootloaderMaster::getState () const
{
	return mState;
}

// -----------------------------------------------------------------------------
unsigned short CanBootloaderMaster::getLastErrorCode () const
{
	return mBootloaderErrorCode;
}

// -----------------------------------------------------------------------------
CanBootloaderMaster::FlashLib CanBootloaderMaster::getFlashLibStatus () const
{
	return mFlashLib;
}

