/* ----------------------------------------------------------------------------
   CANopen Master implementation for Katana 1.2 Robots

   Copyright (C) 2007 Neuronics AG
   Lukas Burger, 2007
   ------------------------------------------------------------------------- */
#include <Golem/Device/Katana450/canopen_master.h>
#include <Golem/Device/Katana450/axis_controller_emcy.h>
#include <Golem/Sys/Timer.h>

#include <fstream>
#include <iostream>
#include <sstream>

// ----------------------------------------------------------------------------
CANopenMaster::CANopenMaster()
{
	mDebug = true;

	mCANbusActive = false;
	mCANbusReady  = false;

	for (unsigned int i=0; i<NUM_OF_NODES; ++i)
	{
		mCANopenNode.push_back(tr1::shared_ptr<CANopenMasterNode>(new CANopenMasterNode(i+1)));
	}
	mCanBusInterface = tr1::shared_ptr<CanBusInterface>(new CanBusInterface());
	mCanBusInterface->setDevice("/dev/pcanusb0"); // TODO: set the default device	
}

// ----------------------------------------------------------------------------
CANopenMaster::~CANopenMaster()
{
	CanBusStop();
}

// ----------------------------------------------------------------------------
std::string CANopenMaster::getVersion ()
{
        return mVersion.getVersion();
}

// ----------------------------------------------------------------------------
void CANopenMaster::reset()
{
	CanBusStop();
	for (unsigned int i=0; i<NUM_OF_NODES; ++i)
	{
		mCANopenNode[i]->reset();
	}
}

// ============================================================================
// CAN Bus
// ============================================================================

// ----------------------------------------------------------------------------
void CANopenMaster::CanBusSetDevice (std::string device)
{
	mCanBusInterface->setDevice(device);
        
}

// ----------------------------------------------------------------------------
bool CANopenMaster::CanBusStart (unsigned int pollingPeriod)
{
	
	mPollingPeriod = pollingPeriod;
	
	unsigned int timer = 5000; // [us]

	if ( CanBusIsReady() )
		return true;

	mCANbusActive = true;
	mCANbusThread = tr1::shared_ptr<boost::thread>(new boost::thread(boost::bind(&CANopenMaster::CANbus_thread, this)));

	while (timer > 0)
	{
		golem::SysTimer::sleep(1);//usleep (1000);
		timer--;
		if ( CanBusIsReady() )
			return true;
	}
	return false;
}

// ----------------------------------------------------------------------------
void CANopenMaster::CanBusStop ()
{
	mCANbusActive = false;
	if (mCANbusThread)
		mCANbusThread->join();
}

// ----------------------------------------------------------------------------
bool CANopenMaster::CanBusIsReady () const
{
//	std::cout << "CanBusIsReady: " << (mCANbusActive and mCANbusReady) << std::endl;
	return (mCANbusActive && mCANbusReady);
}

// ----------------------------------------------------------------------------
CanErrorFlags CANopenMaster::CanBusGetStatus ()
{
	return mCanBusInterface->status();
}

// ----------------------------------------------------------------------------
std::string   CANopenMaster::CanBusFlags2str    (CanErrorFlags flags)
{
	return mCanBusInterface->status2str (flags);
}

// ----------------------------------------------------------------------------
CanBusInterface::CanStatistic CANopenMaster::CanBusgetStatistic ()
{
	return mCanBusInterface->statistic();
}

// ----------------------------------------------------------------------------
std::string   CANopenMaster::CanBusStat2str    (CanBusInterface::CanStatistic stat)
{
	return mCanBusInterface->statistic2str (stat);
}

// ============================================================================
// AxisController specific
// ============================================================================

// ----------------------------------------------------------------------------
bool CANopenMaster::getAxisVersion (unsigned char nodeId, CANopen_ui8 bootloader[3], CANopen_ui8 firmware[3],CANopen_ui16 &hardware, unsigned int timeout )
{
	if ( NMTisAlive(nodeId) == false )
	{
		return false;
	}

	if ( NMTgetState(nodeId) != NMT_State_Operational )
	{
		return false;
	}

	CanFrame frame;
	frame.id = 0x1c0 + static_cast<unsigned int>(nodeId); // TODO: recode this!!!
	frame.size = 8;
	//get Frame via remote transmit
	if ( !mCanBusInterface->send(frame, true) )
	{
//		return false; // there can be an error, but also a warning that does not mean message hat not bean sent
	}

	unsigned int timer = 0;
	while ( PDOchanged (TX_DATA_VERSION_FW_MAJOR, nodeId) == false )
	{
		timer++;
		if ( timer > timeout )
		{
			return false;
		}
		golem::SysTimer::sleep(1);//usleep (1000);
	}

	bootloader[0] = mCANopenNode[nodeId-1]->PDOreadVal_ui8(TX_DATA_VERSION_BOOT_MAJOR);
	bootloader[1] = mCANopenNode[nodeId-1]->PDOreadVal_ui8(TX_DATA_VERSION_BOOT_MINOR);
	bootloader[2] = mCANopenNode[nodeId-1]->PDOreadVal_ui8(TX_DATA_VERSION_BOOT_RELEASE);
	
	firmware[0] = mCANopenNode[nodeId-1]->PDOreadVal_ui8(TX_DATA_VERSION_FW_MAJOR);
	firmware[1] = mCANopenNode[nodeId-1]->PDOreadVal_ui8(TX_DATA_VERSION_FW_MINOR);
	firmware[2] = mCANopenNode[nodeId-1]->PDOreadVal_ui8(TX_DATA_VERSION_FW_RELEASE);
	
	hardware = mCANopenNode[nodeId-1]->PDOreadVal_ui16(TX_DATA_VERSION_HARDWARE);

	return true;
}

// ----------------------------------------------------------------------------
std::string CANopenMaster::getAxisVersionString (unsigned char nodeId)
{
	std::ostringstream stream;
	CANopen_ui8 bootloader[3];
	CANopen_ui8 firmware[3];
	CANopen_ui16 hardware;
	if ( getAxisVersion(nodeId, bootloader, firmware, hardware) )
	{
		stream << "Bootloader: ";
		stream << static_cast<unsigned int>(bootloader[0]) << ".";
		stream << static_cast<unsigned int>(bootloader[1]) << ".";
		stream << static_cast<unsigned int>(bootloader[2]);
		stream << " - ";
		stream << "Firmware: ";
		stream << static_cast<unsigned int>(firmware[0]) << ".";
		stream << static_cast<unsigned int>(firmware[1]) << ".";
		stream << static_cast<unsigned int>(firmware[2]);
		stream << " - ";
		stream << "Hardware: ";
		stream << (hardware);
	} else {
		stream << "ERROR: could not get Version from Node " << static_cast<unsigned int>(nodeId);
	}
	return stream.str();
}

// ============================================================================
//! Bootloader
// ============================================================================

// ----------------------------------------------------------------------------
bool CANopenMaster::BootloaderIsAlive (unsigned char nodeId) const
{
	return mCANopenNode[nodeId-1]->mCanBootloaderMaster.isAlive();
}

// ----------------------------------------------------------------------------
BootloaderFsm::EBootloaderState CANopenMaster::BootloaderGetState (unsigned char nodeId) const
{
	return mCANopenNode[nodeId-1]->mCanBootloaderMaster.getState();
}

// ----------------------------------------------------------------------------
unsigned short CANopenMaster::BootloaderGetLastErrorCode  (unsigned char nodeId) const
{
	return mCANopenNode[nodeId-1]->mCanBootloaderMaster.getLastErrorCode();
}

// ----------------------------------------------------------------------------
CanBootloaderMaster::FlashLib CANopenMaster::BootloaderGetFlashLibStatus (unsigned char nodeId) const
{
	return mCANopenNode[nodeId-1]->mCanBootloaderMaster.getFlashLibStatus();
}

// ----------------------------------------------------------------------------
void CANopenMaster::BootloaderSendCmd (BootloaderFsm::EBootloaderCommand cmd, unsigned char nodeId)
{
	if ( cmd == BootloaderFsm::BOOTLOADER_CMD_ERASE_UMBRIEL || cmd == BootloaderFsm::BOOTLOADER_CMD_ERASE_A )
	{
		std::cout << "Sorry I can not allow you to erase the bootloader umbriel: Command ignored." << std::endl;
		return;
	}
	
	CanFrame frame;
	frame.id = CanBootloaderPDO::BootloaderCommand | nodeId;
	frame.size = 2;
	frame.setByte(0, cmd);
	mCanBusInterface->send( frame );
}

// ============================================================================
// NMT
// ============================================================================

// ----------------------------------------------------------------------------
bool CANopenMaster::NMTsendCmd (ECanOpenNmtCmd cmd, unsigned char nodeId)
{
	CanFrame frame;
	frame.id = 0;
	frame.size = 2;
	frame.data[0] = static_cast<char>(cmd);
	frame.data[1] = nodeId;
	return mCanBusInterface->send(frame) != 0;
}

// ----------------------------------------------------------------------------
ECanOpenNmtState CANopenMaster::NMTgetState (unsigned char nodeId) const
{
	return mCANopenNode[nodeId-1]->NMTgetState();
}

// ----------------------------------------------------------------------------
void CANopenMaster::NMTreset (unsigned char nodeId)
{
	mCANopenNode[nodeId-1]->NMTreset();
}

// ----------------------------------------------------------------------------
bool CANopenMaster::NMTisAlive (unsigned char nodeId) const
{
	bool alive = false;
	// sleep two heartbeat periods to assure the capture or heartbeats
//	usleep(1000000); // TODO: make heartbeat time configurable anr accessable
	if (nodeId == 0) // check all nodes
	{
		unsigned int cnt = 0;
		for (std::vector<CANopenMasterNode>::size_type i=0; i < mCANopenNode.size(); i++)
		{
			if ( mCANopenNode[nodeId-1]->NMTisAlive() )
				cnt++;
		}
		if ( cnt == mCANopenNode.size() )
			alive = true;
	} else {
		alive = mCANopenNode[nodeId-1]->NMTisAlive();
	}
	return alive;
}

// ----------------------------------------------------------------------------
unsigned int CANopenMaster::NMTgetBootupCounter (unsigned char nodeId) const
{
	return mCANopenNode[nodeId-1]->NMTgetBootupCounter();

}

// ----------------------------------------------------------------------------
unsigned int CANopenMaster::NMTgetNotAliveCounter (unsigned char nodeId) const
{
	return mCANopenNode[nodeId-1]->NMTgetNotAliveCounter();
}

// ============================================================================
// EMCY
// ============================================================================

// ----------------------------------------------------------------------------
void CANopenMaster::EMCYreset (unsigned char nodeId)
{
	return mCANopenNode[nodeId-1]->EMCYreset();
}

// ----------------------------------------------------------------------------
unsigned int CANopenMaster::EMCYcount (unsigned char nodeId) const
{
	return mCANopenNode[nodeId-1]->EMCYcount();
}

// ----------------------------------------------------------------------------
unsigned long CANopenMaster::EMCYcountTotal (unsigned char nodeId) const
{
	return mCANopenNode[nodeId-1]->EMCYcountTotal();
}

// ----------------------------------------------------------------------------
CanOpenEmcyDesc CANopenMaster::EMCYpop (unsigned char nodeId)
{
	return mCANopenNode[nodeId-1]->EMCYpop ();
}

// ----------------------------------------------------------------------------
std::string     CANopenMaster::EMCY2str  (CanOpenEmcyDesc const &desc) const
{
	return AxisControllerEMCY::emcyDsc2str (desc);
}

// ============================================================================
// PDO
// ============================================================================

// ----------------------------------------------------------------------------
bool CANopenMaster::PDOchanged(ECANopenPdoData val, unsigned char nodeId)
{
	if ( nodeId > 0 && nodeId <= NUM_OF_NODES )
	{
		return mCANopenNode[nodeId-1]->PDOvalChanged(val);
	}
	return false;
}

// ----------------------------------------------------------------------------
void CANopenMaster::PDOsetActive(ECANopenPdo pdo, unsigned char nodeId)
{
	if ( nodeId > 0 && nodeId <= NUM_OF_NODES )
	{
		mCANopenNode[nodeId-1]->PDOsetActive(pdo);
	}
}

// ----------------------------------------------------------------------------
void CANopenMaster::PDOsetPassiv(ECANopenPdo pdo, unsigned char nodeId)
{
	if ( nodeId > 0 && nodeId <= NUM_OF_NODES )
	{
		mCANopenNode[nodeId-1]->PDOsetPassiv(pdo);
	}
}

// ----------------------------------------------------------------------------
bool CANopenMaster::PDOisActive(ECANopenPdo pdo, unsigned char nodeId)
{
	if ( nodeId > 0 && nodeId <= NUM_OF_NODES )
	{
		return mCANopenNode[nodeId-1]->PDOisActive(pdo);
	}
	return false;
}

// ----------------------------------------------------------------------------
CANopen_ui8  CANopenMaster::PDOreadVal_ui8  (ECANopenPdoData pdoVal, unsigned char nodeId)
{
	return mCANopenNode[nodeId-1]->PDOreadVal_ui8 (pdoVal);
}

// ----------------------------------------------------------------------------
CANopen_i8   CANopenMaster::PDOreadVal_i8   (ECANopenPdoData pdoVal, unsigned char nodeId)
{
	return mCANopenNode[nodeId-1]->PDOreadVal_i8 (pdoVal);
}

// ----------------------------------------------------------------------------
CANopen_ui16 CANopenMaster::PDOreadVal_ui16 (ECANopenPdoData pdoVal, unsigned char nodeId)
{
	return mCANopenNode[nodeId-1]->PDOreadVal_ui16 (pdoVal);
}

// ----------------------------------------------------------------------------
CANopen_i16  CANopenMaster::PDOreadVal_i16  (ECANopenPdoData pdoVal, unsigned char nodeId)
{
	return mCANopenNode[nodeId-1]->PDOreadVal_i16 (pdoVal);
}

// ----------------------------------------------------------------------------
CANopen_ui32 CANopenMaster::PDOreadVal_ui32 (ECANopenPdoData pdoVal, unsigned char nodeId)
{
	return mCANopenNode[nodeId-1]->PDOreadVal_ui32 (pdoVal);
}

// ----------------------------------------------------------------------------
CANopen_i32  CANopenMaster::PDOreadVal_i32  (ECANopenPdoData pdoVal, unsigned char nodeId)
{
	return mCANopenNode[nodeId-1]->PDOreadVal_i32 (pdoVal);
}

// ----------------------------------------------------------------------------
void  CANopenMaster::PDOwriteVal_ui8 (CANopen_ui8  const & val, ECANopenPdoData pdoVal, unsigned char nodeId)
{
	mCANopenNode[nodeId-1]->PDOwriteVal_ui8(val, pdoVal);
}

// ----------------------------------------------------------------------------
void CANopenMaster::PDOwriteVal_i8 (CANopen_i8   const & val, ECANopenPdoData pdoVal, unsigned char nodeId)
{
	mCANopenNode[nodeId-1]->PDOwriteVal_i8(val, pdoVal);
}

// ----------------------------------------------------------------------------
void CANopenMaster::PDOwriteVal_ui16 (CANopen_ui16 const & val, ECANopenPdoData pdoVal, unsigned char nodeId)
{
	mCANopenNode[nodeId-1]->PDOwriteVal_ui16(val, pdoVal);
}

// ----------------------------------------------------------------------------
void CANopenMaster::PDOwriteVal_i16  (CANopen_i16  const & val, ECANopenPdoData pdoVal, unsigned char nodeId)
{
	mCANopenNode[nodeId-1]->PDOwriteVal_i16(val, pdoVal);
}

// ----------------------------------------------------------------------------
void CANopenMaster::PDOwriteVal_ui32  (CANopen_ui32 const & val, ECANopenPdoData pdoVal, unsigned char nodeId)
{
	mCANopenNode[nodeId-1]->PDOwriteVal_ui32(val, pdoVal);
}

// ----------------------------------------------------------------------------
void CANopenMaster::PDOwriteVal_i32  (CANopen_i32  const & val, ECANopenPdoData pdoVal, unsigned char nodeId)
{
	mCANopenNode[nodeId-1]->PDOwriteVal_i32(val, pdoVal);
}

// ----------------------------------------------------------------------------
void CANopenMaster::PDOsend (ECANopenPdoData pdoVal, unsigned char nodeId)
{
	mCanBusInterface->send ( mCANopenNode[nodeId-1]->readCanFrame(pdoVal) );
}

// ----------------------------------------------------------------------------
void CANopenMaster::PDOsend (ECANopenPdo pdo, unsigned char nodeId)
{
	mCanBusInterface->send ( mCANopenNode[nodeId-1]->readCanFrame(pdo) );
}

// ============================================================================
// private
// ============================================================================

// ----------------------------------------------------------------------------
void CANopenMaster::CANbus_thread ()
{
	CanBusMsg                           msg;
	unsigned int                        nodeId;
	unsigned int                        funcCode;
	CanBusInterface::CANERR_FLAGS_UNION errFlags;
	CanBusInterface::Extended_Status    extStatus;

	std::ofstream logfile;

	if ( mDebug )
		logfile.open ("/tmp/CANbus_thread.log",  std::ios_base::out );
	else
		logfile.open ("/dev/null");
	
	logfile << "CANbus_thread started." << std::endl;
	logfile << std::hex;

	while (mCANbusActive)
	{
		if ( mCANbusReady )
		{
			extStatus = mCanBusInterface->getExtStatus();
			for ( int i=0; i<extStatus.pendingReads; i++ )
			{
				msg = mCanBusInterface->read();
			 	errFlags.all = msg.errFlags;
				if ( msg.errFlags == 0 )
				{
					nodeId   = msg.canFrame.id & CAN_NODE_ID_MASK;
					funcCode = msg.canFrame.id & CAN_FUNC_CODE_MASK;
			
					if ( nodeId > 0 && nodeId <= NUM_OF_NODES )
					{
					mCANopenNode[nodeId-1]->writeCanBusMsg(msg);
					} else {
						// TODO: do something!
					}
				} else {

					logfile << mCanBusInterface->status2str(errFlags.all) << std::endl << std::flush;

					if ( errFlags.bit.illhandle )
					{
						logfile << "CANbus illegal handle:" << msg.errFlags << std::endl << std::flush;
						mCANbusReady   = false;
						mCanBusInterface->close();
						//mCANbusActive  = false;
					}
	
					if ( errFlags.bit.overrun )
					{
						// TODO: receive message lost!
						logfile << "CANbus Receive Message lost:" << msg.errFlags << std::endl << std::flush;
					}

					if ( errFlags.bit.qrcvempty )
					{
						continue;
					}
				}
			}
		} else {
			if ( mCanBusInterface->open() )
			{
				mCanBusInterface->flush();
				logfile << "CANbus open()" << std::endl;
				mCANbusReady = true;
			}
		}
		golem::SysTimer::sleep(mPollingPeriod/1000);//usleep(mPollingPeriod);
	}
	mCanBusInterface->close();
    mCANbusReady = false;
	logfile << "CANbus_thread good bye." << std::endl << std::endl;
	logfile.close();
};
