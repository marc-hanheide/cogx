/* ----------------------------------------------------------------------------
   CANopen Master implementation for Katana 1.2 Robots

   Copyright (C) 2007 Neuronics AG
   Lukas Burger, 2007
   ------------------------------------------------------------------------- */
#ifndef _CANOPEN_MASTER_
#define _CANOPEN_MASTER_

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <string>
#include <vector>
#ifdef WIN32
#include <memory>
#else
#include <tr1/memory>
#endif

#include <Golem/Device/Katana450/canbus_interface.h>
#include <Golem/Device/Katana450/canopen_master_node.h>
#include <Golem/Device/Katana450/neuronics_version.h>

namespace tr1 {
#ifdef WIN32
	using boost::shared_ptr;
#else
	using std::tr1::shared_ptr;
#endif
}

class CANopenMaster
{
public:

	static const unsigned int NUM_OF_NODES = 6;

	NeuronicsVersion mVersion;

	std::vector<tr1::shared_ptr<CANopenMasterNode> > mCANopenNode;
	
private:

	bool mDebug;

	tr1::shared_ptr<boost::thread> mCANbusThread;

	bool mCANbusActive;
	bool mCANbusReady;

	tr1::shared_ptr<CanBusInterface> mCanBusInterface;

	unsigned int mPollingPeriod;

public:
	CANopenMaster  ();
	~CANopenMaster ();

	std::string getVersion ();

	void reset ();

	//!	CAN bus handling
	void        CanBusSetDevice (std::string device);
	bool        CanBusStart     (unsigned int timeout=2400);
	void        CanBusStop      ();
	bool        CanBusIsReady   () const;

	CanErrorFlags CanBusGetStatus     ();
	std::string   CanBusFlags2str     (CanErrorFlags flags);

	CanBusInterface::CanStatistic  CanBusgetStatistic  ();
	std::string                    CanBusStat2str      (CanBusInterface::CanStatistic stat);

	//!	AxisController specific (get Version by remote Frame)
	bool        getAxisVersion (unsigned char nodeId, CANopen_ui8 bootloader[3], CANopen_ui8 firmware[3], CANopen_ui16 &hardware, unsigned int timeout=50);
	std::string getAxisVersionString (unsigned char nodeId);

	//! Bootloader
	bool                            BootloaderIsAlive           (unsigned char nodeId) const;
	BootloaderFsm::EBootloaderState BootloaderGetState          (unsigned char nodeId) const;
	unsigned short                  BootloaderGetLastErrorCode  (unsigned char nodeId) const;
	CanBootloaderMaster::FlashLib   BootloaderGetFlashLibStatus (unsigned char nodeId) const;

	void BootloaderSendCmd (BootloaderFsm::EBootloaderCommand cmd, unsigned char nodeId);
	
	//!	NMT
	bool             NMTsendCmd            (ECanOpenNmtCmd cmd, unsigned char nodeId=0);
	ECanOpenNmtState NMTgetState           (unsigned char nodeId) const;
	void             NMTreset              (unsigned char nodeId = 0);
	bool             NMTisAlive            (unsigned char nodeId = 0) const;
	unsigned int     NMTgetBootupCounter   (unsigned char nodeId) const;
	unsigned int     NMTgetNotAliveCounter (unsigned char nodeId) const;

	//!	EMCY
	void            EMCYreset      (unsigned char nodeId);
	unsigned int    EMCYcount      (unsigned char nodeId) const;
	unsigned long   EMCYcountTotal (unsigned char nodeId) const;
	CanOpenEmcyDesc EMCYpop        (unsigned char nodeId);
	std::string     EMCY2str       (CanOpenEmcyDesc const &desc) const;

	//!	PDO interface
	// --------------------------------------------------------------------
	//!	Test a stored CanFrame for change
	bool PDOchanged(ECANopenPdoData val, unsigned char nodeId);

	void PDOsetActive (ECANopenPdo pdo, unsigned char nodeId);
	void PDOsetPassiv (ECANopenPdo pdo, unsigned char nodeId);
	bool PDOisActive  (ECANopenPdo pdo, unsigned char nodeId);

	CANopen_ui8  PDOreadVal_ui8  (ECANopenPdoData pdoVal, unsigned char nodeId);
	CANopen_i8   PDOreadVal_i8   (ECANopenPdoData pdoVal, unsigned char nodeId);
	CANopen_ui16 PDOreadVal_ui16 (ECANopenPdoData pdoVal, unsigned char nodeId);
	CANopen_i16  PDOreadVal_i16  (ECANopenPdoData pdoVal, unsigned char nodeId);
	CANopen_ui32 PDOreadVal_ui32 (ECANopenPdoData pdoVal, unsigned char nodeId);
	CANopen_i32  PDOreadVal_i32  (ECANopenPdoData pdoVal, unsigned char nodeId);

	void PDOwriteVal_ui8  (CANopen_ui8  const & val, ECANopenPdoData pdoVal, unsigned char nodeId);
	void PDOwriteVal_i8   (CANopen_i8   const & val, ECANopenPdoData pdoVal, unsigned char nodeId);
	void PDOwriteVal_ui16 (CANopen_ui16 const & val, ECANopenPdoData pdoVal, unsigned char nodeId);
	void PDOwriteVal_i16  (CANopen_i16  const & val, ECANopenPdoData pdoVal, unsigned char nodeId);
	void PDOwriteVal_ui32 (CANopen_ui32 const & val, ECANopenPdoData pdoVal, unsigned char nodeId);
	void PDOwriteVal_i32  (CANopen_i32  const & val, ECANopenPdoData pdoVal, unsigned char nodeId);

	void PDOsend (ECANopenPdoData pdoVal, unsigned char nodeId);
	void PDOsend (ECANopenPdo     pdo,    unsigned char nodeId);

	//!	SDO interface
	// TODO

private:

	void CANbus_thread ();
};

#endif // _CANOPEN_MASTER_
