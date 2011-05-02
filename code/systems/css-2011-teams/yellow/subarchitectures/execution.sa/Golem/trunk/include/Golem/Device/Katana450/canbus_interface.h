/* ----------------------------------------------------------------------------
   CAN Bus Interface for Peak PCAN CANbus Adapter

   Copyright (C) 2007 Neuronics AG
   Lukas Burger, 2007
   ------------------------------------------------------------------------- */
#ifndef _CANBUS_INTERFACE_H_
#define _CANBUS_INTERFACE_H_

#include <boost/format.hpp>
#include <fstream>

// CAN bus stuff
#ifdef WIN32
#include <Golem/Defs/System.h>
#include <PCAN-Light/Include/Pcan_usb.h>
#else // WIN32
#include <libpcan.h>
#endif // WIN32

#include <fcntl.h>    // O_RDWR

// our own stuff
#include <Golem/Device/Katana450/canframe.h>
#include <Golem/Device/Katana450/can_bus_msg.h>
#include <Golem/Device/Katana450/neuronics_version.h>

class CanBusInterface
{
public:

	// see file pcan.h
	struct  CANERR_FLAGS
	{	                             // bits  description
		unsigned int xmtfull:1;      // 0 transmit buffer full
		unsigned int overrun:1;      // 1 overrun in receive buffer
		unsigned int buslight:1;     // bus error, errorcounter limit reached
		unsigned int busheavy:1;     // bus error, errorcounter limit reached
		unsigned int busoff:1;       // bus error, 'bus off' state entered
		unsigned int qrcvempty:1;    // receive queue is empty
		unsigned int qoverrun:1;     // receive queue overrun
		unsigned int qxmtfull:1;     // transmit queue full 
		unsigned int regtest:1;      // test of controller registers failed
		unsigned int novxd:1;        // win95/98/me only
		unsigned int resource:1;     // can't create resource
		unsigned int illhandle:3;    // wrong handle, handle error
		unsigned int illparamtype:1; // illegal parameter
		unsigned int illparamval:1;  // value out of range
	};

	union CANERR_FLAGS_UNION
	{
		unsigned int        all;
		struct CANERR_FLAGS bit;
	};

	struct CanStatistic
	{
		unsigned int ReadCounter;   // counts all reads to this device from start
		unsigned int WriteCounter;  // counts all writes
		unsigned int ErrorCounter;  // counts all errors
		unsigned int ErrorFlag;     // gathers all errors
		unsigned int LastError;     // the last local error for this device
		unsigned int OpenPaths;     // number of open paths for this device

		CanStatistic () : ReadCounter(0),
		                  WriteCounter(0),
		                  ErrorCounter(0),
		                  ErrorFlag(0),
		                  LastError(0),
		                  OpenPaths(0)
		{}
	};

	struct Extended_Status
	{
		CanErrorFlags errFlags;     // PEAK CAN bus controller error flasg
		int           pendingReads;
		int           pendingWrites;

		Extended_Status () : errFlags(0), pendingReads(0), pendingWrites(0) {}
	};

	NeuronicsVersion mVersion;

private:

	std::ofstream mLogfile;

	std::string mDevice;

	HANDLE mCanHandle;

public:

	CanBusInterface();

	~CanBusInterface();

	/** get Version of the module
	 * @see NeuronicsVersion
	 */
	std::string getVersion ();

	/** \brief set CAN bus device
	 * \param device CAN bus device e.g. "/dev/pcanusb0"
	 */
	void setDevice(std::string device);

	//! open the CAN bus device
	bool open ();

	//! check if the CAN bus device is ready
	bool isReady () const;

	//! close the CAN bus device	
	void close   ();

	/** \brief send a CAN bus message
	 * \param frame reference to the CanFrame that will be sent
	 * \param flag to mark it as a remote request
	 * \return any CAN error may occured
	 */
	CanErrorFlags send (CanFrame const &frame, bool remoteFrame = false) const;

	//!	return the size of the receive buffer on the CAN device
	Extended_Status getExtStatus () const;

	//!  pcan.h: LINUX_CAN_Read()
	//	reads a message WITH TIMESTAMP from the CAN bus. If there is no message 
	//	to read the current request blocks until either a new message arrives 
	//	or a error occures.
	CanBusMsg read () const;

	//!	pcan.h: LINUX_CAN_Read_Timeout()
	//	reads a message WITH TIMESTAMP from the CAN bus. If there is no message 
	//	to read the current request blocks until either a new message arrives 
	//	or a timeout or a error occures.
	//	nMicroSeconds  > 0 -> Timeout in microseconds
	//	nMicroSeconds == 0 -> polling
	//	nMicroSeconds  < 0 -> blocking, same as LINUX_CAN_Read()
	CanBusMsg read_timeout (int timeout) const;

	//!	flush receive queue on the device
	void flush ();

	/**
	 * \return CanErrorFlags CAN Error Flag
	 */
	CanErrorFlags status () const;
	
	/** \brief translate CAN error flag into a string
	 *  \param flags CAN Error Flag
	 * \return string Human readeble error string
	 */
	std::string   status2str (CanErrorFlags flags) const;

	/**
	 * \return CanStatistic
	 */
	CanStatistic statistic () const;
	
	/** \brief translate CAN statistic flag into a string
	 *  \param s CanStatistic
	 * \return string Human readeble statistic string
	 */
	std::string  statistic2str (CanStatistic s) const;

	/** \brief translate CanFrme into a string
	 *  \param frame CanFrame
	 * \return string Human readeble CanFrame
	 */
	std::string CanFrame2str  (CanFrame frame) const;
	
	/** \brief translate CanBusMsg into a string
	 *  \param msg CanBusMsg
	 * \return string Human readeble CanBusMsg
	 */
	std::string CanBusMsg2str (CanBusMsg msg) const;
	
	/** \brief translate CanBusMsg into a string
	 *  \param msg CanBusMsg
	 * \return string for logfile of CanBusMsg
	 */
	std::string CanBusMsg2log (CanBusMsg msg) const;

private:

	/** check if a CAN message from the driver is a standart can frame
	 * \param m: reference of the TPCANMsg that will be checked
	 * \return true if it is a standart message
	 */
	bool isStdMsg(TPCANMsg const &m) const;

	/** \brief translate a TPCANMsg into a CanFrame
	 * \param m:  reference of the TPCANMsg that will be translated
	 * \return CanFrame
	 */
	CanFrame Msg2Frame(TPCANMsg const &m) const;

	/** \brief translate a CanFrame into a TPCANMsg
	 * \param frame: input CanFrame
	 * \param m:  reference of the TPCANMsg that will be written
	 */
	void Frame2Msg(CanFrame const &frame, TPCANMsg &m) const;
};

#endif // _CANBUS_INTERFACE_H_
