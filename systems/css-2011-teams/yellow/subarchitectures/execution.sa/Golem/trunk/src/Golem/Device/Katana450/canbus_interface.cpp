/* ----------------------------------------------------------------------------
   CAN Bus Interface for Peak PCAN CANbus Adapter

   Copyright (C) 2007 Neuronics AG
   Lukas Burger, 2007
   ------------------------------------------------------------------------- */
#include <Golem/Device/Katana450/canbus_interface.h>

#include <sstream>
#include <iomanip>  // std::setw, std::setfill 

// ----------------------------------------------------------------------------
CanBusInterface::CanBusInterface()
{
    mDevice    = std::string("/dev/pcanusb0"); //TODO: setDefault
    mCanHandle = 0;
}

// ----------------------------------------------------------------------------
CanBusInterface::~CanBusInterface()
{
	if (mCanHandle) {
#ifdef WIN32
		//CAN_Close();
#else
		CAN_Close(mCanHandle);
#endif
		mCanHandle = 0;
	}
}

// ----------------------------------------------------------------------------
std::string CanBusInterface::getVersion ()
{
	return mVersion.getVersion();
}

// ----------------------------------------------------------------------------
void CanBusInterface::setDevice (std::string device)
{
	mDevice = device;
}

// ----------------------------------------------------------------------------
bool CanBusInterface::open ()
{
	if (! mCanHandle)
	{
#ifdef WIN32
#else
		mCanHandle = LINUX_CAN_Open(mDevice.c_str(), O_RDWR);
#endif
		if (! mCanHandle) {
			return false;
		}
	}

	unsigned int bitrate = CAN_BAUD_1M;

	// mask the under 8 bit of the error flag because this are errors
	// that occur even if the device was opened correclty
#ifdef WIN32
	//CAN_Init(bitrate, CAN_INIT_TYPE_ST);
#else
	CAN_Init(mCanHandle, bitrate, CAN_INIT_TYPE_ST);
#endif
	return isReady();
}

// ----------------------------------------------------------------------------
bool CanBusInterface::isReady () const
{
	if (!mCanHandle)
		return false;
	/** the last byte is showing errors like:
	 * CAN_ERR_OK             0x0000  // no error
	 * CAN_ERR_XMTFULL        0x0001  // transmit buffer full
	 * CAN_ERR_OVERRUN        0x0002  // overrun in receive buffer
	 * CAN_ERR_BUSLIGHT       0x0004  // bus error, errorcounter limit reached
	 * CAN_ERR_BUSHEAVY       0x0008  // bus error, errorcounter limit reached
	 * CAN_ERR_BUSOFF         0x0010  // bus error, 'bus off' state entered
	 * CAN_ERR_QRCVEMPTY      0x0020  // receive queue is empty
	 * CAN_ERR_QOVERRUN       0x0040  // receive queue overrun
	 * CAN_ERR_QXMTFULL       0x0080  // transmit queue full
	 * when we have this errors we still have a working and open CAN bus device.
	 * Therefore we mask this errors out here.
	 */ 	
	if ((status() & 0x00) == 0)
		return true;
	else
		return false;
}

// ----------------------------------------------------------------------------
void CanBusInterface::close ()
{
	if (mCanHandle)
	{
#ifdef WIN32
		//CAN_Close();
#else
		CAN_Close(mCanHandle);
#endif
		mCanHandle = 0;
	}
}

// ----------------------------------------------------------------------------
CanErrorFlags CanBusInterface::send (CanFrame const &frame, bool remoteFrame) const
{
	TPCANMsg Message;

	Frame2Msg(frame, Message);

	if ( remoteFrame )
	{
		Message.MSGTYPE = MSGTYPE_RTR;
	}

#ifdef WIN32
	return 0;//CAN_Write(&Message);
#else
	return CAN_Write(mCanHandle, &Message);
#endif
}

// ----------------------------------------------------------------------------
CanBusInterface::Extended_Status CanBusInterface::getExtStatus () const
{
	Extended_Status extStat;
#ifdef WIN32
	extStat.errFlags = 0;
#else
	extStat.errFlags = LINUX_CAN_Extended_Status(mCanHandle, &extStat.pendingReads, &extStat.pendingWrites);
#endif
	return extStat;
}

// ----------------------------------------------------------------------------
CanBusMsg CanBusInterface::read () const
{
	CanBusMsg CanBusMsg;

#ifdef WIN32
#else
	TPCANRdMsg Message;
	CanBusMsg.errFlags     = LINUX_CAN_Read(mCanHandle, &Message);
	CanBusMsg.canFrame     = Msg2Frame(Message.Msg);
	CanBusMsg.rt_timestamp = true;
	CanBusMsg.dwTime       = Message.dwTime;
	CanBusMsg.wUsec        = Message.wUsec;
#endif

	return CanBusMsg;
}

// ----------------------------------------------------------------------------
CanBusMsg CanBusInterface::read_timeout (int timeout) const
{
	CanBusMsg CanBusMsg;

#ifdef WIN32
#else
	TPCANRdMsg Message;
	CanBusMsg.errFlags     = LINUX_CAN_Read_Timeout(mCanHandle, &Message, timeout);
	CanBusMsg.canFrame     = Msg2Frame(Message.Msg);
	CanBusMsg.rt_timestamp = true;
	CanBusMsg.dwTime       = Message.dwTime;
	CanBusMsg.wUsec        = Message.wUsec;
#endif
	
	return CanBusMsg;
}

// ----------------------------------------------------------------------------
void CanBusInterface::flush()
{
	for ( int i=getExtStatus().pendingReads; i>0; i-- )
	{
		read();
	}
}

// ----------------------------------------------------------------------------
CanErrorFlags CanBusInterface::status () const
{
#ifdef WIN32
	return 0;//CAN_Status();
#else
	return CAN_Status(mCanHandle);
#endif
}

// ----------------------------------------------------------------------------
std::string CanBusInterface::status2str (CanErrorFlags err) const
{
	std::string errString;
	// all errors codes from file pcan.h
	if (err != CAN_ERR_OK )
	{
		if ( err & CAN_ERR_XMTFULL )       errString += "transmit buffer full; "                  ;
		if ( err & CAN_ERR_OVERRUN )       errString += "overrun in receive buffer; "             ;
		if ( err & CAN_ERR_BUSLIGHT )      errString += "bus error, errorcounter limit reached; " ;
		if ( err & CAN_ERR_BUSHEAVY )      errString += "bus error, errorcounter limit reached; " ;
		if ( err & CAN_ERR_BUSOFF )        errString += "bus error, 'bus off' state entered; "    ;
		if ( err & CAN_ERR_QRCVEMPTY )     errString += "receive queue is empty; "                ;
		if ( err & CAN_ERR_QOVERRUN )      errString += "receive queue overrun; "                 ;
		if ( err & CAN_ERR_QXMTFULL )      errString += "transmit queue full; "                   ;
		if ( err & CAN_ERR_REGTEST )       errString += "test of controller registers failed; "   ;
#ifndef WIN32
		if ( err & CAN_ERR_NOVXD )         errString += "Win95/98/ME only; "                      ;
#endif
		if ( err & CAN_ERR_RESOURCE )      errString += "can't create resource; "                 ;
		if ( err & CAN_ERR_ILLPARAMTYPE)   errString += "illegal parameter; "                     ;
		if ( err & CAN_ERR_ILLPARAMVAL )   errString += "value out of range; "                    ;
#ifndef WIN32
		if ( err & CAN_ERRMASK_ILLHANDLE ) errString += "wrong handle, handle error; "            ;
#endif
	} else {
		errString += "no error; ";
	}
	return errString;
}

// ----------------------------------------------------------------------------
CanBusInterface::CanStatistic CanBusInterface::statistic (void) const
{
	CanStatistic stat;

#ifdef WIN32
#else
	TPDIAG s;
	// TODO: How to give an ERROR back?
	LINUX_CAN_Statistics(mCanHandle, &s);

	stat.ReadCounter  = s.dwReadCounter;
	stat.WriteCounter = s.dwWriteCounter;
	stat.ErrorCounter = s.dwErrorCounter;
	stat.ErrorFlag    = s.wErrorFlag;
	stat.LastError    = s.nLastError;
	stat.OpenPaths    = s.nOpenPaths;
#endif

	return stat;
}

// ----------------------------------------------------------------------------
std::string CanBusInterface::statistic2str (CanStatistic s) const
{
	boost::format format("count of reads  = %i\ncount of writes = %i\ncount of errors = %i\nlast CAN status = %i\nlast error      = %i\nopen paths      = %i\n");

	return (format % s.ReadCounter % s.WriteCounter % s.ErrorCounter % s.ErrorFlag % s.LastError % s.OpenPaths).str();
}

// ----------------------------------------------------------------------------
std::string CanBusInterface::CanFrame2str (CanFrame frame) const
{
	std::ostringstream stream;

	unsigned short size = frame.size;
	if ( size >= CANBUS_DATA_BYTE )
	{
		size = 8;
	}

	stream << std::hex << frame.id << "\t" << static_cast<unsigned int>(frame.size);
	for (unsigned int i=0; i<size; i++)
		stream << "\t"  << std::hex << std::setw(2) << static_cast<unsigned int>(frame.data[i]);
	for (unsigned int i=size; i<CANBUS_DATA_BYTE; i++)
		stream << "\t";

	return stream.str();
}

// ----------------------------------------------------------------------------
std::string CanBusInterface::CanBusMsg2str (CanBusMsg msg) const
{
	std::ostringstream stream;

	if ( msg.rt_timestamp )
	{
		stream << std::setw(10) << msg.dwTime << "." << std::setw(3) << std::left << msg.wUsec << "\t";
	} else {
		stream << std::setw(13) << " ";
	}
	stream << "\t";
	if ( msg.errFlags != 0 )
	{
		stream << CanFrame2str(msg.canFrame);
	}
	stream << "\t# ";
	stream << status2str(msg.errFlags);

	return stream.str();
}

// ----------------------------------------------------------------------------
std::string CanBusInterface::CanBusMsg2log (CanBusMsg msg) const
{
	std::ostringstream stream("");

	if ( msg.errFlags != 0 )
	{
	    if ( msg.rt_timestamp )
	    {
		stream << msg.dwTime << "." << msg.wUsec;
	    }

	    stream << "\t";

    	    unsigned short size = msg.canFrame.size;
    	    if ( size >= CANBUS_DATA_BYTE )
        	size = 8;
    	    stream << std::hex << msg.canFrame.id << "\t" << static_cast<unsigned int>(msg.canFrame.size);
    	    for (unsigned int i=0; i<size; i++)
        	stream << "\t"  << std::hex << std::setw(2) << static_cast<unsigned int>(msg.canFrame.data[i]);
	}
	return stream.str();
}

// ============================================================================
// private
// ============================================================================

// ----------------------------------------------------------------------------
bool CanBusInterface::isStdMsg(TPCANMsg const &m) const
{
	if ( m.MSGTYPE == MSGTYPE_STANDARD )
	{
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------
CanFrame CanBusInterface::Msg2Frame(TPCANMsg const &m) const
{
	CanFrame frame;
	frame.id = (unsigned short)m.ID;
	frame.size = static_cast<unsigned char>(m.LEN);

	for (int i=0; i<frame.size; i++)
	{
		frame.data[i] = m.DATA[i];
	}
	return frame;
}

// ----------------------------------------------------------------------------
void CanBusInterface::Frame2Msg(CanFrame const &frame, TPCANMsg &m) const
{
	m.MSGTYPE = MSGTYPE_STANDARD;
	m.ID  = frame.id;
	m.LEN = (BYTE)frame.size;

	for (int i=0; i<frame.size; i++)
	{
		m.DATA[i] = frame.data[i];
	}
}
