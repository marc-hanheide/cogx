/* ----------------------------------------------------------------------------
   CANopen Master EMCY Queue for Katana 1.2 Robots

   Copyright (C) 2007 Neuronics AG
   Lukas Burger, 2007
   ------------------------------------------------------------------------- */
#ifndef _CANOPEN_EMCY_QUEUE_
#define _CANOPEN_EMCY_QUEUE_

#include <queue>
#include <string>

#include <Golem/Device/Katana450/canopen.h>
#include <Golem/Device/Katana450/canframe.h>
#include <Golem/Device/Katana450/can_bus_msg.h>

/** \brief EMCY buffer
 * This class is a buffer for arriving CANopen EMCY messages. 
 */
class EmcyQueue
{
private:

	//! number of arrived EMCY messages
	unsigned long mEMCYcounter;

	//! EMCY queue
	std::queue<CanOpenEmcyDesc> mEmcyDescQueue;

	//! an empty CanOpenEmcyDesc
	CanOpenEmcyDesc mNullEmcyDesc;

public:

	//! Constructor
	EmcyQueue ();

	//! reset mEMCYcounter and flush mEmcyDescQueue
	void reset ();

	/**	get size of the EMCY queue
	 * return size of the EMCY queue
	 */
	unsigned int size () const;

	/**	get the number of arrived EMCY messages
	 * \return number of arrived EMCY messages
	 */
	unsigned long counter () const;

	/** \brief push a CanFrame into the queue
	 * The EMCY message will be extraceted and pushed into the queue
	 * \param msg: CanBusMsg
	 */
	void pushCanBusMsg (CanBusMsg const &msg);

	/** get the first EMCY from the queue and delete it
	 * \return CanOpenEmcyDesc
	 */
	CanOpenEmcyDesc pop ();

private:

	//!	empty the EMCY queue
	void flush ();

	/** translate a CanBusMsg into a CanOpenEmcyDesc
	 * \param msg: CanBusMsg
	 * \return EMCY description: CanOpenEmcyDesc
	 */
	CanOpenEmcyDesc CanBusMsg2EmcyDesc (CanBusMsg const &msg) const;
};

#endif // _CANOPEN_EMCY_QUEUE_
