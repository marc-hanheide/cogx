/* ----------------------------------------------------------------------------
   CANopen Master EMCY Queue for Katana 1.2 Robots

   Copyright (C) 2007 Neuronics AG
   Lukas Burger, 2007
   ------------------------------------------------------------------------- */
#include <Golem/Device/Katana450/canopen_emcy_queue.h>

EmcyQueue::EmcyQueue()
{
	reset();
}

// ----------------------------------------------------------------------------
void EmcyQueue::reset ()
{
	mEMCYcounter = 0;
	flush();
}

// ----------------------------------------------------------------------------
unsigned int EmcyQueue::size () const
{
	return mEmcyDescQueue.size();
}

// ----------------------------------------------------------------------------
unsigned long EmcyQueue::counter () const
{
	return mEMCYcounter;
}

// ----------------------------------------------------------------------------
void EmcyQueue::pushCanBusMsg (CanBusMsg const &msg)
{
	mEmcyDescQueue.push(CanBusMsg2EmcyDesc(msg));
	mEMCYcounter++;
}

// ----------------------------------------------------------------------------
CanOpenEmcyDesc EmcyQueue::pop ()
{
	if ( mEmcyDescQueue.size() > 0 )
	{
		CanOpenEmcyDesc desc = mEmcyDescQueue.front();
		mEmcyDescQueue.pop();
		return desc;
	} else {
		return mNullEmcyDesc;
	}
}

// ----------------------------------------------------------------------------
// private functions
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void EmcyQueue::flush ()
{
	while ( mEmcyDescQueue.size() > 0 )
	{
		mEmcyDescQueue.pop();
	}
}

// ----------------------------------------------------------------------------
CanOpenEmcyDesc EmcyQueue::CanBusMsg2EmcyDesc (CanBusMsg const &msg) const
{
	CanOpenEmcyDesc emcyDsc;

	emcyDsc.emcyCode     = static_cast<ECanOpenEmcyCode>(static_cast<unsigned int>(msg.canFrame.getByte(0)) | static_cast<unsigned int>(msg.canFrame.getByte(1)) << 8);
	emcyDsc.errorFlag    = 0;
	emcyDsc.rt_timestamp = msg.rt_timestamp;
	emcyDsc.dwTime       = msg.dwTime;
	emcyDsc.wUsec        = msg.wUsec;
	for (unsigned short i=0; i<CANOPEN_EMCY_MSG_BYTES; i++ )
	{
		emcyDsc.setByte(i, msg.canFrame.getByte(i+3) );
	}

	return emcyDsc;
}
