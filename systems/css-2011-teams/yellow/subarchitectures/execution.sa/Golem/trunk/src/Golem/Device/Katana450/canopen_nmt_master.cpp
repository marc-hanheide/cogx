/* ----------------------------------------------------------------------------
   CANopen NMT Master for Katana 1.2 Robots

   Copyright (C) 2007 Neuronics AG
   Lukas Burger, 2007
   ------------------------------------------------------------------------- */
#include <Golem/Device/Katana450/canopen_nmt_master.h>

// ----------------------------------------------------------------------------
CANopenNmtMaster::CANopenNmtMaster(int period, int overtime) : mHeatBeatPeriod(period), mOverTime(overtime)
{
	reset();
}

// ----------------------------------------------------------------------------
void CANopenNmtMaster::reset ()
{
	mTimer.Set_And_Start(1);
	while ( ! mTimer.Elapsed() ) {};
	mState = NMT_State_Nop;
	mBootCounter = 0;
	mHeartBeatCounter = 0;
	mNotAliveCounter = 0;
}

// ----------------------------------------------------------------------------
void CANopenNmtMaster::setState (ECanOpenNmtState state)
{
	if ( state == NMT_State_Bootup )
	{
		mBootCounter++;
	}

	if ( mTimer.Elapsed() && mHeartBeatCounter != 0 )
	{
		mNotAliveCounter++;
	}

	mState = state;
	mTimer.Set_And_Start(mHeatBeatPeriod * mOverTime / 100);
	mHeartBeatCounter++;
}

// ----------------------------------------------------------------------------
bool CANopenNmtMaster::isAlive () const
{
	return (! mTimer.Elapsed());
}

// ----------------------------------------------------------------------------
ECanOpenNmtState CANopenNmtMaster::getState () const
{
	return mState;
}

// ----------------------------------------------------------------------------
unsigned int  CANopenNmtMaster::getBootupCounter () const
{
	return mBootCounter;
}

// ----------------------------------------------------------------------------
unsigned int  CANopenNmtMaster::getNotAliveCounter () const
{
	return mNotAliveCounter;
}
