/* ----------------------------------------------------------------------------
   CANopen NMT Master for Katana 1.2 Robots

   Copyright (C) 2007 Neuronics AG
   Lukas Burger, 2007
   ------------------------------------------------------------------------- */
#ifndef _CANOPEN_NMT_MASTER_
#define _CANOPEN_NMT_MASTER_

#include <Golem/Device/Katana450/Timer.h>

#include <Golem/Device/Katana450/canopen.h>

/** \brief class to handle the NMT heartbeats of a CANopen Node
 * This class processes the CANopen HeartBeats of a CANopen node.
 * It provides the information about boot counts of the node,
 * heartbeat failures and if the node is up.
 */
class CANopenNmtMaster
{
private:

	//! the last seen state of the CANopen node monitored
	ECanOpenNmtState mState;

	//! the regular HeartBeat Period of the CANopen node in [ms]
	unsigned int     mHeatBeatPeriod; 
	
	//! the over time allowed before node is maked as not alive in [%]
	unsigned int     mOverTime;
	
	//! counts nummber of Bootup NMT arrived
	unsigned int     mBootCounter;
	
	//! counts nummber of NMT HeartBeat received with getState(state)
	unsigned long    mHeartBeatCounter;
	
	//! counts nummber of isAlive->isNotAlive transitions
	unsigned int     mNotAliveCounter;
	
	//! timer used to monitor the heartbeat period
	KNI::Timer       mTimer;

public:

	/** Constructor
	 * \param period: HeartBeat period
	 * \param overtime: allowed overtime in %
	 */
	CANopenNmtMaster(int period = CANOPEN_HEARTBEAT_PERIOD, int overtime = CANOPEN_HEARTBEAT_OVERTIME_ALLOWED);

	//!	reset to init state nop and clear all counters
	void reset ();

	/** set a NMT state the arrives with a heartbeat CANframe
	 * \param state: ECanOpenNmtState
	 */
	void setState (ECanOpenNmtState state);

	/** check if CANopen node is alive
	 * \return alive status (true = alive)
	 */
	bool isAlive () const;

	/** read last NMT state of the CANopen node
	 * \return state: ECanOpenNmtState
	 */
	ECanOpenNmtState getState () const;
	
	/** read bootup counter 
	 * \return mBootCounter
	 */
	unsigned int getBootupCounter () const;
	
	/** read not alive counter 
	 * \return mNotAliveCounter
	 */
	unsigned int getNotAliveCounter () const;
};

#endif // _CANOPEN_NMT_MASTER_
