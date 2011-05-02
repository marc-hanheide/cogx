#ifndef _CAN_BUS_MSG_H_
#define _CAN_BUS_MSG_H_

typedef unsigned int CanErrorFlags;

struct CanBusMsg
{
	CanFrame       canFrame;
	bool           rt_timestamp; // realtime timestamp valid
	unsigned int   dwTime;       // a timestamp in msec (32bit)
	unsigned short wUsec;        // remainder in micro-seconds (16bit)
	CanErrorFlags  errFlags;     // PEAK CAN bus controller error flasg

	CanBusMsg()
	{
		rt_timestamp  = false;
		dwTime        = 0;
		wUsec         = 0;
		errFlags      = 0;
	};
};

#endif // _CAN_BUS_MGS_H_
