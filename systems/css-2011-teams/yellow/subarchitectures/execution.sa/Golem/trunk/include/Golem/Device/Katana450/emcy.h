#ifndef _CANOPEN_EMCY_H_
#define _CANOPEN_EMCY_H_

const unsigned short CANOPEN_EMCY_MSG_BYTES = 5;

struct CanOpenEmcyDesc {
	ECanOpenEmcyCode   emcyCode;     // CANopen EMCY code (defined in out own enum)
	unsigned char      errorFlag;    // CANopen ErrorFlag
	bool               rt_timestamp; // true when rt timestamp dwTime.wUsec used
	unsigned int       dwTime;       // a timestamp in msec (32bit)
	unsigned short     wUsec;        // remainder in micro-seconds (16bit)
	unsigned char      data[CANOPEN_EMCY_MSG_BYTES]; // the remaining 5 bytes can be used for out own error msg

	CanOpenEmcyDesc() :
		emcyCode     (EMCY_NOP),
		errorFlag    (0),
		rt_timestamp (false),
		dwTime       (0),
		wUsec        (0)
		{
			for (unsigned short i=0; i<CANOPEN_EMCY_MSG_BYTES; i++ )
			{
				data[i] = 0;
			}
		};

	unsigned char getByte(unsigned short byte) const
	{
		if ( byte < CANOPEN_EMCY_MSG_BYTES )
		{
			return data[byte];
		}
		return 0;
	};

	void setByte(unsigned short byte, unsigned char val)
	{
		if ( byte < CANOPEN_EMCY_MSG_BYTES )
		{
			data[byte] = val;
		}
	};

};

#endif // _CANOPEN_EMCY_H_
