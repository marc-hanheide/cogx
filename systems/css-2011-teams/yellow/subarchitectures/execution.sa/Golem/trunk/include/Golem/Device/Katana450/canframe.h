#ifndef _CAN_FRAME_H_
#define _CAN_FRAME_H_

static const unsigned short CANBUS_DATA_BYTE = 8;

//!	CAN 2.0A data frame struct
struct CanFrame {
	unsigned short id;
	unsigned short size;
	unsigned char  data[CANBUS_DATA_BYTE];

	CanFrame()
	{
		id   = 0;
		size = 0;
		for (unsigned short i = 0; i < CANBUS_DATA_BYTE; i++)
		{
			data[i] = 0;
		}
	}

	unsigned char getByte(unsigned short byte) const
	{
		if ( byte < CANBUS_DATA_BYTE )
		{
			return data[byte];
		}
		return 0;
	};

	void setByte(unsigned short byte, unsigned char val)
	{
		if ( byte < CANBUS_DATA_BYTE )
		{
			data[byte] = val;
		}
	};
};

#endif
