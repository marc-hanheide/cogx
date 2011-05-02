#include <Golem/Device/Katana450/pdo_interface.h>
#include <stddef.h>
// ----------------------------------------------------------------------------
CanOpenPdoInterface::CanOpenPdoInterface(void)
{
	// size of the PdoList (num. of my PDOs)
	mSizePdoIndex =0;

	// offset of the smallest Function Code
	mPdoOffset = 0;

	// num. of PDO between higest and lowest PDO ID
	mPdoRange = 0;

	mSizeDataIndex= 0;
}

// ----------------------------------------------------------------------------
CanOpenPdoInterface::~CanOpenPdoInterface(void)
{
	delete[] mDataIndex;
	for (int i = 0; i < mSizePdoIndex; i++) {
		delete[] mPdoIndex[i].data;
	}
	delete[] mPdoIndex;
	delete[] mPdoLookup;
	delete mDirection;
}

// ----------------------------------------------------------------------------
void CanOpenPdoInterface::init(Direction::ENodeType type)
{
	mDirection = new Direction(type);
	
	Uint8 sizeUranusDataList = sizeof(CANOPEN_PDO_DATA_LIST) / sizeof(CANopenPdoDataDesc);

	// find the _used_ max number of data enums
	Uint8 dataEnumMax = 0;
	for (int i = 0; i < sizeUranusDataList; i++) {
		if (CANOPEN_PDO_DATA_LIST[i].name > dataEnumMax)
		{
			dataEnumMax = CANOPEN_PDO_DATA_LIST[i].name;
		}
	}

	// the numbers of DATAs is one more then the max enum (enum starts @ zero)
	dataEnumMax++;
	mSizeDataIndex = dataEnumMax;

	mDataIndex = new dataIndex[mSizeDataIndex];
	#ifdef _URANUS_H_
	if (mDataIndex == NULL) panic();
	#endif
	for (int i = 0; i < mSizeDataIndex; i++) {
		mDataIndex[i].used   = false;
		mDataIndex[i].pdo    = (ECANopenPdo) 0;
		mDataIndex[i].offset = 0;
		mDataIndex[i].size   = 0;
		mDataIndex[i].type   = (ECanOpenType) 0;
	}

	/*	populate the Data Index table

		the Data Lookup Table is to find (by enum name) the PDO a data value 
		belongs to and to find the offset an size of the data balue in the PDO.
	*/
	for (int i = 0; i < mSizeDataIndex; i++)
	{
		for (int j = 0; j < sizeUranusDataList; j++)
		{
			if ( i == (Uint8) CANOPEN_PDO_DATA_LIST[j].name )
			{
				mDataIndex[i].used   = true;
				mDataIndex[i].pdo    = CANOPEN_PDO_DATA_LIST[j].pdo;
				mDataIndex[i].offset = CANOPEN_PDO_DATA_LIST[j].offset;
				mDataIndex[i].size   = CANOPEN_PDO_DATA_LIST[j].size;
				mDataIndex[i].type   = CANOPEN_PDO_DATA_LIST[j].type;
				break;
			}
		}
	}

	// get the size of the Uranus PDP List
	Uint8 sizeUranusPdoList = sizeof(URANUS_PDO_LIST) / sizeof(CANopenPdoDesc);

	// get smallest and the biggest PDO num from pdoList
	// and find the biggest Enum used in the Uranus PDO Index
	Uint16 pdoStart  = 0x07FF;
	Uint16 pdoStop   = 0x0000;
	Uint8  pdoEnumMax = 0;
	for (int i = 0; i < sizeUranusPdoList; i++) {
		if (URANUS_PDO_LIST[i].functionCode < pdoStart)
		{
			pdoStart = URANUS_PDO_LIST[i].functionCode;
		}
		if (URANUS_PDO_LIST[i].functionCode > pdoStop)
		{
			pdoStop = URANUS_PDO_LIST[i].functionCode;
		}
		if (URANUS_PDO_LIST[i].pdo > pdoEnumMax)
		{
			pdoEnumMax = URANUS_PDO_LIST[i].pdo;
		}
	}
	// the numbers of PDOs is one more then the max enum (enum starts @ zero)
	pdoEnumMax++;
	mSizePdoIndex = pdoEnumMax;

	// get the Offset of the first FunctionCode
	mPdoOffset = ( pdoStart >> CAN_NODE_ID_BITS );

	//	calculate the range of PDOs used on the controller.
	/*	example:
		PDOs: 1, 2, 3, 6, 8, 9
		pdoStart = 1, pdoStop = 9
		mPdoRange = 9 - 1 + 1 = 9 ( Range 1 to 9 are 9 elements ) 
	*/
	mPdoRange = ( pdoStop >> CAN_NODE_ID_BITS ) - mPdoOffset + 1;

	mPdoIndex = new pdoIndex[mSizePdoIndex];
	#ifdef _URANUS_H_
	if (mPdoIndex == NULL) panic();
	#endif
	for (int i = 0; i < mSizePdoIndex; i++) {
		mPdoIndex[i].used         = false;
		mPdoIndex[i].active       = true;
		mPdoIndex[i].functionCode = 0;
		mPdoIndex[i].direction    = Direction::TX;
		mPdoIndex[i].trigger      = NO_TRIGGER;
		mPdoIndex[i].size         = 0;
		mPdoIndex[i].data         = NULL;
		mPdoIndex[i].changed      = false;
		mPdoIndex[i].mailbox      = NULL;
	}

	/*	populate the PDO Index table

		usage of the PDO Index Table:
		you hae the NAME (enum) of a PDO an like to know the
		other details of the PDO. Voila, the PdoIndex array is your friend.
	*/
	for (int i = 0; i < mSizePdoIndex; i++)
	{
		for (int j = 0; j < sizeUranusPdoList; j++)
		{
			if ( i == (Uint8)URANUS_PDO_LIST[j].pdo )
			{
				mPdoIndex[i].used         = true;
				mPdoIndex[i].functionCode = URANUS_PDO_LIST[j].functionCode;
				mPdoIndex[i].direction    = URANUS_PDO_LIST[j].direction;
				mPdoIndex[i].trigger      = URANUS_PDO_LIST[j].trigger;

				// calculate the size from all data in the PDO
				// this info is stored in the Data Index
				for (int k = 0; k < mSizeDataIndex; k++)
				{
					if ( i == (Uint8)mDataIndex[k].pdo &&
					     mDataIndex[k].used )
					{
						mPdoIndex[i].size += mDataIndex[k].size;
					}
				}

				// create the storage for the data as char array
				mPdoIndex[i].data         = new char[mPdoIndex[i].size];
				#ifdef _URANUS_H_
				if (mPdoIndex[i].data == NULL) panic();
				#endif
				// set new char array to 0
				for (int k=0; k < mPdoIndex[i].size; k++)
				{
					mPdoIndex[i].data[k] = 0;
				}

				mPdoIndex[i].changed      = false;
				mPdoIndex[i].mailbox      = NULL;
				break;
			}
		}
	}

	/*	The PDO Lookup Table is your friend when you need to know
		which PDO enum belongs to a FunctionCode.

		This table has to cover tho whole PDO Range of the Axis Contoller
		and it has holes, where the PDO enum is 0.

		To convert the Function Code into the Lookup Table Index use
		the getIndex function:
		index = FunctionCode >> CAN_NODE_ID_BITS - mPdoOffset
	*/
	mPdoLookup = new ECANopenPdo[mPdoRange];
	#ifdef _URANUS_H_
	if (mPdoLookup == NULL) panic();
	#endif
	for (unsigned int i = 0; i < mPdoRange; i++)
	{
		mPdoLookup[i] = (ECANopenPdo)0;
		for (unsigned int j = 0; j < mSizePdoIndex; j++)
		{
			if ( i == getIndex(mPdoIndex[j].functionCode) )
			{
				mPdoLookup[i] = static_cast<ECANopenPdo>(j);
			}
		}
	}
}

// ----------------------------------------------------------------------------
void CanOpenPdoInterface::reset ()
{
	for (int i = 0; i < mSizePdoIndex; i++)
	{
		if ( mPdoIndex[i].used )
		{
			for (int k=0; k < mPdoIndex[i].size; k++)
			{
				mPdoIndex[i].data[k] = 0;
			}
			mPdoIndex[i].changed = false;
		}
	}
}

/*****************************************************************************
 *
 * CAN Interface
 *
 *****************************************************************************/

// ----------------------------------------------------------------------------
void CanOpenPdoInterface::setActive(ECANopenPdo pdo)
{
	mPdoIndex[pdo].active = true;
}

// ----------------------------------------------------------------------------
void CanOpenPdoInterface::setPassiv(ECANopenPdo pdo)
{
	mPdoIndex[pdo].active = false;
}

// ----------------------------------------------------------------------------
bool CanOpenPdoInterface::isActive(ECANopenPdo pdo) const
{
	return mPdoIndex[pdo].active;
}

// ----------------------------------------------------------------------------
bool CanOpenPdoInterface::pushCanFrame(CanFrame const &frame) 
{
	Uint8 pdoIndex = (Uint8)getIndex(frame.id);
	if (pdoIndex > mPdoRange)
		return false;
	Uint8 index = mPdoLookup[pdoIndex];

	// do not allow index bigger then array
	if (index > mSizePdoIndex) return false;

	if ( mPdoIndex[index].used   &&
	     mPdoIndex[index].active &&
	     mPdoIndex[index].direction == mDirection->rx())
	{
		if (frame.size == mPdoIndex[index].size )
		{
			for (int i=0; i<frame.size; i++)
			{
				mPdoIndex[index].data[i] = frame.data[i];
			}
			mPdoIndex[index].changed = true;
			return true;
		}
		// else ERROR can frame size does not match
	}
	// else ERROR tryed to push CAN message into TX PDO memory
	return false;
}

// ----------------------------------------------------------------------------
bool CanOpenPdoInterface::popCanFrame(CanFrame &frame)
{
	Uint8 pdoIndex = (Uint8)getIndex(frame.id);
	if (pdoIndex > mPdoRange)
		return false;
	Uint8 index = mPdoLookup[pdoIndex];

	// do not allow index bigger then array
	if (index > mSizePdoIndex) return false;

	if ( mPdoIndex[index].used   &&
	     mPdoIndex[index].active &&
	     mPdoIndex[index].direction == mDirection->tx())
	{
		frame.size = mPdoIndex[index].size;
		for (int i=0; i<frame.size; i++)
		{
			frame.data[i] = mPdoIndex[index].data[i];
		}
		mPdoIndex[index].changed = false;
		return true;
	}
	// else ERROR tryed to pop CAN message from RX PDO memory
	return false;
}

// ----------------------------------------------------------------------------
CanFrame CanOpenPdoInterface::popCanFrame(ECANopenPdo pdo, unsigned char nodeId )
{
	CanFrame frame;
	frame.id = mPdoIndex[pdo].functionCode + nodeId;
	if ( mPdoIndex[pdo].used   &&
	     mPdoIndex[pdo].active &&
	     mPdoIndex[pdo].direction == mDirection->tx())
	{
		frame.size = mPdoIndex[pdo].size;
		for (int i=0; i<frame.size; i++)
		{
			frame.data[i] = mPdoIndex[pdo].data[i];
		}
		mPdoIndex[pdo].changed = false;
	}
	return frame;
}

// ----------------------------------------------------------------------------
CanFrame CanOpenPdoInterface::popCanFrame(ECANopenPdoData pdoVal, unsigned char nodeId )
{
	return popCanFrame( mDataIndex[pdoVal].pdo, nodeId );
}

// ----------------------------------------------------------------------------
bool CanOpenPdoInterface::changed(ECANopenPdo pdo)
{
	return mPdoIndex[pdo].changed;
}

// ----------------------------------------------------------------------------
bool CanOpenPdoInterface::changed(ECANopenPdoData val)
{
	return changed(mDataIndex[val].pdo);
}

/*****************************************************************************
 *
 * privat helper function
 *
 *****************************************************************************/
ECanOpenType CanOpenPdoInterface::getTypeofPDOData (ECANopenPdoData pdoData)
{
	return mDataIndex[pdoData].type;
}
// -----------------------------------------------------------------------------
ECANopenPdo CanOpenPdoInterface::getPDOofData (ECANopenPdoData pdoData)
{
	return mDataIndex[pdoData].pdo;
}
