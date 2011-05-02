#ifndef _CANOPEN_PDO_INTERFACE_H_
#define _CANOPEN_PDO_INTERFACE_H_

#include <typeinfo>

#include <Golem/Device/Katana450/canframe.h>
#include <Golem/Device/Katana450/canopen.h>
class CanMailBox;

class CanOpenPdoInterface
{
public:

	/*!	PDO index element

		The idee is the create an array where the index is the PDO function code
		and the element held the size, direction, changed status and the content of the PDO
		and a pointer to a char array[size] with the data bytes.
	*/
	struct pdoIndex {
		bool                     used;
		bool                     active;
		Uint16                   functionCode;
		Direction::EDir          direction;
		ECANopenPdoTriggerMethod trigger;
		Uint8                    size;
		bool                     changed;
		char *                   data;
		CanMailBox *             mailbox;
	};

	//!	create the PDO index with the size defined in the canopen_config.h file
	pdoIndex * mPdoIndex;

	//! size of the PdoList (num. of my PDOs)
	Uint8 mSizePdoIndex;
	
	//! offset of the smallest Function Code
	Uint8 mPdoOffset;
	
	//! num. of PDO between higest and lowest PDO ID
	Uint8 mPdoRange;
	
	//! array size used to index all PDO data enumerated in @see ECANopenPdoData
	Uint8 mSizeDataIndex;

	//! struct to lookup a PDO by enummerator as index
	struct dataIndex {
		bool           used;
		ECANopenPdo    pdo;
		Uint8          offset;
		Uint8          size;
		ECanOpenType   type;
	};

	//! lookup table to find which Axis Contoller Data is stored in which PDO
	dataIndex * mDataIndex;

	/** The PDO Lookup Table is your friend when you need to know
	 * which PDO enum belongs to a FunctionCode.
	 */
	ECANopenPdo * mPdoLookup;

private:

	//! stored the direction in which this PDOinterface works
	Direction * mDirection;

public:

	//! \brief constructor
	CanOpenPdoInterface (void);

	/** initialize the PDO interface
	 * \param direction
	 */
	void init (Direction::ENodeType = Direction::SLAVE);

	//!	reset all content in the PDO interface to 0
	void reset ();

	//! destructor
	~CanOpenPdoInterface(void);

	/** activate a specific PDO
	 * \param pdo
	 */
	void setActive (ECANopenPdo pdo);
	
	/** deactivate a specific PDO
	 * \param pdo
	 */
	void setPassiv (ECANopenPdo pdo);
	
	/** check if a specific PDO is active
	 * \param pdo
	 * \return true=active, false=not active
	 */
	bool isActive  (ECANopenPdo pdo) const;

	//!	store a CanFrame in the PDO interface memory
	bool pushCanFrame (CanFrame const &frame);

	/**	get a CanFrame from the PDO interface memory
	 * Preconfigure the CanFrame with the COB ID and handle it to the function
	 * the size and the data bytes will be stores to the CanFrame handled to the function.
	 * \param frame reference to the preconfigured CanFrame
	 * \return true=ok, false=error
	 */
	bool     popCanFrame (CanFrame &frame);

	/**	get a CanFrame from the PDO interface memory
	 * get a CanFrame from PDOinderface by the PDO enumerator
	 * \param pdo enum
	 * \return CanFrame
	 */
	CanFrame popCanFrame ( ECANopenPdo pdo,        unsigned char  nodeId = 0 );

	/**	get a CanFrame from the PDO interface memory
	 * get a CanFrame from PDOinderface by the PDOdata enumerator
	 * \param pdoData enum
	 * \return CanFrame
	 */
	CanFrame popCanFrame ( ECANopenPdoData pdoVal, unsigned char  nodeId = 0 );

	/**	Test a stored CanFrame for change.
	 * Every write access to the PDO interface memory will set the change bit of the
	 * affected CanFrame and every read access will delete this bit.
	 * \param pdo enum
	 * \return true=changed, false=unchanged
	 */
	bool changed(ECANopenPdo pdo);

	/**	Test a stored CanFrame for change.
	 * DO NOT USE THIS! It resets the changed flag for the whole PDO, not only for the value. --MRE
	 */
	bool changed(ECANopenPdoData val);

	//!	Application interface to the PDO interface memory.
	//	This funtions are wrapper funtions to access to the single values stored
	//	in the CanFrames ind the PDO interface.
	
	//! get Type of used PDO data
	ECanOpenType getTypeofPDOData(ECANopenPdoData pdoData);

	//! Finds out which PDO some data value belongs to.
	ECANopenPdo getPDOofData (ECANopenPdoData pdoData);

	/*!	push a single value into the PDO interface memory

		Since different types can be stored in a CanFrame data field different pushVal
		functions has to be implemented.

		This function sets the change bit of the affected CanFrame in the PDO interface memory
		to true.

		@param val the value to be stored into the PDO interface memory
		@param pdoVal the name of the single value (parameter). (necessary to get index and offset
		off the value.
	*/
	template<typename _T>
	bool pushVal (_T const & val, ECANopenPdoData pdoVal)
	{
		if ( mPdoIndex[ mDataIndex[pdoVal].pdo ].direction == mDirection->tx() &&
		     mDataIndex[pdoVal].used &&
		     mPdoIndex[ mDataIndex[pdoVal].pdo ].active &&
			 pdoVal <= mSizeDataIndex )
		{
			for (int i = 0; i < mDataIndex[pdoVal].size; i++)
			{
				mPdoIndex[ mDataIndex[pdoVal].pdo ].data[ i + mDataIndex[pdoVal].offset ] = getByte(val, i);
			}
			mPdoIndex[ mDataIndex[pdoVal].pdo ].changed = true;
			return true;
		} else {
			return false;
		}
	}

	/*!	pop a single value from the PDO interface memory

		Since different types can be stored in a CanFrame data field different popVal
		functions has to be implemented.

		This function sets the cnaged bit of the affected CanFrame in the PDO interface memory
		to false.

		@param val the value from the PDO interface memeory will be stored into the passed reference
		@param pdoVal the name of the single value (parameter). (necessary to get index and offset
		off the value.
	*/
	template<typename _T>
	bool popVal (_T & val, ECANopenPdoData pdoVal)
	{
		if (mDataIndex[pdoVal].used &&
		     mPdoIndex[ mDataIndex[pdoVal].pdo ].active &&
			 pdoVal <= mSizeDataIndex )
		{
			for (int i = 0; i < mDataIndex[pdoVal].size; i++)
			{
				setByte(val, mPdoIndex[ mDataIndex[pdoVal].pdo ].data[ i + mDataIndex[pdoVal].offset ], i);
			}
			mPdoIndex[ mDataIndex[pdoVal].pdo ].changed = false;
			return true;
		} else {
			return false;
		}
	}

	template<typename _T>
	_T popVal(ECANopenPdoData pdoVal)
	{
		_T tmp = 0;
		popVal(tmp, pdoVal);
		return tmp;
	}
	
private:

	//!	calculates the Function Code from CAN COB ID and returns the index number 
	//	to access the PDOindex array
	Uint16 getIndex(Uint16 CanCobId)
	{
		return (CanCobId >> CAN_NODE_ID_BITS) - mPdoOffset;
	}

	//!	returns the byte[index] (8 bit) as char from the input variable data 
	template<typename _T>
	char getByte(_T data,  Uint8 index) const
	{
		Uint8 bitMasqClear = 0xFF;

		for (int i = 0; i < index; i++)
		{
			data = data >> 8;
		}

		return (char) data & bitMasqClear;
	}

	//!	sets the byte[index] (8 bit) of the input variable val with data
	template<typename _T>
	void setByte(_T &val,  char data, Uint8 index) const
	{
		if ( typeid(_T) == typeid(Uint8) )
		{
			val = (_T)data;
		} else {
			_T bitMasqClear = 0xFF;
			_T bitMasqSet   = (Uint16)data;

			for (int i = 0; i < index; i++)
			{
				bitMasqClear = bitMasqClear << 8;
				bitMasqSet   = bitMasqSet   << 8;
			}

			val = static_cast<_T>(val & ~bitMasqClear) | bitMasqSet;
		}
	}
};

#endif // _CANOPEN_PDO_INTERFACE_H_
