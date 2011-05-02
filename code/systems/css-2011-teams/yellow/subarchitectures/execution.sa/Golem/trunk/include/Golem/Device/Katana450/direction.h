#ifndef _DIRECTION_H_
#define _DIRECTION_H_

//!\brief Class for the communication direction
class Direction
{
	public:
		enum ENodeType {MASTER, SLAVE};
		enum EDir      {TX = 0, RX = 1};
	private:
		EDir mTx;
		EDir mRx;
	public:
		Direction(ENodeType type)
		{
			mTx = TX;
			mRx = RX;
			if ( type == MASTER )
			{
				mTx = RX;
				mRx = TX;
			};
		};
		EDir tx (void) {return mTx;};
		EDir rx (void) {return mRx;};
};

#endif // _DIRECTION_H_

