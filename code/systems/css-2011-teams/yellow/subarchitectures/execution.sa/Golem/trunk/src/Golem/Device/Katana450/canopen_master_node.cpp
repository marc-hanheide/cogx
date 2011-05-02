/* ----------------------------------------------------------------------------
   CANopen Master Node for Katana 1.2 Robots

   Copyright (C) 2007 Neuronics AG
   Lukas Burger, 2007
   ------------------------------------------------------------------------- */
#include <Golem/Device/Katana450/canopen_master_node.h>
#include <Golem/Device/Katana450/axis_controller_emcy.h>

// To hanlde bootloader PDOs we need to know about them
#include <Golem/Device/Katana450/canBootloaderPDO.h>

#include <sstream>
//#include <iostream>

// ----------------------------------------------------------------------------
CANopenMasterNode::CANopenMasterNode(unsigned char nodeId) : mNodeId(nodeId)
{
	mPdoInterface.init(Direction::MASTER);
	reset();
}

// ----------------------------------------------------------------------------
std::string CANopenMasterNode::getVersion ()
{
	return mVersion.getVersion();
}
	
// ----------------------------------------------------------------------------
void CANopenMasterNode::reset ()
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	mNmtMaster.reset();
	mEmcyQueue.reset();
	mPdoInterface.reset();
//	mCanOpenSdoClient.reset();
}

// ----------------------------------------------------------------------------
unsigned char CANopenMasterNode::getNodeId() const
{
	return mNodeId;
}

// ----------------------------------------------------------------------------
void CANopenMasterNode::writeCanBusMsg(CanBusMsg msg)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	unsigned short nodeId   = msg.canFrame.id & CAN_NODE_ID_MASK;
	unsigned short funcCode = msg.canFrame.id & CAN_FUNC_CODE_MASK;

	if ( nodeId == mNodeId )
	{
		switch (funcCode)
		{
			case NMT_HeartBeat:
				mNmtMaster.setState(static_cast<ECanOpenNmtState>(msg.canFrame.getByte(0)));
				break;

			case NMT_Emergency:
				mEmcyQueue.pushCanBusMsg(msg);
				break;

			case NMT_MasterMessage:
				// TODO: what to do with such a message?
				break;

			case CanBootloaderPDO::Bootloaderstate:
			case CanBootloaderPDO::BootloaderError:
			case CanBootloaderPDO::BootloaderFlashStatus:
//			case CanBootloaderPDO::BootloaderWriteAck:
				// TODO: handle bootloader state PDOs
				mCanBootloaderMaster.pushCanFrame(msg.canFrame);
				break;
				
			default:
				mPdoInterface.pushCanFrame(msg.canFrame);
				break;
		}
	} else {
		// TODO: unknown CAN frame
	}
}

// ----------------------------------------------------------------------------
CanFrame CANopenMasterNode::readCanFrame(ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

//	std::cout << "pdoVal: " << pdoVal << ", Node Id: " << static_cast<unsigned int>(mNodeId) << std::endl;
	return mPdoInterface.popCanFrame(pdoVal, mNodeId);
}

// ----------------------------------------------------------------------------
CanFrame CANopenMasterNode::readCanFrame(ECANopenPdo pdo)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	return mPdoInterface.popCanFrame(pdo, mNodeId);
}

// ============================================================================
// NMT
// ============================================================================

// ----------------------------------------------------------------------------
void CANopenMasterNode::NMTreset ()
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	mNmtMaster.reset();
}

// ----------------------------------------------------------------------------
bool CANopenMasterNode::NMTisAlive () const
{
	return mNmtMaster.isAlive();
}

// ----------------------------------------------------------------------------
ECanOpenNmtState CANopenMasterNode::NMTgetState () const
{
	return mNmtMaster.getState();
}

// ----------------------------------------------------------------------------
unsigned int CANopenMasterNode::NMTgetBootupCounter () const
{
	return mNmtMaster.getBootupCounter();
}

// ----------------------------------------------------------------------------
unsigned int CANopenMasterNode::NMTgetNotAliveCounter () const
{
	return mNmtMaster.getNotAliveCounter();
}

// ============================================================================
// EMCY
// ============================================================================

// ----------------------------------------------------------------------------
void CANopenMasterNode::EMCYreset () 
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	return mEmcyQueue.reset();
}

// ----------------------------------------------------------------------------
unsigned int CANopenMasterNode::EMCYcount () const
{
	return mEmcyQueue.size();
}

// ----------------------------------------------------------------------------
unsigned long CANopenMasterNode::EMCYcountTotal () const
{
	return mEmcyQueue.counter();
}

// ----------------------------------------------------------------------------
CanOpenEmcyDesc CANopenMasterNode::EMCYpop ()
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	return mEmcyQueue.pop();
}

// ----------------------------------------------------------------------------
std::string CANopenMasterNode::EMCY2str  (CanOpenEmcyDesc const &desc) const
{
	return AxisControllerEMCY::emcyDsc2str (desc);
}

// ============================================================================
// PDO
// ============================================================================

// ----------------------------------------------------------------------------
bool CANopenMasterNode::PDOvalChanged(ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	return mPdoInterface.changed(pdoVal);
}

// ----------------------------------------------------------------------------
bool CANopenMasterNode::PDOchanged(ECANopenPdo pdo)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	return mPdoInterface.changed(pdo);
}

// ----------------------------------------------------------------------------
void CANopenMasterNode::PDOsetActive(ECANopenPdo pdo)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	mPdoInterface.setActive(pdo);
}

// ----------------------------------------------------------------------------
void CANopenMasterNode::PDOsetPassiv(ECANopenPdo pdo)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	mPdoInterface.setPassiv(pdo);
}

// ----------------------------------------------------------------------------
bool CANopenMasterNode::PDOisActive(ECANopenPdo pdo) const
{
	return mPdoInterface.isActive(pdo);
}

// ----------------------------------------------------------------------------
CANopen_ui8  CANopenMasterNode::PDOreadVal_ui8  (ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	CANopen_ui8 tmp = 0;
	mPdoInterface.popVal(tmp, pdoVal);
	return tmp;
}

// ----------------------------------------------------------------------------
CANopen_i8   CANopenMasterNode::PDOreadVal_i8   (ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	CANopen_i8 tmp = 0;
	mPdoInterface.popVal(tmp, pdoVal);
	return tmp;
}

// ----------------------------------------------------------------------------
CANopen_ui16 CANopenMasterNode::PDOreadVal_ui16 (ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	CANopen_ui16 tmp = 0;
	mPdoInterface.popVal(tmp, pdoVal);
	return tmp;
}

// ----------------------------------------------------------------------------
CANopen_i16  CANopenMasterNode::PDOreadVal_i16  (ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	CANopen_i16 tmp = 0;
	mPdoInterface.popVal(tmp, pdoVal);
	return tmp;
}

// ----------------------------------------------------------------------------
CANopen_ui32 CANopenMasterNode::PDOreadVal_ui32 (ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	CANopen_ui32 tmp = 0;
	mPdoInterface.popVal(tmp, pdoVal);
	return tmp;
}

// ----------------------------------------------------------------------------
CANopen_i32  CANopenMasterNode::PDOreadVal_i32  (ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	CANopen_i32 tmp = 0;
	mPdoInterface.popVal(tmp, pdoVal);
	return tmp;
}

// ----------------------------------------------------------------------------
bool CANopenMasterNode::PDOwriteVal_ui8 (CANopen_ui8  const & val, ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	return mPdoInterface.pushVal(val, pdoVal);
}

// ----------------------------------------------------------------------------
bool CANopenMasterNode::PDOwriteVal_i8 (CANopen_i8   const & val, ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	return mPdoInterface.pushVal(val, pdoVal);
}

// ----------------------------------------------------------------------------
bool CANopenMasterNode::PDOwriteVal_ui16 (CANopen_ui16 const & val, ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	return mPdoInterface.pushVal(val, pdoVal);
}

// ----------------------------------------------------------------------------
bool CANopenMasterNode::PDOwriteVal_i16 (CANopen_i16  const & val, ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	return mPdoInterface.pushVal(val, pdoVal);
}

// ----------------------------------------------------------------------------
bool CANopenMasterNode::PDOwriteVal_ui32 (CANopen_ui32 const & val, ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	return mPdoInterface.pushVal(val, pdoVal);
}

// ----------------------------------------------------------------------------
bool CANopenMasterNode::PDOwriteVal_i32 (CANopen_i32  const & val, ECANopenPdoData pdoVal)
{
	boost::mutex::scoped_lock scoped_lock(mNodeMutex);

	return mPdoInterface.pushVal(val, pdoVal);
}

// ----------------------------------------------------------------------------
ECanOpenType CANopenMasterNode::getTypeofPDOData(ECANopenPdoData pdoData)
{
	return mPdoInterface.getTypeofPDOData(pdoData);
}
// ----------------------------------------------------------------------------
ECANopenPdo CANopenMasterNode::getPDOofData(ECANopenPdoData pdoData)
{
	return mPdoInterface.getPDOofData(pdoData);
}

// ============================================================================
// private PDO
// ============================================================================

// ----------------------------------------------------------------------------
//void CANopenMasterNode::PDOwriteCanFrame (CanFrame const &frame)
//{
//	mPdoInterface.pushCanFrame(frame);
//}

// ----------------------------------------------------------------------------
//void CANopenMasterNode::PDOreadCanFrame  (CanFrame &frame)
//{
//	mPdoInterface.popCanFrame(frame);
//}

// ============================================================================
// private
// ============================================================================

// ----------------------------------------------------------------------------
std::string CANopenMasterNode::CanFrame2str (CanFrame frame)
{
	std::ostringstream stream;
	stream << std::hex << frame.id << " " << static_cast<unsigned int>(frame.size) << " ";
	for (unsigned int i=0; i<frame.size; i++)
		stream << static_cast<unsigned int>(frame.data[i]) << " ";
	stream << std::endl;
	return stream.str();
}
