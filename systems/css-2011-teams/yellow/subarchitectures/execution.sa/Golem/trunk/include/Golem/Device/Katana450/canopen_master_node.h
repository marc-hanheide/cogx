/* ----------------------------------------------------------------------------
   CANopen Master Node for Katana 1.2 Robots

   Copyright (C) 2007 Neuronics AG
   Lukas Burger, 2007
   ------------------------------------------------------------------------- */
#ifndef _CANOPEN_MASTER_NODE_
#define _CANOPEN_MASTER_NODE_

#include <boost/thread/mutex.hpp>
#include <iostream>

#include <Golem/Device/Katana450/canopen.h>
#include <Golem/Device/Katana450/pdo_interface.h>
#include <Golem/Device/Katana450/can_bootloader_master.h>

//! assert CANopen types.h in canopen.h:
#include <boost/static_assert.hpp>
namespace arch_conditions {
   BOOST_STATIC_ASSERT(sizeof(Uint8)        == 1);
   BOOST_STATIC_ASSERT(sizeof(CANopen_ui8)  == 1);
   BOOST_STATIC_ASSERT(sizeof(Uint16)       == 2);
   BOOST_STATIC_ASSERT(sizeof(CANopen_ui16) == 2);
   BOOST_STATIC_ASSERT(sizeof(CANopen_ui32) == 4);
}

#include <Golem/Device/Katana450/canopen_nmt_master.h>
#include <Golem/Device/Katana450/canopen_emcy_queue.h>
#include <Golem/Device/Katana450/neuronics_version.h>

/** \brief A CANopen Master Node
 * This class processes all CANopen messages from a CANopen Node
 * and gives access to all sort of data provided by the node
 * and allowes to contoll the node and sent data to it.
 */
class CANopenMasterNode
{
public:

	//! Neuronics Software Version Information
	NeuronicsVersion mVersion;

	//! class that handles the CAN Bootloader PDO objects	
	CanBootloaderMaster mCanBootloaderMaster;

private:

	//! CANopen Node id
	unsigned char mNodeId;

	//! mutex to ansure the thread safety of the class
	boost::mutex mNodeMutex;

	//! class that handles the CANopen NMT objects
	CANopenNmtMaster mNmtMaster;

	//! class that handles the CANopen EMCY objects
	EmcyQueue mEmcyQueue;

	//! class that handles the CANopen PDO objects	
	CanOpenPdoInterface mPdoInterface;

public:
	
	/** \brief Constructor
	 * \param nodeId: CANopen Node Id
	 */
	CANopenMasterNode (unsigned char nodeId);

	/** \brief Read Neuronics software Version
	 * \return Version String
	 */
	std::string getVersion ();

	//! initialize the CANopenMasterNode
	void reset ();

	/** \brief read CANopen Node Id
	 * \return CANopen node id
	 */
	unsigned char getNodeId () const;

	/** \brief write a CAN Bus Message to the CANopen Master Node
	 * \param msg: CanBusMsg
	 * 
	 */
	void writeCanBusMsg (CanBusMsg msg);

	/** \brief read a CAN Bus Message from the CANopen Master Node
	 * \param pdoVal: ECANopenPdoData
	 * \return CanFrame
	 */
	CanFrame readCanFrame(ECANopenPdoData pdoVal);

	/** \brief read a CAN Bus Message from the CANopen Master Node
	 * \param pdo: ECANopenPdo
	 * \return CanFrame
	 */
	CanFrame readCanFrame(ECANopenPdo pdo);

	// NMT functions
	// --------------------------------------------------------------------
	
	//! CANopenNmtMaster.reset()
	void NMTreset ();
	
	//! CANopenNmtMaster.isAlive()
	bool NMTisAlive () const;
	
	//! CANopenNmtMaster.getState()
	ECanOpenNmtState NMTgetState () const;
	
	//! CANopenNmtMaster.getBootupCounter()
	unsigned int NMTgetBootupCounter () const;
	
	//! CANopenNmtMaster.getNotAliveCounter()
	unsigned int NMTgetNotAliveCounter () const;

	// EMCY functions
	// --------------------------------------------------------------------
	
	
	//! EmcyQueue.reset()
	void EMCYreset ();
	
	//! EmcyQueue.size()
	unsigned int EMCYcount () const;
	
	//! EmcyQueue.counter()
	unsigned long EMCYcountTotal () const;
	
	//! EmcyQueue.pop()
	CanOpenEmcyDesc EMCYpop ();
	
	//! AxisControllerEMCY::emcyDsc2str()
	std::string EMCY2str (CanOpenEmcyDesc const &desc) const;

	// PDO interface
	// --------------------------------------------------------------------
	
	/**	Test a stored CanFrame for change.
	 * \param pdoVal: ECANopenPdoData
	 * \return true if changed
	 */
	bool PDOvalChanged(ECANopenPdoData pdoVal);

	/**	Test a stored CanFrame for change.
	 * \param pdo: ECANopenPdo
	 * \return true if changed
	 */
	bool PDOchanged(ECANopenPdo pdo);

	//! CanOpenPdoInterface.setActive
	void PDOsetActive (ECANopenPdo pdo);

	//! CanOpenPdoInterface.setPassiv
	void PDOsetPassiv (ECANopenPdo pdo);

	//! CanOpenPdoInterface.isActive
	bool PDOisActive  (ECANopenPdo pdo) const;

	//! read PDO value from PDO interface
	CANopen_ui8  PDOreadVal_ui8  (ECANopenPdoData pdoVal);
	CANopen_i8   PDOreadVal_i8   (ECANopenPdoData pdoVal);
	CANopen_ui16 PDOreadVal_ui16 (ECANopenPdoData pdoVal);
	CANopen_i16  PDOreadVal_i16  (ECANopenPdoData pdoVal);
	CANopen_ui32 PDOreadVal_ui32 (ECANopenPdoData pdoVal);
	CANopen_i32  PDOreadVal_i32  (ECANopenPdoData pdoVal);

	//! write PDO value into PDO interface
	bool PDOwriteVal_ui8  (CANopen_ui8  const & val, ECANopenPdoData pdoVal);
	bool PDOwriteVal_i8   (CANopen_i8   const & val, ECANopenPdoData pdoVal);
	bool PDOwriteVal_ui16 (CANopen_ui16 const & val, ECANopenPdoData pdoVal);
	bool PDOwriteVal_i16  (CANopen_i16  const & val, ECANopenPdoData pdoVal);
	bool PDOwriteVal_ui32 (CANopen_ui32 const & val, ECANopenPdoData pdoVal);
	bool PDOwriteVal_i32  (CANopen_i32  const & val, ECANopenPdoData pdoVal);
	
	//! helper Function to get datatype out of the corresponding PDO Data
	ECanOpenType getTypeofPDOData(ECANopenPdoData pdoData);
	//! helper Function to find out which PDO contains a given data value
	ECANopenPdo getPDOofData(ECANopenPdoData pdoData);

private:

	//!	store a CanFrame in the PDO interface memory
	void PDOwriteCanFrame (CanFrame const &frame);

	/**	get a CanFrame from the PDO interface memory
	 * Preconfigure the CanFrame with the COB ID and handle it to the function
	 * the size and the data bytes will be stores to the CanFrame handled to the function.
	 */
	void PDOreadCanFrame  (CanFrame &frame);

	//!	get a string from a CanFrame
	std::string CanFrame2str (CanFrame frame);

protected:

	//! copy constructor
	CANopenMasterNode(const CANopenMasterNode&);
};

#endif // _CANOPEN_MASTER_NODE_
