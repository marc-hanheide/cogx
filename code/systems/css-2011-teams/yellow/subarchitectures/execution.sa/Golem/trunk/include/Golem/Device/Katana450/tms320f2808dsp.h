#ifndef _TMS320F2808DSP_H_
#define _TMS320F2808DSP_H_

#include <string>

// ----------------------------------------------------------------------------
namespace TMS320F2808DSP {

	/** \brief struct with the CAN bus error flags of the DSP2808 chip
	 * this implemements the flag register of the dsp2808 chip
	 * @see file Axis Controller Manual EMCY CAN Bus Error
	 */
	struct  AXISCANBUSERR_FLAGS {   // bits  description
		unsigned int EW:1;      // Warning status
		unsigned int EP:1;      // Error-passive state
		unsigned int BO:1;      // Bus-off status
		unsigned int ACKE:1;    // Acknowledge error
		unsigned int SE:1;      // Stuff error
		unsigned int CRCE:1;    // CRC error
		unsigned int SA1:1;     // Stuck at dominant error
		unsigned int BE:1;      // Bit error flag
		unsigned int FE:1;      // Form error flag
		unsigned int rsvd:7;    // reserved
	};
	
	union AxisCanBusErrorFlags {
		unsigned short             all;
		struct AXISCANBUSERR_FLAGS bit;
		unsigned char              byte[2];
	};

	//! \brief translates the CAN bus error register into a error string
	std::string CANbusErrFlag2String (AxisCanBusErrorFlags err);
};

#endif // _TMS320F2808DSP_H_
