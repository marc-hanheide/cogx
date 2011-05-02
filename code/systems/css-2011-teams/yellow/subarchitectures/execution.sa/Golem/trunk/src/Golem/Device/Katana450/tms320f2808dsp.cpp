
#include <Golem/Device/Katana450/tms320f2808dsp.h>

// ----------------------------------------------------------------------------
namespace TMS320F2808DSP {
	std::string CANbusErrFlag2String (AxisCanBusErrorFlags err)
	{
		std::string errString = "";
	
		err.bit.rsvd = 0;
		if (err.all != 0 )
		{
			errString += "CAN bus: ";
			if ( err.bit.EW   )       errString += "Warning status; "          ;
			if ( err.bit.EP   )       errString += "Error-passive state; "     ;
			if ( err.bit.BO   )       errString += "Bus-off status; "          ;
			if ( err.bit.ACKE )       errString += "Acknowledge error; "       ;
			if ( err.bit.SE   )       errString += "Stuff error; "             ;
			if ( err.bit.CRCE )       errString += "CRC error; "               ;
			if ( err.bit.SA1  )       errString += "Stuck at dominant error; " ;
			if ( err.bit.BE   )       errString += "Bit error flag; "          ;
			if ( err.bit.FE   )       errString += "Form error flag; "         ;
		} else {
			errString += "no CAN bus error; ";
		}
		return errString;
	}
}
