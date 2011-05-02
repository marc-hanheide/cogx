#include <Golem/Device/Katana450/axis_controller_emcy.h>
#include <Golem/Device/Katana450/tms320f2808dsp.h>

#include <sstream>
#include <iomanip>  // std::setw, std::setfill

namespace AxisControllerEMCY {

	std::string emcyDsc2str (CanOpenEmcyDesc const &emcyDsc)
	{
		std::ostringstream stream;
		unsigned int cobId;
		unsigned int overTime;
	
		TMS320F2808DSP::AxisCanBusErrorFlags CANbusErr;
		unsigned short CANbusTec;
		unsigned short CANbusRec;

		if ( emcyDsc.rt_timestamp )
		{
			stream << emcyDsc.dwTime << "." << std::left << std::setw(3) << emcyDsc.wUsec << "; ";
		}

		switch(emcyDsc.emcyCode)
		{
			case EMCY_NOP:
				stream << "no EMCY";
				break;
	
			case EMCY_RCV_MSG_LOST:
				cobId = emcyDsc.getByte(0) + (emcyDsc.getByte(1) << 8);
				stream << "receive message lost with COB ID " << cobId;
				break;

			case EMCY_SND_MSG_LOST:
				cobId = emcyDsc.getByte(0) + (emcyDsc.getByte(1) << 8);
				stream << "receive message lost " << cobId;
				break;
	
			case EMCY_NMT_STATE_ERR:
				stream << "unknown CANopen NMT state " << emcyDsc.getByte(0);
				break;
	
			case EMCY_CAN_BUS_ERR:
				CANbusErr.byte[0] = emcyDsc.getByte(0);
				CANbusErr.byte[1] = emcyDsc.getByte(1);
				CANbusTec = emcyDsc.getByte(2);
				CANbusRec = emcyDsc.getByte(3);
				stream  << "REC=" << std::setw(3) << CANbusRec << " TEC=" << std::setw(3) << CANbusTec << "; " << TMS320F2808DSP::CANbusErrFlag2String(CANbusErr);
				break;
	
			case EMCY_SCHEDULE_ERR:
				overTime = emcyDsc.getByte(0) + (emcyDsc.getByte(1) << 8);
				stream << "scheduling over time " << overTime;
				break;
	
			default:
				stream << "unknown EMCY type: ";
				stream << emcyDsc.emcyCode;
				stream << "; ";
				stream << emcyDsc.errorFlag;
				for (unsigned short i=0; i<CANOPEN_EMCY_MSG_BYTES; i++ )
				{
					stream << "; ";
					stream << emcyDsc.getByte(i);
				}
				break;
		}
		return stream.str();
	}
}
