#ifndef _AXISCONTROLLER_EMCY_H_
#define _AXISCONTROLLER_EMCY_H_

#include <string>

#include <Golem/Device/Katana450/canopen.h>

namespace AxisControllerEMCY {

	//! \brief translates a CanOpenEmcyDesc into a human readable string
	std::string emcyDsc2str (CanOpenEmcyDesc const &emcyDsc);

}

#endif // _AXISCONTROLLER_EMCY_H_
