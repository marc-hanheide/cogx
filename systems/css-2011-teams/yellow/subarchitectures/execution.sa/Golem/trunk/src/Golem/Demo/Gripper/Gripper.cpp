/** @file Gripper.cpp
 * 
 * Demonstration program testing Katana gripper.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/Katana/Katana.h>
#include <Golem/Tools/Data.h>
#include <Golem/Demo/Common/Tools.h>
#include <iostream>

using namespace golem;

//------------------------------------------------------------------------------

void armGripper(KatanaGripper &gripper, MessageStream* stream) {
	KatanaGripper::SensorDataSet zero, reading, threshold;

	gripper.gripperRecvSensorData(reading);
	for (size_t i = 0; i < reading.size(); i++)
		stream->write("Sensor #%d: {%d, %d}", i, reading[i].index, reading[i].value);

	threshold = zero = reading;
	for (size_t i = 0; i < threshold.size(); i++)
		threshold[i].value += 5;

	stream->write("Closing gripper, high sensitivity...");
	if(gripper.gripperClose(threshold))
	  stream->write(" OK.");
	else
	  stream->write(" failed.");
	
	gripper.gripperRecvSensorData(reading);
	for (size_t i = 0; i < reading.size(); i++)
		stream->write("Sensor #%d: {%d, %d}", i, reading[i].index, reading[i].value);
	
	stream->write("Opening gripper ...");
	if(gripper.gripperOpen())
	  stream->write(" OK.");
	else
	  stream->write(" failed.");
	
	threshold = zero;
	for (size_t i = 0; i < threshold.size(); i++)
		threshold[i].value += 100;
	
	stream->write("Closing gripper, low sensitivity...");
	if(gripper.gripperClose(threshold))
	  stream->write(" OK.");
	else
	  stream->write(" failed.");
	
	gripper.gripperRecvSensorData(reading);
	for (size_t i = 0; i < reading.size(); i++)
		stream->write("Sensor #%d: {%d, %d}", i, reading[i].index, reading[i].value);
	
	stream->write("Opening gripper ...");
	if(gripper.gripperOpen())
	  stream->write(" OK.");
	else
	  stream->write(" failed.");
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	try {
		// Determine configuration file name
		std::string cfg;
		if (argc == 1) {
			// default configuration file name
			cfg.assign(argv[0]);
#ifdef WIN32
			size_t pos = cfg.rfind(".exe"); // Windows only
			if (pos != std::string::npos) cfg.erase(pos);
#endif
			cfg.append(".xml");
		}
		else
			cfg.assign(argv[1]);

		// Create XML parser and load configuration file
		XMLParser::Desc parserDesc;
		XMLParser::Ptr pParser = parserDesc.create();
		try {
			FileReadStream fs(cfg.c_str());
			pParser->load(fs);
		}
		catch (const Message& msg) {
			std::cerr << msg.str() << std::endl;
			std::cout << "Usage: " << argv[0] << " <configuration_file>" << std::endl;
			return 1;
		}

		// Find program XML root context
		XMLContext* pXMLContext = pParser->getContextRoot()->getContextFirst("golem");
		if (pXMLContext == NULL)
			throw Message(Message::LEVEL_CRIT, "Unknown configuration file: %s", cfg.c_str());

		// Create program context
		golem::Context::Desc contextDesc;
		XMLData(contextDesc, pXMLContext);
		golem::Context::Ptr pContext = contextDesc.create(); // throws

		//-----------------------------------------------------------------------------

		// Create arm controller description
		// Load Katana driver
		Arm::Desc::Ptr pDesc = Arm::Desc::load(*pContext, "GolemDeviceKatana300");

		// Create arm controller
		pContext->getMessageStream()->write(Message::LEVEL_INFO, "Initialising %s...", pDesc->name.c_str());
		Arm::Ptr pArm = pDesc->create(*pContext);
		KatanaGripper* pKatanaGripper = dynamic_cast<KatanaGripper*>(&*pArm);
		if (pKatanaGripper == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to obtain pointer to KatanaGripper", cfg.c_str());
		
		// Display arm information
		armInfo(*pArm);
		
		// Gripper demo
		armGripper(*pKatanaGripper, pContext->getMessageStream());

		// Wait for some time
		PerfTimer::sleep(SecTmReal(5.0));

		pContext->getMessageStream()->write(Message::LEVEL_INFO, "Good bye!");
	}
	catch (const Message& msg) {
		std::cerr << msg.str() << std::endl;
	}
	catch (const std::exception &ex) {
		std::cerr << Message(Message::LEVEL_CRIT, "C++ exception: %s", ex.what()).str() << std::endl;
	}

	return 0;
}
