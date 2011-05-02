/** @file Joint.cpp
 * 
 * Demonstration program which moves the arm to the zero pose.
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the Katana arm simulator
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Ctrl/Arm.h>
#include <Golem/Tools/Data.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Demo/Common/Tools.h>
#include <iostream>

using namespace golem;

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
		if (pContext == NULL)
			return 1;

		// Do not display LEVEL_DEBUG messages (only with level at least LEVEL_ERROR)
		//context()->getLogger()->setMsgFilter(MessageFilter::Ptr(new LevelFilter<Message>(Message::LEVEL_ERROR)));

		//-----------------------------------------------------------------------------

		// Get arm driver name
		std::string driver;
		XMLData("driver", driver, pXMLContext->getContextFirst("arm"));
		// Load driver
		Arm::Desc::Ptr pArmDesc = Arm::Desc::load(*pContext, driver);

		// Create arm controller
		pContext->getMessageStream()->write(Message::LEVEL_INFO, "Initialising arm controller...");
		Arm::Ptr pArm = pArmDesc->create(*pContext);

		// Display arm information
		armInfo(*pArm);
		
		// Move the arm to the zero pose
		armZeroMove(*pArm, SecTmReal(0.5), SecTmReal(0.5), SecTmReal(5.0));

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
