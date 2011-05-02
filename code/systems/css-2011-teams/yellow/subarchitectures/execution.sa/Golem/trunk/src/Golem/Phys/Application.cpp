/** @file Application.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Phys/Application.h>
#include <Golem/Phys/Data.h>
#include <Golem/Phys/Msg.h>
#include <Golem/Tools/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Application::Application() {
}

Application::~Application() {
}

int Application::main(int argc, char *argv[]) {
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
		pParser = parserDesc.create();
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
		pXMLContext = pParser->getContextRoot()->getContextFirst("golem");
		if (pXMLContext == NULL)
			throw MsgApplication(Message::LEVEL_CRIT, "Unknown configuration file: %s", cfg.c_str());

		// Create program context
		golem::Context::Desc contextDesc;
		XMLData(contextDesc, pXMLContext);
		pContext = contextDesc.create(); // throws
		
		// Create Universe
		Universe::Desc universeDesc;
		universeDesc.name = "Golem";
		XMLData(universeDesc, pXMLContext->getContextFirst("universe"));
		universeDesc.argc = argc;
		universeDesc.argv = argv;
		pUniverse = universeDesc.create(*pContext);
		
		// Create scene
		Scene::Desc sceneDesc;
		sceneDesc.name = "Demonstration program";
		XMLData(sceneDesc, pXMLContext->getContextFirst("scene"));
		pScene = pUniverse->createScene(sceneDesc);
		
		// Launch universe
		pUniverse->launch();

		// run application
		run(argc, argv);
		
		return 0;
	}
	catch (const Message& msg) {
		std::cerr << msg.str() << std::endl;
	}
	catch (const std::exception &ex) {
		std::cerr << Message(Message::LEVEL_CRIT, "C++ exception: %s", ex.what()).str() << std::endl;
	}

	return 1;
}

//------------------------------------------------------------------------------
