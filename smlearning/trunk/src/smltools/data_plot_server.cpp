#include <Ice/Ice.h>
#include <smltools/PlotAppI.h>

using namespace smlearning;

int main(int argc, char **argv)
{
	QApplication a(argc, argv);
	int status = 0;
	Ice::CommunicatorPtr ic;
	try {
		ic = Ice::initialize(argc, argv);
		Ice::ObjectAdapterPtr adapter = ic->createObjectAdapterWithEndpoints("DataPlotterAdapter", "default -p 8174");
		Ice::ObjectPtr plotApp = new PlotAppI (argc, argv, ic);
		adapter->add(plotApp,
			     ic->stringToIdentity("DataPlotter"));
		adapter->activate();
		std::cout << "DataPlotter running" << std::endl;
		a.exec();
		ic->waitForShutdown();
	} catch (const Ice::Exception& e) {
		std::cerr << e << std::endl;
		status = 1;
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
		status = 1;
	}
	if (ic) {
		try {
			ic->destroy();
		} catch (const Ice::Exception& e) {
			std::cerr << e << std::endl;
			status = 1;
		}
	}
	return status;	
	//return a.exec();
}
