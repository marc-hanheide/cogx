#include <Ice/Ice.h>
//#include <tools/data_plot_test.h>
#include <PlotApp.hh>
//#include <qapplication.h>
//#include <qmainwindow.h>
//#include <vector>

using namespace smlearning::plotting;

int main(int argc, char* argv[])
{
	int status = 0;
	Ice::CommunicatorPtr ic;
	try {
		ic = Ice::initialize(argc, argv);
		Ice::ObjectPrx base = ic->stringToProxy("SimpleDataPlotter:default -p 8173");
		MainWindowPrx myWidget = MainWindowPrx::checkedCast(base);
		if (!myWidget)
			throw "Invalid proxy";
		SeqDouble lpData;
		SeqDouble eData;
		int max_size = 40;
		myWidget->init (max_size, lpData, eData);
		
		myWidget->resize(600,400);
		myWidget->start();
		myWidget->show();

	} catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
		status = 1;
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
		status = 1;
	}
	if (ic)
		ic->destroy();

	return status;

}
