#include <Ice/Ice.h>
#include <tests/data_plot_test.h>
//#include <qapplication.h>
//#include <qmainwindow.h>



int main(int argc, char **argv)
{
	QApplication a(argc, argv);
	int status = 0;
	Ice::CommunicatorPtr ic;
	try {
		ic = Ice::initialize(argc, argv);
		Ice::ObjectAdapterPtr adapter = ic->createObjectAdapterWithEndpoints("SimpleDataPlotterAdapter", "default -p 8173");
		Ice::ObjectPtr myWidgets = new MainWindowI (argc, argv, ic);
		adapter->add(myWidgets,
			     ic->stringToIdentity("SimpleDataPlotter"));
		adapter->activate();
		std::cout << "SimpleDataPlotter running" << std::endl;
// 		(MainWindowI)myWidgets.w = new QMainWindow();
// 		myWidgets.w->resize (640,480);
// 		myWidgets.w->show ();
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
