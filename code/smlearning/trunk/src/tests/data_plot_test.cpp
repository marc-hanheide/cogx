#include <tests/data_plot_test.h>

int main(int argc, char **argv)
{
	QApplication a(argc, argv);
	
	vector<double> lpData;
	vector<double> eData;
	int max_size = 40;
	MyWidgets* myWidgets = new MyWidgets;
	myWidgets->mainWindow = new MainWindow;
	myWidgets->mainWindow->init (max_size, lpData, eData);
	
	myWidgets->mainWindow->resize(600,400);
	myWidgets->mainWindow->start();
	myWidgets->mainWindow->show();
	
	return a.exec();
}
