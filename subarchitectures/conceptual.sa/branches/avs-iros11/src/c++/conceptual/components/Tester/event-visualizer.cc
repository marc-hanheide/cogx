#include "EventVisualizer.h"

#include <QApplication>
#include <QFileDialog>

using namespace std;


int main(int argc, char **argv)
{
	QApplication app(argc, argv);

	QString fileName = QFileDialog::getOpenFileName(0, "Open Conceptual.SA Events File","", "Conceptual Events (*.cevents)");

	vector<string> roomCats;
	vector<string> shapes;
	vector<string> appearances;
	vector<string> visualizedObjects;

	EventVisualizer visualizer(0, roomCats, shapes, appearances, visualizedObjects);
	visualizer.exec();
}
