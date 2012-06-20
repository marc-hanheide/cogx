#include "EventVisualizer.h"
#include "ConceptualEvent.h"

#include <QApplication>
#include <QFileDialog>

using namespace std;
using namespace conceptual;


int main(int argc, char **argv)
{
	QApplication app(argc, argv);

	QString fileName = QFileDialog::getOpenFileName(0, "Open Conceptual.SA Events File","", "Conceptual Events (*.cevents)");
	if (!fileName.isEmpty())
	{
		QFile file(fileName);
		if (file.open(QIODevice::ReadOnly))
		{
			QDataStream in(&file);
			vector<string> roomCats;
			vector<string> shapes;
			vector<string> sizes;
			vector<string> appearances;
			vector<string> visualizedObjects;
			in >> roomCats;
			in >> shapes;
			in >> sizes;
			in >> appearances;
			in >> visualizedObjects;
			QList<conceptual::ConceptualEvent> events;
			in >> events;
			file.close();

			EventVisualizer visualizer(0, roomCats, shapes, sizes, appearances, visualizedObjects);
			visualizer.generate(events);
			visualizer.exec();
		}
	}
}

