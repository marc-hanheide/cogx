#include "EventVisualizer.h"
#include "ConceptualWidget.h"
#include "AddGroundtruthDialog.h"

#include <QFileDialog>
#include <QtSvg/QSvgGenerator>

using namespace std;
using namespace conceptual;


EventVisualizer::EventVisualizer(QWidget *parent,
		const std::vector<std::string> &roomCats, const std::vector<std::string> &shapes, const std::vector<std::string> &sizes,
		const std::vector<std::string> &appearances, const std::vector<std::string> &visualizedObjects)
    : QDialog(parent)
{
	ui.setupUi(this);
	_roomCats = &roomCats;
	ui.eventView->setDisplayedEntities(roomCats, shapes, sizes, appearances, visualizedObjects);
	ui.legendView->setDisplayedEntities(roomCats, shapes, sizes, appearances, visualizedObjects);

	connect(ui.saveSvgButton, SIGNAL(clicked()), this, SLOT(saveSvgButtonClicked()));
	connect(ui.savePngButton, SIGNAL(clicked()), this, SLOT(savePngButtonClicked()));
	connect(ui.addGroundtruthButton, SIGNAL(clicked()), this, SLOT(addGroundtruthButtonClicked()));
	connect(ui.placesCheckBox, SIGNAL(toggled(bool)), this, SLOT(generate()));
	connect(ui.widthSpinBox, SIGNAL(valueChanged(int)), this, SLOT(generate()));
	connect(ui.verticalIndicatorsCheckBox, SIGNAL(toggled(bool)), this, SLOT(generate()));
	connect(ui.locationEventsCheckBox, SIGNAL(toggled(bool)), this, SLOT(generate()));
	connect(ui.verticalLinesCheckBox, SIGNAL(toggled(bool)), this, SLOT(generate()));

}

EventVisualizer::~EventVisualizer()
{

}


void EventVisualizer::generate(const QList<conceptual::ConceptualEvent> &events)
{
	_lastEvents = events;
	_curPlaceId = events.last().curPlaceId;
	_curRoomId = events.last().curRoomId;
	for (long _e=0; _e<events.size(); ++_e)
	{
		const conceptual::ConceptualEvent &event = events[_e];
		if (event.curRoomId>=0)
			_roomIds.insert(event.curRoomId);
	}
	generate();
}


void EventVisualizer::generate()
{
	ui.eventView->updateEvents(_lastEvents, ui.placesCheckBox->isChecked(),
			ui.widthSpinBox->value(), ui.verticalIndicatorsCheckBox->isChecked(),
			ui.verticalLinesCheckBox->isChecked(), ui.locationEventsCheckBox->isChecked());
	ui.legendView->updateLegend();
}


void EventVisualizer::saveSvgButtonClicked()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
	                            "", tr("SVG Images (*.svg)"));
	if (!fileName.isEmpty())
	{
		QGraphicsScene scene;
		ui.eventView->drawEvents(&scene, _lastEvents, ui.placesCheckBox->isChecked(),
				ui.widthSpinBox->value(), ui.verticalIndicatorsCheckBox->isChecked(),
				ui.verticalLinesCheckBox->isChecked(), ui.locationEventsCheckBox->isChecked());
		ui.eventView->drawLegend(&scene);

		QSvgGenerator generator;
		generator.setFileName(fileName);
		generator.setSize(QSize(200, 200));
		generator.setViewBox(QRect(0, 0, 200, 200));
		generator.setTitle("Room Category Visualization");
		QPainter painter;
		painter.begin(&generator);
		scene.render(&painter);
		painter.end();
	}
}


void EventVisualizer::savePngButtonClicked()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
	                            "", tr("PNG Images (*.png)"));
	if (!fileName.isEmpty())
	{
		QGraphicsScene scene;
		ui.eventView->drawEvents(&scene, _lastEvents, ui.placesCheckBox->isChecked(),
				ui.widthSpinBox->value(), ui.verticalIndicatorsCheckBox->isChecked(),
				ui.verticalLinesCheckBox->isChecked(), ui.locationEventsCheckBox->isChecked());
		ui.eventView->drawLegend(&scene);

		QImage image(QSize(scene.width(), scene.height())*5, QImage::Format_RGB32);
		QPainter painter;
		painter.setRenderHints(QPainter::Antialiasing);
		painter.begin(&image);
		scene.render(&painter);
		painter.end();
		image.save(fileName, "PNG");
	}
}


void EventVisualizer::addGroundtruthButtonClicked()
{
	QStringList categories;
	for (unsigned int i=0 ; i<_roomCats->size(); ++i)
		categories.append(QString::fromStdString((*_roomCats)[i]));
	QList<int> roomIds;
	for (std::set<int>::iterator it = _roomIds.begin(); it!=_roomIds.end(); ++it)
		roomIds.append(*it);

	AddGroundtruthDialog *d = new AddGroundtruthDialog(this, roomIds, categories, _curRoomId);
	d->exec();
	if (d->result() == QDialog::Accepted)
	{
		int roomId = d->getRoomId();
		int categoryIndex = d->getCategoryIndex();
		ui.eventView->addGroundTruth(roomId, categoryIndex);
	}

	generate();
}

