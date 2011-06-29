#include "EventVisualizer.h"
#include "ConceptualWidget.h"
#include "AddGroundtruthDialog.h"

#include <QFileDialog>
#include <QtSvg/QSvgGenerator>
#include <QTimer>
//#include <QtOpenGL/QGLWidget>

using namespace std;
using namespace conceptual;


EventVisualizer::EventVisualizer(QWidget *parent,
		const std::vector<std::string> &roomCats, const std::vector<std::string> &shapes, const std::vector<std::string> &sizes,
		const std::vector<std::string> &appearances, const std::vector<std::string> &visualizedObjects)
    : QDialog(parent)
{
	ui.setupUi(this);
	_roomCats = &roomCats;
//	ui.eventView->setViewport(new QGLWidget);
//	ui.legendView->setViewport(new QGLWidget);
	ui.eventView->setDisplayedEntities(roomCats, shapes, sizes, appearances, visualizedObjects);
	ui.legendView->setDisplayedEntities(roomCats, shapes, sizes, appearances, visualizedObjects);
	_timer = new QTimer(this);

	connect(ui.saveSvgButton, SIGNAL(clicked()), this, SLOT(saveSvgButtonClicked()));
	connect(ui.savePngButton, SIGNAL(clicked()), this, SLOT(savePngButtonClicked()));
	connect(ui.addGroundtruthButton, SIGNAL(clicked()), this, SLOT(addGroundtruthButtonClicked()));
	connect(ui.placesCheckBox, SIGNAL(toggled(bool)), this, SLOT(generate()));
	connect(ui.widthSpinBox, SIGNAL(valueChanged(int)), this, SLOT(generate()));
	connect(ui.verticalIndicatorsCheckBox, SIGNAL(toggled(bool)), this, SLOT(generate()));
	connect(ui.locationEventsCheckBox, SIGNAL(toggled(bool)), this, SLOT(generate()));
	connect(ui.verticalLinesCheckBox, SIGNAL(toggled(bool)), this, SLOT(generate()));

	connect(ui.playButton, SIGNAL(clicked()), this, SLOT(playButtonClicked()));
	connect(ui.pauseButton, SIGNAL(clicked()), this, SLOT(pauseButtonClicked()));
	connect(ui.nextButton, SIGNAL(clicked()), this, SLOT(nextButtonClicked()));
	connect(ui.prevButton, SIGNAL(clicked()), this, SLOT(prevButtonClicked()));
	connect(ui.startButton, SIGNAL(clicked()), this, SLOT(startButtonClicked()));
	connect(ui.endButton, SIGNAL(clicked()), this, SLOT(endButtonClicked()));
	connect(_timer, SIGNAL(timeout()), this, SLOT(timerTimedout()));

	_hasLegend = false;
}

EventVisualizer::~EventVisualizer()
{

}


void EventVisualizer::generate(const QList<conceptual::ConceptualEvent> &events)
{
	_lastEvents = events;
	_curPlaceId = events.last().curPlaceId;
	_curRoomId = events.last().curRoomId;
	_curTime = events.last().time;
	for (long _e=0; _e<events.size(); ++_e)
	{
		const conceptual::ConceptualEvent &event = events[_e];
		if (event.curRoomId>=0)
			_roomIds.insert(event.curRoomId);
	}

	generate();
	if (!_hasLegend)
		ui.legendView->updateLegend();
}


void EventVisualizer::generate()
{
	ui.eventView->updateEvents(_lastEvents, ui.placesCheckBox->isChecked(),
			ui.widthSpinBox->value(), ui.verticalIndicatorsCheckBox->isChecked(),
			ui.verticalLinesCheckBox->isChecked(), ui.locationEventsCheckBox->isChecked(), _curTime);
	ui.timeLabel->setText(QString::number(_curTime,'f',2));
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
				ui.verticalLinesCheckBox->isChecked(), ui.locationEventsCheckBox->isChecked(), _curTime);
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
				ui.verticalLinesCheckBox->isChecked(), ui.locationEventsCheckBox->isChecked(), _curTime);
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


void EventVisualizer::playButtonClicked()
{
	_timer->start(1000);
}

void EventVisualizer::pauseButtonClicked()
{
	_timer->stop();
}

void EventVisualizer::nextButtonClicked()
{
	int i;
	for (i=0; i<_lastEvents.count(); ++i)
	{
		if (_lastEvents[i].time>_curTime)
			break;
	}
	if (i>=_lastEvents.count())
		i=_lastEvents.count()-1;
	_curTime = _lastEvents[i].time;
	generate();
}

void EventVisualizer::prevButtonClicked()
{
	int j=0;
	for (int i=0; i<_lastEvents.count(); ++i)
	{
		if (_lastEvents[i].time<=_curTime)
		{
			j=i;
		}
	}
	if (j>0)
		j--;
	_curTime = _lastEvents[j].time;
	generate();
}

void EventVisualizer::startButtonClicked()
{
	_curTime = _lastEvents.first().time;
	generate();
}

void EventVisualizer::endButtonClicked()
{
	_curTime = _lastEvents.last().time;
	generate();
}


void EventVisualizer::timerTimedout()
{
	_curTime+=1;
	generate();
}
