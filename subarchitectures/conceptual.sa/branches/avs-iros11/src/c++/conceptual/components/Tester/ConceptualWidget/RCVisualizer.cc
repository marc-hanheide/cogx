#include "RCVisualizer.h"
#include "ConceptualWidget.h"

#include <QFileDialog>
#include <QtSvg/QSvgGenerator>


RCVisualizer::RCVisualizer(ConceptualWidget *parent, conceptual::Tester *component)
    : QDialog(parent), _parent(parent), _component(component)
{
	ui.setupUi(this);
	connect(ui.saveImageButton, SIGNAL(clicked()), this, SLOT(saveImageButtonClicked()));

	generate();
}


RCVisualizer::~RCVisualizer()
{

}


void RCVisualizer::generate()
{
	QGraphicsScene *scene = new QGraphicsScene(this);
	pthread_mutex_lock(&_parent->_eventsMutex);

	unsigned long eventCount = _parent->_events.size();
	for (unsigned long e=0; e<eventCount; ++e)
	{
		const ConceptualWidget::Event &event = _parent->_events[e];

		scene->addLine(e*10,0,e*10,100);
	}

	pthread_mutex_unlock(&_parent->_eventsMutex);
	ui.graphicsView->setScene(scene);
}


void RCVisualizer::saveImageButtonClicked()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
	                            "", tr("SVG Images (*.svg)"));
	if (!fileName.isEmpty())
	{
		QSvgGenerator generator;
		generator.setFileName(fileName);
		generator.setSize(QSize(200, 200));
		generator.setViewBox(QRect(0, 0, 200, 200));
		generator.setTitle("Room Category Visualization");
		QPainter painter;
		painter.begin(&generator);
		ui.graphicsView->scene()->render(&painter);
		painter.end();
	}
}

