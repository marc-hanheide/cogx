#include "RCVisualizer.h"

#include <QFileDialog>
#include <QtSvg/QSvgGenerator>


RCVisualizer::RCVisualizer(QWidget *parent, conceptual::Tester *component)
    : QDialog(parent), _component(component)
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

/*	// G
	// TODO: HAVE THE DISTRIBUTION IN A VECTOR RATHER THAN MAP
*/
	scene->addLine(0,0,0,100);


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

