#include "CategoricalWidget.h"
#include "Tester.h"

using namespace cast;

CategoricalWidget::CategoricalWidget(QWidget *parent, conceptual::Tester *component)
    : QWidget(parent), _component(component)
{
	ui.setupUi(this);
	qRegisterMetaType< QVector<double> >("QVector<double>");
	qRegisterMetaType< QList<double> >("QList<double>");

	// Color map for images
	for (int i=0; i<256; ++i)
		_colorMap.append(qRgb(i, i, i));
}


CategoricalWidget::~CategoricalWidget()
{

}


void CategoricalWidget::newVisualResults(CategoricalData::VisualResultsPtr vrPtr)
{
	if (vrPtr->status == CategoricalData::DsValid)
	{
		QStringList names;
		QList<double> values;
		for (size_t i=0; i<vrPtr->results.size(); ++i)
		{
			names.append(vrPtr->results[i].className.c_str());
			values.append(vrPtr->results[i].confidence);
		}
		QMetaObject::invokeMethod(this, "newAppearanceResults", Qt::QueuedConnection,
		                           Q_ARG(QStringList, names),
		                           Q_ARG(QList<double>, values));
	}
}


void CategoricalWidget::newLaserResults(CategoricalData::LaserResultsPtr lrPtr)
{
	if (lrPtr->status == CategoricalData::DsValid)
	{
		{
			QStringList names;
			QList<double> values;
			for (size_t i=0; i<lrPtr->results.size(); ++i)
			{
				names.append(lrPtr->results[i].className.c_str());
				values.append(lrPtr->results[i].confidence);
			}
			QMetaObject::invokeMethod(this, "newShapeResults", Qt::QueuedConnection,
					Q_ARG(QStringList, names),
					Q_ARG(QList<double>, values));
		}
		if (lrPtr->useSize)
		{
			QStringList names;
			QList<double> values;
			for (size_t i=0; i<lrPtr->sizeResults.size(); ++i)
			{
				names.append(lrPtr->sizeResults[i].className.c_str());
				values.append(lrPtr->sizeResults[i].confidence);
			}
			QMetaObject::invokeMethod(this, "newSizeResults", Qt::QueuedConnection,
					Q_ARG(QStringList, names),
					Q_ARG(QList<double>, values));
		}
	}
}


void CategoricalWidget::newLaserScan(CategoricalData::LaserScanPtr lsPtr)
{
	if ((lsPtr->status == CategoricalData::DsValid) && (ui.showScanCheckBox->isChecked()))
	{
	    // Convert scan info
	    QVector<double> range(lsPtr->scanBuffer.ranges.size());
	    for(unsigned int i=0; i<lsPtr->scanBuffer.ranges.size(); ++i)
	      range[i]=lsPtr->scanBuffer.ranges[i];
	    double max=lsPtr->scanBuffer.maxRange;
	    double startAngle=lsPtr->scanBuffer.startAngle;
	    double angleStep=lsPtr->scanBuffer.angleStep;

		QMetaObject::invokeMethod(ui.laserScanView, "addScan", Qt::QueuedConnection,
		                           Q_ARG(double, startAngle),
		                           Q_ARG(double, angleStep),
		                           Q_ARG(double, max),
		                           Q_ARG(QVector<double>, range),
		                           Q_ARG(double, static_cast<double>(lsPtr->realTimeStamp.s) + static_cast<double>(lsPtr->realTimeStamp.us)/1000000.0));
		QMetaObject::invokeMethod(ui.scanFrameLabel, "setText", Qt::QueuedConnection,
		                           Q_ARG(QString, QString::number(lsPtr->frameNo)));
	}
}


void CategoricalWidget::newOdometry(CategoricalData::OdometryPtr oPtr)
{
	if ((oPtr->status == CategoricalData::DsValid) && (ui.showOdometryCheckBox->isChecked()))
	{
		QMetaObject::invokeMethod(ui.odometryView, "addOdometry", Qt::QueuedConnection,
		                           Q_ARG(double, oPtr->odometryBuffer.odompose[0].x),
		                           Q_ARG(double, oPtr->odometryBuffer.odompose[0].y),
		                           Q_ARG(double, oPtr->odometryBuffer.odompose[0].theta),
		                           Q_ARG(double, static_cast<double>(oPtr->realTimeStamp.s) + static_cast<double>(oPtr->realTimeStamp.us)/1000000.0));
		QMetaObject::invokeMethod(ui.odometryFrameLabel, "setText", Qt::QueuedConnection,
		                           Q_ARG(QString, QString::number(oPtr->frameNo)));
	}
}


void CategoricalWidget::newImage(CategoricalData::ImagePtr iPtr)
{
	if ((iPtr->status == CategoricalData::DsValid) && (ui.showImageCheckBox->isChecked()))
	{
		// Convert image to pixmap
		int w=iPtr->imageBuffer.width;
		int h=iPtr->imageBuffer.height;
		QImage tmpImage(w/2, h/2, QImage::Format_Indexed8);
		tmpImage.setColorTable(_colorMap);

		for (int i=0; i<h; i+=2)
			for (int j=0; j<w; j+=2)
				tmpImage.setPixel(j/2, i/2, (unsigned char)(iPtr->imageBuffer.data[i*w+j]) );

		QMetaObject::invokeMethod(this, "setImage", Qt::QueuedConnection,
		                           Q_ARG(QImage, tmpImage),
		                           Q_ARG(int, iPtr->frameNo));
	}
}


void CategoricalWidget::setImage(QImage img, int frame)
{
	ui.imageLabel->setPixmap(QPixmap::fromImage(img));
	ui.imageFrameLabel->setText(QString::number(frame));
}


void CategoricalWidget::newShapeResults(QStringList shapes, QList<double> values)
{
	ui.shapeTreeWidget->clear();
	for(int i=0; i<shapes.count(); ++i)
	{
		QStringList row;
		row.append(shapes[i]);
		row.append(QString::number(values[i], 'f', 2));
		ui.shapeTreeWidget->addTopLevelItem(
				new QTreeWidgetItem(row));
	}
}


void CategoricalWidget::newSizeResults(QStringList sizes, QList<double> values)
{
	ui.sizeTreeWidget->clear();
	for(int i=0; i<sizes.count(); ++i)
	{
		QStringList row;
		row.append(sizes[i]);
		row.append(QString::number(values[i], 'f', 2));
		ui.sizeTreeWidget->addTopLevelItem(
				new QTreeWidgetItem(row));
	}
}


void CategoricalWidget::newAppearanceResults(QStringList appearances, QList<double> values)
{
	ui.appearanceTreeWidget->clear();
	for(int i=0; i<appearances.count(); ++i)
	{
		QStringList row;
		row.append(appearances[i]);
		row.append(QString::number(values[i], 'f', 2));
		ui.appearanceTreeWidget->addTopLevelItem(
				new QTreeWidgetItem(row));
	}
}

