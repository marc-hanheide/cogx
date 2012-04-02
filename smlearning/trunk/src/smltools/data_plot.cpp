/** @file data_plot.cpp
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 *           2010      Sergio Roa
 
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.

 
 */

#include <smltools/data_plot.h>

namespace smlearning {


//
//  Initialize main window
//
DataPlot::DataPlot(int size, QString title, vector<double> lpD, vector<double> eD, QWidget *parent):
	QwtPlot(parent),
	plot_size (size)
{
	// Disable polygon clipping
	QwtPainter::setDeviceClipping(false);

	// We don't need the cache here
	canvas()->setPaintAttribute(QwtPlotCanvas::PaintCached, false);
	canvas()->setPaintAttribute(QwtPlotCanvas::PaintPacked, false);

#if QT_VERSION >= 0x040000
#ifdef Q_WS_X11
	/*
	  Qt::WA_PaintOnScreen is only supported for X11, but leads
	  to substantial bugs with Qt 4.2.x/Windows
	*/
	canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif
#endif

	alignScales();
    
	//  Initialize data
	d_x = new double[plot_size];
	learnprogData = new double[plot_size];
	errorData = new double[plot_size];
	int lpstartIndex, estartIndex;
	if (lpD.size() < plot_size)
		lpstartIndex = plot_size - lpD.size();
	else
		lpstartIndex = 0;
	if (eD.size() < plot_size)
		estartIndex = plot_size - eD.size();
	else
		estartIndex = 0;
	
	for (int i = 0; i< plot_size; i++) {
		d_x[i] = i;     // iterations axis
		if (i < lpstartIndex)
			learnprogData[i] = 0.0;
		else
			learnprogData[i] = lpD[i-lpstartIndex];
		if (i < estartIndex)
			errorData[i] = 0.0;
		else
			errorData[i] = eD[i-estartIndex];
	}

	// Assign a title
	setTitle(title);
	insertLegend(new QwtLegend(), QwtPlot::BottomLegend);

	// Insert new curves
	QwtPlotCurve *lpCurve = new QwtPlotCurve("Learning progress");
	lpCurve->attach(this);

	QwtPlotCurve *eCurve = new QwtPlotCurve("Norm. Sum of Squares Error");
	eCurve->attach(this);

	// Set curve styles
	lpCurve->setPen(QPen(Qt::blue));
	eCurve->setPen(QPen(Qt::red));

	// Attach (don't copy) data. Both curves use the same x array.
	lpCurve->setRawData(d_x, learnprogData, plot_size);
	eCurve->setRawData(d_x, errorData, plot_size);

#if 0
	//  Insert zero line at y = 0
	QwtPlotMarker *mY = new QwtPlotMarker();
	mY->setLabelAlignment(Qt::AlignRight|Qt::AlignTop);
	mY->setLineStyle(QwtPlotMarker::HLine);
	mY->setYValue(0.0);
	mY->attach(this);
#endif

	// Axis 
	setAxisTitle(QwtPlot::xBottom, "Last iterations");
	setAxisScale(QwtPlot::xBottom, 0, plot_size);

	setAxisTitle(QwtPlot::yLeft, "Values");
	setAxisScale(QwtPlot::yLeft, -0.05, 1.0);
    
}

//
//  Set a plain canvas frame and align the scales to it
//
void DataPlot::alignScales()
{
	// The code below shows how to align the scales to
	// the canvas frame, but is also a good example demonstrating
	// why the spreaded API needs polishing.

	canvas()->setFrameStyle(QFrame::Box | QFrame::Plain );
	canvas()->setLineWidth(1);

	for ( int i = 0; i < QwtPlot::axisCnt; i++ ) {
		QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);
		if ( scaleWidget )
			scaleWidget->setMargin(0);

		QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);
		if ( scaleDraw )
			scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
	}
}

}; /* namespace smlearning */
