#include <tools/data_plot.h>

namespace smlearning {


//
//  Initialize main window
//
DataPlot::DataPlot(int size, QString title, vector<double> d, QWidget *parent):
	QwtPlot(parent),
	plot_size (size)
//     d_interval(0),
//     d_timerId(-1)
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
    data = new double[plot_size];
    for (int i = 0; i< plot_size; i++)
    {
	    d_x[i] = i;     // time axis
	    if (i >= d.size())
		    data[i] = 0.0;
	    else
		    data[i] = d[i];
    }

    // Assign a title
    setTitle(title);
    insertLegend(new QwtLegend(), QwtPlot::BottomLegend);

    // Insert new curves
    QwtPlotCurve *curve = new QwtPlotCurve("Learning progress");
    curve->attach(this);

    //QwtPlotCurve *cLeft = new QwtPlotCurve("Data Moving Left");
    //cLeft->attach(this);

    // Set curve styles
    curve->setPen(QPen(Qt::blue));
    //cLeft->setPen(QPen(Qt::blue));

    // Attach (don't copy) data. Both curves use the same x array.
    curve->setRawData(d_x, data, plot_size);
    //cLeft->setRawData(d_x, d_z, PLOT_SIZE);

#if 0
    //  Insert zero line at y = 0
    QwtPlotMarker *mY = new QwtPlotMarker();
    mY->setLabelAlignment(Qt::AlignRight|Qt::AlignTop);
    mY->setLineStyle(QwtPlotMarker::HLine);
    mY->setYValue(0.0);
    mY->attach(this);
#endif

    // Axis 
    setAxisTitle(QwtPlot::xBottom, "Iteration");
    setAxisScale(QwtPlot::xBottom, 0, plot_size);

    setAxisTitle(QwtPlot::yLeft, "Values");
    setAxisScale(QwtPlot::yLeft, -1.5, 1.5);
    
    //setTimerInterval(0.0); 
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

    for ( int i = 0; i < QwtPlot::axisCnt; i++ )
    {
        QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);
        if ( scaleWidget )
            scaleWidget->setMargin(0);

        QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);
        if ( scaleDraw )
            scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
    }
}

}; /* namespace smlearning */
