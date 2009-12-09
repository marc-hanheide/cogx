#ifndef _DATA_PLOT_H
#define _DATA_PLOT_H 1

#include <qwt_plot.h>
#include <stdlib.h>
#include <qwt_painter.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_widget.h>
#include <qwt_legend.h>
#include <qwt_scale_draw.h>
#include <qwt_math.h>

namespace smlearning {

using namespace std;

class DataPlot : public QwtPlot
{
    Q_OBJECT
public:
	double *learnprogData;
	double *errorData;
	int plot_size;
	DataPlot(int plot_size, QString title, vector<double> lpD, vector<double> eD, QWidget* = NULL);
	~DataPlot () {
		delete d_x;
		delete learnprogData;
		delete errorData;
	}


private:
	void alignScales();

	double *d_x;
};

}; /* namespace smlearning */




#endif
