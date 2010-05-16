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
