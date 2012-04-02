/** @file PlotAppI.h
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 *           2009      Sergio Roa
 
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


#include <qapplication.h>
#include <qmainwindow.h>
#include <qtoolbar.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qcombobox.h>
#include <qspinbox.h>
#include <qmetatype.h>

#include <vector>
#include <string>
#include <iostream>
#include <assert.h>

#include <smltools/data_plot.h>
#include <PlotApp.hh>

using namespace std;

namespace smlearning {

struct RegionData {
	vector<double> learnprogData;
	vector<double> errorData;
};

class InitEvent : public QEvent
{
public:


	int regionsNr;
	int size;
	InitEvent(int r, int s ) :
		QEvent ((QEvent::Type)1001),
		regionsNr (r),
		size(s)
	{
	}
	
};

class ResizeEvent : public QEvent
{
public:

	int width;
	int height;
	ResizeEvent(int w, int h) :
		QEvent ((QEvent::Type)1002),
		width(w),
		height(h)
	{
	}
	
};

class ShowEvent : public QEvent
{
public:
	ShowEvent() : QEvent ((QEvent::Type)1003) { }

};

class UpdateDataEvent : public QEvent
{
public:
	int region;
	const ::smlearning::plotting::SeqDouble lpData;
	const ::smlearning::plotting::SeqDouble eData;
	UpdateDataEvent(int r, const smlearning::plotting::SeqDouble& lpD, const smlearning::plotting::SeqDouble& eD) :
		QEvent ((QEvent::Type)1004),
		region (r),
		lpData (lpD),
		eData (eD)
	{
	}

};
	
class PlotApp: public QMainWindow
{
	Q_OBJECT
	
	DataPlot *plot;
	int plot_size;
	enum Mode { User, Robot };
	QComboBox *modeComboBox;
	QSpinBox *regionSpinBox;
	//QToolBar *toolBar;
	//QWidget *hBox;
	QHBoxLayout *layout;
	Mode mode;
	int region;
	//vector<RegionData> regionsData;
	map<int, RegionData> regionsData;
	
public:
	PlotApp () {	}
	void init (int regionsNr, int size)
	{
		// regionsData.resize (regionsNr);
		// RegionData initialData;
		// regionsData[0] = initialData;
		for (int i=0; i<regionsNr; i++) {
			RegionData r;
			regionsData[i] = r;
		}
		
		plot_size = size;
		qRegisterMetaType<vector<double> >("vector<double>");
		QToolBar *toolBar = new QToolBar(this);
		toolBar->setFixedHeight(80);
		
#if QT_VERSION < 0x040000
		setDockEnabled(TornOff, true);
		setRightJustification(true);
#else
		toolBar->setAllowedAreas(Qt::TopToolBarArea | Qt::BottomToolBarArea);
#endif
		QWidget *hBox = new QWidget(toolBar);

		regionSpinBox = new QSpinBox;
		regionSpinBox->setRange(0, regionsNr-1);
		regionSpinBox->setEnabled (false);	
		region = 0;
	
		QLabel *regionLabel = new QLabel(tr("Sensorimotor &Region:"), hBox );
		regionLabel->setBuddy(regionSpinBox);

		modeComboBox = new QComboBox;
		modeComboBox->addItem(tr("User"));
		modeComboBox->addItem(tr("Robot"));
		modeComboBox->setCurrentIndex (1);
		mode = Robot;
		QLabel *modeLabel = new QLabel(tr("&Mode: "), hBox );
		modeLabel->setBuddy(modeComboBox);
		
		/*QHBoxLayout **/layout = new QHBoxLayout(hBox);
		layout->addWidget(regionLabel);
		layout->addWidget(regionSpinBox);
		layout->addWidget(new QWidget(hBox), 10); // spacer);
		layout->addWidget(modeLabel);
		layout->addWidget(modeComboBox);
		
#if QT_VERSION >= 0x040000
		toolBar->addWidget(hBox);
#endif
		addToolBar(toolBar);
		vector<double> lpData;
		vector<double> eData;
		
		plot = new DataPlot(plot_size, QString ("Region ") + QString::number(region), lpData, eData, this);
		setCentralWidget(plot);
		
// 		connect(&thread, SIGNAL(update(vector<double>, vector<double>)),
// 			plot, SLOT(replot(void)) );

// 		connect(&thread, SIGNAL(update(vector<double>, vector<double>)),
// 			this, SLOT(updateData(vector<double>, vector<double>)) );

		connect(this, SIGNAL(repaintPlot()),
			plot, SLOT(replot()) );

		connect(modeComboBox, SIGNAL(activated(int)),
			this, SLOT(modeChanged(int)));

		connect(this, SIGNAL(enableRegionSpinBox(bool)),
			regionSpinBox, SLOT(setEnabled(bool)));

		connect(regionSpinBox, SIGNAL(valueChanged(int)),
			this, SLOT(regionChanged(int)));
	
		connect(this, SIGNAL(changeRegion(int)),
			regionSpinBox, SLOT(setValue(int)));

	}

	virtual void customEvent(QEvent* e) {
		if (e->type() == 1001) {
			cout << "initializing..." << endl;
			InitEvent* ie = dynamic_cast<InitEvent*>(e);
			init (ie->regionsNr, ie->size);
		}
		else if (e->type() == 1002) {
			cout << "resizing..." << endl;
			ResizeEvent* re = dynamic_cast<ResizeEvent*>(e);
			resize (re->width, re->height);
		}
		else if (e->type() == 1003) {
			cout << "showing..." << endl;
			show ();
		}
		else if (e->type() == 1004) {
			cout << "updating..." << endl;
			UpdateDataEvent* ue = dynamic_cast<UpdateDataEvent*>(e);
			updateData  (ue->region, ue->lpData, ue->eData);
		}

	}

	void updatePlotData (vector<double> lpD, vector<double> eD) {
		int lpstartIndex, estartIndex;
		if (lpD.size() < plot_size)
			lpstartIndex = plot_size - lpD.size();
		else
			lpstartIndex = 0;
		if (eD.size() < plot_size)
			estartIndex = plot_size - eD.size();
		else
			estartIndex = 0;
		cout << "updating data..." << endl;
		double min = 10000.0;
		double max = -10000.0;
		for (int i=0; i< plot_size; i++) {
			if (i < lpstartIndex) 
				plot->learnprogData[i] = 0.0;				
			else {
				plot->learnprogData[i] = lpD[i-lpstartIndex];	
			}
			if (plot->learnprogData[i] < min)
				min = plot->learnprogData[i];
			if (plot->learnprogData[i] > max)
				max = plot->learnprogData[i];
			if (i < estartIndex)
				plot->errorData[i] = 0.0;
			else
				plot->errorData[i] = eD[i-estartIndex];
			if (plot->errorData[i] < min)
				min = plot->errorData[i];
			if (plot->errorData[i] > max)
				max = plot->errorData[i];
			
			
		}
		plot->setAxisScale(QwtPlot::yLeft, min-fabs(min/20.0), max+fabs(max/20.0));

	}

	void updateData(int r, vector<double> lpD, vector<double> eD)
	{
		if (regionSpinBox->maximum() < r)
			regionSpinBox->setMaximum (r);

		// update the display
		regionsData[r].learnprogData = lpD;
		regionsData[r].errorData = eD;
		if (mode == User && region == r || mode == Robot) {
			updatePlotData (lpD, eD);
			plot->setTitle (QString("Region " + QString::number(r)) );
			emit repaintPlot();
			//plot->replot();
			repaint();
			region = r;
		}
	}

	
public slots:

	void modeChanged (int index) {
		cout << "starting changing to mode " << index << endl;
		mode = (PlotApp::Mode)index;
		if (mode == Robot)
			emit enableRegionSpinBox (false);
		else if (mode == User) {
			emit enableRegionSpinBox (true);
			emit changeRegion (region);
		}
		cout << "changing to mode " << mode << endl;
			
	}
	

	void regionChanged (int value) {
		region = value;
		updatePlotData (regionsData[region].learnprogData, regionsData[region].errorData);
		//plot->replot();
		plot->setTitle (QString("Region " + QString::number(region)));
		emit repaintPlot();
		repaint();
		cout << "changing to region " << region << endl;
	}

signals:
	void enableRegionSpinBox (bool);
	void repaintPlot();
	void changeRegion(int);
};


class PlotAppI : virtual public smlearning::plotting::PlottingApp {
public:
	PlotApp *w;
	PlotAppI(int argc, char *argv[], Ice::CommunicatorPtr communicator) {
		w = new PlotApp();
	}

	void init ( int regionsNr, int size, const Ice::Current& ) {
		QApplication::postEvent(w, new InitEvent(regionsNr, size) );

	}
	
	void resize ( int width, int height, const Ice::Current& ) {
		QApplication::postEvent(w, new ResizeEvent(width, height) );
	}

	void show (const Ice::Current& ) {
		QApplication::postEvent(w, new ShowEvent() );
	}

	void updateData ( int region, const smlearning::plotting::SeqDouble& lpData, const smlearning::plotting::SeqDouble& eData, const Ice::Current& ) {
		QApplication::postEvent(w, new UpdateDataEvent(region, lpData, eData) );

	}
	

};

class PlotAppIFace {
public:
	PlotApp *w;
	PlotAppIFace() {
		w = new PlotApp();
	}

	void init ( int regionsNr, int size ) {
		QApplication::postEvent(w, new InitEvent(regionsNr, size) );

	}
	
	void resize ( int width, int height ) {
		QApplication::postEvent(w, new ResizeEvent(width, height) );
	}

	void show ( ) {
		QApplication::postEvent(w, new ShowEvent() );
	}

	void updateData ( int region, const vector<double>& lpData, const vector<double>& eData ) {
		QApplication::postEvent(w, new UpdateDataEvent(region, lpData, eData) );

	}
	

};



}; /* namespace smlearning */
