#include <qapplication.h>
#include <qmainwindow.h>
#include <qwt_counter.h>
#include <qtoolbar.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qthread.h>
#include <qmutex.h>
#include <qwaitcondition.h>
#include <qmetatype.h>

#include <vector>
#include <string>
#include <iostream>
#include <assert.h>

#include <tools/data_plot.h>

using namespace std;
using namespace smlearning;

class EvaluationThread : public QThread {

	Q_OBJECT 
	int plot_size;
public:

	EvaluationThread (QObject *parent = 0) : QThread (parent) {
		restart = false;
		abort = false;
	}
	~EvaluationThread () {
		mutex.lock();
		abort = true;
		condition.wakeOne();
		mutex.unlock();
// 		wait();
	}
	void generate_data (int size) {
		plot_size = size;
		if (!isRunning ()) {
			start(LowPriority);
		}
		else {
			restart = true;
			condition.wakeOne();
		}
	}
	
protected:
	void run () {

		vector<double> data;
		int i=0;
		forever {
			if (i > 0)				
				data.push_back(data[i%plot_size-1]+0.005);
			else
				data.push_back(-1.5);
			if (data.size() > plot_size)
				data.erase (data.begin());
			emit update (data);
			sleep (1);
			i++;
			if (i % plot_size == 0)
				i--;
		}
	}
signals:
	void update (vector<double> data);
private:
	QMutex mutex;
	QWaitCondition condition;
	bool restart;
	bool abort;
	
};


class MainWindow: public QMainWindow
{
	Q_OBJECT
	
	DataPlot *plot;
	EvaluationThread thread;
	int plot_size;
	
public:
	MainWindow(int size, vector<double> data) :
		QMainWindow (),
		plot_size (size)
	{
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
		//QLabel *label = new QLabel("Timer Interval", hBox);
		//QwtCounter *counter = new QwtCounter(hBox);
		//counter->setRange(-1.0, 100.0, 1.0);
		
		//QHBoxLayout *layout = new QHBoxLayout(hBox);
		//layout->addWidget(label);
		//layout->addWidget(counter);
		//layout->addWidget(new QWidget(hBox), 10); // spacer);
		
#if QT_VERSION >= 0x040000
		//toolBar->addWidget(hBox);
#endif
		//addToolBar(toolBar);
		
		plot = new DataPlot(plot_size, QString ("Learning Progress"), data, this);
		setCentralWidget(plot);
		
		//connect(&thread, SIGNAL(update(vector<double>)),
		//	plot, SLOT(replot(void)) );

		connect(&thread, SIGNAL(update(vector<double>)),
			this, SLOT(updateData(vector<double>)) );

		//counter->setValue(20.0);
	}


	void start() {
		thread.generate_data (plot_size);
	}

	
public slots:
	void updateData(vector<double> d)
	{
		// update the display
		assert (d.size() <= plot_size);
		for (int i=0; i< d.size(); i++) {
			cout << "updating..." << endl;
			cout << d[i] << endl;
			plot->data[i] = d[i];
		}
		plot->replot();
		repaint();
	}

	


};

