#include <qapplication.h>
#include <qmainwindow.h>
// #include <qwt_counter.h>
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

#include <smltools/data_plot.h>
#include <PlotApp.hh>

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

		vector<double> lpData;
		vector<double> eData;
		int i=0;
		forever {
			if (i > 0) {			
				lpData.push_back(lpData[i%plot_size-1]+0.005);
				eData.push_back(eData[i%plot_size-1]-0.005);
			}
			else {
				lpData.push_back(-1.5);
				eData.push_back(1.5);
			}
			if (lpData.size() > plot_size)
				lpData.erase (lpData.begin());
			if (eData.size() > plot_size)
				eData.erase (eData.begin());
				
			emit update (lpData, eData);
			sleep (1);
			i++;
			if (i % plot_size == 0)
				i--;
		}
	}
signals:
	void update (vector<double> lpData, vector<double> eData);
private:
	QMutex mutex;
	QWaitCondition condition;
	bool restart;
	bool abort;
	
};

class InitEvent : public QEvent
{
public:

	const ::smlearning::plotting::SeqDouble lpData;
	const ::smlearning::plotting::SeqDouble eData;
	
	int size;
	InitEvent(int s, const smlearning::plotting::SeqDouble& lpD, const smlearning::plotting::SeqDouble& eD) :
		QEvent ((QEvent::Type)1001),
		lpData(lpD),
		eData(eD),
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

class StartEvent : public QEvent
{
public:
	StartEvent() : QEvent ((QEvent::Type)1004) { }

};



class MainWindow: public QMainWindow
{
	Q_OBJECT
	
	DataPlot *plot;
	EvaluationThread thread;
	int plot_size;
	
public:
	MainWindow () {	}
	void init (int size, vector<double> lpData, vector<double> eData)
	{
		plot_size = size;
		qRegisterMetaType<vector<double> >("vector<double>");
// 		QToolBar *toolBar = new QToolBar(this);
// 		toolBar->setFixedHeight(80);
		
#if QT_VERSION < 0x040000
		setDockEnabled(TornOff, true);
		setRightJustification(true);
#else
// 		toolBar->setAllowedAreas(Qt::TopToolBarArea | Qt::BottomToolBarArea);
#endif
		QWidget *hBox = new QWidget(this);
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
		
		plot = new DataPlot(plot_size, QString ("Learning Progress"), lpData, eData, this);
		setCentralWidget(plot);
		
		connect(&thread, SIGNAL(update(vector<double>, vector<double>)),
			plot, SLOT(replot(void)) );

		connect(&thread, SIGNAL(update(vector<double>, vector<double>)),
			this, SLOT(updateData(vector<double>, vector<double>)) );

		//counter->setValue(20.0);
	}

	virtual void customEvent(QEvent* e) {
		if (e->type() == 1001) {
			cout << "initializing..." << endl;
			InitEvent* ie = dynamic_cast<InitEvent*>(e);
			init (ie->size, ie->lpData, ie->eData);
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
			cout << "generating data..." << endl;
			start ();
		}

	}

	void start() {
		thread.generate_data (plot_size);
	}

	
public slots:
	void updateData(vector<double> lpD, vector<double> eD)
	{
		// update the display
// 		assert (d.size() <= plot_size);
		for (int i=0; i< plot->plot_size; i++) {
			if (i < lpD.size()) {
				cout << "updating lp data..." << endl;
				cout << lpD[i] << endl;			
				plot->learnprogData[i] = lpD[i];
			}
			if (i < eD.size()) {
				cout << "updating e data..." << endl;
				cout << eD[i] << endl;			
				plot->errorData[i] = eD[i];				
			}
		}
		repaint();
	}


};


class MainWindowI : virtual public smlearning::plotting::MainWindow {
public:
	::MainWindow *w;
	MainWindowI(int argc, char *argv[], Ice::CommunicatorPtr communicator) {
		w = new ::MainWindow();
	}

	void init ( int size, const smlearning::plotting::SeqDouble& lpData, const smlearning::plotting::SeqDouble& eData, const Ice::Current& ) {
		QApplication::postEvent(w, new InitEvent(size, lpData, eData) );

	}
	
	void resize ( int width, int height, const Ice::Current& ) {
		QApplication::postEvent(w, new ResizeEvent(width, height) );
	}

	void start ( const Ice::Current& ) {
		QApplication::postEvent(w, new StartEvent() );
	}

	void show (const Ice::Current& ) {
		QApplication::postEvent(w, new ShowEvent() );
	}

};


class MyWidgets {
public:
	::MainWindow *mainWindow;
};
