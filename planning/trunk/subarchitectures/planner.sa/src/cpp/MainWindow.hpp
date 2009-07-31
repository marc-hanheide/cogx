#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <qmainwindow.h>

class MainWidget;
class WMControl;

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    static MainWindow* getInstance();

    void init(WMControl* Planner);
    void showPlan(std::string plan);

public slots:

signals:

protected: 
    static MainWindow* instance;
    MainWidget* canvas;
    WMControl* planner;

    MainWindow();

    void initCentralWidget();
    void setProperties();

protected slots:

};

#endif
