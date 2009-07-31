#ifndef DEBUG
#define DEBUG
#endif

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <qmainwindow.h>

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    static MainWindow* getInstance();

    void init();

    public slots:

    signals:
    void sig_changeWorkspace();
    void sig_newDomain();
    void sig_newTask();

protected: 
    static MainWindow* instance;

    QMenu* menuFile;
    QMenu* domainFile;
    QMenu* taskFile;

    QAction* actChangeWorkspace;
    QAction* actNewDomain;
    QAction* actNewTask;

    MainWindow();

    void initActions();
    void initMenuBar();
    void initCentralWidget();
    void setProperties();

protected slots:
    void changeWorkspace();
    void newDomain();
    void newTask();
};

#endif
