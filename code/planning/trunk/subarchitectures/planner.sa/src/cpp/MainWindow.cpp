#include "MainWindow.hpp"
#include "MainWidget.hpp"

#include <QtGui>

#include <iostream>

using namespace std;

/**
Public
**/

MainWindow* MainWindow::getInstance() {
    if(instance == NULL)
	instance = new MainWindow();
    return instance;
}

/**
Protected
**/

MainWindow* MainWindow::instance = NULL;

MainWindow::MainWindow() : QMainWindow() {
}

void MainWindow::init() {
#ifdef DEBUG
    cerr << "MainWindow::init() started" << endl;
#endif

    initActions();
    initMenuBar();
    statusBar();
    initCentralWidget();
    setProperties();

#ifdef DEBUG
    cerr << "MainWindow::init() finished" << endl;
#endif
}

void MainWindow::initActions() {
#ifdef DEBUG
    cerr << "  MainWindow::initActions() started" << endl;
#endif

    actChangeWorkspace = new QAction("&Change Workspace", this);
    actChangeWorkspace->setStatusTip("Change to Another Workspace");

    actNewDomain = new QAction("&New Domain", this);
    actNewDomain->setStatusTip("Create a New Domain in the Current Workspace");

    actNewTask = new QAction("&New Task", this);
    actNewTask->setStatusTip("Create a New Task in the Current Domain");

    connect(actChangeWorkspace, SIGNAL(triggered()), this, SLOT(changeWorkspace()));
    connect(actChangeWorkspace, SIGNAL(triggered()), this, SIGNAL(sig_changeWorkspace()));
    connect(actNewDomain, SIGNAL(triggered()), this, SLOT(newDomain()));
    connect(actNewDomain, SIGNAL(triggered()), this, SIGNAL(sig_newDomain()));
    connect(actNewTask, SIGNAL(triggered()), this, SLOT(newTask()));
    connect(actNewTask, SIGNAL(triggered()), this, SIGNAL(sig_newTask()));

#ifdef DEBUG
    cerr << "  MainWindow::initActions() finished" << endl;
#endif
}

void MainWindow::initMenuBar() {
#ifdef DEBUG
    cerr << "  MainWindow::initMenuBar() started" << endl;
#endif

    menuFile = menuBar()->addMenu("&File");
    menuFile->addAction(actChangeWorkspace);

    domainFile = menuBar()->addMenu("&Domain");
    domainFile->addAction(actNewDomain);

    taskFile = menuBar()->addMenu("&Task");
    taskFile->addAction(actNewTask);

#ifdef DEBUG
    cerr << "  MainWindow::initMenuBar() finished" << endl;
#endif
}

void MainWindow::initCentralWidget() {
    setCentralWidget(MainWidget::getInstance());
}

void MainWindow::setProperties() {
    resize(320,240);
    show();
    setFocus();
}

/**
Private Slots
**/

void MainWindow::changeWorkspace() {
#ifdef DEBUG
    cerr << "Slot MainWindow::changeWorkspace() triggered and started" << endl;
#endif

#ifdef DEBUG
    cerr << "Slot MainWindow::changeWorkspace() finished" << endl;
#endif
}

void MainWindow::newDomain() {
#ifdef DEBUG
    cerr << "Slot MainWindow::newDomain() triggered and started" << endl;
#endif

#ifdef DEBUG
    cerr << "Slot MainWindow::newDomain() finished" << endl;
#endif
}

void MainWindow::newTask() {
#ifdef DEBUG
    cerr << "Slot MainWindow::newTask() triggered and started" << endl;
#endif

#ifdef DEBUG
    cerr << "Slot MainWindow::newTask() finished" << endl;
#endif
}
