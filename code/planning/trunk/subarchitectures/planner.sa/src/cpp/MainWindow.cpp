#include "MainWindow.hpp"
#include "MainWidget.hpp"
#include "WMControl.cpp"

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

void MainWindow::showPlan(string plan) {
    canvas-> showPlan();
}

/**
Protected
**/

MainWindow* MainWindow::instance = NULL;

MainWindow::MainWindow() : QMainWindow() {
}

void MainWindow::init(WMControl* Planner) {
    planner = Planner;

    initCentralWidget();
    setProperties();
}

void MainWindow::initCentralWidget() {
    canvas = MainWidget::getInstance();
    setCentralWidget(canvas);
}

void MainWindow::setProperties() {
    resize(640,480);
    show();
    setFocus();
}
