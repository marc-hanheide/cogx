#include "MainWidget.hpp"

#include <QtGui>

#include <iostream>

using namespace std;

/**
Public
**/

MainWidget* MainWidget::getInstance() {
    if(instance == NULL)
	instance = new MainWidget();
    return instance;
}

void MainWidget::showPlan(string plan) {

}

/**
Protected
**/

MainWidget* MainWidget::instance = NULL;

MainWidget::MainWidget() : QWidget() {
}
