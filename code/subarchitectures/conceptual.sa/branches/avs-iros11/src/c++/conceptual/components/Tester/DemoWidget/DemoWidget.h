#ifndef DEMOWIDGET_H
#define DEMOWIDGET_H

#include <QtGui/QWidget>
#include "ui_DemoWidget.h"


namespace conceptual
{
	class Tester;
}


class DemoWidget : public QWidget
{
    Q_OBJECT

public:
    DemoWidget(QWidget *parent, conceptual::Tester *component);
    ~DemoWidget();

private:
    Ui::DemoWidgetClass ui;
    conceptual::Tester *_component;

};

#endif // DEMOWIDGET_H
