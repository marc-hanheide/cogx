#ifndef NAVWIDGET_H
#define NAVWIDGET_H

#include <QtGui/QWidget>
#include "ui_NavWidget.h"


namespace conceptual
{
	class Tester;
}


class NavWidget : public QWidget
{
    Q_OBJECT

public:
    NavWidget(QWidget *parent, conceptual::Tester *component);
    ~NavWidget();

private:
    Ui::NavWidgetClass ui;
    conceptual::Tester *_component;

};

#endif // NAVWIDGET_H
