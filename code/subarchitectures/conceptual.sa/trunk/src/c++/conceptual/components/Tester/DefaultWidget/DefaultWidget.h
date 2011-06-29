#ifndef DEFAULTWIDGET_H
#define DEFAULTWIDGET_H

#include <QtGui/QWidget>
#include "ui_DefaultWidget.h"


namespace conceptual
{
	class Tester;
}


class DefaultWidget : public QWidget
{
    Q_OBJECT

public:
    DefaultWidget(QWidget *parent, conceptual::Tester *component);
    ~DefaultWidget();

private:
    Ui::DefaultWidgetClass ui;
    conceptual::Tester *_component;

};

#endif // DEFAULTWIDGET_H
