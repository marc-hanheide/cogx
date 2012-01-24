#ifndef NAVWIDGET_H
#define NAVWIDGET_H

#include <QtGui/QWidget>
#include "ui_NavWidget.h"
#include "SpatialData.hpp"

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

public slots:

	void newNavCommand(SpatialData::NavCommandPtr navCommandPtr);

	void newNavCommand(QString str);


private slots:

	void gotoxyButtonClicked();


private:
    Ui::NavWidgetClass ui;
    conceptual::Tester *_component;

};

#endif // NAVWIDGET_H
