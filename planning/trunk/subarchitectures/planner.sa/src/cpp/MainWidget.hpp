#ifndef DEBUG
#define DEBUG
#endif

#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

#include <qwidget.h>

class MainWidget : public QWidget {
Q_OBJECT

public:
    static MainWidget* getInstance();

    void showPlan(std::string plan);

public slots:

signals:

protected: 
    static MainWidget* instance;

    MainWidget();

protected slots:
};

#endif
