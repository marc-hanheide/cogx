#ifndef AVSMainWidget_H
#define AVSMainWidget_H

#include <QtGui/QDialog>
#include "ui_AVSMainWidget.h"
#include "SpatialData.hpp"
#include "ConceptualData.hpp"
#include "Tester.h"


class AVSMainWidget : public QDialog
{
    Q_OBJECT

public:
    AVSMainWidget(QWidget *parent, conceptual::Tester * component);
    ~AVSMainWidget();
    bool parseQuery(std::string queryString, std::vector<std::string> &variables);
public slots:
    void generateViewConesButtonClicked();
    void processConeGroup();
    void postVisualObjectClicked();

private:
    Ui::AVSMainWidget ui;
    conceptual::Tester* m_component;
};

#endif // MAINDIALOG_H
