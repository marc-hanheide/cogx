#ifndef AVSMainWidget_H
#define AVSMainWidget_H

#include <QtGui/QDialog>
#include "ui_AVSMainWidget.h"
#include "SpatialData.hpp"
#include "ConceptualData.hpp"
#include "Tester.h"

namespace conceptual
{
	class Tester;
}


class AVSMainWidget : public QDialog
{
    Q_OBJECT

public:
    AVSMainWidget(QWidget *parent, conceptual::Tester * component);
    ~AVSMainWidget();
public slots:
    void generateViewConesButtonClicked();
    void processConeGroup();
    bool parseQuery(std::string queryString, std::vector<std::string> &variables);

private:
    Ui::AVSMainWidget ui;
    conceptual::Tester* m_component;
};

#endif // MAINDIALOG_H
