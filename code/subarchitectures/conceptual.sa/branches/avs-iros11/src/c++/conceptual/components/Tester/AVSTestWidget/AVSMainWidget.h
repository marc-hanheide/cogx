#ifndef AVSMainWidget_H
#define AVSMainWidget_H

#include <QtGui/QDialog>
#include "ui_MainDialog.h"
#include "SpatialData.hpp"
#include "ConceptualData.hpp"
#include "Tester"

namespace conceptual
{
	class Tester;
}


class AVSMainWidget : public QDialog
{
    Q_OBJECT

public:
    AVSMainWidget(conceptual::Tester * component);
    ~AVSMainWidget();
public slots:
    void generateViewConesButtonClicked();
    void processConeGroup();
    bool parseQuery(std::string queryString, std::vector<std::string> &variables);

private:
    Ui::MainAVSWidget ui;
    conceptual::Tester* m_component;
};

#endif // MAINDIALOG_H
