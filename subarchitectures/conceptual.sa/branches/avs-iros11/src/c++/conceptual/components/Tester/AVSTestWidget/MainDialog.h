#ifndef MAINDIALOG_H
#define MAINDIALOG_H

#include <QtGui/QDialog>
#include "ui_MainDialog.h"
#include "SpatialData.hpp"


namespace spatial
{
	class AVS_ContinualPlanner;
}


class MainDialog : public QDialog
{
    Q_OBJECT

public:
    MainDialog(spatial::AVS_ContinualPlanner * component);
    ~MainDialog();
public slots:
    void generateViewConesButtonClicked();
    void processConeGroup();
    bool parseQuery(std::string queryString, std::vector<std::string> &variables);

private:
    Ui::MainDialogClass ui;
    spatial::AVS_ContinualPlanner* m_component;
};

#endif // MAINDIALOG_H
