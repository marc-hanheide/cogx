/**
 * @author Andrzej Pronobis
 *
 * Main dialog.
 */

#ifndef MAINDIALOG_H
#define MAINDIALOG_H

#include <QtGui/QDialog>
#include <queue>


#include "ui_MainDialog.h"

namespace conceptual
{
	class Tester;
}


class MainDialog : public QDialog, public Ui::MainDialogClass
{
    Q_OBJECT

public:
    MainDialog(conceptual::Tester *component);
    ~MainDialog();

public:

    void newWorldState();


signals:

	void setWsFrequencySignal(double freq);


private slots:

	void setWsFrequency(double freq);
	void sendQueryButtonClicked();

private:
    Ui::MainDialogClass ui;
    conceptual::Tester *_component;

    std::queue<qint64> _wsUpdateTimes;

};



#endif // MAINDIALOG_H
