/**
 * @author Andrzej Pronobis
 *
 * Main dialog.
 */

#ifndef MAINDIALOG_H
#define MAINDIALOG_H

// Conceptual.SA
#include "ui_MainDialog.h"
#include "ConceptualData.hpp"

// Qt&Std
#include <QtGui/QDialog>
#include <queue>

class QTimer;

namespace conceptual
{
	class Tester;
}

class ObjectPlacePropertyDialog;
class ObjectSearchResultDialog;

class MainDialog : public QDialog, public Ui::MainDialogClass
{
    Q_OBJECT

    friend class ObjectPlacePropertyDialog;
    friend class ObjectSearchResultDialog;

public:
    MainDialog(conceptual::Tester *component);
    ~MainDialog();

public:

    void newWorldState(ConceptualData::WorldStatePtr wsPtr);


private slots:

	void sendQueryButtonClicked();
	void refreshVarsButtonClicked();
	void refreshWsButtonClicked();
	void showGraphButtonClicked();
	void varListCurrentTextChanged(const QString &curText);
	void factorListCurrentTextChanged(const QString &curText);
	void wsTimerTimeout();
	void addObjectPlacePropertyButtonClicked();
	void addObjectSearchResultButtonClicked();


private:
    conceptual::Tester *_component;

    QTimer *_wsTimer;
    int _wsCount;

    std::queue<qint64> _wsUpdateTimes;

	pthread_mutex_t _worldStateMutex;
	ConceptualData::WorldStatePtr _wsPtr;

};



#endif // MAINDIALOG_H
