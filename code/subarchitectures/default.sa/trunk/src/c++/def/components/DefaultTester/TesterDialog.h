/**
 * @author Andrzej Pronobis
 *
 * Declaration of the def::TesterDialog class.
 */

#ifndef DEFAULT_TESTERDIALOG_H
#define DEFAULT_TESTERDIALOG_H

// Default.SA
#include "ui_TesterDialog.h"
#include "DefaultData.hpp"
// Qt
#include <QDialog>

namespace def
{

class Tester;

class TesterDialog : public QDialog, public Ui_TesterDialog
{
  Q_OBJECT

public:

  /** Constructor. */
  TesterDialog(Tester *tester,
		  bool hfcServerDestination, bool queryHandlerDestination,
		  QWidget *parent = 0);


signals:

	void queryResultsUpdatedSignal();


private slots:

  void sendButtonClicked();
  void queryResultsUpdated();

  /** This function checks if the dialog should be closed. */
  void exitTimerFired();


private:

  Tester *_tester;

  /** Results of the recent QDL query. */
  DefaultData::QdlQueryResults _qdlQueryResults;

  /** Recent query. */
  std::string _query;
};


}

#endif // DEFAULT_TESTERDIALOG_H

