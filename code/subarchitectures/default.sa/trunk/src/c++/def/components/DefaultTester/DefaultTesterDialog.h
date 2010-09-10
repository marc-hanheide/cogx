/**
 * @author Andrzej Pronobis
 *
 * Declaration of the DefaultTesterDialog class.
 */

#ifndef DEFAULTTESTERDIALOG_H
#define DEFAULTTESTERDIALOG_H

// Default.SA
#include "ui_DefaultTesterDialog.h"
#include "DefaultData.hpp"
// Qt
#include <QDialog>

namespace def
{

class DefaultTester;

class DefaultTesterDialog : public QDialog, public Ui_DefaultTesterDialog
{
  Q_OBJECT

public:

  /** Constructor. */
  DefaultTesterDialog(DefaultTester *defaultTester,
		  bool hfcServerDestination, bool qdlQueryHandlerDestination,
		  QWidget *parent = 0);


signals:

	void queryResultsUpdatedSignal();


private slots:

  void sendButtonClicked();
  void queryResultsUpdated();

  /** This function checks if the dialog should be closed. */
  void exitTimerFired();


private:

  DefaultTester *_defaultTester;

  /** Results of the recent query. */
  DefaultData::QueryResults _queryResults;

  /** Recent query. */
  std::string _query;
};


}

#endif // DEFAULTTESTERDIALOG_H

