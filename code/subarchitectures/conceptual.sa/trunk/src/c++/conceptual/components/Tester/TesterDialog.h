/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::TesterDialog class.
 */

#ifndef CONCEPTUAL_TESTERDIALOG_H
#define CONCEPTUAL_TESTERDIALOG_H

// Conceptual.SA
#include "ui_TesterDialog.h"
#include "ConceptualData.hpp"
// Qt
#include <QDialog>

namespace conceptual
{

class Tester;

class TesterDialog : public QDialog, public Ui_TesterDialog
{
  Q_OBJECT

public:

  /** Constructor. */
  TesterDialog(Tester *conceptualTester,
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
  DefaultData::DiscreteProbabilityDistribution _queryResults;

  /** Recent query. */
  std::string _query;
};


}

#endif // CONCEPTUAL_TESTERDIALOG_H

