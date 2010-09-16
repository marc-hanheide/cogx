/**
 * @author Andrzej Pronobis
 *
 * Declaration of the ConceptualTesterDialog class.
 */

#ifndef CONCEPTUALTESTERDIALOG_H
#define CONCEPTUALTESTERDIALOG_H

// Conceptual.SA
#include "ui_ConceptualTesterDialog.h"
#include "ConceptualData.hpp"
// Qt
#include <QDialog>

namespace def
{

class ConceptualTester;

class ConceptualTesterDialog : public QDialog, public Ui_ConceptualTesterDialog
{
  Q_OBJECT

public:

  /** Constructor. */
  ConceptualTesterDialog(ConceptualTester *conceptualTester,
		  QWidget *parent = 0);


signals:

	void queryResultsUpdatedSignal();


private slots:

  void sendButtonClicked();
  void queryResultsUpdated();

  /** This function checks if the dialog should be closed. */
  void exitTimerFired();


private:

  ConceptualTester *_conceptualTester;

  /** Results of the recent QDL query. */
  DefaultData::DiscreteProbabilityDistribution _queryResults;

  /** Recent query. */
  std::string _query;
};


}

#endif // CONCEPTUALTESTERDIALOG_H

