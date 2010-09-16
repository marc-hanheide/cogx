/**
 * @author Andrzej Pronobis
 *
 * Definition of the ConceptualTesterDialog class.
 */

// Conceptual.SA
#include "ConceptualTesterDialog.h"
#include "ConceptualTester.h"
// Qt
#include <QTimer>
#include <QTableWidgetItem>
// Std
#include <iostream>
#include <math.h>

namespace def
{

using namespace std;

// ------------------------------------------------------
ConceptualTesterDialog::ConceptualTesterDialog(ConceptualTester *conceptualTester, QWidget *parent):
    QDialog(parent), _conceptualTester(conceptualTester)
{
  // Setup ui
  setupUi(this);

  // Connect signals to slots
  connect(sendPushButton, SIGNAL(clicked()), this, SLOT(sendButtonClicked()));
  connect(this, SIGNAL(queryResultsUpdatedSignal()), this, SLOT(queryResultsUpdated()));

  // Exit timer, checks if the dialog should be closed
  QTimer *exitTimer = new QTimer(this);
  connect(exitTimer, SIGNAL(timeout()), this, SLOT(exitTimerFired()));
  exitTimer->start(100);
}

// ------------------------------------------------------
void ConceptualTesterDialog::sendButtonClicked()
{
	_query = queryLineEdit->text().toStdString();
	_queryResults = _conceptualTester->sendQuery(_query);

	// Emit signal
	emit queryResultsUpdatedSignal();
}


// ------------------------------------------------------
void ConceptualTesterDialog::queryResultsUpdated()
{
/*	// Init the table
	resultsTableWidget->clear();
	resultsTableWidget->setHorizontalHeaderLabels(QStringList());
	resultsTableWidget->setColumnCount(_qdlQueryResults.bt[0].size());
	resultsTableWidget->setRowCount(_qdlQueryResults.bt.size());

	// Get variables from query
	QString query;
	query=QString().fromStdString(_query).trimmed();
	query=query.mid(6);
	query=query.left(query.indexOf("where", 0, Qt::CaseInsensitive));
	QStringList queryVars = query.simplified().split(" ");
	// Map the variables to the right order
	vector<int> varMap(queryVars.size());
	for (int i = 0; i < queryVars.size(); ++i)
		varMap[_qdlQueryResults.varPosMap[queryVars.at(i).toStdString()]]=i;

	// Fill the table
	resultsTableWidget->setHorizontalHeaderLabels(queryVars);
	for (unsigned int i = 0; i < _qdlQueryResults.bt.size(); i++)
	{
		vector<string> currLine = _qdlQueryResults.bt[i];
		for (unsigned int j = 0; j < currLine.size(); j++)
		{
			QTableWidgetItem *twi = new QTableWidgetItem(currLine[j].c_str());
			resultsTableWidget->setItem(i, varMap[j], twi);
		}
	}
	resultsTableWidget->resizeColumnsToContents ();*/
}


// ------------------------------------------------------
void ConceptualTesterDialog::exitTimerFired()
{
  if (!_conceptualTester->isRunning())
  {
    done(0);
  }
}



} // namespace def
