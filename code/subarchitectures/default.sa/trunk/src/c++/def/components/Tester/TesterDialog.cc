/**
 * @author Andrzej Pronobis
 *
 * Definition of the def::TesterDialog class.
 */

// Default.SA
#include "TesterDialog.h"
#include "Tester.h"
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
TesterDialog::TesterDialog(Tester *tester,
		bool hfcServerDestination, bool queryHandlerDestination, QWidget *parent):
    QDialog(parent), _tester(tester)
{
  // Setup ui
  setupUi(this);
  hfcServerRadioButton->setEnabled(hfcServerDestination);
  qdlQueryHandlerRadioButton->setEnabled(queryHandlerDestination);
  hfcServerRadioButton->setChecked(hfcServerDestination);
  qdlQueryHandlerRadioButton->setChecked(!hfcServerDestination && queryHandlerDestination);

  // Connect signals to slots
  connect(sendPushButton, SIGNAL(clicked()), this, SLOT(sendButtonClicked()));
  connect(this, SIGNAL(queryResultsUpdatedSignal()), this, SLOT(queryResultsUpdated()));

  // Exit timer, checks if the dialog should be closed
  QTimer *exitTimer = new QTimer(this);
  connect(exitTimer, SIGNAL(timeout()), this, SLOT(exitTimerFired()));
  exitTimer->start(100);
}

// ------------------------------------------------------
void TesterDialog::sendButtonClicked()
{
	Tester::QueryDestination destination =
			(hfcServerRadioButton->isChecked())?Tester::QD_HFC_SERVER:Tester::QD_QUERY_HANDLER;

	_query = queryLineEdit->text().toStdString();
	_qdlQueryResults = _tester->sendQuery(_query, destination);

	// Emit signal
	emit queryResultsUpdatedSignal();
}


// ------------------------------------------------------
void TesterDialog::queryResultsUpdated()
{
	// Init the table
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
	resultsTableWidget->resizeColumnsToContents ();
}


// ------------------------------------------------------
void TesterDialog::exitTimerFired()
{
  if (!_tester->isRunning())
  {
    done(0);
  }
}



} // namespace def
