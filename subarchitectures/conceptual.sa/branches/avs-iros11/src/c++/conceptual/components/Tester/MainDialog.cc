/**
 * @author Andrzej Pronobis
 *
 * Main dialog.
 */

#include "MainDialog.h"
#include "Tester.h"
#include "ConceptualData.hpp"
#include "SpatialProbabilities.hpp"

// Qt & std
#include <QDateTime>
#include <QCompleter>
#include <QStringListModel>
#include <queue>

using namespace conceptual;
using namespace std;


// -------------------------------------------------------
MainDialog::MainDialog(Tester *component)
    : QDialog(0), _component(component)
{
	setupUi(this);
	queryResultTreeWidget->setHeaderLabel("");
	queryResultTreeWidget->header()->setResizeMode(QHeaderView::ResizeToContents);

	// Signals and slots
	connect(this, SIGNAL(setWsFrequencySignal(double)), this, SLOT(setWsFrequency(double)));
	connect(sendQueryButton, SIGNAL(clicked()), this, SLOT(sendQueryButtonClicked()));
	connect(refreshVarsButton, SIGNAL(clicked()), this, SLOT(refreshVarsButtonClicked()));
	connect(refreshWsButton, SIGNAL(clicked()), this, SLOT(refreshWsButtonClicked()));
	connect(showGraphButton, SIGNAL(clicked()), this, SLOT(showGraphButtonClicked()));
	connect(variablesListWidget, SIGNAL(currentTextChanged(const QString &)), this, SLOT(varListCurrentTextChanged(const QString &)));
}


// -------------------------------------------------------
MainDialog::~MainDialog()
{

}


// -------------------------------------------------------
void MainDialog::newWorldState()
{
	qint64 time = QDateTime::currentMSecsSinceEpoch();
	_wsUpdateTimes.push(time);
	// Remove old entries
	while ((time - _wsUpdateTimes.front()) > 1000)
		_wsUpdateTimes.pop();

	// Get the average frequency
	double freq = static_cast<double>(time - _wsUpdateTimes.front()) / static_cast<double>(1000*_wsUpdateTimes.size());

	emit setWsFrequencySignal(freq);
}


// -------------------------------------------------------
void MainDialog::setWsFrequency(double freq)
{
	wsFreqLabel->setText(QString::number(freq));
}


// -------------------------------------------------------
void MainDialog::sendQueryButtonClicked()
{
	_component->log("Sending query " + queryComboBox->currentText().toStdString() + ".");

	queryResultTreeWidget->clear();

	SpatialProbabilities::ProbabilityDistribution result =
			_component->sendQueryHandlerQuery(queryComboBox->currentText().toStdString(),
					!standardQueryRadioButton->isChecked());

	if (result.massFunction.size()>0)
	{
		// Setup header
		vector<int> varNos;
		QStringList labels;
		for (map<string, int>::iterator it = result.variableNameToPositionMap.begin();
				it!=result.variableNameToPositionMap.end(); ++it)
		{
			varNos.push_back(it->second);
			labels.append(QString::fromStdString(it->first));
		}
		labels.append("p");

		queryResultTreeWidget->setHeaderLabels(labels);

		// Fill in values
		QList<QTreeWidgetItem *> items;
		for(unsigned int i=0; i<result.massFunction.size(); ++i)
		{
			double probability = result.massFunction[i].probability;

			QStringList values;
			for(unsigned int j=0; j<varNos.size(); ++j)
			{
				QString valueStr;
				if (result.massFunction[i].variableValues[j]->ice_isA("::SpatialProbabilities::StringRandomVariableValue"))
				{
					string value =
							SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
									result.massFunction[i].variableValues[j])->value;
					valueStr = QString::fromStdString(value);
				}
				if (result.massFunction[i].variableValues[j]->ice_isA("::SpatialProbabilities::IntRandomVariableValue"))
				{
					int value =
							SpatialProbabilities::IntRandomVariableValuePtr::dynamicCast(
									result.massFunction[i].variableValues[j])->value;
					valueStr = QString::number(value);
				}
				if (result.massFunction[i].variableValues[j]->ice_isA("::SpatialProbabilities::BoolRandomVariableValue"))
				{
					bool value =
							SpatialProbabilities::BoolRandomVariableValuePtr::dynamicCast(
									result.massFunction[i].variableValues[j])->value;
					valueStr = (value)?"true":"false";
				}
				values.append(valueStr);
			}
			values.append(QString::number(probability));
			items.append(new QTreeWidgetItem((QTreeWidget*)0, values));
		}

		queryResultTreeWidget->insertTopLevelItems(0, items);

		// Add to combo
		queryComboBox->insertItem(0, queryComboBox->currentText());
	}
	else
	{
		queryResultTreeWidget->setHeaderLabels(QStringList(" "));
		queryResultTreeWidget->insertTopLevelItem(0,
				new QTreeWidgetItem((QTreeWidget*)0, QStringList("Incorrect querry!")) );
	}
}


// -------------------------------------------------------
void MainDialog::refreshVarsButtonClicked()
{
	_component->log("Refreshing variable list");

	variablesListWidget->clear();
	ConceptualData::VariableInfos vis = _component->getChainGraphVariables();

	for (map<int, ConceptualData::VariableInfo>::iterator i=vis.begin(); i!=vis.end(); ++i)
	{
		variablesListWidget->addItem(QString::fromStdString(i->second.name));
	}
}

// -------------------------------------------------------
void MainDialog::varListCurrentTextChanged(const QString &curText)
{
	if (!curText.isEmpty())
	{
		queryComboBox->setEditText("p("+curText+")");
		sendQueryButtonClicked();
	}
}


// -------------------------------------------------------
void MainDialog::refreshWsButtonClicked()
{

}


// -------------------------------------------------------
void MainDialog::showGraphButtonClicked()
{

}

