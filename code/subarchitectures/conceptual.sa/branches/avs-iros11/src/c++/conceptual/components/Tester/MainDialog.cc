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


	connect(this, SIGNAL(setWsFrequencySignal(double)), this, SLOT(setWsFrequency(double)));
	connect(sendQueryButton, SIGNAL(clicked()), this, SLOT(sendQueryButtonClicked()));
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
	queryResultTreeWidget->clear();

	SpatialProbabilities::ProbabilityDistribution result =
			_component->sendQueryHandlerQuery(queryLineEdit->text().toStdString(),
					!standardQueryRadioButton->isChecked());

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
			cout << (result.massFunction[i].variableValues[j]->ice_staticId()) << endl;
			cout << (result.massFunction[i].variableValues[j]->ice_id()) << endl;
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
}


