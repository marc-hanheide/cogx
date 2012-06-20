#include "ObjectSearchResultDialog.h"
#include "ConceptualWidget.h"
#include "Tester.h"
#include "SpatialData.hpp"
#include <cast/core/CASTData.hpp>

#include <boost/algorithm/string/erase.hpp>
#include <boost/lexical_cast.hpp>

using namespace boost;
using namespace std;


ObjectSearchResultDialog::ObjectSearchResultDialog(ConceptualWidget *parent, conceptual::Tester *component)
    : QDialog(parent), _component(component)
{
	ui.setupUi(this);
	connect(this, SIGNAL(accepted()), this, SLOT(on_ObjectSearchResultDialogClass_accepted()));

	_objectCategories = component->getObjectCategories();

	pthread_mutex_lock(&parent->_worldStateMutex);
	for (unsigned int i=0; i<parent->_wsPtr->rooms.size(); ++i)
		ui.roomComboBox->addItem("room-"+QString::number(parent->_wsPtr->rooms[i].roomId));
	pthread_mutex_unlock(&parent->_worldStateMutex);

	// Fill in the default values
	ui.supportObjectCategoryComboBox->addItem("");
	for (unsigned int i=0; i<_objectCategories.size(); ++i)
	{
		ui.supportObjectCategoryComboBox->addItem(QString::fromStdString(_objectCategories[i]));
		ui.objectCategoryComboBox->addItem(QString::fromStdString(_objectCategories[i]));
	}
}

ObjectSearchResultDialog::~ObjectSearchResultDialog()
{

}

void ObjectSearchResultDialog::on_ObjectSearchResultDialogClass_accepted()
{
	vector< boost::shared_ptr< cast::CASTData< SpatialData::ObjectSearchResult > > > results;
	_component->getWorkingMemoryEntries<SpatialData::ObjectSearchResult> ("spatial.sa", 0 , results);

    // Check if we have such result already
	string searchedObjectCategory = ui.objectCategoryComboBox->currentText().toStdString();
	string supportObjectCategory = ui.supportObjectCategoryComboBox->currentText().toStdString();
	string supportObjectId = ui.supportObjectIdLineEdit->text().toStdString();
	string roomId = ui.roomComboBox->currentText().toStdString();
	erase_first(roomId, "room-");
	int rId = lexical_cast<int>(roomId);
	SpatialData::SpatialRelation sr;
	switch(ui.relationComboBox->currentIndex())
	{
	case 1:
		sr = SpatialData::INOBJECT;
		break;
	case 2:
		sr = SpatialData::ON;
		break;
	default:
		sr = SpatialData::INROOM;
		break;
	}
	double beta = static_cast<double>(ui.betaSpinBox->value())/100.0;
	for (unsigned int i=0; i<results.size(); ++i)
	{
		if ((results[i]->getData()->searchedObjectCategory == searchedObjectCategory) &&
			(results[i]->getData()->supportObjectId == supportObjectId) &&
			(results[i]->getData()->supportObjectCategory == supportObjectCategory) &&
			(results[i]->getData()->relation == sr) &&
			(results[i]->getData()->roomId == rId))
		{
			results[i]->getData()->beta = beta;
			_component->overwriteWorkingMemory<SpatialData::ObjectSearchResult>(results[i]->getID(), "spatial.sa", results[i]->getData());
			return;
		}
	}

	SpatialData::ObjectSearchResultPtr osr = new SpatialData::ObjectSearchResult();
	osr->beta = beta;
	osr->roomId = rId;
	osr->searchedObjectCategory = searchedObjectCategory;
	osr->supportObjectCategory = supportObjectCategory;
	osr->supportObjectId = supportObjectId;
	osr->relation = sr;

	_component->addToWorkingMemory<SpatialData::ObjectSearchResult>(
			_component->newDataID(), "spatial.sa", osr);
}
