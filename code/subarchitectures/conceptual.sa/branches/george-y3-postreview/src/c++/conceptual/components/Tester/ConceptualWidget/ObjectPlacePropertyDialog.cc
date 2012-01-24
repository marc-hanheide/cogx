#include "ObjectPlacePropertyDialog.h"
#include "Tester.h"
#include "ConceptualWidget.h"
#include "SpatialProperties.hpp"
#include "SpatialData.hpp"


ObjectPlacePropertyDialog::ObjectPlacePropertyDialog(ConceptualWidget *parent, conceptual::Tester *component)
    : QDialog(parent), _component(component)
{
	ui.setupUi(this);
	connect(this, SIGNAL(accepted()), this, SLOT(on_ObjectPlacePropertyDialogClass_accepted()));

	_objectCategories = component->getObjectCategories();

	pthread_mutex_lock(&parent->_worldStateMutex);
	for (unsigned int i=0; i<parent->_wsPtr->rooms.size(); ++i)
	{
		_placesForRooms.push_back(parent->_wsPtr->rooms[i].places[0].placeId);
		ui.roomComboBox->addItem("room-"+QString::number(parent->_wsPtr->rooms[i].roomId));
	}
	pthread_mutex_unlock(&parent->_worldStateMutex);

	// Fill in the default values
	ui.supportObjectCategoryComboBox->addItem("");
	for (unsigned int i=0; i<_objectCategories.size(); ++i)
	{
		ui.supportObjectCategoryComboBox->addItem(QString::fromStdString(_objectCategories[i]));
		ui.objectCategoryComboBox->addItem(QString::fromStdString(_objectCategories[i]));
	}
}


ObjectPlacePropertyDialog::~ObjectPlacePropertyDialog()
{

}


void ObjectPlacePropertyDialog::on_ObjectPlacePropertyDialogClass_accepted()
{
	SpatialProperties::ObjectPlacePropertyPtr opp = new SpatialProperties::ObjectPlaceProperty();
	opp->inferred = false;
	opp->category = ui.objectCategoryComboBox->currentText().toStdString();
	opp->placeId = _placesForRooms[ui.roomComboBox->currentIndex()];
	switch(ui.relationComboBox->currentIndex())
	{
	case 1:
		opp->relation = SpatialData::INOBJECT;
		break;
	case 2:
		opp->relation = SpatialData::ON;
		break;
	default:
		opp->relation = SpatialData::INROOM;
		break;
	}
	opp->supportObjectCategory = ui.supportObjectCategoryComboBox->currentText().toStdString();
	opp->supportObjectId = ui.supportObjectIdLineEdit->text().toStdString();
	SpatialProperties::DiscreteProbabilityDistributionPtr pd = new SpatialProperties::DiscreteProbabilityDistribution();
	SpatialProperties::ValueProbabilityPair vap1;
		SpatialProperties::BinaryValuePtr bv1 = new SpatialProperties::BinaryValue();
		bv1->value = true;
		vap1.probability = 1.0;
		vap1.value = bv1;
	pd->data.push_back(vap1);
	SpatialProperties::ValueProbabilityPair vap2;
		SpatialProperties::BinaryValuePtr bv2 = new SpatialProperties::BinaryValue();
		bv2->value = false;
		vap2.probability = 0.0;
		vap2.value = bv2;
	pd->data.push_back(vap2);

	opp->distribution = pd;
		SpatialProperties::BinaryValuePtr bv3 = new SpatialProperties::BinaryValue();
		bv3->value = true;
	opp->mapValue = bv3;
	opp->mapValueReliable = true;

	_component->addToWorkingMemory<SpatialProperties::ObjectPlaceProperty>(
			_component->newDataID(), "spatial.sa", opp);
}
