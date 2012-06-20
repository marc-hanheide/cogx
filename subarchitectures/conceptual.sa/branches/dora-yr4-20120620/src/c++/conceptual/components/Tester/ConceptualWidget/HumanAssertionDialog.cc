#include "HumanAssertionDialog.h"
#include "Tester.h"
#include "ConceptualWidget.h"
#include "SpatialProperties.hpp"
#include "SpatialData.hpp"

HumanAssertionDialog::HumanAssertionDialog(ConceptualWidget *parent, conceptual::Tester *component)
    : QDialog(parent), _component(component)
{
	ui.setupUi(this);
	connect(this, SIGNAL(accepted()), this, SLOT(dialogAccepted()));

	_humanAssertions = component->getHumanAssertions();

	pthread_mutex_lock(&parent->_worldStateMutex);
	for (unsigned int i=0; i<parent->_wsPtr->rooms.size(); ++i)
	{
		_placesForRooms.push_back(parent->_wsPtr->rooms[i].places[0].placeId);
		ui.roomComboBox->addItem("room-"+QString::number(parent->_wsPtr->rooms[i].roomId));
	}
	pthread_mutex_unlock(&parent->_worldStateMutex);

	// Fill in the default values
	for (unsigned int i=0; i<_humanAssertions.size(); ++i)
	{
		ui.humanAssertionComboBox->addItem(QString::fromStdString(_humanAssertions[i]));
	}
}

HumanAssertionDialog::~HumanAssertionDialog()
{

}


void HumanAssertionDialog::dialogAccepted()
{
	SpatialProperties::RoomHumanAssertionPlacePropertyPtr opp = new SpatialProperties::RoomHumanAssertionPlaceProperty();
	opp->inferred = false;
	opp->assertion = ui.humanAssertionComboBox->currentText().toStdString();
	opp->placeId = _placesForRooms[ui.roomComboBox->currentIndex()];
	opp->mapValueReliable = false;

	_component->addToWorkingMemory<SpatialProperties::RoomHumanAssertionPlaceProperty>(
			_component->newDataID(), "spatial.sa", opp);
}
