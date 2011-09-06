#include "DefaultWidget.h"
#include "Tester.h"


DefaultWidget::DefaultWidget(QWidget *parent, conceptual::Tester *component)
    : QWidget(parent), _component(component)
{
	ui.setupUi(this);

	DefaultData::StringSeq humanAssertions = component->getHumanAssertions();
	for (unsigned int i=0; i<humanAssertions.size(); ++i)
		ui.humanAssertionsListWidget->addItem(QString::fromStdString(humanAssertions[i]));

	DefaultData::StringSeq appearances = component->getAppearances();
	for (unsigned int i=0; i<appearances.size(); ++i)
		ui.appearancesListWidget->addItem(QString::fromStdString(appearances[i]));

	DefaultData::StringSeq shapes = component->getShapes();
	for (unsigned int i=0; i<shapes.size(); ++i)
		ui.shapesListWidget->addItem(QString::fromStdString(shapes[i]));

	DefaultData::StringSeq sizes = component->getSizes();
	for (unsigned int i=0; i<sizes.size(); ++i)
		ui.sizesListWidget->addItem(QString::fromStdString(sizes[i]));

	DefaultData::StringSeq rooms = component->getRoomCategories();
	for (unsigned int i=0; i<rooms.size(); ++i)
		ui.roomsListWidget->addItem(QString::fromStdString(rooms[i]));

	DefaultData::StringSeq objects = component->getObjectPropertyVariables();
	for (unsigned int i=0; i<objects.size(); ++i)
		ui.objectsListWidget->addItem(QString::fromStdString(objects[i]));

}

DefaultWidget::~DefaultWidget()
{

}
