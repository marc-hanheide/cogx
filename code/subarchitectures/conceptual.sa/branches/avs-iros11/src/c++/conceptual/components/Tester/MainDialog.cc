#include "MainDialog.h"
#include "ConceptualWidget.h"

MainDialog::MainDialog(conceptual::Tester *component)
    : QDialog(0), _component(component)
{
	ui.setupUi(this);
	_conceptualWidget = new ConceptualWidget(this, component);
	ui.tabWidget->addTab(_conceptualWidget, "Conceptual Tester");
}

MainDialog::~MainDialog()
{

}
