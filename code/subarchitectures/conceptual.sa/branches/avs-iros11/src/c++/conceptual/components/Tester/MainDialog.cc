#include "MainDialog.h"
#include "ConceptualWidget.h"
#include "AVSMainWidget.h"

MainDialog::MainDialog(conceptual::Tester *component)
    : QDialog(0), _component(component)
{
	ui.setupUi(this);
	_conceptualWidget = new ConceptualWidget(this, component);
	_avsMainWidget = new AVSMainWidget(this, component);
	ui.tabWidget->addTab(_conceptualWidget, "Conceptual Tester");
	ui.tabWidget->addTab(_avsMainWidget, "AVS Tester");
}

MainDialog::~MainDialog()
{

}
