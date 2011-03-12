#include "MainDialog.h"
#include "ConceptualWidget.h"
#include "AVSMainWidget.h"
#include "DefaultWidget.h"

MainDialog::MainDialog(conceptual::Tester *component)
    : QDialog(0, Qt::WindowMinMaxButtonsHint), _component(component)
{
	ui.setupUi(this);
	_conceptualWidget = new ConceptualWidget(this, component);
	_defaultWidget = new DefaultWidget(this, component);
	_avsMainWidget = new AVSMainWidget(this, component);
	ui.tabWidget->addTab(_conceptualWidget, "Conceptual.SA Tester");
	ui.tabWidget->addTab(_defaultWidget, "Default.SA Tester");
	ui.tabWidget->addTab(_avsMainWidget, "AVS Tester");
}

MainDialog::~MainDialog()
{

}
