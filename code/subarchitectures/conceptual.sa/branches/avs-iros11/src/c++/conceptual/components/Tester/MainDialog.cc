#include "MainDialog.h"
#include "ConceptualWidget.h"
#include "AVSMainWidget.h"
#include "NavWidget.h"
#include "DefaultWidget.h"
#include "DemoWidget.h"
#include "CategoricalWidget.h"

MainDialog::MainDialog(conceptual::Tester *component)
    : QDialog(0, Qt::WindowMinMaxButtonsHint), _component(component)
{
	ui.setupUi(this);
	_demoWidget = new DemoWidget(this, component);
	_conceptualWidget = new ConceptualWidget(this, component);
	_categoricalWidget = new CategoricalWidget(this, component);
	_defaultWidget = new DefaultWidget(this, component);
	_avsMainWidget = new AVSMainWidget(this, component);
	_navWidget = new NavWidget(this, component);
	ui.tabWidget->addTab(_demoWidget, "Semantic Mapping Demo");
	ui.tabWidget->addTab(_conceptualWidget, "Conceptual.SA Tester");
	ui.tabWidget->addTab(_categoricalWidget, "Categorical.SA Tester");
	ui.tabWidget->addTab(_defaultWidget, "Default.SA Tester");
	ui.tabWidget->addTab(_avsMainWidget, "AVS Tester");
	ui.tabWidget->addTab(_navWidget, "Naigation Tester");

	connect(_conceptualWidget, SIGNAL(newEventInfo(const QList<conceptual::ConceptualEvent>&)),
			_demoWidget, SLOT(updateEvents(const QList<conceptual::ConceptualEvent>&)));
	connect(_conceptualWidget, SIGNAL(locationChanged(int)),
			_demoWidget, SLOT(locationChanged(int)));

}


MainDialog::~MainDialog()
{

}
