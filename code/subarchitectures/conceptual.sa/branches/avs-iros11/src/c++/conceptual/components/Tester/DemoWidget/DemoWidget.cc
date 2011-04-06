#include "DemoWidget.h"
#include "Tester.h"


DemoWidget::DemoWidget(QWidget *parent, conceptual::Tester *component)
    : QWidget(parent), _component(component)
{
	ui.setupUi(this);
}

DemoWidget::~DemoWidget()
{

}
