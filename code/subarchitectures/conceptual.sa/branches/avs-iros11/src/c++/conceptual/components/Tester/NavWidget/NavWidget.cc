#include "NavWidget.h"

NavWidget::NavWidget(QWidget *parent, conceptual::Tester *component)
    : QWidget(parent), _component(component)
{
	ui.setupUi(this);
}

NavWidget::~NavWidget()
{

}
