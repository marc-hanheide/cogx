#include "DefaultWidget.h"
#include "Tester.h"


DefaultWidget::DefaultWidget(QWidget *parent, conceptual::Tester *component)
    : QWidget(parent)
{
	ui.setupUi(this);
}

DefaultWidget::~DefaultWidget()
{

}
