#include "NavWidget.h"
#include "Tester.h"
#include <Transformation/Pose3D.hh>


NavWidget::NavWidget(QWidget *parent, conceptual::Tester *component)
    : QWidget(parent), _component(component)
{
	ui.setupUi(this);
}

NavWidget::~NavWidget()
{

}


void NavWidget::gotoxyButtonClicked()
{
	Cure::Pose3D pose;
	pose.setX(ui.xSpinBox->value());
	pose.setY(ui.ySpinBox->value());
	pose.setTheta(ui.thetaSpinBox->value());

	_component->postNavCommand(pose, SpatialData::GOTOPOSITION);
}


void NavWidget::newNavCommand(QString str)
{
	ui.navCommandsListWidget->addItem(str);
}


void NavWidget::newNavCommand(SpatialData::NavCommandPtr navCommandPtr)
{
	QString str;
	switch(navCommandPtr->cmd)
	{
	case SpatialData::GOTOPLACE:
		str+="GOTOPLACE";
		break;
	case SpatialData::GOTOPOSITION:
		str+="GOTOPOSITION";
		break;
	case SpatialData::GOFORWARD:
		str+="GOFORWARD";
		break;
	case SpatialData::GOBACK:
		str+="GOBACK";
		break;
	case SpatialData::TURN:
		str+="TURN";
		break;
	case SpatialData::TURNTO:
		str+="TURNTO";
		break;
	case SpatialData::STOP:
		str+="STOP";
		break;
	case SpatialData::BLOCKCONTROL:
		str+="BLOCKCONTROL";
		break;
	default:
		break;
	}

	str+=" Pose:";
	str+=QString::number(navCommandPtr->pose[0]);
	str+=QString::number(navCommandPtr->pose[1]);
	str+=" Status:";

	switch(navCommandPtr->status)
	{
		case SpatialData::COMMANDPENDING:
			str+="COMMANDPENDING";
			break;
		case SpatialData::COMMANDINPROGRESS:
			str+="COMMANDINPROGRESS";
			break;
		case SpatialData::COMMANDABORTED:
			str+="COMMANDABORTED";
			break;
		case SpatialData::COMMANDFAILED:
			str+="COMMANDFAILED";
			break;
		case SpatialData::COMMANDSUCCEEDED:
			str+="COMMANDSUCCEEDED";
			break;
		case SpatialData::UNKNOWN:
			str+="UNKNOWN";
			break;
	}

	QMetaObject::invokeMethod(this, "newNavCommand", Qt::QueuedConnection,
	                           Q_ARG(QString, str));

}
