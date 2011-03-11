#include "AddGroundtruthDialog.h"

AddGroundtruthDialog::AddGroundtruthDialog(QWidget *parent, QList<int> roomIds, QStringList categories, int curRoomId)
    : QDialog(parent)
{
	ui.setupUi(this);
	int index=0;
	for (int i=0; i<roomIds.size(); ++i)
	{
		ui.roomComboBox->addItem(QString::number(roomIds[i]));
		if (roomIds[i] == curRoomId)
			index = i;
	}
	ui.categoryComboBox->addItems(categories);
	ui.roomComboBox->setCurrentIndex(index);

	connect(this, SIGNAL(accepted()), this, SLOT(dialogAccepted()));
}

AddGroundtruthDialog::~AddGroundtruthDialog()
{

}


void AddGroundtruthDialog::dialogAccepted()
{

}


QString AddGroundtruthDialog::getCategory()
{
	return ui.categoryComboBox->currentText();
}

int AddGroundtruthDialog::getCategoryIndex()
{
	return ui.categoryComboBox->currentIndex();
}


int AddGroundtruthDialog::getRoomId()
{
	return ui.roomComboBox->currentText().toInt();
}



