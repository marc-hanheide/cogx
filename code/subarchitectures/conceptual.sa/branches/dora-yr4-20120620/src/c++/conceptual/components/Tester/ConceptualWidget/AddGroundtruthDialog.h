#ifndef ADDGROUNDTRUTHDIALOG_H
#define ADDGROUNDTRUTHDIALOG_H

#include <QtGui/QDialog>
#include "ui_AddGroundtruthDialog.h"

class AddGroundtruthDialog : public QDialog
{
    Q_OBJECT

public:
    AddGroundtruthDialog(QWidget *parent, QList<int> roomIds, QStringList categories, int curRoomId);
    ~AddGroundtruthDialog();

    QString getCategory();
    int getCategoryIndex();
     int getRoomId();

private slots:

	void dialogAccepted();


private:
    Ui::AddGroundtruthDialogClass ui;
};

#endif // ADDGROUNDTRUTHDIALOG_H
