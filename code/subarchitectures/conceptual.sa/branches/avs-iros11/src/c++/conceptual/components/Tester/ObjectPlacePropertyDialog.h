#ifndef OBJECTPLACEPROPERTYDIALOG_H
#define OBJECTPLACEPROPERTYDIALOG_H

#include <QtGui/QDialog>
#include "ui_ObjectPlacePropertyDialog.h"

class ObjectPlacePropertyDialog : public QDialog
{
    Q_OBJECT

public:
    ObjectPlacePropertyDialog(QWidget *parent = 0);
    ~ObjectPlacePropertyDialog();

private:
    Ui::ObjectPlacePropertyDialogClass ui;
};

#endif // OBJECTPLACEPROPERTYDIALOG_H
