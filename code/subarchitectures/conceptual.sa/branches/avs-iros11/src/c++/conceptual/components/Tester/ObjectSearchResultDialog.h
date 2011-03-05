#ifndef OBJECTSEARCHRESULTDIALOG_H
#define OBJECTSEARCHRESULTDIALOG_H

#include <QtGui/QDialog>
#include "ui_ObjectSearchResultDialog.h"

class ObjectSearchResultDialog : public QDialog
{
    Q_OBJECT

public:
    ObjectSearchResultDialog(QWidget *parent = 0);
    ~ObjectSearchResultDialog();

private:
    Ui::ObjectSearchResultDialogClass ui;
};

#endif // OBJECTSEARCHRESULTDIALOG_H
