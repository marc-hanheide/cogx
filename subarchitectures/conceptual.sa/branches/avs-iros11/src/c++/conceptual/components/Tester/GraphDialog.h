#ifndef GRAPHDIALOG_H
#define GRAPHDIALOG_H

#include <QtGui/QDialog>
#include "ui_GraphDialog.h"

class GraphDialog : public QDialog
{
    Q_OBJECT

public:
    GraphDialog(QWidget *parent = 0);
    ~GraphDialog();

private:
    Ui::GraphDialogClass ui;
};

#endif // GRAPHDIALOG_H
