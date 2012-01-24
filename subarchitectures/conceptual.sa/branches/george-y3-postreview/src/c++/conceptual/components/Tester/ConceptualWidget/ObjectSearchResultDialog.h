#ifndef OBJECTSEARCHRESULTDIALOG_H
#define OBJECTSEARCHRESULTDIALOG_H

#include <QtGui/QDialog>
#include "ui_ObjectSearchResultDialog.h"
#include "DefaultData.hpp"


namespace conceptual
{
	class Tester;
}
class ConceptualWidget;

class ObjectSearchResultDialog : public QDialog
{
    Q_OBJECT

public:
    ObjectSearchResultDialog(ConceptualWidget *parent, conceptual::Tester *component);
    ~ObjectSearchResultDialog();

private slots:
    void on_ObjectSearchResultDialogClass_accepted();

private:
    Ui::ObjectSearchResultDialogClass ui;
    conceptual::Tester *_component;
    DefaultData::StringSeq _objectCategories;

};

#endif // OBJECTSEARCHRESULTDIALOG_H
