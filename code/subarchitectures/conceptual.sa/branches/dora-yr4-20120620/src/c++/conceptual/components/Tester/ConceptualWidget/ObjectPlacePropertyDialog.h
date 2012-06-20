#ifndef OBJECTPLACEPROPERTYDIALOG_H
#define OBJECTPLACEPROPERTYDIALOG_H

#include "ui_ObjectPlacePropertyDialog.h"
#include "DefaultData.hpp"

#include <QtGui/QDialog>

namespace conceptual
{
	class Tester;
}
class ConceptualWidget;

class ObjectPlacePropertyDialog : public QDialog
{
    Q_OBJECT

public:
    ObjectPlacePropertyDialog(ConceptualWidget *parent, conceptual::Tester *component);
    ~ObjectPlacePropertyDialog();

private slots:
    void on_ObjectPlacePropertyDialogClass_accepted();

private:
    Ui::ObjectPlacePropertyDialogClass ui;

    conceptual::Tester *_component;

    DefaultData::StringSeq _objectCategories;

    std::vector<int> _placesForRooms;

};

#endif // OBJECTPLACEPROPERTYDIALOG_H
