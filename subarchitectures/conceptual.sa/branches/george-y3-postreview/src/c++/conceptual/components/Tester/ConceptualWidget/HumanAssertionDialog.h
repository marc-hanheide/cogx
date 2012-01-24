#ifndef HUMANASSERTIONDIALOG_H
#define HUMANASSERTIONDIALOG_H

#include "ui_HumanAssertionDialog.h"
#include "DefaultData.hpp"

#include <QtGui/QDialog>

namespace conceptual
{
	class Tester;
}
class ConceptualWidget;


class HumanAssertionDialog : public QDialog
{
    Q_OBJECT

public:
    HumanAssertionDialog(ConceptualWidget *parent, conceptual::Tester *component);
    ~HumanAssertionDialog();

private slots:
    void dialogAccepted();

private:
    Ui::HumanAssertionDialogClass ui;

    conceptual::Tester *_component;

    DefaultData::StringSeq _humanAssertions;

    std::vector<int> _placesForRooms;

};

#endif // HUMANASSERTIONDIALOG_H
