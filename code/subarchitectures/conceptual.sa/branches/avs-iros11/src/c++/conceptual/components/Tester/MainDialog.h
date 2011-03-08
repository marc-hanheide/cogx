#ifndef MAINDIALOG_H
#define MAINDIALOG_H

#include <QtGui/QDialog>
#include "ui_MainDialog.h"

class ConceptualWidget;
class AVSMainWidget;

namespace conceptual
{
	class Tester;
}



class MainDialog : public QDialog
{
    Q_OBJECT

public:
    MainDialog(conceptual::Tester *component);
    ~MainDialog();

    ConceptualWidget *getConceptualWidget()
    {return _conceptualWidget;}


private:
    Ui::MainDialogClass ui;
    conceptual::Tester *_component;

    ConceptualWidget *_conceptualWidget;
    AVSMainWidget *_avsMainWidget;
};

#endif // MAINDIALOG_H
