#ifndef MAINDIALOG_H
#define MAINDIALOG_H

#include <QtGui/QDialog>
#include "ui_MainDialog.h"

class ConceptualWidget;
class AVSMainWidget;
class DefaultWidget;
class NavWidget;
class DemoWidget;
class CategoricalWidget;

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

    NavWidget *getNavWidget()
    {return _navWidget;}

    CategoricalWidget *getCategoricalWidget()
    {return _categoricalWidget;}

    DemoWidget *getDemoWidget()
    {return _demoWidget;}


private:
    Ui::MainDialogClass ui;
    conceptual::Tester *_component;

    ConceptualWidget *_conceptualWidget;
    AVSMainWidget *_avsMainWidget;
    DefaultWidget *_defaultWidget;
    NavWidget *_navWidget;
    DemoWidget *_demoWidget;
    CategoricalWidget *_categoricalWidget;
};

#endif // MAINDIALOG_H
