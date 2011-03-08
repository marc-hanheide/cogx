#ifndef GRAPHDIALOG_H
#define GRAPHDIALOG_H

#include "ConceptualData.hpp"
#include "ui_GraphDialog.h"

#include <QtGui/QDialog>
#include <QProcess>

class SvgView;

namespace conceptual
{
	class Tester;
}


class GraphDialog : public QDialog, public Ui::GraphDialogClass
{
    Q_OBJECT

public:
    GraphDialog(QWidget *parent, conceptual::Tester *component);
    ~GraphDialog();

    void refresh(const ConceptualData::VariableInfos &vis, const ConceptualData::FactorInfos &fis);


private slots:

	void refreshButtonClicked();
	void layoutProcessFinished( int exitCode, QProcess::ExitStatus exitStatus );

private:

	void killLayoutProcess();
	void startLayouting(QByteArray dot);


private:

	conceptual::Tester *_component;

	SvgView *_svgView;

	QProcess* _layoutProcess;
};

#endif // GRAPHDIALOG_H
