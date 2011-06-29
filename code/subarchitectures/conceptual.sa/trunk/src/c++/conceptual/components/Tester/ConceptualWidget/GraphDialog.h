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
	void saveButtonClicked();
	void layoutProcessFinished( int exitCode, QProcess::ExitStatus exitStatus );


private:

	void killLayoutProcess();
	void startLayouting(QByteArray dot);
	std::string getFactorColor(std::string factorName);
	std::string getVariableColor(std::string factorName);
	int wildcmp(const char *wild, const char *string);
	bool isDebugFactor(std::string factorName);


private:

	conceptual::Tester *_component;

	SvgView *_svgView;

	QProcess* _layoutProcess;

	QByteArray _svgData;
};

#endif // GRAPHDIALOG_H
