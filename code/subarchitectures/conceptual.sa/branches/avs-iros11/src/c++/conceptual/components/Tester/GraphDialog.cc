#include "GraphDialog.h"
#include "Tester.h"
#include "SvgView.h"


using namespace std;

GraphDialog::GraphDialog(QWidget *parent, conceptual::Tester *component)
    : QDialog(parent), _component(component)
{
	setupUi(this);
	processWidget->hide();

	_svgView = new SvgView(this);
	viewLayout->addWidget(_svgView);
	_layoutProcess = new QProcess( this );

	connect(refreshButton, SIGNAL(clicked()), this, SLOT(refreshButtonClicked()));
    connect( _layoutProcess, SIGNAL( finished( int, QProcess::ExitStatus ) ),
             this, SLOT( layoutProcessFinished( int, QProcess::ExitStatus ) ) );
    connect( killButton, SIGNAL( clicked() ), this, SLOT( killLayoutProcess() ) );
}


GraphDialog::~GraphDialog()
{

}


void GraphDialog::refreshButtonClicked()
{
	ConceptualData::VariableInfos vis = _component->getChainGraphVariables();
	ConceptualData::FactorInfos fis = _component->getChainGraphFactors();
	refresh(vis, fis);
}


void GraphDialog::refresh(const ConceptualData::VariableInfos &vis, const ConceptualData::FactorInfos &fis)
{
	// Create the DOT
	stringstream out;

    out << "graph FactorGraph { overlap=scalexy splines=true" << endl;
    out << "node[shape=ellipse];" << endl;
    for( map<int,ConceptualData::VariableInfo>::const_iterator it = vis.begin(); it!=vis.end(); it++ )
    {
    	out << "\t" << "x"<< it->first << "[label=\"" << it->second.name << "\",fillcolor=yellow,style=filled];" << endl;
    }
    out << "node[shape=box];" << endl;
    for( map<int,ConceptualData::FactorInfo>::const_iterator it = fis.begin(); it!=fis.end(); it++ )
    	out << "\t" << "f" << it->first << "[label=\"" << it->second.name << "\"];" << endl;
    for( map<int,ConceptualData::FactorInfo>::const_iterator it = fis.begin(); it!=fis.end(); it++ )
    {
        for( size_t I = 0; I < it->second.variables.size(); I++ )
        {
            out << "\t" << "f" << it->first << " -- " << "x" << it->second.variables[I] << ";" << endl;
        }
    }
    out << "}" << endl;

    // Run layouting
    startLayouting(QByteArray::fromRawData(out.str().c_str(), out.str().size()));
}



void GraphDialog::startLayouting(QByteArray dot)
{
	killLayoutProcess();

	QStringList procargs;
	procargs.push_back( "-Tsvg" );

	_layoutProcess->start( "neato", procargs );
	processWidget->show();
	if (!_layoutProcess->waitForStarted())
		 return;

	_layoutProcess->write(dot);
	_layoutProcess->closeWriteChannel();
}


void GraphDialog::killLayoutProcess()
{
    if( _layoutProcess->state() != QProcess::NotRunning )
    {
        _layoutProcess->kill();
        _layoutProcess->waitForFinished();
    }
}

void GraphDialog::layoutProcessFinished( int exitCode, QProcess::ExitStatus exitStatus )
{
    processWidget->hide();

    if( exitStatus == QProcess::NormalExit && exitCode == 0 )
    {
        _svgView->load( _layoutProcess->readAll() );
    }
}
