#include "GraphDialog.h"
#include "Tester.h"
#include "SvgView.h"

#include <boost/lexical_cast.hpp>


using namespace std;
using namespace boost;

GraphDialog::GraphDialog(QWidget *parent, conceptual::Tester *component)
    : QDialog(parent), _component(component)
{
	setupUi(this);
	processWidget->hide();

	_svgView = new SvgView(this);
	viewLayout->addWidget(_svgView);
	_layoutProcess = new QProcess( this );

	connect(refreshButton, SIGNAL(clicked()), this, SLOT(refreshButtonClicked()));
	connect(saveButton, SIGNAL(clicked()), this, SLOT(saveButtonClicked()));
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

	// Get unnique factor and variable numbers
	set<string> factors;
	map<string,int> factorNos;
	set<string> variables;
	map<string,int> variableNos;
	for( map<int,ConceptualData::FactorInfo>::const_iterator it = fis.begin(); it!=fis.end(); it++ )
		factors.insert(it->second.name);
	for( map<int,ConceptualData::VariableInfo>::const_iterator it = vis.begin(); it!=vis.end(); it++ )
		variables.insert(it->second.name);
	int i=0;
	for( set<string> ::iterator it = factors.begin(); it!=factors.end(); it++ )
	{
		i++;
		factorNos[*it]=i;
	}
	i=0;
	for( set<string>::iterator it = variables.begin(); it!=variables.end(); it++ )
	{
		i++;
		variableNos[*it]=i;
	}

	bool full = fullRadioButton->isChecked();
	bool showAll = showAllCheckBox->isChecked();

	out << "graph FactorGraph { overlap=scalexy splines=true" << endl;
	out << "node[shape=ellipse];" << endl;
	for( map<int,ConceptualData::VariableInfo>::const_iterator it = vis.begin(); it!=vis.end(); it++ )
	{
		out << "\t" << "x"<< it->first << "[label=\"" <<
				((full)?it->second.name:lexical_cast<string>(variableNos[it->second.name])) <<
						"\",fillcolor=\""<<
						getVariableColor(it->second.name)<<"\",style=filled];" << endl;
	}
	out << "node[shape=box];" << endl;
	for( map<int,ConceptualData::FactorInfo>::const_iterator it = fis.begin(); it!=fis.end(); it++ )
	{
		if ((!showAll) && (isDebugFactor(it->second.name)))
			continue;
		out << "\t" << "f" << it->first << "[label=\"" <<
				((full)?it->second.name:lexical_cast<string>(factorNos[it->second.name])) <<
						"\",fillcolor=\""<<
						getFactorColor(it->second.name) <<"\",style=filled];" << endl;
	}
	for( map<int,ConceptualData::FactorInfo>::const_iterator it = fis.begin(); it!=fis.end(); it++ )
	{
		if ((!showAll) && (isDebugFactor(it->second.name)))
			continue;

		for( size_t I = 0; I < it->second.variables.size(); I++ )
		{
			out << "\t" << "f" << it->first << " -- " << "x" << it->second.variables[I] << ";" << endl;
		}
	}
	out << "}" << endl;

    // Run layouting
    startLayouting(QByteArray(out.str().c_str(), out.str().size()));
}


void GraphDialog::saveButtonClicked()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
	                            "", tr("SVG Images (*.svg)"));
	if (!fileName.isEmpty())
	{
		QFile file(fileName);
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		         return;
		file.write(_svgData);
		file.close();
	}
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
    	QByteArray result = _layoutProcess->readAll();
        _svgView->load( result );
        _svgData = result;
    }
}


std::string GraphDialog::getFactorColor(std::string factorName)
{
	if (factorName == "f(room_category1,room_category2)")
		return "#F08080"; // LightCoral
	else if (factorName == "f(room_category1,shape_property)")
		return "#98FB98"; // PaleGreen
	else if (factorName == "f(room_category1,size_property)")
		return "#FF69B4"; // HotPink
	else if (factorName == "f(room_category1,appearance_property)")
		return "#ADD8E6"; // LightBlue
	else if (wildcmp("f(room*_category,room*_object_*_unexplored)", factorName.c_str()))
		return "#D3D3D3"; // LightGrey
	else if (factorName == "ObservedObjectPropertyFactor")
		return "#FFA500"; // Orange
	else if (factorName == "ObservedShapePropertyFactor")
		return "#98FB98"; // PaleGreen
	else if (factorName == "ObservedSizePropertyFactor")
		return "#FF69B4"; // HotPink
	else if (factorName == "ObservedAppearancePropertyFactor")
		return "#ADD8E6"; // LightBlue

	return "white";
}


std::string GraphDialog::getVariableColor(std::string factorName)
{
	if (wildcmp("room*_category", factorName.c_str()))
		return "#F08080"; // LightCoral
	else if (wildcmp("place*_shape_property", factorName.c_str()))
		return "#98FB98"; // PaleGreen
	else if (wildcmp("place*_size_property", factorName.c_str()))
		return "#FF69B4"; // HotPink
	else if (wildcmp("place*_appearance_property", factorName.c_str()))
		return "#ADD8E6"; // LightBlue
	else if (wildcmp("room*_object_*_unexplored", factorName.c_str()))
		return "#D3D3D3"; // LightGrey

	return "white";
}


// -------------------------------------------------------
bool GraphDialog::isDebugFactor(std::string factorName)
{
	if (factorName=="SingleRoomFactor")
		return true;

	return false;
}


// -------------------------------------------------------
int GraphDialog::wildcmp(const char *wild, const char *string)
{
  // Written by Jack Handy - jakkhandy@hotmail.com

  const char *cp = NULL, *mp = NULL;

  while ((*string) && (*wild != '*')) {
    if ((*wild != *string) && (*wild != '?')) {
      return 0;
    }
    wild++;
    string++;
  }

  while (*string) {
    if (*wild == '*') {
      if (!*++wild) {
        return 1;
      }
      mp = wild;
      cp = string+1;
    } else if ((*wild == *string) || (*wild == '?')) {
      wild++;
      string++;
    } else {
      wild = mp;
      string = cp++;
    }
  }

  while (*wild == '*') {
    wild++;
  }
  return !*wild;
}
