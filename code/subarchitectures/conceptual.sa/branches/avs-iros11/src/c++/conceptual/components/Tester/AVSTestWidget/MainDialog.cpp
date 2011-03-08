#include "MainDialog.h"
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
#include "AVS_ContinualPlanner.h"

using namespace std;
using namespace cast;
using namespace boost;
using namespace boost::assign;
using namespace boost::algorithm;

MainDialog::MainDialog(spatial::AVS_ContinualPlanner *component)
    : QDialog(0), m_component(component)
{
	ui.setupUi(this);
	connect(ui.generateViewCones, SIGNAL(clicked()), this, SLOT(generateViewConesButtonClicked()));
	connect(ui.processConeGroup, SIGNAL(clicked()), this, SLOT(processConeGroup()));
}

MainDialog::~MainDialog()
{

}

void MainDialog::processConeGroup(){
	m_component->log("processConeGroup");
	QString coneGroupId = ui.coneGroupId->text();
	m_component->processConeGroup(coneGroupId.toStdString());
}
void MainDialog::generateViewConesButtonClicked(){
	m_component->log("generateViewConesButtonClicked");

	SpatialData::RelationalViewPointGenerationCommandPtr newVPCommand = new SpatialData::RelationalViewPointGenerationCommand;
	QString queryString = ui.locationgenerateViewCones->text();
	std::vector<string> variables;
	//parse string:

	if(parseQuery(queryString.toStdString(),variables)){
	for (unsigned int i=0; i < variables.size(); i++){
		m_component->log(" %s ", variables[i].c_str());
	}
	if (variables.size() == 4){
	newVPCommand->roomId = lexical_cast<int>(variables[1]);
	newVPCommand->searchedObjectCategory = variables[3];
	newVPCommand->supportObject = "";
	newVPCommand->relation = SpatialData::INROOM;
	}
	else if (variables.size() == 6){
		newVPCommand->roomId = lexical_cast<int>(variables[1]);
		newVPCommand->searchedObjectCategory = variables[3];
		newVPCommand->supportObjectCategory = variables[5];
		newVPCommand->supportObject = variables[6]; // supportObjectid
		newVPCommand->relation = (variables[4] == "on" ? SpatialData::ON : SpatialData::INOBJECT);
	}
	m_component->generateViewCones(newVPCommand);
	}

}


bool MainDialog::parseQuery(string queryString, vector<string> &variables)
{
	// Check the query string
	if ( (queryString.length()<4) || (queryString[0]!='r') )
			{
				m_component->log("Malformed ChainGraphInferencer query string \'%s\'! e.g. room_<roomid>_object_cup or room_<roomid>_object_cup_on_table_<tableid>!", queryString.c_str());
				return false;
			}
	// Extract variable names
	string variableString=queryString;
	split( variables, variableString, is_any_of("_") );
	return true;

}
