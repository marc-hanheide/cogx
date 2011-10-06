#include "AVSMainWidget.h"
#include <cast/core/CASTUtils.hpp>
#include <VisionData.hpp>
#include <Pose3.h>
#include <VisionUtils.h>

#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>


using namespace std;
using namespace cast;
using namespace boost;
using namespace boost::assign;
using namespace boost::algorithm;

AVSMainWidget::AVSMainWidget(QWidget *parent, conceptual::Tester *component)
    : QDialog(parent), m_component(component)
{
	ui.setupUi(this);
	connect(ui.generateViewCones, SIGNAL(clicked()), this, SLOT(generateViewConesButtonClicked()));
	connect(ui.processConeGroup, SIGNAL(clicked()), this, SLOT(processConeGroup()));
	connect(ui.postVisualObject, SIGNAL(clicked()), this, SLOT(postVisualObjectClicked()));
	ui.lineEdit_3->setText("table");
	ui.locationgenerateViewCones->setText("room_0_object_table");
}

AVSMainWidget::~AVSMainWidget()
{

}

void 
AVSMainWidget::processConeGroup()
{
	m_component->log("processConeGroup");
	QString coneGroupId = ui.coneGroupId->text();
	SpatialData::ProcessConeGroupPtr cmd = new SpatialData::ProcessConeGroup;
	m_component->addToWorkingMemory(m_component->newDataID(), "spatial.sa", cmd); // TODO
}

void AVSMainWidget::postVisualObjectClicked(){
	m_component->log("postVisualObjectClicked");
	QString objectlabel = ui.lineEdit_3->text();
	VisionData::VisualObjectPtr obj =  cogx::createVisualObject();
  // create a very simple distribution: label and unknown
  obj->identLabels.push_back(objectlabel.toStdString());
  obj->identLabels.push_back("unknown");
  // note: distribution must of course sum to 1
  obj->identDistrib.push_back(1.);
  obj->identDistrib.push_back(0.);
  obj->detectionConfidence = 1.0;
  // the information gain if we know the label, just set to 1, cause we don't
  // have any alternative thing to do
  obj->identGain = 1.;
  // ambiguity in the distribution: we use the distribution's entropy
  obj->identAmbiguity = 0.;
//  for(size_t i = 0; i < obj->identDistrib.size(); i++)
//    if(fpclassify(obj->identDistrib[i]) != FP_ZERO)
//      obj->identAmbiguity -= obj->identDistrib[i]*::log(obj->identDistrib[i]);


  obj->pose.pos.x = 0.1;
  obj->pose.pos.y = 0;
  obj->pose.pos.z = 0;
  setIdentity(obj->pose.rot);

  obj->componentID = m_component->getComponentID();

	m_component->addToWorkingMemory(m_component->newDataID(), "vision.sa", obj); // TODO
}
void AVSMainWidget::generateViewConesButtonClicked(){
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
		m_component->log("Posting fake VPCommand without supportObject");
	newVPCommand->roomId = lexical_cast<int>(variables[1]);
	newVPCommand->searchedObjectCategory = variables[3];// TODO
	newVPCommand->supportObject = "";
	newVPCommand->relation = SpatialData::INROOM;
	}
	else if (variables.size() == 7){
		m_component->log("Posting fake VPCommand with supportObject");
		newVPCommand->roomId = lexical_cast<int>(variables[1]);
		newVPCommand->searchedObjectCategory = variables[3];// TODO
		newVPCommand->supportObjectCategory = variables[5];// TODO
		newVPCommand->supportObject = variables[6]; // supportObjectid
		newVPCommand->relation = (variables[4] == "on" ? SpatialData::ON : SpatialData::INOBJECT);
	}
	else{
		m_component->log("Not a correct query");
		return;
	}
	 SpatialData::AVSInterfacePrx agg2(m_component->getIceServer<SpatialData::AVSInterface> ("avs.cpplanner"));
	 agg2->simulateViewCones(newVPCommand);


	}

}


bool AVSMainWidget::parseQuery(string queryString, vector<string> &variables)
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
