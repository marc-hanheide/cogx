#include "binding/ontology/BindingOntologyFactory.hpp"
#include "binding/ontology/BindingLocalOntology.hpp"
#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/utils/BindingUtils.hpp"
#include "BindingDotViewer.hpp"
#include "binding/feature-utils/Features.hpp"
#include "binding/utils/BindingUtils.hpp"

#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>


using namespace std;
using namespace boost;
using namespace boost::assign;
using namespace cast;
using namespace dot;
using namespace BindingFeatures;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::BindingDotViewer(_id);
  }
}

namespace Binding {

BindingDotViewer::BindingDotViewer(const string &_id) : 
  WorkingMemoryAttachedComponent(_id),
  AbstractBinder(_id),
  m_dotPath(""),
  m_dotCount(0),
  m_lastStatusVersion(-1)
{
  // too many events and no rest will make Jack a dull boy when he gets too many dot files
  m_queueBehaviour = cdl::DISCARD;  
}

BindingDotViewer::~BindingDotViewer() {
}

void 
BindingDotViewer::start() {

  AbstractBinder::start();

  //  addChangeFilter(BindingLocalOntology::BINDER_STATUS_TYPE,
  //cdl::OVERWRITE,
  //cdl::LOCAL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<BindingDotViewer>(this,
  //&BindingDotViewer::generateDotGraphIfStable));
  addChangeFilter(createLocalTypeFilter<BindingData::BinderStatus>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<BindingDotViewer>(this,
								     &BindingDotViewer::generateDotGraphIfStable));
  
}

void 
BindingDotViewer::configure(map<string, string>& _config)
{
  AbstractBinder::configure(_config);
  m_dotPath = _config["-d"];  
  
  if(m_dotPath == "")
    cerr << "Warning: BindingDotViewer running without doing anything, specify file path with -d flag\n";
  
  string proxy(_config["-proxy"]);
  if(proxy == "debug") {
    m_dotParameters.proxies = DotParameters::show_full;
  } else if(proxy == "source") {
    m_dotParameters.proxies = DotParameters::only_source;
  } else if(proxy == "points") {
    m_dotParameters.proxies = DotParameters::only_dots;
  } else if(proxy == "none") {
    m_dotParameters.proxies = DotParameters::no_show;
  }

  string binding_union(_config["-union"]);
  if(binding_union == "debug") {
    m_dotParameters.unions = DotParameters::show_full;
//  } else if(binding_union == "source") {
//    m_dotParameters.unions = DotParameters::only_source;
  } else if(binding_union == "short") {
    m_dotParameters.unions = DotParameters::show_short;
  } 
  
  
}

void 
BindingDotViewer::generateDotGraph(const cdl::WorkingMemoryChange &) 
{
  if(!m_dotPath.empty()) {
    stringstream filename;
    filename << m_dotPath << "/bindingWM_" << m_subarchitectureID << "_" << setw(4) << setfill('0') << m_dotCount << ".dot";
    ofstream dot_file (filename.str().c_str());
    if(!dot_file) {
      throw runtime_error("File error: " + filename.str() + "\n");
    }
    dot_file << dotGraph();
    dot_file.close();
    log("Stored: " + filename.str());
    m_dotCount++;
  }  
}

// calls generateDotGraph only if the status of the binder is that it is stable
void
BindingDotViewer::generateDotGraphIfStable(const cdl::WorkingMemoryChange& _wmc) {
  try {
    int m_version = getVersionNumber(_wmc.m_address);
    if(m_lastStatusVersion < m_version) {    
      shared_ptr<const BindingData::BinderStatus>
	status(loadBindingDataFromWM<BindingData::BinderStatus>(_wmc));
      if(status->m_stable) { // || status->m_scoringTasks == 0) {
	m_lastStatusVersion = m_version;
	generateDotGraph(_wmc);
      }
    }
  }
  catch(const DoesNotExistOnWMException&) {} // no problem  

}


string
bestUnionsToString(const BindingData::BestUnionsForProxy& _best) 
{
  stringstream str;
//  str << "\\nbest.m_proxy: " << _best.m_proxyID << "\\n";
  str << "best.score: " << Binding::scoreToString(_best.m_score) << "\\n";
  str << "best.unionIDs: {";

  for(unsigned int i = 0; i < _best.m_unionIDs.length() ; ++i) {
    str << _best.m_unionIDs[i];
    if(i < _best.m_unionIDs.length() - 1)
      str << ", ";
  }
  str << ", v(" << _best.m_proxyUpdatesWhenThisComputed <<")}";
  return str.str();
}

string
nonMatchingUnionsToString(const BindingData::NonMatchingUnions& _nonmatching) 
{
  stringstream str;
  str << "nonmatching.unionIDs: {";
  for(unsigned int i = 0; i < _nonmatching.m_nonMatchingUnionIDs.length() ; ++i) {
    str << _nonmatching.m_nonMatchingUnionIDs[i];
    if(i < _nonmatching.m_nonMatchingUnionIDs.length() - 1)
      str << ", ";
  }
  str << "}";
  return str.str();
}


string
BindingDotViewer::dotGraph() {

  const BindingFeatureOntology& ont(BindingFeatureOntology::construct());
  
  m_prox2uni.clear();

  stringstream str;
  //   typedef
  //   vector<CASTData<BindingData::BindingUnion>*> CBV;
  //   CBV unions;
  //   getWorkingMemoryEntries(BindingOntology::BINDING_PROXY_TYPE,
  //   0, &unions);
  str << dot_head("Binding WM contents " + lexical_cast<string>(m_dotCount) + " (" + m_subarchitectureID + ")");
  
  typedef vector<shared_ptr<const CASTData<BindingData::BindingUnion> > > UnionPtrs;
  UnionPtrs unions;
  getWorkingMemoryEntries<BindingData::BindingUnion>(//BindingLocalOntology::BINDING_UNION_TYPE,
			  0,
			  unions);
  
  typedef vector<shared_ptr<const CASTData<BindingData::BindingProxy> > > ProxyPtrs;
  ProxyPtrs proxies;
  getWorkingMemoryEntries<BindingData::BindingProxy>(//BindingLocalOntology::BINDING_PROXY_TYPE,
			  0,
			  proxies);


  typedef vector<shared_ptr<const CASTData<BindingData::UnionDisambiguationIssue> > > IssuePtrs;
  IssuePtrs issues;
  getWorkingMemoryEntries<BindingData::UnionDisambiguationIssue>(//BindingLocalOntology::UNION_DISAMBIGUATION_ISSUE_TYPE,
			  0,
			  issues);

  stringstream issue;
  
  for(IssuePtrs::const_iterator i = issues.begin();
      i != issues.end();
      ++i) {
    issue << ((*i)->getData());
  }
  if(issue.str() != "")
    str << dot_node("DISSAMBIGUATIONISSUES",
		    issue.str(),"box");

  /*  typedef vector<shared_ptr<const CASTData<BindingData::FeatureSetComparison> > > ComparisonPtrs;
      ComparisonPtrs comparisons;
      getWorkingMemoryEntries(BindingLocalOntology::FEATURE_SET_COMPARISON_TYPE,
      0,
      comparisons);
      for(ComparisonPtrs::const_iterator c_itr = comparisons.begin();  
      c_itr != comparisons.end();  
      ++c_itr) {
      string comparisonID((*c_itr)->getID());
      shared_ptr<const BindingData::FeatureSetComparison>
      comparison((*c_itr)->data());

      str << boxed_dot_arrow(string(comparison->m_proxyID), 
      string(comparison->m_unionID),
      "->",
      string("updated scoring: ") + lexical_cast<string>(comparison->m_updated) + " " + comparisonID);
      }
  */

  for(UnionPtrs::const_iterator b_itr = unions.begin();
      b_itr != unions.end();
      ++ b_itr) {
    string unionID((*b_itr)->getID());
    //shared_ptr<const BindingData::BindingUnion> binding_union((*b_itr)->getData());
    try {
      //shared_ptr<const BindingData::BindingUnion> binding_union(m_unionCache.getPtr(unionID));
      const LBindingUnion& binding_union(m_unionLocalCache[unionID]);

      const BindingData::ProxyPorts& inports(binding_union.inPorts());	    
      for(unsigned int i = 0 ; i < inports.m_ports.length() ; ++i) {
	string ID(inports.m_ports[i].m_proxyID);
	if(m_proxyLocalCache[ID].bound())
	  ID = string(m_proxyLocalCache[ID]->m_unionID);
	str << dot_arrow(ID,
			 unionID,
			 "->",
			 "",
			 string("headlabel=\"INPORT:") + string(inports.m_ports[i].m_label) + "\"");
      }
      
      const BindingData::ProxyPorts& outports(binding_union.rawOutPorts());	          
      for(unsigned int i = 0 ; i < outports.m_ports.length() ; ++i) {
	string ID(outports.m_ports[i].m_proxyID);
	if(m_proxyLocalCache[ID].bound())
	  ID = string(m_proxyLocalCache[ID]->m_unionID);

	str << dot_arrow(unionID,
			 ID,
			 "->",
			 "",
			 string("taillabel=\"") + string(outports.m_ports[i].m_label) + "\"");
      
      }      

      string label; 
      string feature_string;
      if(m_dotParameters.unions == DotParameters::show_full) {
	feature_string = featureSetToDotLabel(binding_union.featureSetWithRepetitions(),set<string>(),AbstractFeature::show_all);
	label = string(unionID) + " (" + to_string(binding_union->m_type) + ")\\n " + 
	  feature_string + string("\\n updates: ") + lexical_cast<string>(binding_union->m_updates);
      } else if(m_dotParameters.unions == DotParameters::show_short) {
	set<string> exclude;
	exclude.insert(ont.featureName(typeid(BindingFeatures::DebugString)));
	  //BindingFeatureOntology::DEBUG_STRING_TYPE);
	//exclude.insert(BindingOntology::THIS_PROXY_ID_TYPE);
	//exclude.insert(BindingOntology::SOURCE_ID_TYPE);
	//exclude.insert(BindingOntology::CREATION_TIME_TYPE);
	feature_string = featureSetToDotLabel(binding_union.featureSet(), exclude);
	label = feature_string;
      } else {
	cerr << "\nERROR: only show_full and show_short vbinding viewing modes implemented.\n";
	abort();
      }
      string prop;
      if(binding_union->m_type == BindingData::GROUP) {
	prop = "style=filled,fillcolor=lightgrey";
      }
      if(binding_union->m_type == BindingData::RELATION) {
	prop = "style=filled,fillcolor=antiquewhite";
      }
      
      str << dot_node(string(unionID), label, "box", prop);
    
      
      const BindingData::WorkingMemoryIDList& proxyIDs(binding_union->m_proxyIDs);
      for(unsigned int i = 0 ; i < proxyIDs.length() ; i++) {
	if(m_dotParameters.proxies != DotParameters::no_show) {
	  str << dot_arrow(string(proxyIDs[i]),unionID);
	}
	m_prox2uni[string(proxyIDs[i])] = unionID;
      }
      
    }
    catch(const DoesNotExistOnWMException&) {} // no problem  
  }
  
  // groups
  for(UnionPtrs::const_iterator b_itr = unions.begin();
      b_itr != unions.end();
      ++ b_itr) {
    string unionID((*b_itr)->getID());
    //shared_ptr<const BindingData::BindingUnion> binding_union((*b_itr)->getData());
    try {
      //shared_ptr<const BindingData::BindingUnion> binding_union(m_unionCache.getPtr(unionID));
      const LBindingUnion& binding_union(m_unionLocalCache[unionID]);
      
      set<string> groupIDs = _retrieve_singular_groupIDs(binding_union);
      if(!groupIDs.empty()) {
	for(set<string>::const_iterator i = groupIDs.begin() ; i != groupIDs.end(); ++i) {
	  str << dot_arrow(m_prox2uni[*i],unionID,"->", "", "style=dotted");
	}
      }
    }
    catch(const DoesNotExistOnWMException&) {}; // no problem  
  }
  

  
  for(ProxyPtrs::const_iterator c_itr = proxies.begin();
      c_itr != proxies.end();
      ++ c_itr) {

    string proxyID((*c_itr)->getID());
    //shared_ptr<const BindingData::BindingProxy> 
    //  proxy((*c_itr)->getData());

    try{
      
      const LBindingProxy& proxy(m_proxyLocalCache[proxyID]);    

      string hypothetical;
      if(proxy->m_hypothetical) {
	hypothetical = "\\n*** HYPOTHETICAL PROXY ***";
      }
      
      string best_string;
      if(string(proxy->m_bestUnionsForProxyID) != "") {
	shared_ptr<const BindingData::BestUnionsForProxy>
	  best(loadBindingDataFromWM<BindingData::BestUnionsForProxy>(proxy->m_bestUnionsForProxyID));
	
	best_string += bestUnionsToString(proxy.bestUnionsForProxy());
      }
      

//      shared_ptr<const BindingData::NonMatchingUnions>
//	nonmatching_ptr(loadBindingDataFromWM<BindingData::NonMatchingUnions>(proxy->m_nonMatchingUnionID));
      
      const BindingData::NonMatchingUnions& nonMatchingUnions(proxy.nonMatchingUnions());
      
      string nonmatching = nonMatchingUnionsToString(nonMatchingUnions);
      best_string += "\\n" + nonmatching;
      
      const BindingFeatureOntology& ont(BindingFeatureOntology::construct());
      set<string> exclude;
      
      // ugly and expensive:
      string groupID = _retrieve_singular_groupID(proxy);
      string style = "dashed";
      string colour;
      if(proxy->m_type == BindingData::GROUP) {
	style += ",filled";
	colour = "fillcolor=lightgrey";
      }
      if(proxy->m_type == BindingData::RELATION) {
	style += ",filled";
	colour = "fillcolor=antiquewhite";
      }
      
      for(unsigned int i = 0 ; i < proxy->m_outPorts.m_ports.length() ; ++i) {
	str << dot_arrow(proxyID,
			 string(proxy->m_outPorts.m_ports[i].m_proxyID),
			 "->",
			 "",
			 string("taillabel=\"") + string(string(proxy->m_outPorts.m_ports[i].m_ownerProxyID)) + " " + string(proxy->m_outPorts.m_ports[i].m_label) + "\"");
      }
      
      //shared_ptr<const BindingData::ProxyPorts> inports 
//	(loadBindingDataFromWM<BindingData::ProxyPorts>(proxy->m_inPortsID));
      
      const BindingData::ProxyPorts& inports(proxy.inPorts());
      
      for(unsigned int i = 0 ; i < inports.m_ports.length() ; ++i) {
	str << dot_arrow(string(inports.m_ports[i].m_proxyID),
			 proxyID,
			 "->",
			 "",
			 string("headlabel=\"INPORT:") + string(string(inports.m_ports[i].m_ownerProxyID)) + " " + string(inports.m_ports[i].m_label) + "\"");
      }
      
      switch(m_dotParameters.proxies) {
      case DotParameters::show_full: 
	str << dot_node(proxyID, 
			proxyID + " (" + to_string(proxy->m_type) + ")\\n " + 
			featureSetToDotLabel(proxy.featureSetWithRepetitions(),set<string>(),AbstractFeature::show_all) + 
			"\\nBound to: " + string(proxy->m_unionID) + "\\n" + best_string + hypothetical +
			string("\\n updates: ") + lexical_cast<string>(proxy->m_updates) + 
			", bindingCount: " + lexical_cast<string>(proxy->m_bindingCount) +
			"\\nstate: " + to_string(proxy.proxyState())
			//(proxy.bound()?"bound":"\\n---* NOT BOUND *---")
			,
			"box", 
			"style=\""+style+"\""+colour); 
	if(groupID != "") {
	  str << dot_arrow(groupID,proxyID,"->", "", "style=\"dotted,setlinewidth(8)\"");
	}
	break;
      case DotParameters::only_source:
	//cerr << "DotParameters::only_source not implemented"; abort();
	
	exclude.insert(ont.features().begin(), ont.features().end());
	exclude.erase(ont.featureName(typeid(BindingFeatures::SourceID)));//BindingFeatureOntology::SOURCE_ID_TYPE);
	str << dot_node(proxyID,
			featureSetToDotLabel(proxy.featureSet(),exclude) + hypothetical,
			"box", 
			"style=\""+style+"\""+colour); 
	if(groupID != "") {
	  str << dot_arrow(groupID,proxyID,"->", "", "style=dotted");
	}
	
	
	/*source = i->second.featureSet().find(featureName<SourceID>());
	  if(source!=i->second.featureSet().end()) {
	  str << dot_node(proxyID, toDotLabel(source->second), "parallelogram"); 
	  } else {
	  str << dot_node(proxyID, "***MISSING SOURCE ID***", "parallelogram"); 
	  }*/
	break;
      case DotParameters::only_dots:
	str << dot_node(proxyID, "", "point"); 
	if(groupID != "") {
	  str << dot_arrow(groupID,proxyID,"->", "", "style=dotted");
	}
	break;
      case DotParameters::no_show:
	// noop
	break;
      default:
	throw(runtime_error("unknown dot visualization type in BindingDotViewer::dotGraph()"));
      };
    }
    catch(const DoesNotExistOnWMException&) {}; // no problem  
  }
    
  // and now all relations:
  
  /*
  
  typedef vector<shared_ptr<const CASTData<BindingData::SimpleRelation> > > RelationPtrs;
  RelationPtrs relations;
  getWorkingMemoryEntries(BindingOntology::SIMPLE_RELATION_TYPE,
			  0,
			  relations);

  for(RelationPtrs::const_iterator i = relations.begin();
      i != relations.end();
      ++i) {
    const BindingData::SimpleRelation& relation(*((*i)->getData()));
    string from(relation.m_from);
    string to(relation.m_to);
    string label(relation.m_label); //, arrowprop("arrowhead=none");
    if(m_dotParameters.proxies != DotParameters::no_show) {
      str << boxed_dot_arrow(from, to, "->" , label, "", "ellipse");
    }
    map<string,string>::const_iterator p2u = m_prox2uni.find(from);
    if(p2u != m_prox2uni.end()) { // i.e. we know the proxy is bound, draw relation to binding instead
      from = p2u->second;
    }
    p2u = m_prox2uni.find(to);
    if(p2u != m_prox2uni.end()) { // i.e. we know the proxy is bound
      to = p2u->second;
    }
    str << boxed_dot_arrow(from, to, "->" , label);
	
  }
  
  */
  
  str << dot_foot();
  return str.str();  

  /*  
  // first instance unions with features
  for(map<string,LBindingUnion>::const_iterator i = m_unions.begin();
  i != m_unions.end();
  ++i) {
  // first ordinary features
  set<string> exclude;
  //<remove>    exclude += BindingOntology::RELATION_TYPE;
  str << dot_node(i->first, string(i->first) + "\\n " + toDotLabel(i->second.featureSet(), exclude));    
  //<remove>    // then relations
  //<remove>    FeatureSet::const_iterator relations = i->second.featureSet().find(BindingOntology::RELATION_TYPE);
  //<remove>    if(relations != i->second.featureSet().end()) {
  //<remove>      str << _dotRelations(i->first,*relations);
  //<remove>}
  }
  
  // then proxies
  if(m_dotParameters.proxies != DotParameters::no_show) {
  for(map<string,LBindingProxy>::const_iterator i = m_proxyIDs.begin();
  i != m_proxyIDs.end();
  ++i) {
  FeatureSet::const_iterator source;
  switch(m_dotParameters.proxies) {
  case DotParameters::show_full: 
  str << dot_node(i->first, string(i->first) + "\\n " + 
  toDotLabel(i->second.featureSet()) + 
  string(i->second.instanceBindingProxy().m_unionID),
  "parallelogram"); 
  break;
  case DotParameters::only_source:
  source = i->second.featureSet().find(featureName<SourceID>());
  if(source!=i->second.featureSet().end()) {
  str << dot_node(i->first, toDotLabel(source->second), "parallelogram"); 
  } else {
  str << dot_node(i->first, "***MISSING SOURCE ID***", "parallelogram"); 
  }
  break;
  case DotParameters::only_dots:
  str << dot_node(i->first, "", "point"); 
  break;
  default:
  throw(runtime_error("unknown dot visualization type in BindingDotViewer::dotGraph()"));
  }
  }
  for(map<string,set<string> >::const_iterator i = m_uni2prox.begin();
  i != m_uni2prox.end();
  ++i) {
  for(set<string>::const_iterator j = i->second.begin();
  j != i->second.end();
  ++j) {
  str << dot_arrow(i->first,*j);
  }
  }
  }
  str << _dotRelations();
  */
    
}



ostream&
operator<<(ostream& out, const map<string,set<string> >& _map) 
{
  for(map<string,set<string> >::const_iterator iter = _map.begin() ; 
      iter != _map.end() ; 
      ++iter ) {
    out << iter->first << " : ";
    print_set(out, iter->second);
    out << " || ";
  }
  return out;
}

ostream&
operator<<(ostream& out, const map<string,string>& _map) {
  for(map<string,string>::const_iterator iter = _map.begin() ; 
      iter != _map.end() ; 
      ++iter ) {
    out << iter->first << " : " << iter->second;
    out << " || ";
  }
  return out;
}

string
BindingDotViewer::_retrieve_singular_groupID(const LBindingProxy& _proxy)
{
  for(unsigned int i = 0 ; 
      i < _proxy->m_proxyFeatures.length(); 
      ++i) {
    static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
    if(string(_proxy->m_proxyFeatures[i].m_type) == ontology.featureName(typeid(Singular))) {//featureName<Singular>()) {
      shared_ptr<const BindingFeatures::Singular>
	singular(loadBindingDataFromWM<BindingFeatures::Singular>(_proxy->m_proxyFeatures[i].m_address));
      return string(singular->m_groupID);
    }
  }
  return("");
}

set<string>
BindingDotViewer::_retrieve_singular_groupIDs(const LBindingUnion& _union)
{
  set<string> ret;
  for(unsigned int i = 0 ; 
      i < _union->m_unionFeatures.length(); 
      ++i) {
    static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());

    if(string(_union->m_unionFeatures[i].m_type) == ontology.featureName(typeid(Singular))) {
      shared_ptr<const BindingFeatures::Singular>
	singular(loadBindingDataFromWM<BindingFeatures::Singular>(_union->m_unionFeatures[i].m_address));
      ret.insert(string(singular->m_groupID));
    }
  }
  return ret;
}


  void BindingDotViewer::runComponent() {
    //nah: used for testing
    //sleep(60);
    //log("\n\n\n\ngenerating final view");
    //generateDotGraph(cdl::WorkingMemoryChange());     
  }


} // namespace Binding 
