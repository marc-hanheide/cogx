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
  dotPath(""),
  dotCount(0),
  lastStatusVersion(-1)
{
  // too many events and no rest will make Jack a dull boy when he gets too many dot files
  queueBehaviour = cdl::DISCARD;  
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
  dotPath = _config["-d"];  
  
  if(dotPath == "")
    cerr << "Warning: BindingDotViewer running without doing anything, specify file path with -d flag\n";
  
  string proxy(_config["-proxy"]);
  if(proxy == "debug") {
    dotParameters.proxies = DotParameters::show_full;
  } else if(proxy == "source") {
    dotParameters.proxies = DotParameters::only_source;
  } else if(proxy == "points") {
    dotParameters.proxies = DotParameters::only_dots;
  } else if(proxy == "none") {
    dotParameters.proxies = DotParameters::no_show;
  }

  string binding_union(_config["-union"]);
  if(binding_union == "debug") {
    dotParameters.unions = DotParameters::show_full;
//  } else if(binding_union == "source") {
//    dotParameters.unions = DotParameters::only_source;
  } else if(binding_union == "short") {
    dotParameters.unions = DotParameters::show_short;
  } 
  
  
}

void 
BindingDotViewer::generateDotGraph(const cdl::WorkingMemoryChange &) 
{
  if(!dotPath.empty()) {
    stringstream filename;
    filename << dotPath << "/bindingW" << subarchitectureID << "_" << setw(4) << setfill('0') << dotCount << ".dot";
    ofstream dot_file (filename.str().c_str());
    if(!dot_file) {
      throw runtime_error("File error: " + filename.str() + "\n");
    }
    dot_file << dotGraph();
    dot_file.close();
    log("Stored: " + filename.str());
    dotCount++;
  }  
}

// calls generateDotGraph only if the status of the binder is that it is stable
void
BindingDotViewer::generateDotGraphIfStable(const cdl::WorkingMemoryChange& _wmc) {
  try {
    int version = getVersionNumber(_wmc.address);
    if(lastStatusVersion < version) {    
      shared_ptr<const BindingData::BinderStatus>
	status(loadBindingDataFromWM<BindingData::BinderStatus>(_wmc));
      if(status->stable) { // || status->scoringTasks == 0) {
	lastStatusVersion = version;
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
//  str << "\\nbest.proxy: " << _best.proxyID << "\\n";
  str << "best.score: " << Binding::scoreToString(_best.score) << "\\n";
  str << "best.unionIDs: {";

  for(unsigned int i = 0; i < _best.unionIDs.length() ; ++i) {
    str << _best.unionIDs[i];
    if(i < _best.unionIDs.length() - 1)
      str << ", ";
  }
  str << ", v(" << _best.proxyUpdatesWhenThisComputed <<")}";
  return str.str();
}

string
nonMatchingUnionsToString(const BindingData::NonMatchingUnions& _nonmatching) 
{
  stringstream str;
  str << "nonmatching.unionIDs: {";
  for(unsigned int i = 0; i < _nonmatching.nonMatchingUnionIDs.length() ; ++i) {
    str << _nonmatching.nonMatchingUnionIDs[i];
    if(i < _nonmatching.nonMatchingUnionIDs.length() - 1)
      str << ", ";
  }
  str << "}";
  return str.str();
}


string
BindingDotViewer::dotGraph() {

  const BindingFeatureOntology& ont(BindingFeatureOntology::construct());
  
  prox2uni.clear();

  stringstream str;
  //   typedef
  //   vector<CASTData<BindingData::BindingUnion>*> CBV;
  //   CBV unions;
  //   getWorkingMemoryEntries(BindingOntology::BINDING_PROXY_TYPE,
  //   0, &unions);
  str << dot_head("Binding WM contents " + lexical_cast<string>(dotCount) + " (" + subarchitectureID + ")");
  
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

      str << boxed_dot_arrow(string(comparison->proxyID), 
      string(comparison->unionID),
      "->",
      string("updated scoring: ") + lexical_cast<string>(comparison->updated) + " " + comparisonID);
      }
  */

  for(UnionPtrs::const_iterator b_itr = unions.begin();
      b_itr != unions.end();
      ++ b_itr) {
    string unionID((*b_itr)->getID());
    //shared_ptr<const BindingData::BindingUnion> binding_union((*b_itr)->getData());
    try {
      //shared_ptr<const BindingData::BindingUnion> binding_union(unionCache.getPtr(unionID));
      const LBindingUnion& binding_union(unionLocalCache[unionID]);

      const BindingData::ProxyPorts& inports(binding_union.inPorts());	    
      for(unsigned int i = 0 ; i < inports.ports.length() ; ++i) {
	string ID(inports.ports[i].proxyID);
	if(proxyLocalCache[ID].bound())
	  ID = string(proxyLocalCache[ID]->unionID);
	str << dot_arrow(ID,
			 unionID,
			 "->",
			 "",
			 string("headlabel=\"INPORT:") + string(inports.ports[i].label) + "\"");
      }
      
      const BindingData::ProxyPorts& outports(binding_union.rawOutPorts());	          
      for(unsigned int i = 0 ; i < outports.ports.length() ; ++i) {
	string ID(outports.ports[i].proxyID);
	if(proxyLocalCache[ID].bound())
	  ID = string(proxyLocalCache[ID]->unionID);

	str << dot_arrow(unionID,
			 ID,
			 "->",
			 "",
			 string("taillabel=\"") + string(outports.ports[i].label) + "\"");
      
      }      

      string label; 
      string feature_string;
      if(dotParameters.unions == DotParameters::show_full) {
	feature_string = featureSetToDotLabel(binding_union.featureSetWithRepetitions(),set<string>(),AbstractFeature::show_all);
	label = string(unionID) + " (" + to_string(binding_union->type) + ")\\n " + 
	  feature_string + string("\\n updates: ") + lexical_cast<string>(binding_union->updates);
      } else if(dotParameters.unions == DotParameters::show_short) {
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
      if(binding_union->type == BindingData::GROUP) {
	prop = "style=filled,fillcolor=lightgrey";
      }
      if(binding_union->type == BindingData::RELATION) {
	prop = "style=filled,fillcolor=antiquewhite";
      }
      
      str << dot_node(string(unionID), label, "box", prop);
    
      
      const BindingData::WorkingMemoryIDList& proxyIDs(binding_union->proxyIDs);
      for(unsigned int i = 0 ; i < proxyIDs.length() ; i++) {
	if(dotParameters.proxies != DotParameters::no_show) {
	  str << dot_arrow(string(proxyIDs[i]),unionID);
	}
	prox2uni[string(proxyIDs[i])] = unionID;
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
      //shared_ptr<const BindingData::BindingUnion> binding_union(unionCache.getPtr(unionID));
      const LBindingUnion& binding_union(unionLocalCache[unionID]);
      
      set<string> groupIDs = _retrieve_singular_groupIDs(binding_union);
      if(!groupIDs.empty()) {
	for(set<string>::const_iterator i = groupIDs.begin() ; i != groupIDs.end(); ++i) {
	  str << dot_arrow(prox2uni[*i],unionID,"->", "", "style=dotted");
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
      
      const LBindingProxy& proxy(proxyLocalCache[proxyID]);    

      string hypothetical;
      if(proxy->hypothetical) {
	hypothetical = "\\n*** HYPOTHETICAL PROXY ***";
      }
      
      string best_string;
      if(string(proxy->bestUnionsForProxyID) != "") {
	shared_ptr<const BindingData::BestUnionsForProxy>
	  best(loadBindingDataFromWM<BindingData::BestUnionsForProxy>(proxy->bestUnionsForProxyID));
	
	best_string += bestUnionsToString(proxy.bestUnionsForProxy());
      }
      

//      shared_ptr<const BindingData::NonMatchingUnions>
//	nonmatching_ptr(loadBindingDataFromWM<BindingData::NonMatchingUnions>(proxy->nonMatchingUnionID));
      
      const BindingData::NonMatchingUnions& nonMatchingUnions(proxy.nonMatchingUnions());
      
      string nonmatching = nonMatchingUnionsToString(nonMatchingUnions);
      best_string += "\\n" + nonmatching;
      
      const BindingFeatureOntology& ont(BindingFeatureOntology::construct());
      set<string> exclude;
      
      // ugly and expensive:
      string groupID = _retrieve_singular_groupID(proxy);
      string style = "dashed";
      string colour;
      if(proxy->type == BindingData::GROUP) {
	style += ",filled";
	colour = "fillcolor=lightgrey";
      }
      if(proxy->type == BindingData::RELATION) {
	style += ",filled";
	colour = "fillcolor=antiquewhite";
      }
      
      for(unsigned int i = 0 ; i < proxy->outPorts.ports.length() ; ++i) {
	str << dot_arrow(proxyID,
			 string(proxy->outPorts.ports[i].proxyID),
			 "->",
			 "",
			 string("taillabel=\"") + string(string(proxy->outPorts.ports[i].ownerProxyID)) + " " + string(proxy->outPorts.ports[i].label) + "\"");
      }
      
      //shared_ptr<const BindingData::ProxyPorts> inports 
//	(loadBindingDataFromWM<BindingData::ProxyPorts>(proxy->inPortsID));
      
      const BindingData::ProxyPorts& inports(proxy.inPorts());
      
      for(unsigned int i = 0 ; i < inports.ports.length() ; ++i) {
	str << dot_arrow(string(inports.ports[i].proxyID),
			 proxyID,
			 "->",
			 "",
			 string("headlabel=\"INPORT:") + string(string(inports.ports[i].ownerProxyID)) + " " + string(inports.ports[i].label) + "\"");
      }
      
      switch(dotParameters.proxies) {
      case DotParameters::show_full: 
	str << dot_node(proxyID, 
			proxyID + " (" + to_string(proxy->type) + ")\\n " + 
			featureSetToDotLabel(proxy.featureSetWithRepetitions(),set<string>(),AbstractFeature::show_all) + 
			"\\nBound to: " + string(proxy->unionID) + "\\n" + best_string + hypothetical +
			string("\\n updates: ") + lexical_cast<string>(proxy->updates) + 
			", bindingCount: " + lexical_cast<string>(proxy->bindingCount) +
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
    string from(relation.from);
    string to(relation.to);
    string label(relation.label); //, arrowprop("arrowhead=none");
    if(dotParameters.proxies != DotParameters::no_show) {
      str << boxed_dot_arrow(from, to, "->" , label, "", "ellipse");
    }
    map<string,string>::const_iterator p2u = prox2uni.find(from);
    if(p2u != prox2uni.end()) { // i.e. we know the proxy is bound, draw relation to binding instead
      from = p2u->second;
    }
    p2u = prox2uni.find(to);
    if(p2u != prox2uni.end()) { // i.e. we know the proxy is bound
      to = p2u->second;
    }
    str << boxed_dot_arrow(from, to, "->" , label);
	
  }
  
  */
  
  str << dot_foot();
  return str.str();  

  /*  
  // first instance unions with features
  for(map<string,LBindingUnion>::const_iterator i = unions.begin();
  i != unions.end();
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
  if(dotParameters.proxies != DotParameters::no_show) {
  for(map<string,LBindingProxy>::const_iterator i = proxyIDs.begin();
  i != proxyIDs.end();
  ++i) {
  FeatureSet::const_iterator source;
  switch(dotParameters.proxies) {
  case DotParameters::show_full: 
  str << dot_node(i->first, string(i->first) + "\\n " + 
  toDotLabel(i->second.featureSet()) + 
  string(i->second.instanceBindingProxy().unionID),
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
  for(map<string,set<string> >::const_iterator i = uni2prox.begin();
  i != uni2prox.end();
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
      i < _proxy->proxyFeatures.length(); 
      ++i) {
    static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
    if(string(_proxy->proxyFeatures[i].type) == ontology.featureName(typeid(Singular))) {//featureName<Singular>()) {
      shared_ptr<const BindingFeatures::Singular>
	singular(loadBindingDataFromWM<BindingFeatures::Singular>(_proxy->proxyFeatures[i].address));
      return string(singular->groupID);
    }
  }
  return("");
}

set<string>
BindingDotViewer::_retrieve_singular_groupIDs(const LBindingUnion& _union)
{
  set<string> ret;
  for(unsigned int i = 0 ; 
      i < _union->unionFeatures.length(); 
      ++i) {
    static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());

    if(string(_union->unionFeatures[i].type) == ontology.featureName(typeid(Singular))) {
      shared_ptr<const BindingFeatures::Singular>
	singular(loadBindingDataFromWM<BindingFeatures::Singular>(_union->unionFeatures[i].address));
      ret.insert(string(singular->groupID));
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
