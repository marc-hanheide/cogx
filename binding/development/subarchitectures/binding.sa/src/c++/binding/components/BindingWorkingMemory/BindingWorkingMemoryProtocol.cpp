#include "BindingWorkingMemoryProtocol.hpp"


using namespace std;
using namespace boost;
using namespace cast;

namespace Binding {
  
  const string BindingWorkingMemoryProtocol::SOURCE_DATA_PREFIX(BindingData::BINDING_QUERY_PREFIX_SOURCE_DATA);

  AbstractWorkingMemoryPullQuery * 
  BindingWorkingMemoryProtocol::parseQuery(const string &_query) {
    string prefix = _query.substr(0,2);

    if (prefix == SOURCE_DATA_PREFIX) {
      //cout<<"creating for SOURCE_DATA_PREFIX"<<endl;
      return extractBindingFeature<SourceData>(_query);
    }
    else {
      //cout<<"standard parse"<<endl;
      //cout<<_query<<endl;
      return SubarchitectureWorkingMemoryProtocol::parseQuery(_query);
    }
  }

  template<>   
  WorkingMemoryPullQuery<SourceData> *
  BindingWorkingMemoryProtocol::extractBindingFeature<SourceData>(const string &_query) {
    
      vector<string> tokens;
  
      tokenizeString(_query, tokens, "\\");

      if(7 != tokens.size()) {
	throw(runtime_error("SubarchitectureWorkingMemoryProtocol::extractBindingFeature<SourceData> incorrect number of tokens in query: " + _query));
      }

      SourceData * pSD = new SourceData();
      pSD->m_type = CORBA::string_dup(tokens[4].c_str());
      pSD->m_address.m_subarchitecture = CORBA::string_dup(tokens[5].c_str());
      pSD->m_address.m_id = CORBA::string_dup(tokens[6].c_str());
 
      //cout<<endl;
      //cout<<"m_id: "<<pSD->m_address.m_id<<endl;
      //cout<<endl;

      return new WorkingMemoryPullQuery<SourceData>(tokens[1], 
						    tokens[2], 
						    tokens[3], 
						    0,
						    pSD, 
						    AbstractWorkingMemoryPullQuery::OTHER);
      
  }



}
