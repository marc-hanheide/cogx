/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include "SubarchitectureWorkingMemoryProtocol.hpp"
#include <sstream>
#include <cstdlib>

using namespace std;

namespace cast {

  string SubarchitectureWorkingMemoryProtocol::ID_PREFIX = cdl::ID_QUERY_PREFIX;
  string SubarchitectureWorkingMemoryProtocol::EXISTS_PREFIX = cdl::EXISTS_QUERY_PREFIX;
  string SubarchitectureWorkingMemoryProtocol::OVERWRITE_COUNT_PREFIX = cdl::OVERWRITE_COUNT_QUERY_PREFIX;
  string SubarchitectureWorkingMemoryProtocol::ID_ARRAY_PREFIX = cdl::ID_ARRAY_QUERY_PREFIX;
  string SubarchitectureWorkingMemoryProtocol::TYPE_PREFIX = cdl::TYPE_QUERY_PREFIX;


  string SubarchitectureWorkingMemoryProtocol::createIDQuery(const string &_srcID, 
							     const string &_srcSA, 
							     const string &_subarch, 
							     const string &_id) {
    return ID_PREFIX + "\\" + _srcID + "\\" + _srcSA + "\\" + _subarch + "\\" + _id;
  }

  string SubarchitectureWorkingMemoryProtocol::createExistsQuery(const string &_srcID, 
								 const string &_srcSA, 
								 const string &_subarch, 
								 const string &_id) {
    return EXISTS_PREFIX + "\\" + _srcID + "\\" + _srcSA + "\\" + _subarch + "\\" + _id;
  }

  string SubarchitectureWorkingMemoryProtocol::createOverwriteCountQuery(const string &_srcID, 
									 const string &_srcSA, 
									 const string &_subarch, 
									 const string &_id) {
    return OVERWRITE_COUNT_PREFIX + "\\" + _srcID + "\\" + _srcSA + "\\" + _subarch + "\\" + _id;
  }


  string SubarchitectureWorkingMemoryProtocol::createTypeQuery(const string &_srcID, 
							       const string &_srcSA, 
							       const string &_subarch, 
							       const string &_type, 
							       const int &_count) {
    std::ostringstream o;
    o << TYPE_PREFIX;
    o << "\\";
    o << _srcID;
    o << "\\";
    o << _srcSA;  
    o << "\\";
    o << _subarch;
    o << "\\";
    o << _count;
    o << "\\";
    o << _type;
    return o.str();

  }

  string SubarchitectureWorkingMemoryProtocol::createIDArrayQuery(const string &_srcID, 
								  const string &_srcSA, 
								  const string &_subarch, 
								  const vector<string> &_ids) {

    string s = ID_ARRAY_PREFIX + "\\" + _srcID + "\\" + _srcSA + "\\" + _subarch + "\\";
    for (unsigned int i = 0; i < _ids.size(); i++) {
      s += _ids[i] + "\\";
    }
    return s;
  
  }

  AbstractWorkingMemoryPullQuery * 
  SubarchitectureWorkingMemoryProtocol::parseQuery(const string &_query) {

    string prefix = _query.substr(0,2);

    //cout<<"prefix: "<<prefix<<endl;

    if (prefix == ID_PREFIX) {
      return extractID(_query);
    }
    else if (prefix == EXISTS_PREFIX) {
      return extractExists(_query);
    }
    else if (prefix == OVERWRITE_COUNT_PREFIX) {
      return extractOverwriteCount(_query);
    }
    else if (prefix == ID_ARRAY_PREFIX) {
      return extractIDArray(_query);
    }
    else if (prefix == TYPE_PREFIX) {
      return extractType(_query);
    }
    else {
      return extractUnknown(_query);
    }
  
  }

  AbstractWorkingMemoryPullQuery * 
  SubarchitectureWorkingMemoryProtocol::extractUnknown(const string &_query) {

    vector<string> tokens;
  
    tokenizeString(_query, tokens, "\\");

    if(tokens.size() < 4) {
      throw(runtime_error("Unknown memory protocol for query: " + _query));
    }

    return new WorkingMemoryPullQuery<string>(tokens[1], tokens[2], tokens[3], 
					      0,
					      new string(_query), 
					      AbstractWorkingMemoryPullQuery::OTHER);
  }

  WorkingMemoryPullQuery<string> * 
  SubarchitectureWorkingMemoryProtocol::extractID(const string &_query) {

    vector<string> tokens;
  
    tokenizeString(_query, tokens, "\\");

    if(5 != tokens.size()) {
      cerr<<"SubarchitectureWorkingMemoryProtocol::extractID incorrect number of tokens in query"<<endl;
      return NULL;
    }

    return new WorkingMemoryPullQuery<string>(tokens[1], tokens[2], tokens[3], 
					      0,
					      new string(tokens[4]), 
					      AbstractWorkingMemoryPullQuery::ID);
  }

  WorkingMemoryPullQuery<string> * 
  SubarchitectureWorkingMemoryProtocol::extractExists(const string &_query) {

    vector<string> tokens;
  
    tokenizeString(_query, tokens, "\\");

    if(5 != tokens.size()) {
      cerr<<"SubarchitectureWorkingMemoryProtocol::extractExists incorrect number of tokens in query"<<endl;
      return NULL;
    }

    return new WorkingMemoryPullQuery<string>(tokens[1], tokens[2], tokens[3], 
					      0,
					      new string(tokens[4]), 
					      AbstractWorkingMemoryPullQuery::EXISTS);
  }

  WorkingMemoryPullQuery<string> * 
  SubarchitectureWorkingMemoryProtocol::extractOverwriteCount(const string &_query) {

    vector<string> tokens;
  
    tokenizeString(_query, tokens, "\\");

    if(5 != tokens.size()) {
      cerr<<"SubarchitectureWorkingMemoryProtocol::extractOverwriteCount incorrect number of tokens in query"<<endl;
      return NULL;
    }

    return new WorkingMemoryPullQuery<string>(tokens[1], tokens[2], tokens[3], 
					      0,
					      new string(tokens[4]), 
					      AbstractWorkingMemoryPullQuery::OVERWRITE_COUNT);
  }

  WorkingMemoryPullQuery< vector<string> > * 
  SubarchitectureWorkingMemoryProtocol::extractIDArray(const string &_query) {

    vector<string> tokens;

    tokenizeString(_query, tokens, "\\");
  
    if(tokens.size() < 5) {
      cerr<<"SubarchitectureWorkingMemoryProtocol::extractIDArray incorrect number of tokens in query"<<endl;
      return NULL;
    }
  
    vector<string> *pIDs = new vector<string>();
    for(unsigned int i = 4; i < tokens.size(); i++) {
      pIDs->push_back(tokens[i]);
    }
  
    return new WorkingMemoryPullQuery< vector<string> >(tokens[1], tokens[2], tokens[3], 
							0,
							pIDs, 
							AbstractWorkingMemoryPullQuery::ID_ARRAY);
  }

  WorkingMemoryPullQuery<string> * 
  SubarchitectureWorkingMemoryProtocol::extractType(const string &_query) {
    vector<string> tokens;
  
    tokenizeString(_query, tokens, "\\");
  
    if(6 != tokens.size()) {
      cerr<<"SubarchitectureWorkingMemoryProtocol::extractType incorrect number of tokens in query"<<endl;
      return NULL;
    }

    return new WorkingMemoryPullQuery<string>(tokens[1], tokens[2], tokens[3], 
					      atoi(tokens[4].c_str()),
					      new string(tokens[5]), 
					      AbstractWorkingMemoryPullQuery::TYPE);
  }


  /**
   * Stolen from http://www.linuxselfhelp.com/HOWTO/C++Programming-HOWTO-7.html
   */
  void 
  SubarchitectureWorkingMemoryProtocol::tokenizeString(const string & _str,
						       vector < string > &_tokens,
						       const string & _delimiters) {
    // Skip delimiters at beginning.
    string::size_type lastPos = _str.find_first_not_of(_delimiters, 0);
    // Find first "non-delimiter".
    string::size_type pos = _str.find_first_of(_delimiters, lastPos);
  
    while(string::npos != pos || string::npos != lastPos) {
      // Found a token, add it to the vector.
      _tokens.push_back(_str.substr(lastPos, pos - lastPos));
      // Skip delimiters.  Note the "not_of"
      lastPos = _str.find_first_not_of(_delimiters, pos);
      // Find next "non-delimiter"
      pos = _str.find_first_of(_delimiters, lastPos);
    }
  }


  string SubarchitectureWorkingMemoryProtocol::createQuery(AbstractWorkingMemoryPullQuery * _pQuery) {

    if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::ID) {
   
      WorkingMemoryPullQuery<string> * pQuery = 
	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);
      if(pQuery) {
	return createIDQuery(pQuery->getSourceID(),pQuery->getSourceSA(),pQuery->getSubarchitectureID(),*pQuery->getQueryObject());
      }
      else {
	cerr<<"ERROR: casting query to ID"<<endl;
	return "";
      }

    }
    else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::EXISTS) {
   
      WorkingMemoryPullQuery<string> * pQuery = 
	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);
      if(pQuery) {
	return createExistsQuery(pQuery->getSourceID(),pQuery->getSourceSA(),pQuery->getSubarchitectureID(),*pQuery->getQueryObject());
      }
      else {
	cerr<<"ERROR: casting query to EXISTS"<<endl;
	return "";
      }

    }
    else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::OVERWRITE_COUNT) {
   
      WorkingMemoryPullQuery<string> * pQuery = 
	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);
      if(pQuery) {
	return createOverwriteCountQuery(pQuery->getSourceID(),pQuery->getSourceSA(),pQuery->getSubarchitectureID(),*pQuery->getQueryObject());
      }
      else {
	cerr<<"ERROR: casting query to OVERWRITE_COUNT"<<endl;
	return "";
      }

    }
    else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::ID_ARRAY) {
      //cerr<<"ID Array query"<<endl;
      WorkingMemoryPullQuery< vector<string> > * pQuery = 
	dynamic_cast<WorkingMemoryPullQuery< vector<string> > *>(_pQuery);
      if(pQuery) {
	return createIDArrayQuery(pQuery->getSourceID(),pQuery->getSourceSA(),pQuery->getSubarchitectureID(),*pQuery->getQueryObject());
      }
      else {
	cerr<<"ERROR: casting query to ID array"<<endl;
	return "";
      }

    }
    else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::TYPE) {
      //cerr<<"Type query"<<endl;
      WorkingMemoryPullQuery<string> * pQuery = 
	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);
      if(pQuery) {
	return createTypeQuery(pQuery->getSourceID(),pQuery->getSourceSA(),pQuery->getSubarchitectureID(),*pQuery->getQueryObject(), pQuery->getCount());
      }
      else {
	cerr<<"ERROR: casting query to type"<<endl;
	return "";
      }

    }
    else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::OTHER) {
      //cerr<<"Other query"<<endl;
      WorkingMemoryPullQuery<string> * pQuery = 
	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);
      if(pQuery) {
	return string(*pQuery->getQueryObject());
      }
      else {
	cerr<<"ERROR: casting query to other"<<endl;
	return "";
      }

    }
    else {
      cerr<<"Unknown query type"<<endl;
      return "";
    }

  }

} //namespace cast
