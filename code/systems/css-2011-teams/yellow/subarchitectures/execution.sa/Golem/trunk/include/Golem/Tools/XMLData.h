/** @file XMLData.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_TOOLS_XMLDATA_H_
#define _GOLEM_TOOLS_XMLDATA_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/XMLParser.h>
#include <Golem/Tools/Context.h>
#include <Golem/Tools/Msg.h>
#include <sstream>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads value from a given context */
template <typename VAL> void XMLGetValue(VAL &val, const XMLContext* context) {
	ASSERT(context)

	std::istringstream iss(context->getValue(), std::istringstream::in);	
	iss >> val;
	if (iss.fail())
		throw MsgXMLParser("XMLGetValue(): Value parse error at: %s", context->getName().c_str());
}

/** Reads string from a given context */
template <> void XMLGetValue(std::string &val, const XMLContext* context);

/** Writes value to a given context */
template <typename VAL> void XMLSetValue(const VAL &val, XMLContext* context) {
	ASSERT(context)

	std::ostringstream oss(std::ostringstream::out);	
	oss << val;
	if (oss.fail())
		throw MsgXMLParser("XMLSetValue(): Value conversion error at: %s", context->getName().c_str());
	
	context->setValue(oss.str());
}

/** Writes string to a given context */
template <> void XMLSetValue(const std::string &val, XMLContext* context);

/** Reads/writes value from/to a given context
*/
template <typename VAL> void XMLData(VAL &val, XMLContext* context, bool create = false) {
	if (create)
		XMLSetValue(val, context);
	else
		XMLGetValue(val, context);
}
//------------------------------------------------------------------------------

/** Reads attribute from a given context */
template <typename VAL> void XMLGetAttribute(const std::string &attr, VAL &val, const XMLContext* context) {
	ASSERT(context)

	std::istringstream iss(context->getAttribute(attr), std::istringstream::in);	
	iss >> val;
	if (iss.fail())
		throw MsgXMLParser("XMLGetAttribute(): Attribute parse error at: %s", context->getName().c_str());
}

/** Reads string from a given context */
template <> void XMLGetAttribute(const std::string &attr, std::string &val, const XMLContext* context);

/** Writes attribute to a given context */
template <typename VAL> void XMLSetAttribute(const std::string &attr, const VAL &val, XMLContext* context) {
	ASSERT(context)

	std::ostringstream oss(std::ostringstream::out);	
	oss << val;
	if (oss.fail())
		throw MsgXMLParser("XMLSetValue(): Attribute conversion error at: %s", context->getName().c_str());
	
	context->setAttribute(attr, oss.str());
}

/** Writes string to a given context */
template <> void XMLSetAttribute(const std::string &attr, const std::string &val, XMLContext* context);

/** Reads/writes attribute from/to a given context
*/
template <typename VAL> void XMLData(const std::string &attr, VAL &val, XMLContext* context, bool create = false) {
	if (create)
		XMLSetAttribute(attr, val, context);
	else
		XMLGetAttribute(attr, val, context);
}

//------------------------------------------------------------------------------

/** Reads sequence of values from a given context
*/
template <typename PTR> void XMLGetValue(PTR begin, PTR end, const XMLContext* context, const char* name) {
	ASSERT(context)
	
	std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = context->getContextMap().equal_range(name);
	if (range.first == range.second)
		throw MsgXMLParserNameNotFound("XMLGetValue(): Name not found: %s %s", context->getName().c_str(), name);

	for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second && begin != end; i++, begin++)
		XMLData(*begin, const_cast<XMLContext*>(&i->second), false);

	if (begin != end)
		throw MsgXMLParserIncompleteData("XMLGetValue(): Incomplete data sequence: %s %s", context->getName().c_str(), name);
}
template <typename TYPE, typename PTR> void XMLGetValuePtr(PTR begin, PTR end, const XMLContext* context, const char* name) {
	ASSERT(context)
	
	std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = context->getContextMap().equal_range(name);
	if (range.first == range.second)
		throw MsgXMLParserNameNotFound("XMLGetValuePtr(): Name not found: %s %s", context->getName().c_str(), name);

	for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second && begin != end; i++, begin++) {
		TYPE* ptr = dynamic_cast<TYPE*>(&**begin);
		if (ptr == NULL)
			throw MsgXMLParserInvalidCast("XMLGetValuePtr(): unable to cast a pointer: %s %s", context->getName().c_str(), name);
		XMLData(*ptr, const_cast<XMLContext*>(&i->second), false);
	}

	if (begin != end)
		throw MsgXMLParserIncompleteData("XMLGetValuePtr(): Incomplete data sequence: %s %s", context->getName().c_str(), name);
}


/** Writes sequence of values to a given context
*/
template <typename PTR> void XMLSetValue(PTR begin, PTR end, XMLContext* context, const char* name) {
	ASSERT(context)

	while (begin != end)
		XMLData(*begin++, context->createContext(name), true);
}
template <typename TYPE, typename PTR> void XMLSetValuePtr(PTR begin, PTR end, XMLContext* context, const char* name) {
	ASSERT(context)

	for (;begin != end; begin++) {
		TYPE* ptr = dynamic_cast<TYPE*>(&**begin);
		if (ptr == NULL)
			throw MsgXMLParserInvalidCast("XMLSetValuePtr(): unable to cast a pointer: %s %s", context->getName().c_str(), name);
		XMLData(const_cast<TYPE&>(*ptr), context->createContext(name), true);
	}
}

/** Reads/writes sequence of values from/to a given context
*/
template <typename PTR> void XMLData(PTR begin, PTR end, XMLContext* context, const char* name, bool create = false) {
	if (create)
		XMLSetValue(begin, end, context, name);
	else
		XMLGetValue(begin, end, context, name);
}
template <typename TYPE, typename PTR> void XMLDataPtr(PTR begin, PTR end, XMLContext* context, const char* name, bool create = false) {
	if (create)
		XMLSetValuePtr<TYPE>(begin, end, context, name);
	else
		XMLGetValuePtr<TYPE>(begin, end, context, name);
}

//------------------------------------------------------------------------------

/** Reads sequence of values from a given context
*/
template <typename SEQ> void XMLGetValue(SEQ &seq, const XMLContext* context, const char* name) {
	ASSERT(context)
	
	std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = context->getContextMap().equal_range(name);
	if (range.first == range.second)
		throw MsgXMLParserNameNotFound("XMLGetValue(): Name not found: %s %s", context->getName().c_str(), name);

	for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second; i++) {
		typename SEQ::value_type val;
		XMLData(val, const_cast<XMLContext*>(&i->second), (bool)false);
		seq.insert(seq.end(), val);
	}
}

/** Writes sequence of values to a given context
*/
template <typename SEQ> void XMLSetValue(const SEQ &seq, XMLContext* context, const char* name) {
	ASSERT(context)
	
	for (typename SEQ::const_iterator i = seq.begin(); i != seq.end(); i++) {
		typedef typename SEQ::value_type VAL;
		XMLData(const_cast<VAL&>(*i), context->createContext(name), true);
	}
}

/** Reads/writes sequence of values from/to a given context
*/
template <typename SEQ> void XMLData(SEQ &seq, XMLContext* context, const char* name, bool create = false) {
	if (create)
		XMLSetValue(seq, context, name);
	else
		XMLGetValue(seq, context, name);
}

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_TOOLS_XMLDATA_H_*/
