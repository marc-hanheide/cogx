/** @file Application.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYS_APPLICATION_H_
#define _GOLEM_PHYS_APPLICATION_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/XMLParser.h>
#include <Golem/Phys/Universe.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Application skelton */
class Application {
public:
	/** Constructs Application */
	Application();
	/** Destroys Application */
	virtual ~Application();
	/** Main application method */
	virtual int main(int argc, char *argv[]);

protected:
	/** Abstract run method */
	virtual void run(int argc, char *argv[]) = 0;

	inline XMLParser* xmlparser() {
		return pParser.get();
	}
	inline const XMLParser* xmlparser() const {
		return pParser.get();
	}
	inline XMLContext* xmlcontext() {
		return pXMLContext;
	}
	inline const XMLContext* xmlcontext() const {
		return pXMLContext;
	}
	inline Context* context() {
		return pContext.get();
	}
	inline const Context* context() const {
		return pContext.get();
	}
	inline Universe* universe() {
		return pUniverse.get();
	}
	inline const Universe* universe() const {
		return pUniverse.get();
	}
	inline Scene* scene() {
		return pScene;
	}
	inline const Scene* scene() const {
		return pScene;
	}

private:
	XMLParser::Ptr pParser;
	XMLContext* pXMLContext;
	Context::Ptr pContext;
	Universe::Ptr pUniverse;
	Scene* pScene;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYS_APPLICATION_H_*/
