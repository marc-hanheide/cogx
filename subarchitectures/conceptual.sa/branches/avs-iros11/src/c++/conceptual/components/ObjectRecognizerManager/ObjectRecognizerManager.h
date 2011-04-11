/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::ObjectRecognizerManager class.
 */

#ifndef CONCEPTUAL_OBJECTRECOGNIZERMANAGER_H
#define CONCEPTUAL_OBJECTRECOGNIZERMANAGER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>

namespace conceptual
{

/**
 * @author Andrzej Pronobis
 */
class ObjectRecognizerManager: public cast::ManagedComponent
{

public:

	/** Constructor. */
	ObjectRecognizerManager();

	/** Destructor. */
	virtual ~ObjectRecognizerManager();


protected:

	/** Called by the framework to configure the component. */
	virtual void configure(const std::map<std::string,std::string> & _config);

	/** Called by the framework after configuration, before run loop. */
	virtual void start();

	/** The main run loop. */
	virtual void runComponent();

	/** Called by the framework after the run loop finishes. */
	virtual void stop();


private:
	void overwriteRecognizer3DCommand(const cast::cdl::WorkingMemoryChange & _wmc);
	void addRecognizer3DCommand(std::string label);

	std::vector<std::string> m_labels;

}; // class ObjectRecognizerManager
} // namespace

#endif // CONCEPTUAL_OBJECTRECOGNIZERMANAGER_H




