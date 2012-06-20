/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::ObjectRecognizerManager class.
 */

#ifndef CONCEPTUAL_OBJECTRECOGNIZERMANAGER_H
#define CONCEPTUAL_OBJECTRECOGNIZERMANAGER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <FrontierInterface.hpp>


namespace categorical
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
	int getCurrentPlace();
	void addNewObject(std::string label, int curPlaceId, int curRoomId);
	void worldStateChanged(const cast::cdl::WorkingMemoryChange &wmChange);


	std::vector<std::string> m_labels;
	int _curRoomId;


	FrontierInterface::PlaceInterfacePrx _placeInterfacePrx;
	std::string _placeManagerName;


	typedef std::list< std::pair<std::string, int> > ObjectPlaceList;
	ObjectPlaceList _objectInPlace;

}; // class ObjectRecognizerManager
} // namespace

#endif // CONCEPTUAL_OBJECTRECOGNIZERMANAGER_H




