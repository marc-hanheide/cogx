/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::ChainGraphInferencer class.
 */

#ifndef DEFAULT_CHAINGRAPHINFERENCER_H
#define DEFAULT_CHAINGRAPHINFERENCER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <DefaultData.hpp>
#include <ComaData.hpp>


namespace def
{


/**
 * @author Andrzej Pronobis
 *
 * Main query handler for the Default.SA
 */
class ChainGraphInferencer: public cast::ManagedComponent
{

	class Server: public DefaultData::ChainGraphInferencerServerInterface
	{
	public:

		/** Constructor. */
		Server(ChainGraphInferencer *chainGraphInferencer) : _chainGraphInferencer(chainGraphInferencer)
		{}

		virtual DefaultData::StringSeq getObjectPropertyVariables(const Ice::Current &);

		virtual DefaultData::StringSeq getRoomCategories(const Ice::Current &);

		virtual SpatialProbabilities::ProbabilityDistribution
			getFactor(const std::string &factorStr, const Ice::Current &);

	private:

		/** Pointer to the owner of the server. */
		ChainGraphInferencer *_chainGraphInferencer;
    };




public:

	/** Constructor. */
	ChainGraphInferencer();

	/** Destructor. */
	virtual ~ChainGraphInferencer();


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

	/** Change event. */
	void inferenceQueryAdded(const cast::cdl::WorkingMemoryChange &wmChange);



private:

	struct Query
	{
		DefaultData::InferenceQueryPtr queryPtr;
		cast::cdl::WorkingMemoryAddress wmAddress;
	};

	/** Recently received queries. */
	std::list<Query> _receivedQueries;

	pthread_cond_t _inferenceQueryAddedSignalCond;
	pthread_mutex_t _inferenceQueryAddedSignalMutex;


	/** Id of the forward chainer server component.  */
	std::string _hfcServerName;

	/** ICE proxy to the forward chainer server interface. */
	comadata::HFCInterfacePrx _hfcInterfacePrx;

	/** Results of the query for the default knowledge sent to HFC. */
	comadata::QueryResults _hfcQueryResults;

	struct HFCItem
	{
		std::string room;
		std::string object;
		double probability;
	};

	/** Converted knowledge from the HFC. */
	std::list<HFCItem> _hfcKnowledge;

	double _defaultRoomCategoryConnectivityPotential;
	double _defaultObjectExistenceProbability;


	struct RoomCategoryConnectivity
	{
		std::string roomCategory1;
		std::string roomCategory2;
		double potential;
	};

	/** Room connectivity information as read from the config file. */
	std::list<RoomCategoryConnectivity> _roomCategoryConnectivity;

	/** List of all object property variable names. */
	std::vector<std::string> _objectPropertyVariables;

	/** List of all the room categories. */
	std::vector<std::string> _roomCategories;


}; // class ChainGraphInferencer
} // namespace def

#endif // DEFAULT_CHAINGRAPHINFERENCER_H



