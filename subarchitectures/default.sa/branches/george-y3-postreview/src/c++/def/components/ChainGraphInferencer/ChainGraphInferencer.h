/**
 * @author Andrzej Pronobis
 *
 * Declaration of the def::ChainGraphInferencer class.
 */

#ifndef DEFAULT_CHAINGRAPHINFERENCER_H
#define DEFAULT_CHAINGRAPHINFERENCER_H

#include <cast/architecture/ManagedComponent.hpp>
#include "VariableNameGenerator.h"
#include <DefaultData.hpp>
#include <ComaData.hpp>
#include <SpatialData.hpp>


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

		virtual DefaultData::StringSeq getObjectCategories(const Ice::Current &);

		virtual DefaultData::StringSeq getRoomCategories(const Ice::Current &);

		virtual DefaultData::StringSeq getShapes(const Ice::Current &);

		virtual DefaultData::StringSeq getSizes(const Ice::Current &);

		virtual DefaultData::StringSeq getAppearances(const Ice::Current &);

		virtual DefaultData::StringSeq getHumanAssertions(const Ice::Current &);

		virtual SpatialProbabilities::ProbabilityDistribution
			getFactor(const std::string &factorStr, const Ice::Current &);

	private:

		/** Pointer to the owner of the server. */
		ChainGraphInferencer *_chainGraphInferencer;
    };


	enum LoadObjectsFrom
		{LOF_HFC, LOF_DEFAULTPROB};


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

	/** Loads the avs default knowledge file (defaultprob.txt) */
	void loadAvsDefaultKnowledge();

	/** Generates a list of objects, room categories etc.. */
	void generateLists();

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

	struct ObjectPropertyGivenRoomCategory
	{
		std::string roomCategory;
		std::string objectCategory;
		std::string supportObjectCategory;
		SpatialData::SpatialRelation relation;
		double probability;
	};

	/** Converted knowledge from the HFC. */
	std::list<ObjectPropertyGivenRoomCategory> _objectPropertyGivenRoomCategory;

	double _defaultObjectExistenceProbability;

	struct ObjectObservationModel
	{
		std::string objectCategory;
		std::string supportObjectCategory;
		SpatialData::SpatialRelation relation;
		double truePositiveProbability;
		double trueNegativeProbability;
	};

	/** Converted knowledge from the HFC. */
	std::list<ObjectObservationModel> _objectObservationModel;

	double _defaultObjectTruePositiveProbability;
	double _defaultObjectTrueNegativeProbability;

	double _defaultRoomCategoryConnectivityPotential;

	struct RoomCategoryConnectivity
	{
		std::string roomCategory1;
		std::string roomCategory2;
		double potential;
	};

	/** Room connectivity information as read from the config file. */
	std::list<RoomCategoryConnectivity> _roomCategoryConnectivity;

	/** List of all object categories. */
	std::vector<std::string> _objectCategories;

	/** List of all object property variable names. */
	std::vector<std::string> _objectPropertyVariables;

	/** List of all the room categories. */
	std::vector<std::string> _roomCategories;

	/** List of all the shapes. */
	std::vector<std::string> _shapes;

	/** List of all the sizes. */
	std::vector<std::string> _sizes;

	/** List of all the appearances. */
	std::vector<std::string> _appearances;

	/** List of all the human assertions. */
	std::vector<std::string> _humanAssertions;

	struct ShapePropertyGivenRoomCategory
	{
		std::string roomCategory1;
		std::string shapeProperty;
		double probability;
	};

	std::list<ShapePropertyGivenRoomCategory> _shapePropertyGivenRoomCategory;

	struct DefaultShapePropertyGivenRoomCategory
	{
		std::string shapeProperty;
		double probability;
	};

	std::list<DefaultShapePropertyGivenRoomCategory> _defaultShapePropertyGivenRoomCategory;


	struct SizePropertyGivenRoomCategory
	{
		std::string roomCategory1;
		std::string sizeProperty;
		double probability;
	};

	std::list<SizePropertyGivenRoomCategory> _sizePropertyGivenRoomCategory;

	struct DefaultSizePropertyGivenRoomCategory
	{
		std::string sizeProperty;
		double probability;
	};

	std::list<DefaultSizePropertyGivenRoomCategory> _defaultSizePropertyGivenRoomCategory;


	struct AppearancePropertyGivenRoomCategory
	{
		std::string roomCategory1;
		std::string appearanceProperty;
		double probability;
	};

	std::list<AppearancePropertyGivenRoomCategory> _appearancePropertyGivenRoomCategory;

	struct DefaultAppearancePropertyGivenRoomCategory
	{
		std::string appearanceProperty;
		double probability;
	};

	std::list<DefaultAppearancePropertyGivenRoomCategory> _defaultAppearancePropertyGivenRoomCategory;


	struct HumanAssertionPropertyGivenRoomCategory
	{
		std::string roomCategory1;
		std::string humanAssertionProperty;
		double probability;
	};

	std::list<HumanAssertionPropertyGivenRoomCategory> _humanAssertionPropertyGivenRoomCategory;
	double _defaultMatchingHumanAssertionProbability;


	/** Determines from where the object information should be loaded. */
	LoadObjectsFrom _loadObjectsFrom;

	/** Path to the file with object info. */
	std::string _objectsFileName;


}; // class ChainGraphInferencer
} // namespace def

#endif // DEFAULT_CHAINGRAPHINFERENCER_H



