/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::ChainGraphInferencer class.
 */

#ifndef CONCEPTUAL_CHAINGRAPHINFERENCER_H
#define CONCEPTUAL_CHAINGRAPHINFERENCER_H

// CAST
#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>
#include <DefaultData.hpp>
// LibDAI
#define DAI_WITH_JTREE
#define DAI_WITH_BP

#include <dai/alldai.h>


namespace conceptual
{


/**
 * @author Andrzej Pronobis
 *
 * Main query handler for the Conceptual.SA
 */
class ChainGraphInferencer: public cast::ManagedComponent
{

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

	/** Change event. */
	void worldStateChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Updates the factor graph based on the world state if the world state changed.
	 * Sets factorGraphChanged if the factor graph was changed.
	 * Return true if the world state was valid and inference can be done. */
	bool updateFactorGraph(bool &factorGraphChanged);

	/** Create DAI Variable if not yet created. */
	struct DaiVariable; /* forward declaration */
	DaiVariable& createDaiVariable(const std::string &name, const std::vector<std::string> &values);

	/** Creates a DAI connectivity factor for two rooms. */
	void createDaiConnectivityFactor(int room1Id, int room2Id);
	void createDaiSingleRoomFactor(int room1Id);
	void createDaiObservedObjectPropertyFactor(int room1Id,
			const std::string &objectVariableName, bool objectExists);
	void createDaiShapePropertyGivenRoomCategoryFactor(int room1Id, int placeId);
	void createDaiObservedShapePropertyFactor(int placeId, ConceptualData::ValuePotentialPairs dist);
	void createDaiAppearancePropertyGivenRoomCategoryFactor(int room1Id, int placeId);
	void createDaiObservedAppearancePropertyFactor(int placeId, ConceptualData::ValuePotentialPairs dist);

	/** Adds factors related to the factor graph */
	void addDaiFactors();

	/** Performs all inferences on the factor graph. */
	void runAllInferences();

	/** Prepares inference results based on the graph in which inference was performed. */
	void prepareInferenceResult(std::string queryString,
			SpatialProbabilities::ProbabilityDistribution *resultDistribution);

	/** Reads the default knowledge factors from the Default.SA */
	void getDefaultKnowledge();

	/** Returns the given probability distribution from the default knowledge.
	 * The returned pointer is valid as long _defaultKnowledge is not changed.
	 * At the moment _defaultKnowledge only gets changed on call to getDefaultKnowledge */
	const SpatialProbabilities::ProbabilityDistribution* getDefaultProbabilityDistribution(const std::string &factorName) const;

	/** Returns probability value from the distribution. */
	double getProbabilityValue(const SpatialProbabilities::ProbabilityDistribution &pd,
			std::string var1Value, std::string var2value);

	/** Returns probability value from the distribution. */
	double getProbabilityValue(const SpatialProbabilities::ProbabilityDistribution &pd,
			std::string var1Value, bool var2value);


private:

	struct Query
	{
		ConceptualData::InferenceQueryPtr queryPtr;
		cast::cdl::WorkingMemoryAddress wmAddress;
	};

	/** Recently received queries. */
	std::list<Query> _receivedQueries;

	pthread_cond_t _inferenceQueryAddedSignalCond;
	pthread_mutex_t _inferenceQueryAddedSignalMutex;

	pthread_mutex_t _worldStateMutex;

	/** True if the world state has changed since the last time we checked. */
	bool _worldStateChanged;

	/** Part of the local copy of the world state. */
	ConceptualData::ComaRoomInfos _worldStateRooms;

	/** Part of the local copy of the world state. */
	ConceptualData::RoomConnectivityInfos _worldStateRoomConnections;

	/** Name of the DefaultChainGraphInferencer component.  */
	std::string _defaultChainGraphInferencerName;

	/** Name of the file to which the graph is saved.  */
	std::string _saveGraphFileName;

	/** Name of the file to which the info about variables and their values is saved.  */
	std::string _saveGraphInfoFileName;

	/** ICE proxy to the DefaultData::ChainGraphInferencerInterface. */
	DefaultData::ChainGraphInferencerServerInterfacePrx _defaultChainGraphInferencerServerInterfacePrx;

	/** Default knowledge factors. Map from factor string to the factor itself. */
	std::map<std::string, SpatialProbabilities::ProbabilityDistribution> _defaultKnowledgeFactors;

	/** Names of the object place property variables. */
	DefaultData::StringSeq _objectPropertyVariables;

	/** Names of all room categories. */
	DefaultData::StringSeq _roomCategories;

	/** Names of all shapes. */
	DefaultData::StringSeq _shapes;

	/** Names of all appearances. */
	DefaultData::StringSeq _appearances;


	/** Information about DAI variable. */
	struct DaiVariable
	{
		dai::Var var;
		std::map<int, std::string> valueIdToName;
	};

	/** Map from variable names to DAI variables. */
	std::map<std::string, DaiVariable> _variableNameToDai;

	/** List of all factors in the graph. */
	std::vector<dai::Factor> _factors;

	/** List of all names for all factors. */
	std::vector<std::string> _factorNames;

	/** The main factor graph used for inference. */
	dai::FactorGraph _factorGraph;

	/** Junction tree. */
//	dai::JTree _junctionTree;

	dai::BP _bp;


    dai::PropertySet _daiOptions;

}; // class ChainGraphInferencer
} // namespace def

#endif // CONCEPTUAL_CHAINGRAPHINFERENCER_H



