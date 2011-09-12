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
// Boost&Std
#include <boost/math/special_functions/factorials.hpp>
#include <math.h>
#include <string>

namespace conceptual
{


/**
 * @author Andrzej Pronobis
 *
 * Main query handler for the Conceptual.SA
 */
class ChainGraphInferencer: public cast::ManagedComponent
{

	class TestingServer: public ConceptualData::ChainGraphTestingServerInterface
	{
	public:

		/** Constructor. */
		TestingServer(ChainGraphInferencer *chainGraphInferencer) : _chainGraphInferencer(chainGraphInferencer)
		{}

		ConceptualData::VariableInfos getVariables(const Ice::Current &);
		ConceptualData::FactorInfos getFactors(const Ice::Current &);

	private:

		/** Pointer to the owner of the server. */
		ChainGraphInferencer *_chainGraphInferencer;
    };

	enum ObjectPropertyModel
	{OPM_COUNTING, OPM_OBSERVATION_MODEL};

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
	void createDaiVariable(std::string name, const std::vector<std::string> &values);

	/** Creates and/or sets the value of an observed variable. */
	void createAndSetObservedVariable(std::string name, const std::vector<std::string> &values, int observedValue);

	/** Creates a DAI connectivity factor for two rooms. */
	void createDaiConnectivityFactor(int room1Id, int room2Id);
	/**
	 * Creates an uninformative DAI factor for the room to make each variable always
	 * have at least one factor.
	 */
	void createDaiSingleRoomFactor(int room1Id);
	void createDaiObjectPropertyGivenRoomCategoryFactorOM(int room1Id, const std::string &objectCategory,
			SpatialData::SpatialRelation relation, const std::string &supportObjectCategory,
			const std::string &supportObjectId, double beta);
	void createDaiObservedObjectPropertyFactorOM(int room1Id, const std::string &objectCategory,
			SpatialData::SpatialRelation relation, const std::string &supportObjectCategory, const std::string &supportObjectId,
			bool objectPresent, double beta);
	/** Creates the factor for an observed object counter variable for the explored space. */
	void createDaiObservedObjectPropertyFactorCounting(int room1Id, const std::string &objectCategory,
			SpatialData::SpatialRelation relation, const std::string &supportObjectCategory,
			const std::string &supportObjectId, unsigned int objectCount, double beta);
	/** Creates the factor for the presence of the object in yet unexplored space. */
	void createDaiObjectUnexploredFactor(int room1Id, const std::string &objectCategory,
			SpatialData::SpatialRelation relation, const std::string &supportObjectCategory,
			const std::string &supportObjectId, double beta);
	void createDaiShapePropertyGivenRoomCategoryFactor(int room1Id, int placeId);
	void createDaiSizePropertyGivenRoomCategoryFactor(int room1Id, int placeId);
	void createDaiObservedShapePropertyFactor(int placeId, ConceptualData::ValuePotentialPairs dist);
	void createDaiObservedSizePropertyFactor(int placeId, ConceptualData::ValuePotentialPairs dist);
	void createDaiAppearancePropertyGivenRoomCategoryFactor(int room1Id, int placeId);
	void createDaiObservedAppearancePropertyFactor(int placeId, ConceptualData::ValuePotentialPairs dist);
	void createDaiObservedHumanAssertionPropertyFactor(int room1Id, const std::string humanAssertion);

	/** Adds factors related to the factor graph */
	void addDaiFactors();

	/** Performs all inferences on the factor graph. */
	void runAllInferences();

	/** Performs generation of all imaginary worlds and runs inverence on those. */
	void runImaginaryWorldsGeneration();

	/** Performs imaginary world generation for a single placeholder in a single room. */
	void runImaginaryWorldsGenerationForPlaceholderInRoom(int roomId,
			std::vector< std::vector<double> > &allOutputs);

	void evaluateCurrentRoomImaginaryWorld(int roomId, std::vector<double> &outputs, double prior);
	void evaluateImaginaryWorld(int roomId, int worldId, std::vector<double> &outputs, double prior);

	/** Prepares inference results based on the graph in which inference was performed. */
	void prepareInferenceResult(std::string queryString, std::vector<std::string> queryVariables,
			ConceptualData::ProbabilityDistributions &resultDistribution);

	/** Prepares inference results based on the graph and imaginary worlds. */
	void prepareImaginaryInferenceResult(std::string queryString, std::vector<std::string> queryVariables,
			ConceptualData::ProbabilityDistributions &resultDistribution);

	/** Prepares results of the factor query. */
	void prepareFactorResult(std::string queryString, std::vector<std::string> queryVariables,
			ConceptualData::ProbabilityDistributions &resultDistribution);

	/** Reads the default knowledge factors from the Default.SA */
	void getDefaultKnowledge();

	/** Returns probability value from the distribution. */
	double getProbabilityValue(const SpatialProbabilities::ProbabilityDistribution &pd,
			std::string var1Value, std::string var2value);

	/** Returns probability value from the distribution. */
	double getProbabilityValue(const SpatialProbabilities::ProbabilityDistribution &pd,
			std::string var1Value, bool var2value);

	/** Returns probability value from the distribution. */
	double getProbabilityValue(const SpatialProbabilities::ProbabilityDistribution &pd,
			bool var1Value, bool var2value);

	/** Parses a query into a vector of variables. */
	void parseQuery(std::string queryString, ConceptualData::QueryType type, std::vector<std::string> &variables);

	/** Retrieves the elements of the variable name. */
	void parseVariable(std::string variableName, std::vector<std::string> &elements);

	/** Retrieves the value of the variable from the var=val string. */
	void parseVariableValue(const std::string variableString,
			std::string &variableName, std::string &variableValue);

	/** Adds room connectivity factor for 2 variables representing room categories. */
	void createDaiConnectivityFactor(std::vector<dai::Factor> &factors, dai::Var &var1, dai::Var &var2);

	void generateOutputsUsingImaginaryVariables(dai::BP &bp, dai::VarSet &vars, std::vector<double> &outputs);

	void integrateImaginaryWorldsOutputs(
			const std::vector< std::vector<double> > &allOutputs,
			std::vector<double> &outputs, double prior);

	/** Returns the calculated Lambda of the poisson distrubution for the object, relation and room category. */
	double getPoissonLambda(const std::string &roomCategory, const std::string &objectCategory,
							SpatialData::SpatialRelation relation, const std::string &supportObjectCategory);

	/** Calculates probability for the Poisson distribution. */
	static double getPoissonProabability(double lambda, unsigned int k)
	{
		return (::pow(lambda, static_cast<double>(k)) * ::exp(-lambda)) / (boost::math::factorial<double>(k));
	}

	int wildcmp(const char *wild, const char *string);


private:

	ObjectPropertyModel _objectPropertyModel;

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

	/**
	 * Mutex protecting the graph and all graph related variables. Used mostly
	 *  to protect the graph while testing server interface is used.
	 */
	pthread_mutex_t _graphMutex;

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

	/** If true, placeholder properties will be inferred. */
	bool _inferPlaceholderProperties;

	double _freespacePlaceholderRate;

	bool _addUnobservedShape;
	bool _addUnobservedSize;
	bool _addUnobservedAppearance;
	std::vector<std::string> _addUnobservedObjects;



	/** ICE proxy to the DefaultData::ChainGraphInferencerInterface. */
	DefaultData::ChainGraphInferencerServerInterfacePrx _defaultChainGraphInferencerServerInterfacePrx;

	/** Default knowledge factors. Map from factor string to the factor itself. */
	std::map<std::string, SpatialProbabilities::ProbabilityDistribution> _defaultKnowledgeFactors;

	/** Names of the object place property variables. */
	DefaultData::StringSeq _objectPropertyVariables;

	/** Names of all room categories. The string is the concatenation of all the parameters that
	 *  lead to this lambda. */
	DefaultData::StringSeq _roomCategories;

	/** Names of all shapes. */
	DefaultData::StringSeq _shapes;

	/** Names of all sizes. */
	DefaultData::StringSeq _sizes;

	/** Names of all appearances. */
	DefaultData::StringSeq _appearances;

	/** Names of all human assertions. */
	DefaultData::StringSeq _humanAssertions;

	/** Information about DAI variable. */
	struct DaiVariable
	{
		dai::Var var;
		std::map<int, std::string> valueIdToName;
	};

	/** Map from variable names to DAI variables. */
	std::map<std::string, DaiVariable> _variableNameToDai;

	struct ObservedVariable
	{
		int id;
		unsigned int observedValue;
		std::map<int, std::string> valueIdToName;
	};

	/** Observed variable names to variable info map. */
	std::map<std::string, ObservedVariable> _observedVariableNameToInfo;

	/** List of all factors in the graph. */
	std::vector<dai::Factor> _factors;

	/** List of all names for all factors. */
	std::vector<std::string> _factorNames;

	/** The main factor graph used for inference. */
	dai::FactorGraph _factorGraph;

	/** Inferred probability of category existance for the room category placeholder property. */
	std::map<std::string, double> _placeholderRoomCategoryExistance;

	/** List of additional variables that were requested in the queries that should be created on request. */
	std::set<std::string> _additionalVariables;

	/** Cache for the lambda values. */
	std::map<std::string, double> _poissonLambdaCache;

	/** Junction tree. */
//	dai::JTree _junctionTree;

	dai::BP _bp;

    dai::PropertySet _daiOptions;


}; // class ChainGraphInferencer
} // namespace def

#endif // CONCEPTUAL_CHAINGRAPHINFERENCER_H



