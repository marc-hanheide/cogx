#ifndef ABDUCER_ICE
#define ABDUCER_ICE

module Abducer {

	exception AbducerException {
		string message;
	};
	
	exception ParseException extends AbducerException {};

	//-----------------------------------------------------------------

	class Term;
	sequence<Term> TermSeq;
	
	class Term {
		bool variable;
		string name;
		TermSeq args;
	};
	
	class Predicate {
		string predSym;
		TermSeq args;
	};

	//-----------------------------------------------------------------

	enum ModalityType {
		Event,
		Info,
		AttState,
		K
	};

	enum Agent {
		Human,
		Robot
	};

	enum Sharing {
		Private,
		Attribute,
		Mutual
	};

	class Modality {
		ModalityType type;
	};

	class KModality extends Modality {
		Agent act;
		Agent pat;
		Sharing share;
	};

	sequence<Modality> ModalitySeq;

	//-----------------------------------------------------------------

	class ModalisedFormula {
		ModalitySeq m;
		Predicate p;
	};

	sequence<ModalisedFormula> ModalisedFormulaSeq;

	class Assertion {
		ModalisedFormula head;
		ModalisedFormulaSeq ante;
	};

	sequence<Assertion> AssertionSeq;

	enum Marking {
		Proved,
		Unsolved,
		Assumed,
		Asserted
	};

	class MarkedQuery {
		Marking mark;
		ModalisedFormula body;
	};

	class ProvedQuery extends MarkedQuery {};
	class UnsolvedQuery extends MarkedQuery {
		bool isConst;
		float constCost;
		string costFunction;
	};
	class AssumedQuery extends MarkedQuery {
		// would we perhaps prefer to have the actual used costs
		// in the returned proof?
		bool isConst;
		float constCost;
		string costFunction;
	};
	class AssertedQuery extends MarkedQuery {
		ModalisedFormulaSeq antecedents;
	};

	sequence<MarkedQuery> MarkedQuerySeq;

/*
	class AssumableGoal {
		ModalisedFormula body;
		float assumeCost;
	};

	sequence<AssumableGoal> AssumableGoalSeq;
*/

	class AbductiveProof {
		float cost;
		MarkedQuerySeq body;
	};

	enum ProveResult {
		SUCCESS,
		FAILED,
		ERROR
	};

	interface AbducerServer {
		void clearRules();
		void loadRulesFromFile(string filename);

		void clearFacts();
		void loadFactsFromFile(string filename);
		void addFact(ModalisedFormula f);

		void clearAssumables();
		void addAssumable(string function, ModalisedFormula f, float cost);

		ProveResult prove(MarkedQuerySeq g);
		AbductiveProof getBestProof();
	};

};

#endif
