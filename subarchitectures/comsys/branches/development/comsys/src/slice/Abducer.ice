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

	//-----------------------------------------------------------------

	// marking as used in the abductive proof
	enum Marking {
		Proved,
		Unsolved,
		Assumed,
		Asserted
	};

	// base class for abductive proofs
	class MarkedQuery {
		Marking mark;
		ModalisedFormula body;
	};

	// this predicate has been solved
	class ProvedQuery extends MarkedQuery {};

	// this predicate is yet to be solved
	class UnsolvedQuery extends MarkedQuery {
		// isConst == true -> constCost valid, else costFunction valid
		bool isConst;
		float constCost;
		string costFunction;
	};

	// assumed predicate
	class AssumedQuery extends MarkedQuery {
		// TODO: would we perhaps prefer to have the actual used costs
		// in the returned proof?

		// isConst == true -> constCost valid, else costFunction valid
		bool isConst;
		float constCost;
		string costFunction;
	};

	// asserted predicate
	class AssertedQuery extends MarkedQuery {
		ModalisedFormulaSeq antecedents;
	};

	//-----------------------------------------------------------------

	sequence<MarkedQuery> MarkedQuerySeq;

	// an abductive proof is a list of marked queries and its overall
	// cost
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
