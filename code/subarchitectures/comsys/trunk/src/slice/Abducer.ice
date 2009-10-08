#ifndef ABDUCER_ICE
#define ABDUCER_ICE
 
module Abducer {

	exception AbducerException {
		string message;
	};
	
	exception FileNotFoundException extends AbducerException {};
	exception SyntaxErrorException extends AbducerException {};
	exception NoProofException extends AbducerException {};

	//-----------------------------------------------------------------

	// TERMS & PREDICATES

	enum TermType {
		Function,
		Variable
	};

	// Base class for both types of terms.
	class Term {
		TermType type;
	};

	sequence<Term> TermSeq;

	// Variable.
	class VariableTerm extends Term {
		string name;
	};

	// Function term is a term of the form f(t1,...tn)
	// where f is a functor and t1... are terms.
	class FunctionTerm extends Term {
		string functor;
		TermSeq args;
	};

	class Predicate {
		string predSym;
		TermSeq args;
	};

	//-----------------------------------------------------------------

	// MODALITIES

	enum ModalityType {
		Event,
		Info,
		AttState,
		K
	};

	enum Agent {
		human,
		robot
	};

	enum Sharing {
		Private,
		Attribute,
		Mutual
	};

	// Base modality class.
	class Modality {
		ModalityType type;
	};
	
	sequence<Modality> ModalitySeq;

	class EventModality extends Modality { };
	
	class InfoModality extends Modality { };
	
	class AttStateModality extends Modality { };

	class KModality extends Modality {
		Agent act;
		Agent pat;
		Sharing share;
	};

	//-----------------------------------------------------------------

	class ModalisedFormula {
		ModalitySeq m;
		Predicate p;
	};

	sequence<ModalisedFormula> ModalisedFormulaSeq;

	//-----------------------------------------------------------------

	// PIECES OF PROOF

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

	// PROOF

	sequence<MarkedQuery> MarkedQuerySeq;

	//-----------------------------------------------------------------

	// SERVER INTERFACE

	enum ProveResult {
		ProofFound,
		NoProofFound,
		Error
	};

	interface AbducerServer {
		void clearRules();
		void loadRulesFromFile(string filename)
				throws FileNotFoundException, SyntaxErrorException;

		void clearFacts();
		void clearFactsByModality(ModalityType type);
		void loadFactsFromFile(string filename)
				throws FileNotFoundException, SyntaxErrorException;
		void addFact(ModalisedFormula f);

		void clearAssumables();
		void addAssumable(string function, ModalisedFormula f, float cost);

		ProveResult prove(MarkedQuerySeq g);
		MarkedQuerySeq getBestProof()
				throws NoProofException;
	};

};

#endif
