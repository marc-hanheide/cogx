#ifndef ABDUCER_ICE
#define ABDUCER_ICE

#include <BeliefModels.ice>

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
		AttState
	};

	class Modality {
		ModalityType type;
	};

	//-----------------------------------------------------------------

	class ModalisedFormula {
		Modality m;
		Predicate p;
	};

	sequence<ModalisedFormula> ModalisedFormulaSeq;

	class Assertion {
		ModalisedFormula head;
		ModalisedFormulaSeq ante;
	};

	sequence<Assertion> AssertionSeq;

	class AssumableGoal {
		ModalisedFormula body;
		float assumeCost;
	};

	sequence<AssumableGoal> AssumableGoalSeq;

	sequence<string> stringSeq;

	class AbductiveProof {
		float cost;
		stringSeq assumed;
		stringSeq asserted;
		//ModalisedFormulaSeq assumed;
		//AssertionSeq asserted;
	};

	enum ProofResult {
		SUCCESS,
		FAILED,
		ERROR
	};

	interface AbducerServer {
		void synchronise(beliefmodels::adl::BeliefModel m);

		void clearExplicitRules();
		void loadExplicitRulesFromFile(string filename);

		void clearExplicitFacts();
		void loadExplicitFactsFromFile(string filename);
		void addExplicitFact(ModalisedFormula f);

		ProofResult proveGoal(AssumableGoalSeq g);
		AbductiveProof getBestProof();
	};

};

#endif
