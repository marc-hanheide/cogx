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
	sequence<int> intSeq;

	class AbductiveProof {
		float cost;
		ModalisedFormulaSeq body;
		intSeq assumed;
		intSeq asserted;
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

		ProveResult prove(AssumableGoalSeq g);
		AbductiveProof getBestProof();
	};

};

#endif
