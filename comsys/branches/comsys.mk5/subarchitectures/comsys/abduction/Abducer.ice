module autogen {

	module Abducer {

		class ModalisedFormula {
			string termString;
		};

		sequence<ModalisedFormula> modFormulaSeq;

		class Assertion {
			ModalisedFormula head;
			modFormulaSeq ante;
		};

		sequence<Assertion> assertionSeq;

		class Goal {
			ModalisedFormula body;
			float assumeCost;
		};

		class AbductiveProof {
			float cost;
			modFormulaSeq assumed;
			assertionSeq asserted;
		};

		enum ProofResult {
			SUCCESS,
			FAILED,
			ERROR
		};

		interface AbducerServer {
			void clearFacts();
			void clearRules();

			void addFact(ModalisedFormula fact);
			void addRule(ModalisedFormula rule);

			ProofResult proveGoal(Goal g);
			AbductiveProof getBestProof();
		};

	};
};
