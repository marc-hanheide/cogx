#ifndef ABDUCER_ICE
#define ABDUCER_ICE

#include <BeliefModels.ice>

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

		sequence<Goal> goalSeq;

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
			void synchronise(beliefmodels::adl::BeliefModel m);

			void clearRules();
			void loadRulesFromFile(string filename);

			ProofResult proveGoal(Goal g);
			AbductiveProof getBestProof();
		};

	};
};

#endif
