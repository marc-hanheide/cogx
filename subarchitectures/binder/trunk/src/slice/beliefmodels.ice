
#include <cast/slice/CDL.ice>


/**
 * Slice definition of the belief models formed and maintained by the binder
 *
 * TODO: properly document this specification
 *
 * @author 	Pierre Lison (plison@dfki.de)
 * @version	13.03.2010
 * @started	13.03.2010
 */

module binder {
module autogen {

//////////////////////////////////
// SPATIO-TEMPORAL FRAME
//////////////////////////////////


class SpatioTemporalFrame { };

class SimpleSpatioTemporalFrame extends SpatioTemporalFrame {
	string place;
	cast::cdl::CASTTime startTime;
	cast::cdl::CASTTime endTime;
};


//////////////////////////////////
// EPISTEMIC STATUS
//////////////////////////////////

class EpistemicStatus {} ;

class PrivateEpistemicStatus {
	string agent;
};

sequence<string> Agents;

class AttributedEpistemicStatus {
	string agent;
	Agents attribagents;
};

class SharedEpistemicStatus {
	Agents cgagents;
};

//////////////////////////////////
// FORMULAE
//////////////////////////////////

// include a nominal?
class Formula { };

class ElementaryFormula extends Formula {
	string prop;
};

class ModalFormula extends Formula {
	string modalOp;
	Formula form;
};

class PointerFormula extends Formula {
	string beliefPointer;
};

sequence<Formula> Formulae;
enum BinaryOp {conj, disj};

class ComplexFormula extends Formula {
	Formulae forms;
	BinaryOp op;	
};


//////////////////////////////////
// DISTRIBUTIONS
//////////////////////////////////


struct FormulaProbPair {
	Formula form;
	float prob;
};

sequence<FormulaProbPair> FormulaProbPairs;

// find a way to include the conditional independenc
// why not a dictionary for the probability?
struct ProbDistribution {
	FormulaProbPairs pairs;
};


//////////////////////////////////
// BELIEFS
//////////////////////////////////

enum OntologicalType {percept, punion, mbelief, tunion, sbelief, abelief, cgbelief};

sequence<string> BeliefIds;

struct History {
	BeliefIds ancestors;
	BeliefIds offspring;
};	

class BasicBelief {
	string id;
	SpatioTemporalFrame frame;
	EpistemicStatus estatus;
	OntologicalType ontType;
	ProbDistribution content;
	History hist; 
};	

// TODO: find specification for beliefs split into sub-beliefs, taking cond. (in)dependence into account

}; // end autogen
}; // end binder