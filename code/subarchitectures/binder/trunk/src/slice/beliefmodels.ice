 
// =================================================================
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de) 
//                                                                                                                          
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================
  


#ifndef BELIEF_ICE
#define BELIEF_ICE 
 

#include <cast/slice/CDL.ice>


/**
 * Slice definition of the belief models formed and maintained by the binder
 *
 * The specification is currently divided into 7 modules:
 * - framing: representations for spatio-temporal framing
 * - epstatus: representations for epistemic status
 * - epobject: representations for epistemic objects
 * - formulae: representations for (hybrid logic) formulae
 * - distribs: representations for probability distributions
 * - history: representations for belief histories
 * - beliefs: representations for beliefs
 *
 *
 * @author 	Pierre Lison (plison@dfki.de)
 * @version	19.03.2010
 * @started	13.03.2010
 */

module binder {
module autogen {

	

// ================================
// SPATIO-TEMPORAL FRAME
// ================================


module framing {

/**
 * Abstract class for a spatio-temporal frame
 */
class SpatioTemporalFrame { };


/**
 * A temporal interval defined by a start time and an
 * end time (both specified as CASTTime objects)
 */
struct TemporalInterval {
	cast::cdl::CASTTime startTime;
	cast::cdl::CASTTime endTime;
};
	

/**
 * Simple spatio-temporal frame defined by the conjunction of
 * two constraints:
 * 1) a place identifier,
 * 2) a temporal interval
 */
class SimpleSpatioTemporalFrame extends SpatioTemporalFrame {
	string place;
	TemporalInterval interval;
};

}; // end spatiotemporalframe



// ================================
// EPISTEMIC STATUS
// ================================

module epstatus {


/**
 * Abstract class describing an epistemic status
 */
class EpistemicStatus {} ;


/**
 * Private epistemic status for a given agent
 */
class PrivateEpistemicStatus extends EpistemicStatus {
	string agent;
};


/**
 * Sequence of agents
 */
sequence<string> Agents;

/**
 * Attributed epistemic status of an agent A to a group of 
 * other agents B1...Bn (not including A) 
 */
class AttributedEpistemicStatus extends EpistemicStatus {
	string agent;
	Agents attribagents;
};


/**
 * Shared epistemic status (common ground) of a group of
 * agents A1...An
 */
class SharedEpistemicStatus extends EpistemicStatus{
	Agents cgagents;
};

};  // end epstatus





// ================================
// EPISTEMIC OBJECTS
// ================================


module epobject {

/**
 * Abstract class for epistemic objects 
 */
class EpistemicObject {
	framing::SpatioTemporalFrame frame;
	epstatus::EpistemicStatus estatus;
} ;


// EPISTEMIC OBJECTS WILL BE EXTENDED LATER ON TO INCLUDE NOT
// ONLY BELIEFS, BUT ALSO INTENTIONS, PLANS, ETC

};


module featurecontent {

/**
 * Enumeration of possible features ("modal operators" in the modal 
 * logic terminology) which can be defined on a formula
 *
 * TO BE COMPLETED!!
 */
enum Feature {
	Colour, 
	Shape, 
	ObjectLabel, 
	Size,
	PlaceId,
	PlaceStatus,
	ConnectedTo1,
	ConnectedTo2,
	Position };
	
	
class FeatureValue { };


class StringValue extends FeatureValue {
	string val;
};

class IntegerValue extends FeatureValue {
	int val;
};

class BooleanValue extends FeatureValue {
	bool val;
};

class UnknownValue extends FeatureValue { } ;

class PointerValue extends FeatureValue {
	string beliefId;
};

}; 


// ================================
// FORMULAE
// ================================

module logicalcontent {

/**
 * Abstract class for a formula
 */
 class Formula { 
 	int id;  // nominal (cf. hybrid logic)
 };


/**
 * Elementary formula consisting of a single
 * logical proposition
 */
class ElementaryFormula extends Formula {
	string prop;
};


/**
 * Formula consisting of a pointer to another
 * belief, referenced by its identifier
 */
class PointerFormula extends Formula {
	string beliefPointer;
};


/** 
 * Formula consisting in the negation of another
 * formula
 */
class NegatedFormula extends Formula {
	Formula negForm;
};


/**
 * Formula consisting in a modal operator <op>
 * applied on another formula
 */
class ModalFormula extends Formula {
	string op;
	Formula form;
};


/**
 * A sequence of formulae
 */
sequence<Formula> FormulaSeq;


/**
 * Possible binary operators: ^ and v 
 */
enum BinaryOp {conj, disj};


/**
 * A complex formula is defined as a sequence of formulae
 * which are combined via a binary operator (^ or v)
 */ 
class ComplexFormula extends Formula {
	FormulaSeq forms;
	BinaryOp op;	
};


}; // end formulae



// ================================
// DISTRIBUTIONS
// ================================

module distribs {

/**
 * Abstract class for a (multivariate) probability distribution 
 * over alternative belief contents
 */
class ProbDistribution { };


/**
 * sequence of probability distributions
 */
sequence<ProbDistribution> Distributions;


/**
 * Discrete probability distribution for P(E,X1...Xn), where :
 * - E ("existence") takes two values (true or false) and
 *   P(E) = Pe (E)
 * - X1...Xn is dependent on E in the following way:
 *   P(X1...Xn|E=true) = Pc (X1...Xn) and
 *   P(X1...Xn|E=false) = 0 for all values of X1...Xn
 */
class DistributionWithExistDep extends ProbDistribution {
	ProbDistribution Pe;
	ProbDistribution Pc;
};

/**
 * Discrete probability distribution for P(X1...Xn) where each 
 * random variable Xi is conditionally independent on the other,
 * i.e. P(X1...Xn) = P(X1)...P(xn)
 */
class CondIndependentDistribs extends ProbDistribution {
	Distributions distribs;
};


struct FeatureValueProbPair {
	featurecontent::FeatureValue val;
	float prob;
};

sequence<FeatureValueProbPair> FeatureValueProbPairs;


class FeatureValueDistribution extends ProbDistribution {
	featurecontent::Feature feat;
	FeatureValueProbPairs values;
};
	

/**
 * Continuous probability distribution P(X) defined on the value
 * of a particular feature.  The distribution P(X) is a normal
 * distribution with mean and variance as parameters
 */
class NormalDistribution extends ProbDistribution {
	featurecontent::Feature feat;
	double mean;
	double variance;
};

/**
 * A data structure consisting of a formula associated with a 
 * probability value
 */
struct FormulaProbPair {
	logicalcontent::Formula form;
	float prob;
};


/**
 * A collection of <form,prob> pairs
 */
sequence<FormulaProbPair> FormulaProbPairs;


/**
 * A discrete distribution defined as a collection of
 * <form,prob> pairs
 */
class DiscreteDistribution extends ProbDistribution {
	FormulaProbPairs pairs;
};

}; // end distributions



// ================================
// BELIEF DEVELOPMENT HISTORY
// ================================

module history {

/**
 * Abstract class specifying the history of a given belief
 */
class BeliefHistory { };


/** 
 * If the belief is a percept, i.e. directly derived from
 * a subarchitecture perceptual input, link to the working
 * memory pointer (including the subarchitecture ID)
 */
class PerceptHistory extends BeliefHistory {
	cast::cdl::WorkingMemoryPointer origin;
};

/**
 * Collection of belief identifiers
 */
sequence<string> BeliefIds;


/**
 * If the belief is not a percept, the history is expressed 
 * as a set of pointers to the belief's ancestors and offspring
 */
class BinderHistory extends BeliefHistory {
	BeliefIds ancestors;
	BeliefIds offspring;
};	


}; // end history


// ================================
// BELIEFS
// ================================

module beliefs {

/**
 * Enumeration of possible ontological types for a given belief
 * 
 * NOTE: ontological types now specified as distinct classes, to be able
 * to apply change filters directly on them
 */
// enum OntologicalType {
//	percept, 
//	perceptualunion, 
//	multimodalbelief, 
//	temporalunion, 
//	stablebelief, 
//	attributedbelief, 
//	sharedbelief}; 


/**
 * Definition a belief, consisting of an identifier, a spatio-temporal
 * frame, an epistemic status, an ontological type, a belief development
 * history, and a probabilistic content
 */
class Belief extends epobject::EpistemicObject {
	string id;
	distribs::ProbDistribution content;
	history::BeliefHistory hist; 
};


/**
 * Low-level perceptual belief (uni-modal, current spatio-temporal frame)
 */
class PerceptBelief extends Belief { };


/**
 * Belief constituted of the combination of one or more perceptual beliefs
 */
class PerceptUnionBelief extends Belief { };


/**
 * Multi-modal belief constrained to the current spatio-temporal frame.  
 * Constructed by refining the values in a percept union
 */
class MultiModalBelief extends Belief { };


/**
 * Belief constituted of the combination of one or more multi-modal beliefs
 * over time
 */
class TemporalUnionBelief extends Belief { } ;


/**
 * High-level belief refining and abstracting over temporal unions.  Stable
 * beliefs are thus multi-modal and span an extended spatio-temporal frame
 */
class StableBelief extends Belief { } ;

/**
 * Attributed belief
 */
class AttributedBelief extends Belief { };

/**
 * Shared belief
 */
class SharedBelief extends Belief { } ;
	

}; // end beliefs



}; // end autogen

}; // end binder


#endif



// NOTES PLISON:
// 21.03.2010: the ontological types now specified as separate classes, for efficiency reasons regarding the CAST change filters
