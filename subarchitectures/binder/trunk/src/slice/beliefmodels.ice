 
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
 * TODO: check gj and union specs for comparison
 *
 * @author 	Pierre Lison (plison@dfki.de)
 * @version	19.03.2010
 * @started	13.03.2010
 */

module binder {
module autogen {



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
	Position };
	
	
	

// ================================
// SPATIO-TEMPORAL FRAME
// ================================


module spatiotemporalframing {

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
// FORMULAE
// ================================

module formulae {

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
 * Formula consisting in a modal operator <feat>
 * applied on another formula
 */
class ModalFormula extends Formula {
	Feature feat;
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

/**
 * Continuous probability distribution P(X) defined on the value
 * of a particular feature.  The distribution P(X) is a normal
 * distribution with mean and variance as parameters
 */
class NormalDistribution extends ProbDistribution {
	Feature feat;
	double mean;
	double variance;
};

/**
 * A data structure consisting of a formula associated with a 
 * probability value
 */
struct FormulaProbPair {
	binder::autogen::formulae::Formula form;
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

module beliefhistory {


/**
 * Abstract class specifying the history of a given belief
 */
class History { };


/** 
 * If the belief is a percept, i.e. directly derived from
 * a subarchitecture perceptual input, link to the working
 * memory pointer (including the subarchitecture ID)
 */
class PerceptHistory {
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
class BinderHistory {
	BeliefIds ancestors;
	BeliefIds offspring;
};	


}; // end beliefhistory




// ================================
// BELIEFS
// ================================

module beliefs {


/**
 * Enumeration of possible ontological types for a given belief
 */
enum OntologicalType {
	percept, 
	perceptualunion, 
	multimodalbelief, 
	temporalunion, 
	stablebelief, 
	attributedbelief, 
	sharedbelief};


/**
 * Definition a belief, consisting of an identifier, a spatio-temporal
 * frame, an epistemic status, an ontological type, a belief development
 * history, and a probabilistic content
 */
class Belief {
	string id;
	OntologicalType ontType;
	binder::autogen::spatiotemporalframing::SpatioTemporalFrame frame;
	binder::autogen::epstatus::EpistemicStatus estatus;
	binder::autogen::distribs::ProbDistribution content;
	binder::autogen::beliefhistory::History hist; 
};	

}; // end beliefs



}; // end autogen

}; // end binder


#endif


