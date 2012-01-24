// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de), Pierre Lison (pierre.lison@dfki.de) 
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
 
/**
 * API [ de.dfki.lt.tr.beliefs ] 
 * 
 * Slice definition of multi-agent situated belief models, 
 * with root package name [ de.dfki.lt.tr.beliefs.slice ]
 *
 * The specification provides the following modules:
 * - framing: representations for spatio-temporal framing
 * - epstatus: representations for epistemic status
 * - epobject: representations for epistemic objects
 * - formulae: representations for (hybrid logic) formulae
 * - distribs: representations for probability distributions
 * - history: representations for belief histories
 * - sitbeliefs: representations for situated beliefs
 *
 * @author 	Geert-Jan M. Kruijff (gj@dfki.de)
 * @author 	Pierre Lison (plison@dfki.de)
 * @version	10.05.2010
 * @started	13.03.2010
 */

module de {
module dfki {
module lt {
module tr {
module beliefs {
module slice { 

// ================================
// SPATIO-TEMPORAL FRAME
// ================================


module framing {

/**
 * Abstract class for a spatio-temporal frame
 */
class AbstractFrame { };
 

/**
 * A temporal interval defined by a start time and an
 * end time 
 */
class TemporalInterval {
};
	

/**
 * A spatio-temporal frame defined by the conjunction of 
 * 1) a place identifier,
 * 2) a temporal interval
 */
class SpatioTemporalFrame extends AbstractFrame {
	string place;
	TemporalInterval interval;
	float existProb;
};



}; 
// end spatiotemporalframe

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
["java:type:java.util.LinkedList<String>"] sequence<string> Agents;

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

};  
// end epstatus

// ================================
// EPISTEMIC OBJECTS
// ================================

module epobject {

/**
 * Abstract class for epistemic objects 
 */
class EpistemicObject {
	framing::AbstractFrame frame;
	epstatus::EpistemicStatus estatus;
};


}; 
// end epobject



// ================================
// FORMULAE
// ================================

module logicalcontent {

/**
 * Abstract class for a formula
 */
 class dFormula { 
 	int id;  
	// nominal (cf. hybrid logic)
 };


/**
 * Elementary formula consisting of a single
 * logical proposition
 */
class ElementaryFormula extends dFormula {
	string prop;
};

class GenericPointerFormula extends dFormula {
      string pointer;
};

class BooleanFormula extends dFormula {
	bool val;
};

class IntegerFormula extends dFormula {
	int val;
};

class FloatFormula extends dFormula {
	float val;
};

class UnknownFormula extends dFormula { };

class UnderspecifiedFormula extends dFormula {
	string arglabel;
};



/** 
 * Formula consisting in the negation of another
 * formula
 */
class NegatedFormula extends dFormula {
	dFormula negForm;
};


/**
 * Formula consisting in a modal operator <op>
 * applied on another formula
 */
class ModalFormula extends dFormula {
	string op;
	dFormula form;
};


/**
 * A sequence of formulae
 */
["java:type:java.util.LinkedList<dFormula>"] sequence<dFormula> FormulaSeq;


/**
 * Possible binary operators: ^ and v 
 */
enum BinaryOp {conj, disj};


/**
 * A complex formula is defined as a sequence of formulae
 * which are combined via a binary operator (^ or v)
 */ 
class ComplexFormula extends dFormula {
	FormulaSeq forms;
	BinaryOp op;	
};


}; 
// end formulae



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
 * Abstract class for expressing the alternative values contained
 * in a distribution
 */
class DistributionValues { };


/**
 * A basic probability distribution, consisting of a key ("label")
 * associated with a set of alternative values
 */
 
class  BasicProbDistribution extends ProbDistribution {
	string key;
	DistributionValues values;
};

/**
 * map of probability distributions
 */
dictionary<string,ProbDistribution> Distributions;

/**
 * sequence of probability distributions
 */
["java:type:java.util.LinkedList<ProbDistribution>:java.util.List<ProbDistribution>"] sequence<ProbDistribution> DistributionList;


/**
 * Discrete probability distribution for P(X1...Xn) where each 
 * random variable Xi is conditionally independent on the other,
 * i.e. P(X1...Xn) = P(X1)...P(xn)
 */
class CondIndependentDistribs extends ProbDistribution {
	Distributions distribs;
};


/**
 * Discrete probability distribution for P(X1...Xn) where each 
 * random variable Xi is conditionally independent on the other,
 * i.e. P(X1...Xn) = P(X1)...P(xn)
 */
class CondIndependentDistribList extends ProbDistribution {
	DistributionList distribs;
};

struct FormulaProbPair {
	logicalcontent::dFormula val;
	float prob;
};

    
["java:type:java.util.LinkedList<FormulaProbPair>:java.util.List<FormulaProbPair>"] sequence<FormulaProbPair> FormulaProbPairs;


/**
 * 
 */
class FormulaValues extends DistributionValues {
	FormulaProbPairs values;
};
	
 
/**
 * Continuous probability distribution P(X) defined on the value
 * of a particular feature.  The distribution P(X) is a normal
 * distribution with mean and variance as parameters
 */
class NormalValues extends DistributionValues {
	double mean;
	double variance;
};
 
}; 
// end distributions



// ================================
// BELIEF DEVELOPMENT HISTORY
// ================================

module history {

/**
 * Abstract class specifying the history of a given belief
 */
class AbstractBeliefHistory { };

}; 
// end history


// ================================
// BELIEF
// ================================

module sitbeliefs {

/**
 * Definition a situated belief, consisting of an identifier, a spatio-temporal
 * frame, an epistemic status, an ontological type, a belief development
 * history, and a probabilistic content
 */ 
class dBelief extends epobject::EpistemicObject {
	string id;
	string type;
	distribs::ProbDistribution content;
	history::AbstractBeliefHistory hist; 
};

}; 
// end sitbeliefs


// ================================
// INTENTIONS
// ================================
 
module intentions {

class IntentionalContent {
	epstatus::Agents agents;
	logicalcontent::dFormula preconditions;
	logicalcontent::dFormula postconditions;
	float probValue;
};

["java:type:java.util.LinkedList<IntentionalContent>"] sequence<IntentionalContent> AlternativeIntentions;


class Intention extends epobject::EpistemicObject {
	string id;
	AlternativeIntentions content;
};

}; 
// end intentions

// ================================
// EVENTS
// ================================
 
module events {

class Event extends epobject::EpistemicObject {
	string id;
	distribs::ProbDistribution content;
};

}; 
// end events

// ================================
// END SLICE
// ================================

}; 
}; 
}; 
};
}; 
}; 

#endif
