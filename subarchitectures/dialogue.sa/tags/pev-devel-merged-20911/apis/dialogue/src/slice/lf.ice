// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
// Pierre Lison (pierre.lison@dfki.de)
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

#ifndef LF_ICE
#define LF_ICE

// ===================================================================
// MODULE: de.dfki.lt.tr.dialogue.slice.lf
// 
// Defines the data structures for logical forms: 
// - logical form: LogicalForm, based on LFNominal and LFRelation 
// - packed logical form: PackedLogicalForm   
//
// @author Geert-Jan M. Kruijff	<gj@dfki.de>
// @author Pierre Lison	<pierre.lison@dfki.de>
//
// ===================================================================

module de {
module dfki {
module lt { 
module tr { 
module dialogue { 
module slice { 
module lf { 

// -------------------------------------------------------------------
// A connective can be none (atomic proposition), "DISJUNCTIVE" (OR), 
// XDISJUNCTIVE ("XOR"), or CONJUNCTIVE ("AND"). 
// -------------------------------------------------------------------

enum ConnectiveType { NONE, DISJUNCTIVE, CONJUNCTIVE, XDISJUNCTIVE }; 


// -------------------------------------------------------------------
// A proposition is an atomic structure with an associated connective. 
// Complex propositions are built as lists over such propositions. 
// -------------------------------------------------------------------

class Proposition { 
	string prop;
	ConnectiveType connective;
}; 
// end Proposition

// sequence type declaration

sequence<Proposition> Propositions; 

// -------------------------------------------------------------------
// A feature is a relation with an atomic propositional value
// -------------------------------------------------------------------

class Feature { 
	string feat;
	string value;
}; 

// sequence type declaration

sequence<Feature> Features;

// -------------------------------------------------------------------
// A relation is a sorted modal relation between two nominal variables. We have 
// a boolean flag for whether the dependent "dep" is co-indexed. If we have at
// least one co-indexed dependent in a logical form, the logical form is a graph, 
// else it is a tree. 
// -------------------------------------------------------------------

class LFRelation {
	string head;
	string mode;
	string dep;
	bool coIndexedDep;
}; 
// end LFRelation

// sequence type declaration

sequence<LFRelation> Relations;

// -------------------------------------------------------------------
// A logical form nominal structure is an object representing a formula of the 
// form @_{nomVar:sort}(FORMULA), with FORMULA being a (possibly complex) proposition, 
// zero or more features (of the formal form <Attribute>Value) and zero or more 
// relations (of the formal form <Relation>(nomVar_n:sort_n), with nomVar_n an
// identifiable variable in the namespace of the logical form).   
// -------------------------------------------------------------------

class LFNominal { 
	string nomVar;
	string sort; 
	Proposition prop;
	Features feats;
	Relations rels;
}; 
// end LFNominal

// sequence type declaration

sequence<LFNominal> Nominals;

// -------------------------------------------------------------------
// A logical form is a uniquely identifiable collection of nominals. 
// -------------------------------------------------------------------

class LogicalForm { 
	string logicalFormId;
	Nominals noms;
	float preferenceScore;
	LFNominal root;
	long stringPos;
}; 
// end LogicalForm


// -------------------------------------------------------------------
// A component is a named logical form
// -------------------------------------------------------------------

class LFComponent { 
	string componentName;
	LogicalForm lf; 
}; 
// end LFComponent

// sequence type declarations

sequence<LFComponent> LFComponents;
sequence<LFComponent> LFComponentsVector;
sequence<LogicalForm> LogicalFormsVector;

// -------------------------------------------------------------------
// A packed logical form (PackedLogicalForm) is composed of:
// - an identifier ;
// - a collection of packing nodes ;
// - a reference to the root packing node.
//
// A packing node (PackingNode) is composed of:
// - an unique identifier ;
// - a collection of packed nominals ;
// - a reference (identifier) to the root packed nominal ;
// - a set of LF identifiers specifying the logical forms in which the
//   nominals in the packing node occur ;
// - a set of packing edges connected to one nominal of the packing node;
// - a preference score, ie. a number n, with 0 <= n <= 1.
//  
// A packing edge (PackingEdge) is composed of:
// - an unique identifier ;
// - an edge label ;
// - a reference to the nominal from which the edge originates ;
// - a collection of pairs <packing node, LF identifiers> describing the set of
//   possible packing nodes as target for the edge, together with the LF identifiers
//   in which this connection occur;
// - a boolean flag indicating whether the edge if coindexed.
// - a preference score,  ie. a number n, with 0 <= n <= 1.
//   
// A packed nominal (PackedNominal) is composed of:
//  - an unique identifier ;
//  - a packed ontological sort ;
//  - a proposition ;
//  - a set of packed features ;
//  - a set of relations internal to the packing node.
//  
// A packed feature (PackedFeature) is composed of:
// - a feature name ;
// - a feature value ;
// - a set of logical form identifiers in which the feature value occurs.
// 
// A packed ontotological sort (PackedOntologicalSort) is composed of:
// - an ontological sort ;
// - the set of identifiers in which this sort occur.
// -------------------------------------------------------------------

// sequence type declaration

sequence <string> LogicalFormIds; 

	// -----------------------
	// PACKING EDGES
	// -----------------------

	// packing node targets 
	class PackingNodeTarget {
		string pnId ;
		LogicalFormIds lfIds ;
	} ;

	// collection of packing node targets for a given packing edge
	sequence <PackingNodeTarget> PackingNodeTargets ; 

	// packing edge
	class PackingEdge {
		string peId ;
		string mode ;
		string head ;
		PackingNodeTargets targets ;
		bool coIndexedDep ;  
		float preferenceScore ;
	} ;
	
	// collection of packing edges 
	sequence <PackingEdge> PackingEdges ;

	// -----------------------
	//	PACKED FEATURES, SORTS AND NOMINALS	
	// -----------------------
	
	// packed feature 
	class PackedFeature {
		string feat ;
		string value ;
		LogicalFormIds lfIds ;
	};

	// packed ontological sort
	class PackedOntologicalSort {
    		string sort ;
    		LogicalFormIds lfIds ;
	};

 	sequence <PackedOntologicalSort> PackedOntologicalSorts;

	// collection of packed featurs
	sequence <PackedFeature> PackedFeatures ;
	
	// packed nominal
	class PackedNominal {
		string nomVar ;
	 	PackedOntologicalSorts packedSorts ;
		Proposition prop ;
		Relations rels ;
		PackedFeatures feats ;
		PackingEdges pEdges;		
	 } ;
	 
	// collection of packed nominals ;
	sequence <PackedNominal> PackedNominals ;

	// -----------------------
	// PACKING NODES
	// -----------------------

	// <nominalVariable, PackingEdge> pair, indicating a packing
	// edge originating at the given nominal
	class NominalPackingEdgePair {
		string head ;
		PackingEdge pe ;
	} ;

	// 	collection of <nominalVariable, PackingEdge> pairs ;
	sequence <NominalPackingEdgePair> NominalPackingEdgePairs ;

 	// packing node
 	class PackingNode {
		string pnId ; 
 		LogicalFormIds lfIds ;
 		PackedNominals packedNoms ;
		string root ;
		NominalPackingEdgePairs nomsPePairs ;
		float preferenceScore ;
	};
	
	// collection of packing nodes
	sequence <PackingNode> PackingNodes ;


	// -----------------------
	// PACKED LOGICAL FORM
	// -----------------------
	
 	// packed logical form
	class PackedLogicalForm {
		string packedLFId ;
		PackingNodes pNodes ;
		string root ;
	};

}; 
// end module lf
}; 
// end module slice
}; 
// end module dialogue
}; 
// end module tr
}; 
// end module lt
}; 
// end module dfki
}; 
// end module de

#endif



