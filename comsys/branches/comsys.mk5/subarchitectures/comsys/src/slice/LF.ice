#ifndef HWD_ICE
#define HWD_ICE

// ===================================================================
// MODULE: Comsys
// 
// Defines the data structures for logical forms.  
// 
// History: derived from the older LFEssentials and LFPacking IDL files. 
//
// Authors:		Geert-Jan M. Kruijff	<gj@dfki.de>
//				Pierre Lison			<pierre.lison@dfki.de>
//
// TO DO:
//
// 
// For an edit log, see the bottom of this file. 
//
// ===================================================================

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
}; // end Proposition

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
}; // end LFRelation

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
}; // end LFNominal

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
}; // end LogicalForm


// -------------------------------------------------------------------
// A component is a named logical form
// -------------------------------------------------------------------

class LFComponent { 
	string componentName;
	LogicalForm lf; 
}; // end LFComponent

// sequence type declarations

sequence<LFComponent> LFComponents;
sequence<LFComponent> LFComponentsVector;
sequence<LogicalForm> LogicalFormsVector;





}; // end module

#endif



//EDIT LOG
