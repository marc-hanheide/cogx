// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef GROUNDACTION_HH
#define GROUNDACTION_HH


#include "Problem.hh"
#include "Action.hh"

namespace Planning
{
    /* An action that can be executed at a state. This is an instance
     * in the planning/PDDL sense of an \class{Action}.*/
    class GroundAction
    {
    public:
	/* Propositions that have been encountered during construction
	 * on \class{GroundAction} instances.*/
	static SetOfPropositions setOfEncounteredPropositions;
	
	/*Initialises \member{name}.*/
	GroundAction(const string& actionName,
		     const Constants& argument,
		     const SignedPredicates& actionPrecondition,
		     const SignedPredicates& actionEffects,
		     const VariableToConstant& variableToConstant,
		     const Action<>& actionSchema);/* The Sandman says:
						    * "Just because
						    * it's not used,
						    * doesn't mean it's not
						    * needed ;)"*/

	//void initialiseIntegerRepresentation();
	size_t hash_value() const;

	/*Comparison operators are based on \member{name}, so make
	 * sure this is initialised before calling comparators.*/
	bool operator==(const GroundAction&) const;
	bool operator<(const GroundAction&) const;

	/* Each action has two representations, one in terms of the
	 * actual (string represented) propositions that are in the
	 * precondition and postcondition (i.e., the latter is the
	 * \member{_add} and \member{_del}ete lists). The other is in
	 * terms of the integers that have been assigned to the
	 * propositions (and vise versa). This method configures those
	 * integer representations in \member{add}, \member{del}, and
	 * \member{prec} (resp. \member{_add}, \member{_del}, and
	 * \member{_pred}). Symbols that do not occur in the argument
	 * mapping to integers are ignored in the integer
	 * representation. Hence you want to hope they are fluent. If
	 * something occurs in the add or delete list that is not
	 * indexed to an integer, then an assertion will be
	 * violated.*/
	void generateIntegerRepresentation(PropositionToInt& propositionIndex);
	
	/*Add list (STRIPS).*/
	vector<uint> add;
	vector<Proposition<> > _add;
	
	/*Delete list (STRIPS).*/
	vector<uint> del;
	vector<Proposition<> > _del;
	
	/*Precondition (STRIPS).*/
	vector<uint> prec;
	vector<Proposition<> > _prec;

	/*"String"-like representation (see \module{PredicatesAndPropositions}).*/
	Proposition<> name;
    };

    /*Function for \library{STL} and \library{boost} to access
     * \member{hash_value} of \argument{GroundAction}.*/
    std::size_t hash_value(const GroundAction& );

    ostream& operator<<(ostream&, const GroundAction&);
}

#endif

/*

  

  \begin{quote}
  
  In its very truly great manners of Ludwig van Beethoven very
  heroically the very cruelly ancestral death of Sara Powell Haardt
  had very ironically come amongst his very really grand men and women
  to Rafael Sabatini, George Ade, Margaret Storm Jameson, Ford Madox
  Hueffer, Jean-Jacques Bernard, Louis Bromfield, Frie- drich Wilhelm
  Nietzsche and Helen Brown Norden very titanically.
  
  \end{quote}

  -- John Barton Wolgamot (birth & death dates unknown), In Sara,
     Mencken, Christ and Beethoven There Were Men and Women, 1944. The
     work was recorded by Robert Ashley (with Paul DeMarinis) in
     1972. My younger brother got his hands on a copy of the text at a
     second-hand bookshop while touring New Zealand in 2004.

  
 */
