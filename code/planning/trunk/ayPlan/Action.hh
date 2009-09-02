// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef ACTION_HH
#define ACTION_HH

#include"global.hh"

#include "PredicatesAndPropositions.hh"

namespace Planning
{
    class Domain;/*(see Domain.hh)*/

    /*PDDL action schema.*/
    template<typename BASE = HasStringRepresentation>
    class Action : public BASE
    {
    public:
	Action(const string&,
	       const Arguments&,
	       const SignedPredicates&,
	       const SignedPredicates&,
	       int cost = 0);
	
	Action(const string&,
	       const Arguments&,
	       const SignedPredicates&,
	       const SignedPredicates&,
	       SignedPredicate* costEvaluator);

	const string& getName() const;
	void setName(const string& );
	
	const Arguments& getArguments() const;
	const SignedPredicates& getPreconditions() const;
	const SignedPredicates& getEffects() const;

	/* 1 we can choose, 0 we can not. A variable that we can choose
	 *  is one which does not participate in the argument of a
	 *  constant predicate in the actions precondition. NOTE: we
	 *  assume that \member{variablesInOrder} and
	 *  \member{variableIndex} are built when this is called. If
	 *  this is not the case, calls to
	 *  \member{getVariablesInOrder} and \member{getVariableIndex}
	 *  will configure those structures.*/
	const vector<bool>& getVariableChoices(const Domain&) const;

	/* Variable arguments to action. Makes result if necessary,
	 * and then returns \member{variablesInOrder}.*/
	const Variables& getVariablesInOrder() const;
	
	/* Each variable argument to the action is mapped to an
	 * integer index (see \member{getVariablesInOrder()}). Makes
	 * the result if necessary, and then returns
	 * \member{variableIndex}.*/
	const VariableToUnsignedInt& getVariableIndex() const;

	/* Repeatedly calling this function has it incrementally apply
	 * to \argument{VariableToConstant} the deferent legal
	 * assignments from \argument{vector<Proposition<> >} from
	 * index \argument{lower} to \argument{upper}. When the
	 * assignments have been exhausted, then the return value is
	 * false, otherwise it is true. All assignments are legal, in
	 * the sense that they occur in
	 * \argument{vector<SetOfConstants>}. \argument{predicateIndex}
	 * is the index of \argument{SignedPredicate} as it occurs
	 * parsed into a vector from the original specification of
	 * this action.*/
	bool makePartialAssignment(VariableToConstant& ,
				   const SignedPredicate& , 
				   uint predicateIndex,
				   const vector<Proposition<> >&,
				   uint upper,
				   uint lower,
				   const VariableToUnsignedInt&,
				   const vector<SetOfConstants>&) const;
	
	/* In the case that this action has a constant cost, here we
	 * have that cost (each time) of executing this action.*/
	int cost;

	/* In the case that an action has a variable cost (determined
	 * after instantiation), the symbol that determines that cost
	 * occurs below. (NULL means there is no variable cost)*/
	SignedPredicate* costEvaluator;
    protected:
	
	/* 1 we can choose, 0 we can not. A variable that we can choose
	 * is one which does not participate in the argument of a
	 * constant predicate in the actions precondition.*/
	mutable vector<bool> choiceVariables;//(variablesInOrder.size());
	
	/* A vector of the variable names in the order in which they
	 * were parsed.*/
	mutable Variables variablesInOrder;//(getArity(_arguments));
	
	/* Index associated with a variable in \member{arguments} and
	 * \local{variablesInOrder}.*/
	mutable VariableToUnsignedInt variableIndex;

	void _configureVariablesInOrderAndVariableIndex() const;
	
	
	/* Action as a string.*/
	void computeAsString(const string& str) const;
    
	/* Action identifier.*/
	string name;

	/* Action arguments.*/
	Arguments arguments;

	/* Action precondition*/
	SignedPredicates preconditions;

	/* Action effects.*/
	SignedPredicates effects;
    };
}


#endif
