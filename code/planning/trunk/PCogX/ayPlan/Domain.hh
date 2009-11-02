// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef DOMAIN_HH
#define DOMAIN_HH

#include"global.hh"
#include "PredicatesAndPropositions.hh"
#include "Action.hh"
#include "Functions.hh"

namespace Planning
{
    /* Propositional PDDL domain with action costs (see \class{Problem}
     * for instances).*/
    class Domain
    {
    public:
	Domain();
	
	friend ostream& operator<<(ostream& , const Domain& );
	
	/*Domains have a string identifier.*/
	string name;

	/*\class{PredicateName} symbols (see \module{PredicatesAndPropositions}).*/
	SetOfPredicateNames predicates;

	/*Specification for domain predicates (see \module{PredicatesAndPropositions}).*/
	PredicateNameSpecifications predicateSpecifications;

	/*Domain actions (see \module{Action}).*/
	vector<Action<> > actions;

	/*List of functions mapping to integers (IPC-6, UNUSED).*/
	PositiveIntegerFunctions positiveIntegerFunctions;
	IntegerFunctions integerFunctions;
	RealFunctions realFunctions;

	/*Template argument determines what list
	 * (\member{positiveIntegerFunctions},
	 * \member{integerFunctions}, \member{realFunctions}, ...)
	 * this function is added too.
	 */
// 	template<typename T>
// 	void addFunction(const PredicateName&, const Parameters&, T* _NA_ = 0);//{};

	/* Related to \members{positiveIntegerFunctions,
	 * integerFunctions, realFunctions}(IPC-6 UNUSED).*/
	template < typename T >
	void addFunction(const PredicateName&, const Parameters&);
	
	/* Set of \class{PredicateName}s for symbols that do not appear in the
	 * add or delete effects of an action.*/
	SetOfPredicateNames constantSymbols;
	
	/* Set of \class{PredicateName}s for symbols that only appear
	 * in the add (resp. delete) effects of actions.*/
	SetOfPredicateNames increase_oneTimeFluentSymbols;
	
	/* Set of  \class{PredicateName}s for symbols that only appear in the
	 * delete (resp. add) effects of actions.*/
	SetOfPredicateNames decrease_oneTimeFluentSymbols;

	/* Set of \class{PredicateName}s for symbols that appear both
	 * in the add and delete effects of actions.*/
	SetOfPredicateNames manyTimeFluentSymbols;

	/* Is the argument a predicate constant (i.e., non-fluent).*/
	inline bool isConstant(const PredicateName& predicateName) const
	{return constantSymbols.find(predicateName) != constantSymbols.end();};
	template<typename T>
	inline bool isConstant(const Predicate<T>& predicate) const
	{return isConstant(predicate.getName());};

	/* Is the argument a predicate whose truth value can vary both
	 * to and from the positive.*/
	inline bool isFluent(const PredicateName& predicateName) const
	{return manyTimeFluentSymbols.find(predicateName) != manyTimeFluentSymbols.end();};
	template<typename T>
	inline bool isFluent(const Predicate<T>& predicate) const
	{return isFluent(predicate.getName());};

	/* Is the argument a predicate whose truth value can vary only
	 * to the negative.*/
	inline bool isFluentToNegative(const PredicateName& predicateName) const
	{return decrease_oneTimeFluentSymbols.find(predicateName) != decrease_oneTimeFluentSymbols.end();};
	template<typename T>
	inline bool isFluentToNegative(const Predicate<T>& predicate) const
	{return isFluentToNegative(predicate.getName());};

	/*Is the argument a predicate whose truth value can vary only
	 * to the positive.*/
	inline bool isFluentToPositive(const PredicateName& predicateName) const
	{return increase_oneTimeFluentSymbols.find(predicateName) != increase_oneTimeFluentSymbols.end();};
	template<typename T>
	inline bool isFluentToPositive(const Predicate<T>& predicate) const
	{return isFluentToPositive(predicate.getName());};
	
	
	
	/*******************************************************
	 *
	 * Constants, how they relate to types, and how types relate to
	 * them.
	 *
	 *******************************************************/
	
	/* Specification for domain constants.*/
	TypeOfSymbols* parsed_Constants;

	/* Constants (a constant "id" is its index in \member{constants}).*/
	Constants constants;
	
	/* LHS : string (likely a derivative), RHS : int. RHS is the
	 * index of LHS in \member{types}*/
	MapStringToInt constantIndex;

	/* Maps a type (as integer; see \member{types}) to
	 * objects (as integer; see \member{constants}).*/
	MapIntToInts typesToConstants;
	
	/* Maps a constant (as integer; see \member{constants}) to types
	 *  (as integer; see \member{types}).*/
	MapIntToInts constantsToTypes;

	/* Compact \member{typesToConstants} mirror.*/
	vector<vector<uint> > mirror_typesToConstants;
	
	/* Compact \member{typesToConstants} mirror.*/
	vector<vector<uint> > mirror_constantsToTypes;

	
	
	/* Configure \member{constants}, \member{constantIndex}, and
	 * \member{typesToConstants} from \member{parsed_Constants}.*/
	void configureConstants();

	/* (C is for constant, T is for type) :: C1 may be \subseteq
	 * T2 may be \subseteq T3. Hence C1 \subseteq T3. We want that
	 * reasoning applied to \member{typesToConstants}. NOTE(1)::
	 * Call before \member{unwindConstants()}. NOTE(2)::
	 * Configures \member{constantsToTypes}.*/
	void unwindConstants();
	
	/*******************************************************
	 *
	 * Types, and how types relate to types.
	 *
	 *******************************************************/
	
	/* Specification of domain types.*/
	TypeOfTypes* parsed_Types;

	/* Types (a type "id" is its index in \member{types}).*/
	Types types;
	
	/* LHS : string (likely a derivative), RHS : int. RHS is the
	 * index of LHS in \member{types}*/
	MapStringToInt typeIndex;
	
	/* Maps a type (as integer; see \member{types}) to
	 * objects (as integer; see \member{types}).*/
	MapIntToInts typesToTypes;
	
	/* Compact \member{typesToTypes} mirror.*/
	vector<vector<uint> > mirror_typesToTypes;

	/* Configure \member{types}, \member{typeIndex} and
	 * \member{typesToTypes} from \member{parsed_Types}.*/
	void configureTypes();

	/* (T is for Type). T1 may be \subseteq T2 may be \subseteq
	 * T3. Hence T1 \subseteq T3. We want that reasoning applied
	 * to \member{typesToTypes}. NOTE:: Call before
	 * \member{unwindConstants()}.*/
	void unwindTypes();


	/* Initialised all member mirrors:
	 * \member{mirror_typesToTypes},
	 * \member{mirror_objectsToConstants}, and
	 * \member{mirror_typesToConstants}.*/
	void initialiseMirrors();
	
	/* Adds an action.
	 *
	 * \argument{name} -- name of the action
	 *
	 * \argument{arguments} -- action arguments (symbols) and
	 * their type. Although the order isn't really important
	 * (i.e. as far as I can tell it has no semantic meaning in
	 * the spec. of PDDL), it does remain important for the
	 * competition (IPC-6) because the plan checkers assume that
	 * the arguments are given in order. So, for example, if my
	 * planner describes an action, it mustn't say: action = Move,
	 * block1 = A, block1 = B. Rather we have Move(A, B). The
	 * competition plan checker decides block1=A and block2=B from
	 * this.
	 *
	 * \argument{precondition} -- action's precondition.
	 *
	 * \argument{effects} -- action's effects
	 */
	void addAction(const string& name,
		       const Arguments& arguments,
		       const SignedPredicates& precondition,
		       const SignedPredicates& effects);//const AddAndDeleteList& effects);
	
	/*As above, only takes an extra parameter that is the cost of the action.*/
	void addAction(const string& name,
		       const Arguments& arguments,
		       const SignedPredicates& precondition,
		       const SignedPredicates& effects,
		       int cost);

	/*As above, only this time the action cost parameter is
	 * determined by a function evaluation encapsulated by
	 * \argument{costEvaluator}.*/
	void addAction(const string& name,
		       const Arguments& arguments,
		       const SignedPredicates& precondition,
		       const SignedPredicates& effects,
		       SignedPredicate* costEvaluator);
	
	
	/* Processes all the parsed actions, extracting
	 * \member{constantSymbols}, \member{oneTimeFluentSymbols},
	 * and \member{manyTimeFluentSymbols}. NOTE: This should be
	 * called after the parse is complete. We assume that
	 * \member{predicates} and \member{actions} are complete.*/
	 void processActionsForSymbolTypes();
	
	/*Adds typed constants to the domain. A typed constant is the
	 * argument to a unary predicate, or otherwise is a 0-ary
	 * predicate. Like the table in blocks world for example.*/
	void setConstants( TypeOfSymbols*);

	void setTypes( TypeOfTypes*);
	
	/* There are many features in the latest round of PDDL which _may_
	 * be active in a particular domain specification. These are the
	 * boolean flags that I have taken from the Srathclyde Planning
	 * Group's VAL package. */
	class Requires
	{
	public:
	
	    Requires()
		:equality(false),
		 strips(false),
		 typing(false),
		 disjunctivePreconditions(false),
		 existentiallyQuantifiedPreconditions(false),
		 universallyQuantifiedPreconditions(false),
		 conditionalEffects(false),
		 fluents(false),
		 durativeActions(false),
		 time(false),
		 durationInequalities(false),
		 continuousEffects(false),
		 negativePreconditions(false),
		 derivedPredicates(false),
		 timedInitialLiterals(false),
		 preferences(false),
		 constraints(false),
		 actionCosts(false)
	    {};
	
	
	    /*Requires equality testing?*/
	    bool equality;
	
	    /*Is a strips domain?*/
	    bool strips;
	
	    /*Has types?*/
	    bool typing;
	
	    /*Uses disjunctive preconditions?*/
	    bool disjunctivePreconditions;

	    /*Existentially quantified preconditions?*/
	    bool existentiallyQuantifiedPreconditions;
	
	    /*Universally quantified preconditions?*/
	    bool universallyQuantifiedPreconditions;

	    /*Conditional effects?*/
	    bool conditionalEffects;

	    /* FIX :: I'm not sure what this is about, I have to check
	     * the PDDL documentation.*/
	    bool fluents;

	    /*Can actions be durative?*/
	    bool durativeActions;

	    /*Is there "time" in this domain?*/
	    bool time;

	    /*Are durational inequalities used?*/
	    bool durationInequalities;

	    /*Do actions have continuous effects?*/
	    bool continuousEffects;

	    /*Do actions have negative preconditions?*/
	    bool negativePreconditions;

	    /*Do we use derived predicates?*/
	    bool derivedPredicates;

	    /*Are there timed initial literals?*/
	    bool timedInitialLiterals;

	    /*Do we have preferences?*/
	    bool preferences;

	    /*Are there constraints?*/
	    bool constraints;

	    /*Is there a cost to executing actions?*/
	    bool actionCosts;
	}requires;
    };

    

    //template<>
    //void Domain::addFunction<int>(const PredicateName&, const Parameters&, int* _NA_ = 0){}
    
    ostream& operator<<(ostream& o, const Domain&);
}


#endif
