// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef PLANNER_HH
#define PLANNER_HH

#include "Problem.hh"
#include "Action.hh"
#include "GroundAction.hh"
#include "State.hh"
#include "UnexpandedStack.hh"

namespace Planning
{
    /* A \class{Planner} does a forward search for the goal (see
     * \member{mustBeTrue} and \member{mustBeFalse}) in the
     * state-space \member{states}.
     *
     * NOTE: I assume that there is one \class{Planner} per process.*/
    template<typename StateType = State<GroundAction>, typename GroundActionType = GroundAction>
    class Planner
    {
    public:
	
	/*States..*/
	typedef tr1::unordered_set<StateType, boost::hash<StateType> > SetOfStates;
	
	/*State pointers.*/
	typedef tr1::unordered_set<StateType*, /*state_hash*/deref_hash<StateType>,  deref_equal_to<StateType> > SetOfStatePointers;
    
	/*Action indices.*/
	typedef tr1::unordered_map<GroundActionType, uint, boost::hash<GroundActionType> > GroundActionToInt;

	
	/* Calls \member{make_*}. Also configures:
	 *
	 * \static{StateType::fluentPropositions},
	 * \static{StateType::plannerActions},
	 * \static{StateType::propositionIndices},
	 * \static{StateType::problem}, and
	 * \static{StateType::propositions}
	 */
	Planner(Problem&);

	/*(some member functions are virtual so we need a virtual
	 * destructor).*/
	virtual ~Planner();
	
	/* Expand \argument{State}. Returns true (i.e., non-null) if one of
	 * the states expanded is the goal. NOTE: The argument is not
	 * a _constant_ reference, because oftentimes the expansion
	 * updates details of the state it is expanding, to inform
	 * that state of information concerning its successors.*/
	StateType* expand(StateType&);
	
	/* Base behaviour of this function selects an element from
	 * \member{unexpanded}.*/
	StateType* chooseState();

	/* Does the argument satisfy the goal conditions (see
	 * \member{mustBeTrue} and \member{mustBeFalse}).*/
	bool isGoalState(const StateType&) const;
	
	/* Plans by repeatedly expanding the state returned by
	 * \member{chooseState()} until the goal is achieved or
	 * \member{unexpanded} is exhausted.*/
	void operator()();
	
	/*       PROPOSITIONS         */	
	
	/*Set of domain propositions.*/
	vector<Proposition<> > propositions;
	
	/* LHS : string name of predicate symbol, RHS : is the lower
	 * and upper index bound of instances of LHS in
	 * \member{propositions}.*/
	MapStringToPairInt propositionIndices;

	/*Index into \member{propositions} for LHS Proposition.*/
	PropositionToInt propositionIndex;
	
	/* Set of fluent propositions -- I.e., what propositions
	 * actually uniquely characterise a planning state for the
	 * given \member{startingState} and \member{problem.goal}.*/
	vector<Proposition<> > fluentPropositions;
	MapStringToPairInt fluentPropositionIndices;
	PropositionToInt fluentPropositionIndex;

	/* Remove from \members{fluentPropositions,
	 * fluentPropositionIndices, fluentPropositionIndex}
	 * items whose index is not in the  argument set.*/
	void removeFluents(set<uint> allowedFluentIndices)  ;
	
	/*        GROUND ACTIONS         */
	
	/*Every ground action available in the problem.*/
	vector<GroundActionType> actions;
	
	/* LHS : string name of action symbol, RHS : is the lower
	 * and upper index bound of instances of LHS in
	 * \member{actions}*/
	MapStringToPairInt actionIndices;
	
	/*Index into \member{actions} for LHS GroundAction.*/
	GroundActionToInt actionIndex;

	/* Remove from \members{actions, actionIndices, actionIndex},
	 * actions with indices not from the argument set.*/
	void removeActions(set<uint> allowedActionIndices);
    
	/* If a \member{goalState} is found (non-null) when
	 * \method{operator()} terminates, then this method will yield
	 * a plan. NOTE: the order is reversed as the plan is read
	 * from the \member{goalState} back to the
	 * \member{startingState}.*/
	virtual const vector<GroundActionType>& getPlan();


	/*What \class{Problem} was this \class{Planner} built to solve?*/
	inline const Problem& getProblem() const{return problem;};
	

	/*       STATE SPACE         */

	
	/* States in the search space that have not been "expanded"
	 * (i.e., if $s$ is a state, then it has not been "expanded"
	 * if expand(s) has not been called). */
	UnexpandedStack<StateType, GoalProximityEvaluator<StateType> > unexpanded;

	/*Problem states that have been visited by the planner.*/
	SetOfStatePointers states;

	/*Starting state.*/
	const StateType& getStartingState() const {return startingState;}

	/*Positive part of the goal. NOTE: As far as I know, IPC-6
	 * doesn't have negative conditions in the goal.*/
	const vector<uint>& getMustBeTrue() const {return mustBeTrue;}
    protected:
	/*Can we consider the argument for expansion?*/
	virtual bool considerStateForExpansion(StateType* successor) ;
	
	/* Call this function when we have expanded
	 * \argument{currentState} and generated
	 * \argument{successor}. The latter is logically equivalent to
	 * \argument{oldSuccessor}. State \argument{oldSuccessor} can
	 * already be found in \member{states}. This function decides
	 * whether or not the new-means of achieving
	 * \argument{oldSuccessor} means it should be considered for
	 * entry into \member{unexpanded}.*/
	virtual bool reconsiderStateForExpansion(StateType* successor,
						 StateType* currentState,
						 StateType* oldSuccessor) ;

	/*Is the argument state to be considered a candidate for expansion?*/
	virtual bool redraw(StateType*) {return false;};
	
	/*When the goal is achieved, the goal state is stored here.*/
	StateType* goalState;

	/*The planner produces a plan -- A (reversed) sequence of
	 * actions that yield the \member{goalState}.*/
	vector<GroundActionType> plan;
	
	/*Number of bits required to represent a \member{problem}
	 * state.*/
	uint stateSize;
	
	/*Planning problem (see \module{Problem}).*/
	Problem& problem;

	/*Initial state.*/
	StateType startingState;

	/* In the goal state, some propositions must be true, and
	 * others must be false.*/
	vector<uint> mustBeTrue;
	vector<uint> mustBeFalse;

	/*Bit vector, where every 1 entry means the corresponding
	 * proposition has to be true, and contrariwise (i.e. 1 means
	 * false in the case of \member{goalFalse}.).*/
	vector<ELEM_TYPE> goalTrue;
	vector<ELEM_TYPE> goalFalse;

	/*(see \member{propositions}, \member{propositionIndices}, and
	 *\member{propositionIndex})*/
	void make_propositions();

	/* Before we make the starting state, and before the search for
	 * the goal commences, we make a new set of
	 * \member{fluentPropositions} from \local{propositions} that
	 * are invariant -- i.e., invariant according to
	 * \class{Domain} instance
	 * \member{problem.domain.constantSymbols}.*/
	void prune_propositions();
	
	/* Recursively generates all the possible groundings of the
	 * symbol whose first arguments can take values in
	 * \argument{vector<Constants >}[0], etc... Each is stored in
	 * \member{propositions}. Thus, each is a grounding of a
	 * proposition.*/
	void addGroundings(uint,
			   Constants&,
			   const vector<Constants >&,
			   const PredicateName&);
	
	/*Call when a predicate is constant. Adds to
	 * \member{propositions} all ground instances of the
	 * \argument{PredicateName} that occur in the start state.*/
	void addGroundingsFromStartState(const PredicateName&);

	/* As above only for \member{actions}. 1st and 2nd arguments
	 * are indices into \argument{Constants}, and the constants
	 * that make up an action argument respectively (the latter is
	 * built by recursive calls) -- i.e. Usage should be, 1st
	 * argument $0$, second argument a list of \type{Constants} of
	 * the same length as \argument{Variables}. 3rd argument is a
	 * mapping from a variable name to a constant name. 4th
	 * argument is the list of variable names, in order, that
	 * correspond to the action arguments. 5th argument, as for
	 * \member{addGroundings()} for propositions, is a list of all
	 * the constants that each variable in \argument{Variables}
	 * can take.  6th, 7th, 8th, arguments are the action name,
	 * preconditions, and effects. 9th argument is the action
	 * specification that a ground action built by this function
	 * is an instance of.*/
	void addActionGroundings(uint ,
				 Constants& ,
				 VariableToConstant& ,
				 const Variables& ,
				 const vector<Constants >& ,
				 const string& ,
				 const SignedPredicates& ,
				 const SignedPredicates& ,
				 const Action<>& );

	/*As above, only respecting the allowable action arguments
	 *given the constant (non-fluent) predicates that occur as
	 *preconditions.*/
	void addActionGroundings(set<pair<uint, uint> >::const_iterator,
				 Constants& ,
				 VariableToConstant& ,
				 const Variables& ,
				 const VariableToUnsignedInt& ,
				 vector<Constants >& ,
				 const Action<>&,
				 const SignedPredicates& ,
				 const SignedPredicates& ,
				 const set<pair<uint, uint> >& ,
				 const vector<bool>&,
				 const vector<SetOfConstants>&);
	
	void make_actions();
	void make_startingState();
	void make_goalState();
	void make_goalEvaluator();
	
	/*Ensure every proposition that is mentioned in the
	 * precondition of an action, knows of that action. The result
	 * of this computation is stored in
	 * \module{State::propositionsAllowsActions}*/
	void make_setActionPossibilities();
    };


    template<typename StateType, typename GroundActionType>
    ostream& operator<<(ostream&, const Planner<StateType, GroundActionType>&);
}

#endif


/*

  \begin{quote}
  
  There is a time when the operation of the machine becomes so odious,
  makes you so sick at heart, that you can't take part; you can't even
  tacitly take part, and you've got to put your bodies on the gears and
  upon the wheels, upon the levers, upon all the apparatus and you've
  got to make it stop.  And you've got to indicate to the people who run
  it, to the people who own it, that unless you're free, the machine
  will be prevented from working at all.
  
  \end{quote} 

  -- Mario Savio (1942..1996), Sproul Hall, University of California,
     Berkeley, December 2, 1964. NOTE: This quote features at the
     start of tune The Movies Over, COG, (2008).
 

 */
