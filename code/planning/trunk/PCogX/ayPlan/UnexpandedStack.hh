// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.
#ifndef UNEXPANDEDSTACK_HH
#define UNEXPANDEDSTACK_HH

#include "StateEvaluation.hh"

namespace Planning
{
    /* Stack of states that have yet to be expanded by a state-based
     * planner.*/
    template<typename StateType, typename StateEvaluatorType>
    class UnexpandedStack
    {
    public:
	UnexpandedStack();
	
	/*Function used to evaluate a state.*/
	StateEvaluatorType stateEvaluator;

	/*Costs associated with unexpanded states.*/
	std::set<uint> costs;

	//typedef std::map<StateType*, uint> StatePointerToIndex;
 	typedef tr1::unordered_map<StateType*, uint, boost::hash<StateType*> > StatePointerToIndex;

	//typedef std::map<uint, vector<StateType*> > CostToStates;
 	typedef tr1::unordered_map<uint, vector<StateType*>, boost::hash<uint> > CostToStates;

	/*Remove the \argument{state} from the expansion stack.*/
	void remove(StateType* state);

	/*If the \argument{state} in this stack?*/
	bool contains(StateType* state) const;
	
	/*Are there no states for expansion?*/
	bool empty() const;
	
	/*Add a state to the expansion stack.*/
	void push_back(StateType*);

	/*Pick a state to expand.*/
	StateType* pop();

	/*How many elements are in the unexpanded stack?*/
	uint size() const;// {return count;};
    protected:
	/*How many elements are in the unexpanded stack?*/
	uint count;
	
	/*Index of a state */
	StatePointerToIndex stateIndex;

	/*Each stack of states has a cost that corresponds to the
	 *domain element that maps to it.*/
	CostToStates stacks;
    };
}


#endif

/*
  
  \begin{quote}
  
  ... Before the war <<WW2>> he had done it to Mauriac. The great
  Catholic writer had just published {\em La Fin De La Nuit}. It was a
  classic. It was praised to the skies by the critics. Well received by
  readers. In hindsight, it is one of Mauriac's really fine novels. Then
  along came Sartre, who, in an article in the {\em Nouvelle Revue
  Francais}, explained that the whole art of the older writer rested on
  the postulate that the writer `plays the same role towards his
  creatures as God does towards his`, and that he, Sartre, was in a
  position to assert, from the heights of the legitimacy that the
  publication of {\em Nausea} alone, a year earlier, had just granted
  him, that what we had here was an unforgivable `technical error`
  compounded by a philosophical blunder; the novelist `is not God`; he
  `does not have the right to pronounce his absolute judgements` or to
  manipulate his characters as puppets in the way Mauriac did; he does
  not have the right to enter into their reasons or their unreasons,
  their paltry contradictions so obviously strung together by the
  omniscient narrator's designs on them, and so here is the verdict he
  felt obliged to deliver: {\em La Fin De La Nuit} `is not a novel`, for
  `God is not an artist and neither is Monsieur Mauriac`. The victim
  swallows the insult. He didn't respond, he took it on the chin, and,
  playing a close game, didn't disdain, as the years went by -- for
  example, at the time of Sartre's preface to {\em Aden Arabie} -- to
  pay homage to his adversary's talent. But he waited thirty years
  before risking publishing a new novel. Sartre, he said, tried `from
  the outset, twenty years ago, [...] to throttle me`. And, on another
  occasion, in answer to a journalist from {\em France-Soir} who asked
  him what was the reason behind such a long silence, he replies: {\em
  La Fin De La Nuit} had been `panned by Sartre`, who was `not only a
  very young author, but at the same time the glory of his generation`;
  and without going so far as to say that this `attack` demoralised him,
  he admitted that it had `made me think`. Sartre was to regret the way
  he had panned the novel. He later agreed, in 1960, that `all methods
  are a form of rigging`, and that the celebrated `American methods`
  that he had claimed, at the time, to be upholding in opposition to
  Mauriac's literary interventionism were scarcely less artificial. But
  that's how it was, in the final analysis. It was a period when a
  literary panning, published in a review, could give you something to
  think about for thirty years. A period, in 1939, the middle of the
  twentieth century, when Sartre, still so very young, already had this
  power to destroy a novelist...
  
  \end{quote}
  
  -- Bernard-Henri L\'evy, Sartre: The Philosopher of the Twentieth
     Century, Polity, 2003.


  
 */
