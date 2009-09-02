// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.
#ifndef GROUNDACTIONWITHCOST_HH
#define GROUNDACTIONWITHCOST_HH

#include "GroundAction.hh"

namespace Planning
{
    /* A \parent{GroundAction} with a cost. Comparison operators and
     * hashing is partly determined by the cost of executing the
     * action, and is otherwise determined by \parent{GroundAction} hashing and
     * comparators.*/
    class GroundActionWithCost : public GroundAction
    {
    public:
	GroundActionWithCost(const string& actionName,
			     const Constants& argument,
			     const SignedPredicates& actionPrecondition,
			     const SignedPredicates& actionEffects,
			     const VariableToConstant& variableToConstant,
			     const Action<>& actionSchema);
	
	bool operator==(const GroundActionWithCost&) const;
	bool operator<(const GroundActionWithCost&) const;
	
	size_t hash_value() const;
	
	/*Cost of executing this action.*/
	uint cost;
    };
    
    std::size_t hash_value(const GroundActionWithCost& );
    ostream& operator<<(ostream&, const GroundActionWithCost&);
}

#endif
