// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.
#include "GroundActionWithCost.hh"

using namespace Planning;

GroundActionWithCost::GroundActionWithCost(const string& actionName,
					   const Constants& argument,
					   const SignedPredicates& actionPrecondition,
					   const SignedPredicates& actionEffects,
					   const VariableToConstant& variableToConstant,
					   const Action<>& actionSchema)
    :GroundAction(actionName,
		  argument,
		  actionPrecondition,
		  actionEffects,
		  variableToConstant,
		  actionSchema),
     cost(actionSchema.cost)
{

}
	
bool GroundActionWithCost::operator==(const GroundActionWithCost& groundActionWithCost) const
{
    return ( cost == groundActionWithCost.cost ) &&
	GroundAction::operator==(groundActionWithCost) ;
}

bool GroundActionWithCost::operator<(const GroundActionWithCost& groundActionWithCost) const
{
    if(cost < groundActionWithCost.cost){
	return true;
    } else if (cost == groundActionWithCost.cost) {
	return GroundAction::operator<(groundActionWithCost);
    }

    return false;
    
//     return GroundAction::operator<(groundActionWithCost)
// 	&& (cost < groundActionWithCost.cost);
}
	
size_t GroundActionWithCost::hash_value() const
{
    size_t tmp = GroundAction::hash_value();
    boost::hash_combine(tmp, boost::hash_value(cost));
    
    return tmp;//boost::hash_combine(GroundAction::hash_value(), boost::hash_value(cost));
}

namespace Planning
{
    std::size_t hash_value(const GroundActionWithCost& groundActionWithCost)
    {
	return groundActionWithCost.hash_value();
    }
    
    ostream& operator<<(ostream& o, const GroundActionWithCost& groundActionWithCost)
    {
	o<<dynamic_cast<const GroundAction&>(groundActionWithCost);
    }
    
}

	
