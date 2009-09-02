// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#include "GroundAction.hh"

#include "Planner.hh"

#include "Action_templates.hh"
#include "PredicatesAndPropositions_templates.hh"

using namespace Planning;

#ifndef NATHAN
using namespace tr1;
#endif

SetOfPropositions GroundAction::setOfEncounteredPropositions;

#ifndef NDEBUG
namespace GoundAction_Planner
{    
    void* planner = 0;
}

#endif

GroundAction::GroundAction(const string& name,
			   const Constants& argument,
			   const SignedPredicates& preconditions,
			   const SignedPredicates& effects,
			   const VariableToConstant& variableToConstant,
			   const Action<>& actionSchema)
    :name(PredicateName(const_cast<string&>(name)),argument)
{
#ifndef NDEBUG
    Planner<State<GroundAction>, GroundAction>* planner = reinterpret_cast<Planner<State<GroundAction>, GroundAction>*>(GoundAction_Planner::planner);
#endif
    
    /*Configure the action preconditions.*/
    for(SignedPredicates::const_iterator precondition = preconditions.begin()
	    ; precondition != preconditions.end()
	    ; precondition++){
	if((*precondition)->isPositive()){
	    _prec.push_back((*precondition)->ground_NoSign(variableToConstant));

	    const Proposition<>& testProp =  _prec[_prec.size() - 1];
#ifndef NDEBUG
	    if(planner->propositionIndex.find(testProp) == planner->propositionIndex.end()){
		UNRECOVERABLE_ERROR("Unknown proposition :: "<<testProp<<endl
				    <<"as action precondition."<<endl);
	    }
#endif
	    
	} else if ((*precondition)->getName() == "=") {
	    VERBOSER(22, "Got an equality symbol in an action precondition."<<endl);

	    // Just checked that this block was actually being called for \domain{satellite}.
	    //exit(0);
	} else {
	    UNRECOVERABLE_ERROR("Negative precondition :: "<<name<<"(..."<<**precondition<<"...)");
	}
    }
    
    /*Configure the action effects.*/
    for(SignedPredicates::const_iterator effect = effects.begin()
	    ; effect != effects.end()
	    ; effect++){
	if((*effect)->isPositive()){
	    _add.push_back((*effect)->ground_NoSign(variableToConstant));
	} else {
	    _del.push_back((*effect)->ground_NoSign(variableToConstant));
	}
    }

    /*Add the encountered propositions to \staticMember{setOfEncounteredPropositions}.*/
    for(vector<Proposition<> >::const_iterator proposition = _add.begin(); proposition != _add.end(); proposition++ )
    {setOfEncounteredPropositions.insert(*proposition);};
    for(vector<Proposition<> >::const_iterator proposition = _prec.begin(); proposition != _prec.end(); proposition++ )
    {setOfEncounteredPropositions.insert(*proposition);};
    for(vector<Proposition<> >::const_iterator proposition = _del.begin(); proposition != _del.end(); proposition++ )
    {setOfEncounteredPropositions.insert(*proposition);};
    
}


void GroundAction::generateIntegerRepresentation(PropositionToInt& propositionIndex)
{
    VERBOSER(6, "Generating integer-based representation for ground action :: "<<*this<<endl);
    VERBOSER(15, "Generating integer-based representation for ground action :: "<<name<<endl);
    
    assert(propositionIndex.size() > 0);

    /*Initialise the actions integer representation.*/
    add = vector<uint>();
    del = vector<uint>();
    prec = vector<uint>();
    
    for(uint i = 0; i < _add.size(); i++){
	if(propositionIndex.find(_add[i]) == propositionIndex.end()){
	    UNRECOVERABLE_ERROR("Proposition (add) :: "<<_add[i]<<" was not indexed."<<endl);
	}
	
	add.push_back(propositionIndex[_add[i]]);
    }

    for(uint i = 0; i < _del.size(); i++){
	if(propositionIndex.find(_del[i]) == propositionIndex.end()){
	    UNRECOVERABLE_ERROR("Proposition (delete) :: "<<_del[i]<<" was not indexed."<<endl);
	}
	
	del.push_back(propositionIndex[_del[i]]);
    }

    for(uint i = 0; i < _prec.size(); i++){
	if(propositionIndex.find(_prec[i]) == propositionIndex.end()){
	    //assert(_prec.getName())
	    VERBOSER(21, "Proposition (precondition) :: "<<_prec[i]<<" was not indexed."<<endl
		     <<"This is okay if it is a fluent."<<endl);
	} else {
	    prec.push_back(propositionIndex[_prec[i]]);
	}
    }
}


bool GroundAction::operator==(const GroundAction& action) const
{
    return action.name == this->name;
}
 
bool GroundAction::operator<(const GroundAction& action) const
{
    return action.name < this->name;
}
    

size_t GroundAction::hash_value() const
{
    //boost::hash<Proposition<> > tmp(name);
    return name.hash_value();
    //return tmp(name);
}

namespace Planning
{
    size_t hash_value(const GroundAction& groundAction)
    {
	return groundAction.hash_value();
    }
    
    ostream& operator<<(ostream& o , const GroundAction& groundAction)
    {
	o<<groundAction.name<<"( ";

	o<<" prec[ "<<groundAction._prec<<" ] ";
	o<<" add[ "<<groundAction._add<<" ] ";
	o<<" del[ "<<groundAction._del<<" ] ";
	
	o<<" )";

	return o;
	    
    }
}
