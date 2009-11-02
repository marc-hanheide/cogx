// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.
#include "PredicatesAndPropositions.hh"


#include "PredicatesAndPropositions_templates.hh"

using namespace Planning;


ostream& operator<<(ostream& o, const SetOfConstants& soc)
{

    for(SetOfConstants::const_iterator p = soc.begin()
	    ; p != soc.end()
	    ; p++){
	o<<*p<<", ";
    }
    
    
    return o;
}



ostream& operator<<(ostream& o, const VariableToConstant& vtc)
{
    for(VariableToConstant::const_iterator p = vtc.begin()
	    ; p != vtc.end()
	    ; p++){
	o<<p->first<<"  -> "<<p->second<<endl;
    }
    
    
    return o;
}


ostream& operator<<(ostream& o, const VariableToUnsignedInt& vtc)
{
    for(VariableToUnsignedInt::const_iterator p = vtc.begin()
	    ; p != vtc.end()
	    ; p++){
	o<<p->first<<"  -> "<<p->second<<endl;
    }
    
    return o;
}

ostream& operator<<(ostream& o, const SetOfPredicateNames& setOfPredicateNames)
{
    for(SetOfPredicateNames::const_iterator p = setOfPredicateNames.begin()
	    ; p != setOfPredicateNames.end()
	    ; p++){
	o<<*p<<" {:} ";
    }
    
    return o;
}



uint Planning::getArity(const Arguments& arguments)
{
    uint arity = 0;
    for(Arguments::const_iterator argument = arguments.begin()
	    ; argument != arguments.end()
	    ; argument++ ){
	
	const Variables& variables  = argument->second;
	
	arity += variables.size();
    }
    
    return arity;
}


namespace PREDICATESANDPROPOSITIONS_Nonsense
{
    void foo()
    {
	PredicateName predicateName("some predicate name");
	
	{
	    
	    Constants constants;
	    Predicate<> prop(predicateName, constants);
	    Proposition<> pred(predicateName, constants);
	    SignedPredicate sProp(predicateName, constants);
	    SignedProposition sPred(predicateName, constants);
	}
	
	
	
	Parameters parameters;
	parameters.push_back(new Constant("some constant"));
	parameters.push_back(new Variable("some constant"));
	
	
	Parameters constants;
	constants.push_back(new Constant("some constant"));
	
	{
	    Predicate<HasStringRepresentation> p1(predicateName, parameters);
	}

	{
	    Proposition<HasStringRepresentation> p1(predicateName, constants);
	}
	
	{
	    SignedPredicate p1(predicateName, parameters);
	}

	{
	    SignedProposition p1(predicateName, constants);
	}


	SignedProposition sProp(predicateName, constants);
	SignedPredicate sPred(predicateName, parameters);
	Predicate<Signed<HasStringRepresentation> > p(sProp);
	p = SignedPredicate(sPred);

	{
	    Predicate<> sProp(predicateName, constants);
	    Predicate<> _sProp;
	    _sProp = sProp;
	}
	
	

	{
	    SignedProposition sProp(predicateName, constants);
	    SignedPredicate sPred(predicateName, constants);
	    Predicate<> prop(predicateName, constants);
	    Proposition<> pred(predicateName, constants);
	    
	    SignedProposition _sProp(sProp);
	    SignedPredicate _sPred(sProp);
	    Predicate<> _prop(sProp);
	    Proposition<> _pred(sProp);
	    
	    SignedProposition __sProp(sPred);
	    SignedPredicate __sPred(sPred);
	    Predicate<> __prop(sPred);
	    Proposition<> __pred(sPred);
	    
	    SignedProposition __sProp_(prop);
	    SignedPredicate __sPred_(prop);
	    Predicate<> __prop_(prop);
	    Proposition<> __pred_(prop);

	    SignedProposition __sProp__(pred);
	    SignedPredicate __sPred__(pred);
	    Predicate<> __prop__(pred);
	    Proposition<> __pred__(pred);
	    
	    __pred__.hash_value();
	    __sProp.hash_value();


	    
	    VariableToConstant variableToConstant;
	    
	    sProp.ground_NoSign(variableToConstant);
	    sPred.ground_NoSign(variableToConstant);
	    prop.ground_NoSign(variableToConstant);
	    pred.ground_NoSign(variableToConstant);
	}
    }    
}

