// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef FUNCTIONS_HH
#define FUNCTIONS_HH

#include "PredicatesAndPropositions.hh"

#include<typeinfo>

namespace Planning
{

    /* A function (as far as planning is concerned) is like a
     * \class{predicate} that maps to a finite domain type
     * that is not Boolean. */
    template<typename RANGE = uint, typename BASE = HasStringRepresentation>
    class Function : public Predicate<BASE>
    {
    public:
	/*For STL compatibility only.*/
	Function(){};
    
	/* Constructor as for \parent{Predicate} only we ensure that the
	 * \argument{Parameters} are all of \type{Constant}.*/
	Function(const PredicateName& predicateName, const Parameters& parameters)
	    :Predicate<BASE>(predicateName, parameters){};
	
	size_t hash_value() const ;
    };
    
    /* A invocation is a function whose arguments are all constant. An
     * invocation (as far as planning is concerned) is like a
     * \class{Proposition} that maps to a finite domain type
     * (resp. Boolean). */
    template<typename RANGE = uint, typename BASE = HasStringRepresentation>
    class Invocation : public Function<RANGE, BASE>
    {
    public:
	/*For STL compatibility only.*/
	Invocation(){};
    
	/*Constructor as for \parent{Predicate} only we ensure that the
	 * \argument{Parameters} are all of \type{Constant}.*/
	Invocation(const PredicateName&, const Parameters&, const RANGE& rangeValue = RANGE());
	
	Invocation(const PredicateName& predicateName, const Constants& constants, const RANGE& rangeValue = RANGE())
	    :Function<RANGE, BASE>(predicateName, constants),
	     rangeValue(rangeValue){};

	const RANGE& getRangeValue() const {return rangeValue;}
	
	void setRangeValue(const RANGE& rv) {rangeValue = rv;}
	
	size_t hash_value() const ;
    private:
	RANGE rangeValue;
    };

    /*The following are lists of planning functions that could be
     * useful. Like a \class{Predicate<>}, a function is evaluated
     * according to a state. Unlike a \class{Predicate<>} a function
     * maps to an element from a finite range. For example, that
     * element might be in a subset of the reals, positive natural
     * numbers, or integers.*/
    
    typedef Function<double> RealFunction;
    typedef vector<RealFunction> RealFunctions;
    
    typedef Function<int> IntegerFunction;
    typedef vector<IntegerFunction> IntegerFunctions;
    
    typedef Function<uint> PositiveIntegerFunction;
    typedef vector<PositiveIntegerFunction> PositiveIntegerFunctions;
}

template<typename RANGE, typename BASE>
ostream& operator<<(ostream& o, const Planning::Function<RANGE, BASE>& in)
{
    o<<dynamic_cast<const Planning::Predicate<BASE>&>(in)<<" - "<<typeid(RANGE).name();
    
    return o;
}


#endif
