// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#include "Functions.hh"

using namespace Planning;


template<typename RANGE, typename BASE>
Invocation<RANGE, BASE>::Invocation(const PredicateName& predicateName,
				    const Parameters& parameters,
				    const RANGE& rangeValue)
    :Predicate<BASE>(predicateName, parameters),
     rangeValue(rangeValue)
{
    for_each(parameters.begin(),
	     parameters.end(),
	     Proposition<>::assert_Constant());
}

template<typename RANGE, typename BASE>
size_t Function<RANGE, BASE>::hash_value() const
{
    size_t tmp = Predicate<BASE>::hash_value();
    boost::hash_combine(tmp, typeid(RANGE).name());
    return tmp;
}


template<typename RANGE, typename BASE>
size_t Invocation<RANGE, BASE>::hash_value() const
{
    size_t tmp = Function<RANGE, BASE>::hash_value();
    boost::hash_combine(tmp, rangeValue);
    return tmp;
}


namespace FUNCTIONS_Nonsense
{
    void foo()
    {
    }
}
