// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.
#ifndef PREDICATESANDPROPOSITIONS_TEMPLATES_HH
#define PREDICATESANDPROPOSITIONS_TEMPLATES_HH

namespace Planning
{
    
    template<typename BASE>
    Predicate<> Predicate<BASE>::ground_NoSign(const VariableToConstant& variableToConstant) const
    {
	Constants constants;
	for(Parameters::const_iterator parameter = parameters.begin()
		; parameter != parameters.end()
		; parameter++){

	    if(dynamic_cast<const Constant*>(*parameter)){
		constants.push_back(dynamic_cast<const Constant&>(**parameter));
	    } else if (dynamic_cast<const Variable*>(*parameter)) {
		VariableToConstant::const_iterator tmp
		    = variableToConstant.find(dynamic_cast<const Variable&>(**parameter));

		if(tmp != variableToConstant.end()){
		    constants.push_back(tmp->second);
		} else {
		    UNRECOVERABLE_ERROR("No variable substitute for :: "<<**parameter<<endl);
		}
	    } else {
		UNRECOVERABLE_ERROR("Parameter :: "
				    <<**parameter<<" for symbol :: "
				    <<predicateName<<" is neither constant or variable... ");
	    }
	}

	return Predicate<>(predicateName, constants);
    }


    template<typename BASE>
    size_t Predicate<BASE>::hash_value() const
    {
	size_t tmp = 0;

	VERBOSER(8, "Hashing predicate :: "<<*this<<endl);
    
	for(Parameters::const_iterator parameter = parameters.begin()
		; parameter != parameters.end()
		; parameter++)
	{
	    boost::hash_combine(tmp, **parameter);
	}
    
	boost::hash_combine(tmp,
			    Predicate<BASE>::predicateName);

	return tmp;
    }



    template<typename BASE>
    size_t Proposition<BASE>::hash_value() const
    {
	return Predicate<BASE>::hash_value();
    
	//     size_t tmp = 0;

	//     for(; first != last; ++first)
	//     {
	// 	hash_combine(seed, *first);
	//     }
    
	//     size_t tmp = boost::hash_range(Predicate<BASE>::parameters.begin(),
	// 				   Predicate<BASE>::parameters.end());
    
	//     boost::hash_combine(tmp,
	// 			Predicate<BASE>::predicateName);

	//     return tmp;
    }


    template<typename BASE>
    Signed<BASE>::Signed(const Signed<BASE>& sb)
	:sign(sb.sign),
	 BASE(dynamic_cast<const BASE&>(sb))
    {
    
    }


    template<typename BASE>
    template<typename T>
    Signed<BASE>::Signed(const Signed<T>& sb)
	:sign(sb.sign),
	 BASE(dynamic_cast<const T&>(sb))
    {
    
    }


    template<typename BASE>
    template<typename T>
    Signed<BASE>::Signed(const T& t)
	:sign(false),
	 BASE(dynamic_cast<const T&>(t))
    {
    }

    template<typename BASE>
    Signed<BASE>& Signed<BASE>::operator=(const Signed<BASE>& sb)
    {
	sign = sb.sign;
	return dynamic_cast<Signed<BASE>&>(BASE::operator=(dynamic_cast<const BASE&>(sb)));
    }

    template<typename BASE>
    template<typename T>
    Signed<BASE>& Signed<BASE>::operator=(const Signed<T>& sb)
    {
	sign = sb.sign;
	return T::operator=(dynamic_cast<const T&>(sb));
    }

    template<typename BASE>
    template<typename T>
    Signed<BASE>& Signed<BASE>::operator=(const T& t)
    {
	return T::operator=(dynamic_cast<const T&>(t));
    }

    template<typename BASE>
    Predicate<BASE>& Predicate<BASE>::operator=(const Predicate<BASE>& t)
    {
	VERBOSER(1, "Calling Predicate<BASE>::operator=(const Predicate<BASE>& t).");
	predicateName = t.predicateName;
    
	/*Clear the old parameters.*/
	for_each(parameters.begin(), parameters.end(), delete_pointers<Copyable<string> >());

	parameters = Parameters();//t.parameters.size());
    
	for(Parameters::const_iterator p = t.parameters.begin()
		; p != t.parameters.end()
		; p++){
	    parameters.push_back((*p)->copy());
	}
    
	return dynamic_cast<Predicate<BASE>&>(BASE::operator=(dynamic_cast<const BASE&>(t)));
    }


    template<typename BASE>
    template<typename T>
    Predicate<BASE>& Predicate<BASE>::operator=(const Predicate<T>& t)
    {
	VERBOSER(1, "Calling Predicate<BASE>::operator=(const Predicate<T>& t).");
	predicateName = t.predicateName;
    
	/*Clear the old parameters.*/
	for_each(parameters.begin(), parameters.end(), delete_pointers<Copyable<string> >());

	parameters = Parameters();//t.parameters.size());
    
	for(Parameters::const_iterator p = t.parameters.begin()
		; p != t.parameters.end()
		; p++){
	    parameters.push_back((*p)->copy());
	}
    
	return BASE::operator=(dynamic_cast<const T&>(t));
    }


    template<typename BASE>
    Predicate<BASE>::~Predicate()
    {
	for_each(parameters.begin(), parameters.end(), delete_pointers<Copyable<string> >());
    }


    template<typename BASE>
    void Signed<BASE>::computeAsString(const string& str) const
    {
	if(sign)
	    BASE::computeAsString(str);
	else
	    BASE::computeAsString("( not " + str + " )");
    }

    template<typename BASE>
    void Predicate<BASE>::computeAsString(const string& str) const
    {
	ostringstream oss;
	oss<<'('<<predicateName<<' '<<parameters<<')';

	BASE::computeAsString(oss.str());
    }

    
    template<typename BASE>
    Predicate<BASE>::Predicate(const PredicateName& predicateName, const Parameters& parameters)
	:predicateName(predicateName),
	 parameters(parameters),
	 BASE()
    {
    
    }


    template<typename BASE>
    Predicate<BASE>::Predicate(const PredicateName& predicateName, const Constants& parameters)
	:predicateName(predicateName),
	 BASE()
    {
	/*Make room for the parameters.*/
	this->parameters = Parameters(parameters.size());
    
	transform(parameters.begin(), parameters.end(),
		  this->parameters.begin(),
		  morph_elements<Constant, Constant>());
    }

    template<typename BASE>
    Predicate<BASE>::Predicate(const Predicate<BASE>& pb)
	:predicateName(pb.getName()),
	 BASE(dynamic_cast<const BASE&>(pb))
    {
	     
	for(Parameters::const_iterator p = pb.parameters.begin()
		; p != pb.parameters.end()
		; p++){
	    parameters.push_back((*p)->copy());
	}
    }

    
    template<typename BASE>
    Proposition<BASE>::Proposition(const PredicateName& predicateName, const Parameters& parameters)
	:Predicate<BASE>(predicateName, parameters)
    {
	for_each(parameters.begin(),
		 parameters.end(),
		 assert_Constant());
	//     for(Parameters::const_iterator p = this->parameters.begin()
	// 	    ; p != this->parameters.end()
	// 	    ; p++){
	// 	assert(dynamic_cast<Constant*>(*p));
	//     }
    }



    template<typename BASE>
    Proposition<BASE>::Proposition(const PredicateName& predicateName, const Constants& constants)
	:Predicate<BASE>(predicateName, constants)
    {
    }

    template<typename BASE>
    template<typename T>
    Proposition<BASE>::Proposition(const Predicate<T>& proposition)
	:Predicate<BASE>(proposition){
    
	for_each(Predicate<BASE>::parameters.begin(),
		 Predicate<BASE>::parameters.end(),
		 assert_Constant());
    }


}


#endif
