// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef PREDICATESANDPROPOSITIONS_HH
#define PREDICATESANDPROPOSITIONS_HH

#include"global.hh"

namespace Planning
{
    /*We use the C++ type system to make sure we are using the right
     * string (variable, constant, type) in the right place.*/
    TYPED_STRING(Variable);
    TYPED_STRING(Type);
    TYPED_STRING(PredicateName);
    TYPED_STRING(Constant);

#ifndef NATHAN
    typedef tr1::unordered_map<Variable, Constant, boost::hash<Variable> > VariableToConstant;
    typedef tr1::unordered_map<Variable, uint, boost::hash<Variable> > VariableToUnsignedInt;
    typedef tr1::unordered_set<Constant, boost::hash<Constant> > SetOfConstants;
#else
    typedef map<Variable, Constant> VariableToConstant;
    typedef map<Variable, uint> VariableToUnsignedInt;
    typedef set<Constant> SetOfConstants;
#endif
    
    /*Set of constant symbols.*/
    typedef vector<Constant> Constants;

    /*Container of strings whose planning type is to be later
     * ascertained.*/
    typedef vector<Copyable<string>* > UntypedStrings;

    /*A set of \class{Variable}s.*/
    typedef set<Variable> SetOfVariables;

    /*A set of \class{Type}s.*/
    typedef set<Type> SetOfTypes;

    /*Sequence of \class{Variable}s.*/
    typedef vector<Variable> Variables;

    /*Sequence of \class{Type}s.*/
    typedef vector<Type> Types;

    /*Sequence of \class{PredicateName}s.*/
    typedef vector<PredicateName> PredicateNames;

    /* Part of the specification of a predicate, fluent or operator
     *  arguments. A 1'st and higher order symbols arguments in PDDL are
     *  given as a sequence of relations between variables and
     *  types. Again the relation is given in sequence. Although the
     *  sequence is unimportant to semantics as far as I can tell, it is
     *  important to keep because plan verification relies on symbol
     *  arguments being given in a particular order -- i.e. The order they
     *  occur in the application-argument domain definition. */
    typedef pair<Types, Variables> ArgumentComponent;

    /*Specification of a predicate, fluent or operator arguments*/
    typedef vector<ArgumentComponent> Arguments;

    /*PDDL multi-type to variables. A variable argument to a PDDL operator
     * can be of one or more types. Here we have mapped the \class{Type}s
     * to the \class{Variable}s. This is the datastructure that we are
     * going to use for the operator arguments specification.*/
    typedef map<SetOfTypes, SetOfVariables> MultiPDDLArguments;

    /*LHS is the types (one or more of) the RHS.*/
    typedef pair<Types, Types> TypeOfType;

    /*Here I am keeping the order from the domain specification just for
     * consistency with \type{Arguments}*/
    typedef vector<TypeOfType> TypeOfTypes;

    /*Specification of a predicate in terms of a typed variable list. As
     * far as I can tell there is no use in keeping the order in which the
     * specification of predicates occur in pddl.*/
    typedef map<PredicateName, Arguments> PredicateNameSpecifications;

    /*Parameters can be either \class{Variable}s or \class{Constant}s.*/
    typedef UntypedStrings Parameters;

    /*Sometimes we can associate either a \class{Constant} or \class{Type}
     * with one or more types. LHS is the types we are associating the RHS
     * pointers with.*/
    typedef pair<Types, UntypedStrings> TypeOfSymbol;

    /*Sequence of specifications of symbol types.*/
    typedef vector<TypeOfSymbol> TypeOfSymbols;


    /*Sometimes a symbol can be signed, other-times there is no need to
     *worry about that.*/
    template<typename BASE = HasStringRepresentation>
    class Signed : public BASE
    {
    public:
	/*Sign is positive if unspecified.*/
	Signed(bool sign = true):sign(sign){};

	bool isPositive() const{return sign == true;};
	bool isNegative() const{return !(isPositive());}

	Signed(const Signed<BASE>&);

	template<typename T>
	Signed(const Signed<T>&);

	template<typename T>
	Signed(const T&);

	Signed<BASE>& operator=(const Signed<BASE>&);
	template<typename T>
	Signed<BASE>& operator=(const Signed<T>&);
	template<typename T>
	Signed<BASE>& operator=(const T&);
	
	
	
	/*Make symbol positive.*/
	bool makePositive(){

	    if(!isPositive()){
		this->HasStringRepresentation::computedAsString = false;
		sign = true;
	    }
	    
	    
	    return sign;// = true;
	};

	/*Make symbol negative.*/
	bool makeNegative(){
	    
	    if(!isNegative()){
		this->HasStringRepresentation::computedAsString = false;
		sign = false;
	    }
	    return sign;
	}
	


	/*Change sign of symbol -- Form positive to negative or
	 * contrariwise.*/
	bool changeSign(){
	    this->HasStringRepresentation::computedAsString = false;
	    return sign = !sign;
	};
    protected:
	
	/*NOT is false, NOT-NOT is true.*/
	bool sign;
	
	/*Modifies the \member{asString} of
	 * \ancestor{HasStringRepresentation}. We only really expect this
	 * to be called by the overloaded stream operator.*/
	void computeAsString(const string&) const;  
    };    
    

    /*A \type{Predicate} has a \type{PredicateName} identifier and takes
     *  some arguments that could be \type{Variable}s or
     *  \type{Constant}s. We have that a \child{Proposition} is a
     *  \class{Predicate} (think of a proposition as a predicate with only
     *  constant arguments).
     *
     *
     * NOTE : \template<BASE> must have \class{HasStringRepresentation} as
     * an ancestor.
     **/
    //typedef pair<PredicateName, UntypedStrings> Predicate;
    template<typename BASE = HasStringRepresentation>
    class Predicate : public BASE
    {
    public:
	/*For STL compatibility only.*/
	Predicate():predicateName(""){};
    
	/*Entries in \Argument{Parameters} will be deleted on
	 * destruction.*/
	Predicate(const PredicateName&, const Parameters&);
	Predicate(const PredicateName&, const Constants&);

	Predicate(const Predicate<BASE>&);

	/*Makes a new predicate with all arguments constant by
	 * grounding according to substitutions in \argument{VariableToConstant}.*/
	Predicate<> ground_NoSign(const VariableToConstant&) const;
	
	/*Allocates space and copies \argument{Predicate}
	 * \member{parameters}.*/
	template<typename T>
	Predicate(const Predicate<T>& predicate)
	    :predicateName(predicate.getName()),
	     BASE(dynamic_cast<const T&>(predicate))
	{
	    for(Parameters::const_iterator p = predicate.getParameters().begin()
		    ; p != predicate.getParameters().end()
		    ; p++){
		parameters.push_back((*p)->copy());
	    }
	}

	/*Deletes entries in \member{parameters}*/
	~Predicate();

	
	
	Predicate<BASE>& operator=(const Predicate<BASE>& t);
	template<typename T>
	Predicate<BASE>& operator=(const Predicate<T>& t);
	
	
	/*Get predicates string identifier.*/
	const PredicateName& getName() const {return predicateName;};

	const Parameters& getParameters() const {return parameters;};
	
	size_t hash_value() const;
	
	/* FIX ME :: At the moment ordering is done using the string
	 * representation of this object, however for speed and
	 * sensibility of the order, I think that I am going to have to
	 * make something more specialised at some point. */
	//     bool operator<() const;
	//     bool operator==() const;
    protected:
	/*String identifier.*/
	PredicateName predicateName;
    
	/*Modifies the \member{asString} of
	 * \ancestor{HasStringRepresentation}. We only really expect this
	 * to be called by the overloaded stream operator.*/
	void computeAsString(const string&) const;
    
	/*Predicate's \member{parameters} are untyped. Could be a
	 * \type{Constant}, could be a \type{Variable}.*/
	Parameters parameters;
    };

    /*Datastructure to store collections of predicate names.*/
#ifndef NATHAN
    /*A set of \class{PredicateName}s.*/
    class SetOfPredicateNames : public std::tr1::unordered_set<PredicateName, std::tr1::hash<std::string> > //boost::hash<PredicateName> >

#else
    class SetOfPredicateNames : public std::set<PredicateName>
#endif
    {	
    public:
	
	
	inline const_iterator find(const PredicateName& predicateName) const
	{
#ifndef NATHAN
	    return std::tr1::unordered_set<PredicateName, std::tr1::hash<std::string> >::find(predicateName);
#else
	    return std::set<PredicateName>::find(predicateName);
#endif
	}
	
	template<typename T>
	inline const_iterator find(const Predicate<T>& predicate) const
	{
	    return find(predicate.getName());
	}

	
    };

    
    /*A predicate whose parameters are all \type{Constant}s. A proposition
     *  has a string identifier and takes some constant arguments. We do
     *  not distinguish between the type or name a predicate has, and the
     *  type or the name a proposition has. Here, a proposition is a
     *  ground predicate.*/
    template<typename BASE = HasStringRepresentation>
    class Proposition : public Predicate<BASE>
    {
    public:
	/*During construction, this class asserts that its contents
	 * are of type \class{Constant<string>}. This is the unary
	 * function type for making that assertion.*/
	typedef assert_dynamic_type<Constant, Copyable<string> > assert_Constant;
	
	/*For STL compatibility only.*/
	Proposition(){};
    
	/*Constructor as for \parent{Predicate} only we ensure that the
	 * \argument{Parameters} are all of \type{Constant}.*/
	Proposition(const PredicateName&, const Parameters&);
	Proposition(const PredicateName&, const Constants&);
	
	size_t hash_value() const;
	
	/*Allocates space and copies \argument{Predicate}
	 * \member{parameters}.*/
	template<typename T>
	Proposition(const Predicate<T>& proposition);//:Predicate<BASE>(proposition){};
    };

    /*A predicate that is signed can be negated.*/
    typedef Predicate<Signed<HasStringRepresentation> > SignedPredicate;

    /*A proposition that is signed can be negated.*/
    typedef Proposition<Signed<HasStringRepresentation> > SignedProposition;

    /*Sequence of signed predicates.*/
    typedef vector<SignedPredicate*> SignedPredicates;

    /*Starting state is a sequence of propositions. Anything not in this
     * list is considered to be false in the starting state.*/
    typedef vector<Proposition<> > StartingState;

    /*A goal is specified by a sequence of propositions that need to be
     *true in the final state. Here I allow for those propositions to be
     *negated -- Just in case.*/
    typedef vector<SignedProposition> Goal;

#ifndef NATHAN
    /*Proposition indices.*/
    typedef tr1::unordered_map<Proposition<>,
			       uint,
			       boost::hash<Proposition<> > > PropositionToInt;

    typedef tr1::unordered_set<Proposition<>, boost::hash<Proposition<> > > SetOfPropositions;
#else
    /*Proposition indices.*/
    typedef map<Proposition<>,
		uint > PropositionToInt;

    typedef set<Proposition<> > SetOfPropositions;
#endif
    
    
    /*Get the arity  of \argument{Arguments}*/
    uint getArity(const Arguments&);
    
    template<typename BASE>
    std::size_t hash_value(const Predicate<BASE>& t)
    {
	return t.hash_value();
    }
    
    template<typename BASE>
    std::size_t hash_value(const Proposition<BASE>& t)
    {
	return t.hash_value();
    }
    
}

ostream& operator<<(ostream&, const Planning::SetOfConstants&);

ostream& operator<<(ostream&, const Planning::VariableToConstant&);

ostream& operator<<(ostream&, const Planning::VariableToUnsignedInt&);

ostream& operator<<(ostream&, const Planning::SetOfPredicateNames&);



#endif
