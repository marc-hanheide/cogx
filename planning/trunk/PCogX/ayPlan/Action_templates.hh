// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef ACTION_TEMPLATES_HH
#define ACTION_TEMPLATES_HH

#include"Domain.hh"
#include "PredicatesAndPropositions_templates.hh"

namespace Planning
{
    /*(see Action<BASE>::makePartialAssignment)*/
    namespace MEMORY_Action_makePartialAssignment
    {
	typedef std::tr1::unordered_map<Constants,
					pair<uint, vector<VariableToConstant> >,
					boost::hash<Constants> > FinishAssignment;
    
	typedef std::tr1::unordered_map<TripleInt,
				    FinishAssignment,
				    boost::hash<TripleInt > > MapPairIntToFinishedAssignment;
    
    typedef std::tr1::unordered_map<TripleInt,
				    vector<bool>,
				    boost::hash<TripleInt > > MapPairIntToConstrainedVariables;

extern MapPairIntToFinishedAssignment finishingAssignment;
extern MapPairIntToConstrainedVariables constrainedVariableCache;

void cacheAssignmentCompletion(FinishAssignment& finishAssignment,
				       VariableToConstant& variableToConstant,
				       const SignedPredicate& signedPredicate,
				       uint predicateIndex,
				       const vector<Proposition<> >& models,
				       uint lower,
				       uint upper,
				       const VariableToUnsignedInt& variableToUnsignedInt,
				       const vector<SetOfConstants>& argumentsToCheck);

	extern const Action<>* actionPointer;// = 0;//actionName = "";
    }
    


    template<typename BASE>
    bool Action<BASE>::makePartialAssignment(VariableToConstant& variableToConstant,
					     const SignedPredicate& signedPredicate,
					     uint predicateIndex,
					     const vector<Proposition<> >& models,
					     uint lower,
					     uint upper,
					     const VariableToUnsignedInt& variableToUnsignedInt,
					     const vector<SetOfConstants>& argumentsToCheck) const
    {
	using namespace MEMORY_Action_makePartialAssignment;

	/*For each constant predicate,
	 * and each occurrence of it, we
	 * cache the completion
	 * models.*/
	TripleInt cacheIndex(predicateIndex, lower, upper);
    
	/*Make sure this function knows what action it is talking
	 * about. All the caches are reset when this function is called on
	 * an action different to
	 * \global{MEMORY_Action_makePartialAssignment::actionPointer}. */
	if(actionPointer != this){
	    actionPointer = this;

	    VERBOSER(19, "This is the first time \\function{Action::makePartialAssignment} has seen"<<endl
		     <<" ** action :: "<<this->getName()<<endl);
	
	    finishingAssignment = MapPairIntToFinishedAssignment();
	    constrainedVariableCache = MapPairIntToConstrainedVariables();
	}
    
    
	/*Setup this functions memory
	 * (see
	 * MEMORY_Action_makePartialAssignment::finishingAssignment)
	 * if this has not yet been
	 * done.*/
	if(finishingAssignment.find(cacheIndex) == finishingAssignment.end()){
	    VERBOSER(19, "This is the first time we have been asked to make a partial assignment for :: "<<signedPredicate<<endl
		     <<"** As a precondition for action :: "<<this->getName()<<endl);

	    VERBOSER(17, "Trying for precondition :: "<<signedPredicate.getName()<<endl);
	
	    finishingAssignment[cacheIndex] = FinishAssignment();
	
	    FinishAssignment& finishAssignment = finishingAssignment[cacheIndex];


	    VERBOSER(15, "Calling cached assignment completion."<<endl);
	    cacheAssignmentCompletion(finishAssignment,
				      variableToConstant,
				      signedPredicate,
				      predicateIndex,
				      models,
				      lower,
				      upper,
				      variableToUnsignedInt,
				      argumentsToCheck);
	}

	assert(finishingAssignment.find(cacheIndex) != finishingAssignment.end());
	assert(constrainedVariableCache.find(cacheIndex) != constrainedVariableCache.end());
    
	const Parameters& predicatesParameters = signedPredicate.getParameters();
	    
	FinishAssignment& finishAssignment = finishingAssignment[cacheIndex];
	vector<bool>& givenIndices = constrainedVariableCache[cacheIndex];
    
	Constants indexAssignment;
    
	for(uint argumentIndex = 0
		; argumentIndex < givenIndices.size()
		; argumentIndex++){
	    if(givenIndices[argumentIndex]){
		assert(argumentIndex < predicatesParameters.size());
		assert(dynamic_cast<const Variable*>(predicatesParameters[argumentIndex]));
		assert(variableToConstant
		       .find(dynamic_cast<const Variable&>(*predicatesParameters[argumentIndex]))
		       != variableToConstant.end());
	    
		indexAssignment.push_back(variableToConstant[dynamic_cast<const Variable&>(*predicatesParameters[argumentIndex])]);
	    }
	}

	FinishAssignment::iterator completions = finishAssignment.find(indexAssignment);

	if(completions != finishAssignment.end()){
	    vector<VariableToConstant>& someAssignments = completions->second.second;
	    uint& index = completions->second.first;

	    /*If we make no additional assignments, but rather act simply
	     * a check on the assignments already made...*/
	    if(index == 0 && someAssignments.size() == 0){
		index++;
		return true;
	    }
	
	    if(!(index < someAssignments.size())){

		/*Restart the index into \local{someAssignments} for the next traversal.*/
		index = 0;
	    
		return false;
	    }
	
	    const VariableToConstant& additionalAssignments = someAssignments[index];
	
	    for(VariableToConstant::const_iterator assignment = additionalAssignments.begin()
		    ; assignment != additionalAssignments.end()
		    ; assignment++ ){
		VERBOSER(19, "Adding assignments :: "<<assignment->first<<" -> "<<assignment->second<<endl);
		variableToConstant[assignment->first] = assignment->second;
	    }
	
	    index++;
	} else {
	    VERBOSER(17, "Failing with :: "<<variableToConstant<<endl
		     <<" ** "<<" On assignments for :: "<<signedPredicate.getName()<<endl);
	
	    return false;
	}
    
	return true;
    }

    template<typename BASE>
    const vector<bool>& Action<BASE>::getVariableChoices(const Domain& domain) const
    {
	assert((getArity(arguments)) == 0 || variablesInOrder.size() > 0);
	assert((getArity(arguments)) == 0 || variableIndex.size() > 0);

	if((getArity(arguments)) == 0)
	    return choiceVariables;
    
	if(choiceVariables.size() > 0) return choiceVariables;

	/*1 we can choose, 0 we can not. A variable that we can choose
	 *  is one which does not participate in the argument of a
	 *  constant predicate in the actions precondition.*/
	choiceVariables = vector<bool>(variablesInOrder.size());
    
	/*Assume before hand that we have a choice about every
	 * variable.*/
	for(uint i = 0; i < choiceVariables.size(); i++){choiceVariables[i] = true;}
    
	for(SignedPredicates::const_iterator precondition = preconditions.begin()
		; precondition != preconditions.end()
		; precondition++){
	    if(domain.isConstant((*precondition)->getName()) ||
	       domain.isFluentToNegative((*precondition)->getName())){
	    
		const Parameters& parameters = (*precondition)->getParameters();

		for(Parameters::const_iterator parameter = parameters.begin()
			; parameter != parameters.end()
			; parameter++){
		    assert(dynamic_cast<Variable*>(*parameter));

		    Variable& variable = dynamic_cast<Variable&>(**parameter);
		
		    choiceVariables[variableIndex[variable]] = false;
		}
	    }
	}

	//The action has to take variables, so we need for there to be a
	//bit-mask on choice variables of non-zero size.
	return choiceVariables;
    }

    template<typename BASE>
    const Variables& Action<BASE>::getVariablesInOrder() const
    {
	if(variablesInOrder.size() > 0) return variablesInOrder;

	_configureVariablesInOrderAndVariableIndex();

	assert((getArity(arguments)) == 0 || variablesInOrder.size() > 0);
    
	return variablesInOrder;
    }

    template<typename BASE>
    const VariableToUnsignedInt& Action<BASE>::getVariableIndex() const
    {
	if(variableIndex.size() > 0) return variableIndex;

	_configureVariablesInOrderAndVariableIndex();

    
	assert((getArity(arguments)) == 0 || variableIndex.size() > 0);
	return variableIndex;
    }

    template<typename BASE>
    void Action<BASE>::_configureVariablesInOrderAndVariableIndex() const
    {   
	/*Index...*/
	int argCount = getArity(arguments) - 1;//0;//arguments.size() - 1;
    
	VERBOSER(19, "Action :: "<<getName()<<" has :: "<<argCount<<" as top argument index."<<endl);
	VERBOSER(19, "Action :: "<<getName()<<" has :: "<<getArity(arguments)<<" arguments."<<endl);
	//     //{char ch; cin>>ch;};
    
	/*We assume an action takes some arguments.*/
	if(argCount < 0){
	    WARNING("Action :: "<<getName()<<" takes no arguments."<<endl);
	    return;
	}
    
	assert(argCount >= 0);
    
	variablesInOrder = Variables(getArity(arguments));//.size());
    
	/*Remember the arguments were parsed backwards.*/
	for(Arguments::const_iterator argument = arguments.begin()
		; argument != arguments.end()
		; argument++){
	    
	    /*Set of variable symbols that have are objects in the
	     * union of \local{types}.*/
	    const Variables& variables  = argument->second;
	    
	    for(int varIndex = 0; varIndex < variables.size(); varIndex ++){
		assert(argCount < variablesInOrder.size());
		assert(argCount >= 0);// variablesInOrder.size());
		variablesInOrder[argCount] = variables[varIndex];
		variableIndex[variables[varIndex]] = argCount;
		argCount--;
	    }
	
	    /*Listed the objects associated with one more argument.*/
	    //assert(argCount>=0 || );
	}
    }


    template<typename BASE>
    void Action<BASE>::setName(const string& name)
    {
	this->name = name;
    }


    template<typename BASE>
    const string& Action<BASE>::getName() const
    {
	return name;
    }

    template<typename BASE>
    const Arguments& Action<BASE>::getArguments() const
    {
	return arguments;
    }

    template<typename BASE>
    const SignedPredicates& Action<BASE>::getPreconditions() const
    {
	return preconditions;
    }

    template<typename BASE>
    const SignedPredicates& Action<BASE>::getEffects() const
    {
	return effects;
    }


    template<typename BASE>
    void Action<BASE>::computeAsString(const string& str) const
    {
	ostringstream oss;
	oss<<"(:action "<<name<<endl;

	oss<<":parameters ( "<<arguments<<" )\n";

	oss<<":precondition (and "<<preconditions<<")\n";
	oss<<":effect (and "<<effects<<")\n";
	oss<<')';
    
	BASE::computeAsString(oss.str());
    }


    template<typename BASE>
    Action<BASE>::Action(const string& name,
			 const Arguments& arguments,
			 const SignedPredicates& preconditions,
			 const SignedPredicates& effects,
			 int cost)
	:name(name),
	 arguments(arguments),
	 preconditions(preconditions),
	 effects(effects),
	 cost(cost),
	 costEvaluator(0)
    {}
    
    template<typename BASE>
    Action<BASE>::Action(const string& name,
			 const Arguments& arguments,
			 const SignedPredicates& preconditions,
			 const SignedPredicates& effects,
			 SignedPredicate* costEvaluator)
	:name(name),
	 arguments(arguments),
	 preconditions(preconditions),
	 effects(effects),
	 cost(0),
	 costEvaluator(costEvaluator)
    {}

}

#endif
