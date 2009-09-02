// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#include "Action.hh"

#include "Action_templates.hh"

using namespace Planning;


namespace Planning
{
    /*(see Action<BASE>::makePartialAssignment)*/
    namespace MEMORY_Action_makePartialAssignment
    {
	MapPairIntToFinishedAssignment finishingAssignment;
	MapPairIntToConstrainedVariables constrainedVariableCache;

	const Action<>* actionPointer = 0;
	
	void cacheAssignmentCompletion(FinishAssignment& finishAssignment,
				       VariableToConstant& variableToConstant,
				       const SignedPredicate& signedPredicate,
				       uint predicateIndex,
				       const vector<Proposition<> >& models,
				       uint lower,
				       uint upper,
				       const VariableToUnsignedInt& variableToUnsignedInt,
				       const vector<SetOfConstants>& argumentsToCheck)
	{
	    TripleInt cacheIndex(predicateIndex, lower, upper);
	
	    const Parameters& parameters  = signedPredicate.getParameters();
	
	    /*Indices to variables that \argument{signedPredicate} is
	     * constrained by (resp. variables that
	     * \argument{signedPredicate} makes assignments to from its
	     * model -- via \argument{models[lower .. upper]}) are marked
	     * as TRUE.*/
	    constrainedVariableCache[cacheIndex] = vector<bool>(parameters.size());
	    vector<bool>& givenIndices = constrainedVariableCache[cacheIndex];//(parameters.size());
	    for(uint i =  0; i < givenIndices.size(); i++){
		givenIndices[i] = false;
	    }
	
	
	    uint parameterCount = 0;
	    for(Parameters::const_iterator parameter = parameters.begin()
		    ; parameter != parameters.end()
		    ; parameter++){
		assert(dynamic_cast<const Variable*>(*parameter));
	    
		const Variable& variable = dynamic_cast<const Variable&>(**parameter);

		VERBOSER(15, "Deciding if \\predicate{"<<signedPredicate.getName()
			 <<"} should force an assignment for variable :: "<<variable<<endl);
	    
		VariableToConstant::const_iterator maplet = variableToConstant.find(variable);
	    
		/*If an assignment has already been made to
		 * \argument{signedPredicate} according to
		 * \argument{variableToConstant}.*/
		if(maplet != variableToConstant.end()){
		    assert(parameterCount < givenIndices.size());
		    givenIndices[parameterCount] = true;//.push_back(parameterCount);

		    VERBOSER(15, "Assignment to  :: "<<variable<<" was forced by another precondition."<<endl);
		} else {
		    VERBOSER(15, "Assignment to  :: "<<variable<<" was NOT forced by another precondition."<<endl);
		}
	    
		parameterCount++;
	    }
	
	
	    if(upper <= lower){
		WARNING("Indices to models of :: "<<signedPredicate<<endl
			<<" ** are lower :: "<<lower<<" upper :: "<<upper<<endl
			<<" ** hence we have no models of this predicate, and yet it"<<endl
			<<" ** appears as an action precondition.\n");
		// 	    UNRECOVERABLE_ERROR("Indices to models of :: "<<signedPredicate<<endl
		// 				<<"** are lower :: "<<lower<<" upper :: "<<upper<<endl
		// 				<<"** hence we have no models of this predicate, and yet it"<<endl
		// 				<<"** appears as an action precondition.\n");
	    }
	
	    VERBOSER(15, "For each model of predicate :: "<<signedPredicate.getName()<<endl);
	    for(uint index = lower; index < upper; index++){
		const Proposition<>& proposition = models[index];

#ifndef NDEBUG
		if(proposition.getName() != signedPredicate.getName()
		   && signedPredicate.getName() != "="){
		    UNRECOVERABLE_ERROR("Expecting model of :: "<<signedPredicate.getName()
					<<" and got a model for :: "<<proposition.getName()<<" at index :: "<<index<<endl
					<<" ** for upper :: "<<upper<<" and lower :: "<<lower<<endl);
		}
#endif

		const Parameters& propositionsParameters = proposition.getParameters();
		const Parameters& predicatesParameters = signedPredicate.getParameters();

		assert(givenIndices.size() == propositionsParameters.size());
		assert(givenIndices.size() == predicatesParameters.size());


		VERBOSER(15, "We have proposition :: "<<proposition<<endl);
		
		VERBOSER(15, "For predicate :: "<<signedPredicate<<endl);

		VERBOSER(15, "For each argument to proposition :: "<<proposition<<endl);

	    
		Constants indexAssignment;
		VariableToConstant completedAssignment;

		/* Were we unable to complete the assignment in \local{propositionsParameters}.*/
		bool abort = false;

		/* To avoid computing the same integer multiple times
		 * in the following code, we explicitly cache it.*/
		uint integerCache = 0;
	    
		for(uint argumentIndex = 0
			; argumentIndex < givenIndices.size()
			; argumentIndex++){
		
		    assert(argumentIndex < predicatesParameters.size());
		    assert(argumentIndex < propositionsParameters.size());
		    assert(dynamic_cast<const Variable*>(predicatesParameters[argumentIndex]));
		    assert(dynamic_cast<const Constant*>(propositionsParameters[argumentIndex]));

		    const Constant& constant = dynamic_cast<const Constant&>(*propositionsParameters[argumentIndex]);
		    const Variable& variable = dynamic_cast<const Variable&>(*predicatesParameters[argumentIndex]);
		
		    /*If we DO NOT get to decide on the value of argument \local{argumentIndex}.*/
		    if(givenIndices[argumentIndex]){
			indexAssignment.push_back(constant);
		    }
		    /*If we get to decide on the value of argument \local{argumentIndex}.*/
		    else if(argumentsToCheck[integerCache = variableToUnsignedInt.find(variable)->second].find(constant)
			    != argumentsToCheck[integerCache].end()) {
		    
			completedAssignment[variable] = constant;
		    } else {
			/*Assert that we at least tried to find the argument in a non-empty set.*/
			assert(argumentsToCheck[variableToUnsignedInt.find(variable)->second].size() > 0);

			VERBOSER(17, "*DENIED* for action -- ("<<constant<<") is not a good argument for ("<<variable<<")"<<endl);
			VERBOSER(17, "*DENIED* This was the "<<(integerCache = variableToUnsignedInt.find(variable)->second)<<"'th argument."<<endl);

			const SetOfConstants& setOfConstants
			    = argumentsToCheck[integerCache = variableToUnsignedInt.find(variable)->second];

			VERBOSER(17, "Possibilities are :: "<<argumentsToCheck<<endl);//setOfConstants<<endl);
		    
			abort = true;
			break;
		    }
		}

		if(!abort){

		    /*if we have no dealt with \local{indexAssignment} before.*/
		    if(finishAssignment.find(indexAssignment) == finishAssignment.end()){
			finishAssignment[indexAssignment] = pair<uint, vector<VariableToConstant> >(0, vector<VariableToConstant>());
		    }
		
		    /*Finally, cache the assignment completion.*/
		    finishAssignment[indexAssignment].second.push_back(completedAssignment);
		
		    VERBOSER(17, "At index :: [|"<<indexAssignment
			     <<"|]\n ** we can impose assignment :: \n"<<completedAssignment<<endl);
		}
	    }
	}
	
	
    }
}


namespace ACTION_Nonsense
{
    void foo()
    {
	//Action<HasStringRepresentation>* a;
	Arguments arguments;
	SignedPredicates preconditions;
	SignedPredicates effects;
	
	Action<> a("some", arguments, preconditions, effects);

	SignedPredicate* _signedPredicate;
	Action<> aa("some", arguments, preconditions, effects, _signedPredicate);

	a.getName();
	a.setName("some");
	a.getArguments();
	a.getPreconditions();
	a.getEffects();

	Domain* d = 0;

	a.getVariableChoices(*d);
	a.getVariablesInOrder();
	a.getVariableIndex();
	VariableToConstant* variableToConstant = 0;
	SignedPredicate* signedPredicate = 0;
	vector<Proposition<> >* models = 0;
	VariableToUnsignedInt* variableToUnsignedInt;
	vector<SetOfConstants>* vsoc;
				       
	a.makePartialAssignment(*variableToConstant,
				*signedPredicate,
				0,
				*models,
				0,
				0,
				*variableToUnsignedInt,
				*vsoc);
    }
}
	
