// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#include "Domain.hh"

#include "Action_templates.hh"
#include "PredicatesAndPropositions_templates.hh"

using namespace Planning;

Domain::Domain()
    :parsed_Constants(0),
     parsed_Types(0)
{
}


/* Charles Gretton :: BUG (I think) in gcc requires I explicitly put
 * member function specialisation in a namespace.*/
namespace Planning
{
    
    template <>
    void Domain::addFunction<int>(const PredicateName& predicateName,
				  const Parameters& parameters) {
	integerFunctions.push_back(Function<int>(predicateName, parameters));
    }
    
    template <>
    void Domain::addFunction<uint>(const PredicateName& predicateName,
				   const Parameters& parameters)
    {
	positiveIntegerFunctions.push_back(Function<uint>(predicateName, parameters));
    }

    
    template <>
    void Domain::addFunction<double>(const PredicateName& predicateName,
				     const Parameters& parameters)
    {
	realFunctions.push_back(Function<double>(predicateName, parameters));
    }
    
    template <>
    void Domain::addFunction<float>(const PredicateName& predicateName,
				const Parameters& parameters)
    {
	realFunctions.push_back(Function<double>(predicateName, parameters)); 
    }
}

void Domain::processActionsForSymbolTypes()
{

    /*Assert that we actually have some predicate symbols.*/
    assert(predicates.size() > 0);
    
    /*First-up, we sort some of the \member{predicates} into
     * \member{manyTimeFluentSymbols},
     * \member{decrease_oneTimeFluentSymbols}, and
     * \member{increase_oneTimeFluentSymbols}, by traversing the
     * domain actions.*/
    for(vector<Action<> >::const_iterator _action = actions.begin()
	    ; _action != actions.end()
	    ; _action++){
	const Action<>& action = *_action;

	const SignedPredicates& effects = action.getEffects();

	for(SignedPredicates::const_iterator effect = effects.begin()
		; effect != effects.end()
		; effect++){
	    const SignedPredicate& signedPredicate = **effect;

	    const PredicateName& predicateName = signedPredicate.getName();

	    VERBOSER(26, "Got action effect symbol :: "<<predicateName<<endl);
	    
	    /*If we have already established that \local{predicateName} is a fluent symbol.*/
	    if(manyTimeFluentSymbols.find(predicateName) != manyTimeFluentSymbols.end()){
		continue;/*No point in further classification.*/
	    }
	    
	    /*If the predicate has a positive sign.*/
	    if(signedPredicate.isPositive()){
		if(decrease_oneTimeFluentSymbols.find(predicateName) != decrease_oneTimeFluentSymbols.end()){
		    decrease_oneTimeFluentSymbols.erase(predicateName);
		    manyTimeFluentSymbols.insert(predicateName);
		    
		} else {
		    increase_oneTimeFluentSymbols.insert(predicateName);
		}
	    }
	    /*If the predicate has a negative sign*/
	    else {
		if(increase_oneTimeFluentSymbols.find(predicateName) != increase_oneTimeFluentSymbols.end()){
		    increase_oneTimeFluentSymbols.erase(predicateName);
		    manyTimeFluentSymbols.insert(predicateName);
		    
		} else {
		    decrease_oneTimeFluentSymbols.insert(predicateName);
		}
	    }
	}
    }
    
    
    /*EXTRACT \member{constantSymbols}... Now we loop through all the
     * predicate symbols and store those that don't occur in one of
     * \member{manyTimeFluentSymbols},
     * \member{decrease_oneTimeFluentSymbols}, and
     * \member{increase_oneTimeFluentSymbols} in
     * \member{constantSymbols}.*/
    for(SetOfPredicateNames::const_iterator _predicateName = predicates.begin()
	    ; _predicateName != predicates.end()
	    ; _predicateName++){
	const PredicateName& predicateName = *_predicateName;

	if( manyTimeFluentSymbols.find(predicateName) == manyTimeFluentSymbols.end() &&
	    decrease_oneTimeFluentSymbols.find(predicateName) == decrease_oneTimeFluentSymbols.end() &&
	    increase_oneTimeFluentSymbols.find(predicateName) == increase_oneTimeFluentSymbols.end()){
	    constantSymbols.insert(predicateName);
	}
    }


    VERBOSER(26, " ## We have :: "<<manyTimeFluentSymbols.size()<<" fluent symbols"<<endl
	     <<" ** :: "<<decrease_oneTimeFluentSymbols.size()<<" decrease_oneTimeFluentSymbols"<<endl
	     <<" ** :: "<<increase_oneTimeFluentSymbols.size()<<" increase_oneTimeFluentSymbols"<<endl
	     <<" ** :: "<<constantSymbols.size()<<" constantSymbols"<<endl
	     <<" ** :: "<<constantSymbols<<endl);
    
    VERBOSER(26, "We have :: "<<manyTimeFluentSymbols<<" fluent symbols"<<endl
	     <<" ** :: "<<decrease_oneTimeFluentSymbols<<" decrease_oneTimeFluentSymbols"<<endl
	     <<" ** :: "<<increase_oneTimeFluentSymbols<<" increase_oneTimeFluentSymbols"<<endl
	     <<" ** :: "<<constantSymbols<<" constantSymbols"<<endl);
    DEBUG_GET_CHAR(26);
}

void Domain::configureConstants()
{
    if(!parsed_Constants) return;
    
    for(TypeOfSymbols::const_iterator typeToConstant = parsed_Constants->begin()
	    ; typeToConstant != parsed_Constants->end()
	    ; typeToConstant++){

	const UntypedStrings& _constants = typeToConstant->second;
	
	for(UntypedStrings::const_iterator _constant = _constants.begin()
		; _constant != _constants.end()
		; _constant++){

	    assert(dynamic_cast<const Constant*>(*_constant));
	    
	    const Constant& constant = dynamic_cast<const Constant&>(**_constant);
	    
	    VERBOSER(2, "Trying for constant :: "<<constant<<endl);
	    if(constantIndex.find(constant) == constantIndex.end()) {
		uint id = constants.size();
		constants.push_back(constant);
		constantIndex[constant] = id;
		VERBOSER(2, "Success. \n");
	    } else {
		VERBOSER(2, "Failed. \n");
	    }
	}
    }

    assert(!(parsed_Constants->size()) || constantIndex.size());

    
    for(TypeOfSymbols::const_iterator typeToConstant = parsed_Constants->begin()
	    ; typeToConstant != parsed_Constants->end()
	    ; typeToConstant++){
	
	const UntypedStrings& range_constants = typeToConstant->second;
	const Types& domain_types = typeToConstant->first;

	
	/*A group of types can only be declared to be of one type,
	 *  even though the PDDL grammar is more flexible than that.*/
	assert(domain_types.size() == 1 || domain_types.size() == 0);

	if(domain_types.size()){
	    
	    const Type& domain_type = *domain_types.begin();

	    assert(typeIndex.find(domain_type) != typeIndex.end());
	    uint domain_typeId = typeIndex[domain_type];
	
	    assert(domain_typeId < types.size());

	    for(UntypedStrings::const_iterator _range_constant = range_constants.begin()
		    ; _range_constant != range_constants.end()
		    ; _range_constant++){
		
		assert(dynamic_cast<const Constant*>(*_range_constant));
		const Constant& range_constant
		    = dynamic_cast<const Constant&>(**_range_constant);
	    
		assert(constantIndex.find(range_constant) != constantIndex.end());
		uint range_constantId = constantIndex[range_constant];


		if(typesToConstants.find(domain_typeId) == typesToConstants.end()){
		    typesToConstants[domain_typeId] = SetOfUnsignedInts();
		}


		if(constantsToTypes.find(range_constantId) == constantsToTypes.end()){
		    constantsToTypes[range_constantId] = SetOfUnsignedInts();
		}

		VERBOSER(2, "Adding :: "<<domain_typeId<<" - "<<range_constantId<<endl);
		
		typesToConstants[domain_typeId].insert(range_constantId);
		constantsToTypes[range_constantId].insert(domain_typeId);
	    }
	}
    }
}

void Domain::unwindConstants()
{
    bool madeChange = false;
    /*For each constant*/
    for(MapIntToInts::const_iterator constantsToType = constantsToTypes.begin()
	    ; constantsToType != constantsToTypes.end()
	    ; constantsToType++){
	
	
	uint domain_constantId = constantsToType->first;
	const SetOfUnsignedInts& range_types = constantsToType->second;
	
	
	vector<uint> additionalTypes;
	
	for(SetOfUnsignedInts::const_iterator range_typeId = range_types.begin()
		; range_typeId != range_types.end()
		; range_typeId++){

	    for(MapIntToInts::const_iterator typeToTypes = typesToTypes.begin()
		    ; typeToTypes != typesToTypes.end()
		    ; typeToTypes ++){
		if(range_types.find(typeToTypes->first) != range_types.end())continue;
		
		SetOfUnsignedInts::const_iterator found = typeToTypes->second.find(*range_typeId);
		if(found != typeToTypes->second.end()){
		    additionalTypes.push_back(typeToTypes->first);
		}
		
	    }
	    
	    
// 	    for(SetOfUnsignedInts::const_iterator _range_typeId = typesToTypes[*range_typeId].begin()
// 		    ; _range_typeId != typesToTypes[*range_typeId].end()
// 		    ; _range_typeId++){
// 		if(range_types.find(*_range_typeId) == range_types.end()){
// 		    additionalTypes.push_back(*_range_typeId);
// 		}
// 	    }
	}

	
	if(additionalTypes.size()){
	    madeChange = true;

	    constantsToTypes[domain_constantId].insert(additionalTypes.begin(),
						       additionalTypes.end());
	    
	    for(uint i =0 ; i < additionalTypes.size(); i++)
		typesToConstants[additionalTypes[i]].insert(domain_constantId);
	}
    }
    
	
    
    if(madeChange){
	unwindConstants();
    }
}

void Domain::configureTypes()
{
    if(!parsed_Types)return;
    
    for(TypeOfTypes::const_iterator typeToType = parsed_Types->begin()
	    ; typeToType != parsed_Types->end()
	    ; typeToType++){

	for(Types::const_iterator type = typeToType->first.begin()
		; type != typeToType->first.end()
		; type++){

	    VERBOSER(2, "Trying for type :: "<<*type<<endl);
	    if(typeIndex.find(*type) == typeIndex.end()) {
		uint id = types.size();
		types.push_back(*type);
		typeIndex[*type] = id;
		VERBOSER(2, "Success. \n");
	    } else {
		VERBOSER(2, "Failed. \n");
	    }
	    
	}
	for(Types::const_iterator type = typeToType->second.begin()
		; type != typeToType->second.end()
		; type++){
	    
	    VERBOSER(2, "Trying for type :: "<<*type<<endl);
	    if(typeIndex.find(*type) == typeIndex.end()) {
		uint id = types.size();
		types.push_back(*type);
		typeIndex[*type] = id;
		VERBOSER(2, "Success. \n");
	    } else {
		VERBOSER(2, "Failed. \n");
	    }
	}
    }

    VERBOSER(2, "We have :: "<<parsed_Types->size()
	     <<" entries and :: "<<typeIndex.size()<<" type indices.\n");
    
    assert(!(parsed_Types->size()) || typeIndex.size());
    
    for(TypeOfTypes::const_iterator typeToType = parsed_Types->begin()
	    ; typeToType != parsed_Types->end()
	    ; typeToType++){

	const Types& domain_types = typeToType->first;
	const Types& range_types = typeToType->second;

	/*A group of types can only be declared to be of one type,
	 *  even though the PDDL grammar is more flexible than that.*/
	assert(domain_types.size() == 1 || domain_types.size() == 0);

	if(domain_types.size()){
	    
	    const Type& domain_type = *domain_types.begin();

	    assert(typeIndex.find(domain_type) != typeIndex.end());
	    uint domain_typeId = typeIndex[domain_type];
	
	    assert(domain_typeId < types.size());

	    for(Types::const_iterator range_type = range_types.begin()
		    ; range_type != range_types.end()
		    ; range_type++){
		
		assert(typeIndex.find(*range_type) != typeIndex.end());
		uint range_typeId = typeIndex[*range_type];
	    
		if(typesToTypes.find(domain_typeId) == typesToTypes.end()){
		    typesToTypes[domain_typeId] = SetOfUnsignedInts();
		}

		VERBOSER(2, "Adding :: "<<domain_typeId<<" - "<<range_typeId<<endl);
		
		typesToTypes[domain_typeId].insert(range_typeId);
	    }
	}
    }
}


void Domain::unwindTypes()
{
    bool madeChange = false;
    for(MapIntToInts::const_iterator typeToTypes = typesToTypes.begin()
	    ; typeToTypes != typesToTypes.end()
	    ; typeToTypes++){
	uint domain_typeId = typeToTypes->first;
	const SetOfUnsignedInts& range_types = typeToTypes->second;
	vector<uint> additionalTypes;
	for(SetOfUnsignedInts::const_iterator range_typeId = range_types.begin()
		; range_typeId != range_types.end()
		; range_typeId++){

	    for(SetOfUnsignedInts::const_iterator _range_typeId = typesToTypes[*range_typeId].begin()
		    ; _range_typeId != typesToTypes[*range_typeId].end()
		    ; _range_typeId++){
		if(range_types.find(*_range_typeId) == range_types.end()){
		    additionalTypes.push_back(*_range_typeId);
		}
	    }
	}

	if(additionalTypes.size()){
	    madeChange = true;

	    typesToTypes[domain_typeId].insert(additionalTypes.begin(), additionalTypes.end());
	}
    }

    if(madeChange){
	unwindTypes();
    }
}

void Domain::initialiseMirrors()
{
    assert(types.size() == typeIndex.size());
    //assert(types.size() > 0);//Don't have to have types.

    
    assert(constants.size() == constantIndex.size());
    //assert(constants.size() > 0);//Don't have to have constants.
    
    mirror_typesToTypes = vector<vector<uint> >(types.size());
    for(uint i = 0; i < types.size(); i++) mirror_typesToTypes[i] = vector<uint>(0);
    
    for(MapIntToInts::const_iterator typeToTypes = typesToTypes.begin()
	    ; typeToTypes != typesToTypes.end()
	    ; typeToTypes++){
	assert(typeToTypes->first < mirror_typesToTypes.size());
	
	mirror_typesToTypes[typeToTypes->first] =
	    vector<uint>(typeToTypes->second.begin(), typeToTypes->second.end());
    }


    mirror_typesToConstants = vector<vector<uint> >(types.size());
    for(uint i = 0; i < types.size(); i++) mirror_typesToConstants[i] = vector<uint>(0);
    
    for(MapIntToInts::const_iterator typesToConstant = typesToConstants.begin()
	    ; typesToConstant != typesToConstants.end()
	    ; typesToConstant++){
	assert(typesToConstant->first < mirror_typesToConstants.size());
	
	mirror_typesToConstants[typesToConstant->first] =
	    vector<uint>(typesToConstant->second.begin(), typesToConstant->second.end());
    }


    
    mirror_constantsToTypes = vector<vector<uint> >(constants.size());
    for(uint i = 0; i < constants.size(); i++) mirror_constantsToTypes[i] = vector<uint>(0);

    
    for(MapIntToInts::const_iterator constantsToType = constantsToTypes.begin()
	    ; constantsToType != constantsToTypes.end()
	    ; constantsToType++){
	assert(constantsToType->first < mirror_constantsToTypes.size());
	
	mirror_constantsToTypes[constantsToType->first] =
	    vector<uint>(constantsToType->second.begin(), constantsToType->second.end());
    }

}


void Domain::addAction(const string& name,
		       const Arguments& arguments,
		       const SignedPredicates& precondition,
		       const SignedPredicates& effects)
{
    actions.push_back(Action<>(name, arguments, precondition, effects));
}

void Domain::addAction(const string& name,
		       const Arguments& arguments,
		       const SignedPredicates& precondition,
		       const SignedPredicates& effects,
		       int cost)
{
    actions.push_back(Action<>(name, arguments, precondition, effects, cost));
}

void Domain::addAction(const string& name,
		       const Arguments& arguments,
		       const SignedPredicates& precondition,
		       const SignedPredicates& effects,
		       SignedPredicate* costEvaluator)
{
    actions.push_back(Action<>(name, arguments, precondition, effects, costEvaluator));
}

void Domain::setTypes( TypeOfTypes* types)
{
    parsed_Types = types;   
}

void Domain::setConstants( TypeOfSymbols* constants)
{
    parsed_Constants = constants;
}

namespace Planning
{
    ostream& operator<<(ostream& o, const Domain::Requires& requirements)
    {
	o<<"(:requirements ";

	if(requirements.actionCosts)
	    o<<":action-costs "<<endl;
	if(requirements.equality)
	    o<<":equality "<<endl;
	if(requirements.strips)
	    o<<"strips: "<<endl;
	if(requirements.typing)
	    o<<":typing "<<endl;
	if(requirements. disjunctivePreconditions)
	    o<<":disjunctive-preconditions "<<endl;
	if(requirements.existentiallyQuantifiedPreconditions)
	    o<<":existential-preconditions "<<endl;
	if(requirements.universallyQuantifiedPreconditions)
	    o<<":universal-preconditions "<<endl;
	if(requirements.conditionalEffects)
	    o<<":conditional-effects "<<endl;
	if(requirements.fluents)
	    o<<":fluents "<<endl;
	if(requirements.durativeActions)
	    o<<":durative-actions "<<endl;
	if(requirements.time)
	    o<<":time "<<endl;
	if(requirements.durationInequalities)
	    o<<":duration-inequalities "<<endl;
	if(requirements.continuousEffects)
	    o<<":continuous-effects "<<endl;
	if(requirements.negativePreconditions)
	    o<<":negative-preconditions "<<endl;
	if(requirements.derivedPredicates)
	    o<<"derived-predicates: "<<endl;
	if(requirements.timedInitialLiterals)
	    o<<":timed-initial-literals "<<endl;
	if(requirements.preferences)
	    o<<":preferences "<<endl;
	if(requirements.constraints)
	    o<<":constraints "<<endl;
	
	return o<<" ) ";
    }
    
    ostream& operator<<(ostream& o, const Domain& domain)
    {
	o<<"(define (domain "<<domain.name<<")";

	o<<domain.requires<<endl;

	if(domain.parsed_Types){
	    o<<"(:types "<<endl;
	    o<<*domain.parsed_Types<<endl;
	    o<<"  ) " ;
	}
	

	if(domain.parsed_Constants){
	    o<<"(:constants "<<endl;
	    o<<*domain.parsed_Constants<<endl;
	    o<<" ) ";
	}
	
	o<<"(:predicates "<<endl;
	o<<domain.predicateSpecifications<<endl;//predicates<<endl;
	o<<" ) "<<endl;

	
	o<<"(:functions "<<endl;
	o<<domain.positiveIntegerFunctions<<endl;//predicates<<endl;
	o<<" ) "<<endl;
	
	
	o<<domain.actions<<endl;
	
	o<<" ) ";

	

	o<<"*********************************************************************"<<endl;
	
	o<<"\n\n Type indices :: \n ";
	for_each(domain.typeIndex.begin(),
		 domain.typeIndex.end(),
		 print_elements<pair<string, uint> >(o, domain.typeIndex.size()));

	
	o<<"\n\nType of types :: "<<endl;//<<domain.mirror_typesToTypes<<endl;

	for(uint i = 0; i < domain.mirror_typesToTypes.size(); i++){
	    o<<i<<" := :";
	    for(uint j = 0; j < domain.mirror_typesToTypes[i].size(); j++){
		o<<domain.mirror_typesToTypes[i][j]<<", ";
	    }
	    o<<endl;
	}
	
	
	o<<"*********************************************************************"<<endl;
	
	o<<"\n\n Constant indices :: \n ";
	for_each(domain.constantIndex.begin(),
		 domain.constantIndex.end(),
		 print_elements<pair<string, uint> >(o, domain.constantIndex.size()));

	
	o<<"\n\nType of constants :: "<<endl;//<<domain.mirror_typesToTypes<<endl;

	for(uint i = 0; i < domain.mirror_typesToConstants.size(); i++){
	    o<<i<<" := :";
	    for(uint j = 0; j < domain.mirror_typesToConstants[i].size(); j++){
		o<<domain.mirror_typesToConstants[i][j]<<", ";
	    }
	    o<<endl;
	}
	
	o<<"\n\nConstant of types :: "<<endl;//<<domain.mirror_typesToTypes<<endl;

	for(uint i = 0; i < domain.mirror_constantsToTypes.size(); i++){
	    o<<i<<" := :";
	    for(uint j = 0; j < domain.mirror_constantsToTypes[i].size(); j++){
		o<<domain.mirror_constantsToTypes[i][j]<<", ";
	    }
	    o<<endl;
	}
	
	
	o<<"*********************************************************************"<<endl;
	
// 	for_each(domain.typesToTypes.begin(),
// 		 domain.typesToTypes.end(),
// 		 print_elements<SetOfUnsignedInts >(o, domain.typesToTypes.size()));
	
// 	for_each(domain.mirror_typesToTypes.begin(),
// 		 domain.mirror_typesToTypes.end(),
// 		 print_elements<vector<uint> >(o, domain.mirror_typesToTypes.size()));

	o<<endl;
	
	
	return o;
    }
    
}
