// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#include "Problem.hh"

#include "PredicatesAndPropositions_templates.hh"

using namespace Planning;


Constants& Problem::getObjects(const Types& types, Constants& constants)
{


    
    for(Types::const_iterator type = types.begin()
	    ; type != types.end()
	    ; type++){
	/*Add all the \member{domain} constants.*/
	const vector<uint>& objectsFromType =
	    mirror_typesToObjects[domain.typeIndex[*type]];

	for(uint i = 0; i < objectsFromType.size(); i++){
	    assert(objectsFromType[i] < objects.size());
	    
	    constants.push_back(objects[objectsFromType[i]]);
	}
	
	/*Add all the problem (this) objects*/
	const vector<uint>& constantsFromType =
	    domain.mirror_typesToConstants[domain.typeIndex[*type]];

	for(uint i = 0; i < constantsFromType.size(); i++){
	    assert(constantsFromType[i] < domain.constants.size());
	    
	    constants.push_back(domain.constants[constantsFromType[i]]);//objects[constantsFromType[i]]);
	}
	
    }
    
    return constants;
}

Problem::Problem()
    :parsed_Objects(0)
{
}

void Problem::configureObjects()
{
    if(!parsed_Objects)return;
    
    for(TypeOfSymbols::const_iterator typeToObject = parsed_Objects->begin()
	    ; typeToObject != parsed_Objects->end()
	    ; typeToObject++){

	const UntypedStrings& _objects = typeToObject->second;
	
	for(UntypedStrings::const_iterator _object = _objects.begin()
		; _object != _objects.end()
		; _object++){

	    assert(dynamic_cast<const Constant*>(*_object));
	    
	    const Constant& object = dynamic_cast<const Constant&>(**_object);
	    
	    VERBOSER(2, "Trying for object :: "<<object<<endl);
	    if(objectIndex.find(object) == objectIndex.end()) {
		uint id = objects.size();
		objects.push_back(object);
		objectIndex[object] = id;
		VERBOSER(2, "Success. \n");
	    } else {
		VERBOSER(2, "Failed. \n");
	    }
	}
    }

    assert(!(parsed_Objects->size()) || objectIndex.size());
    
    for(TypeOfSymbols::const_iterator typeToObject = parsed_Objects->begin()
	    ; typeToObject != parsed_Objects->end()
	    ; typeToObject++){
	
	const UntypedStrings& range_objects = typeToObject->second;
	const Types& domain_types = typeToObject->first;

	
	/* A group of types can only be declared to be of one type,
	 * even though the PDDL grammar is more flexible than that.*/
	assert(domain_types.size() == 1 || domain_types.size() == 0);

	if(domain_types.size()){
	    
	    const Type& domain_type = *domain_types.begin();

	    assert(domain.typeIndex.find(domain_type) != domain.typeIndex.end());
	    uint domain_typeId = domain.typeIndex[domain_type];
	
	    assert(domain_typeId < domain.types.size());

	    for(UntypedStrings::const_iterator _range_object = range_objects.begin()
		    ; _range_object != range_objects.end()
		    ; _range_object++){
		
		assert(dynamic_cast<const Constant*>(*_range_object));
		const Constant& range_object
		    = dynamic_cast<const Constant&>(**_range_object);
	    
		assert(objectIndex.find(range_object) != objectIndex.end());
		uint range_objectId = objectIndex[range_object];


		if(typesToObjects.find(domain_typeId) == typesToObjects.end()){
		    typesToObjects[domain_typeId] = SetOfUnsignedInts();
		}


		if(objectsToTypes.find(range_objectId) == objectsToTypes.end()){
		    objectsToTypes[range_objectId] = SetOfUnsignedInts();
		}

		VERBOSER(2, "Adding :: "<<domain_typeId<<" - "<<range_objectId<<endl);
		
		typesToObjects[domain_typeId].insert(range_objectId);
		objectsToTypes[range_objectId].insert(domain_typeId);
	    }
	}
    }
}

void Problem::unwindObjects()
{
    bool madeChange = false;
    
    /*For each object*/
    for(MapIntToInts::const_iterator objectsToType = objectsToTypes.begin()
	    ; objectsToType != objectsToTypes.end()
	    ; objectsToType++){
	
	
	uint domain_objectId = objectsToType->first;
	const SetOfUnsignedInts& range_types = objectsToType->second;
	

	VERBOSER(3, "Looking at object :: "<<domain_objectId<<endl);
	
	vector<uint> additionalTypes;
	
	for(SetOfUnsignedInts::const_iterator range_typeId = range_types.begin()
		; range_typeId != range_types.end()
		; range_typeId++){


	    
	    for(MapIntToInts::const_iterator typeToTypes = domain.typesToTypes.begin()
		    ; typeToTypes != domain.typesToTypes.end()
		    ; typeToTypes ++){
		if(range_types.find(typeToTypes->first) != range_types.end())continue;
		
		SetOfUnsignedInts::const_iterator found = typeToTypes->second.find(*range_typeId);
		if(found != typeToTypes->second.end()){
		    additionalTypes.push_back(typeToTypes->first);
		}
		
	    }
	}

	
	if(additionalTypes.size()){
	    madeChange = true;
	    
	    objectsToTypes[domain_objectId].insert(additionalTypes.begin(),
						       additionalTypes.end());
	    
	    for(uint i =0 ; i < additionalTypes.size(); i++)
		typesToObjects[additionalTypes[i]].insert(domain_objectId);
	}
    }
    
    if(madeChange){
	unwindObjects();
    }
}


void Problem::initialiseMirrors()
{
    
    assert(objects.size() == objectIndex.size());
    //assert(objects.size() > 0);//Don't have to have objects.

    mirror_typesToObjects = vector<vector<uint> >(domain.types.size());
    for(uint i = 0; i < domain.types.size(); i++) mirror_typesToObjects[i] = vector<uint>(0);
    
    for(MapIntToInts::const_iterator typesToObject = typesToObjects.begin()
	    ; typesToObject != typesToObjects.end()
	    ; typesToObject++){
	assert(typesToObject->first < mirror_typesToObjects.size());
	
	mirror_typesToObjects[typesToObject->first] =
	    vector<uint>(typesToObject->second.begin(), typesToObject->second.end());
    }


    
    mirror_objectsToTypes = vector<vector<uint> >(objects.size());
    for(uint i = 0; i < objects.size(); i++) mirror_objectsToTypes[i] = vector<uint>(0);

    
    for(MapIntToInts::const_iterator objectsToType = objectsToTypes.begin()
	    ; objectsToType != objectsToTypes.end()
	    ; objectsToType++){
	assert(objectsToType->first < mirror_objectsToTypes.size());
	
	mirror_objectsToTypes[objectsToType->first] =
	    vector<uint>(objectsToType->second.begin(), objectsToType->second.end());
    }
}



const TypeOfSymbols& Problem::getObjects() const
{
    return *parsed_Objects;
}

const Goal& Problem::getGoal() const
{
    return goal;
}

const StartingState& Problem::getStartingState() const
{
    return  startingState;
}

void Problem::setGoal(const SignedPredicates& predicates)
{
    goal = Goal(predicates.size());
    
    transform(predicates.begin(), predicates.end(),
	      goal.begin(),
	      morph_elements<SignedPredicate, SignedProposition>());    
}

void Problem::setObjects( TypeOfSymbols* typeOfSymbols) 
{
    parsed_Objects = typeOfSymbols;
}


void Problem::setStartingState(const SignedPredicates& signedPredicates)
{
    VERBOSER(1, "We have :: "<<signedPredicates.size()<<" sized starting state.\n");
    
    /*Make room for the starting state.*/
    startingState = StartingState(signedPredicates.size());
    
    transform(signedPredicates.begin(), signedPredicates.end(),
	      startingState.begin(),
	      morph_elements<SignedPredicate, Proposition<> >());
    
}

namespace Planning
{
    ostream& operator<<(ostream& o, const Problem& problem)
    {
	o<<problem.domain<<endl;

	o<<"(define (problem "<<problem.name<<" ) "<<endl;


	o<<"(:domain "<<problem.domain.name<<" ) "<<endl;

	if(problem.parsed_Objects){
	    o<<"(:objects "<<endl;
	    o<<*problem.parsed_Objects<<endl;
	    o<<" ) "<<endl;
	}
	

	
	o<<"(:init "<<endl;
	o<<problem.startingState<<endl;
	o<<" ) "<<endl;

	o<<"(:goal (and "<<endl;
	o<<problem.goal<<endl;
	o<<" ))"<<endl;

	o<<" ) "<<endl;

	
	o<<"*********************************************************************"<<endl;
	
	o<<"\n\n Object indices :: \n ";
	for_each(problem.objectIndex.begin(),
		 problem.objectIndex.end(),
		 print_elements<pair<string, uint> >(o, problem.objectIndex.size()));

	
	o<<"\n\nType of objects :: "<<endl;//<<problem.mirror_typesToTypes<<endl;

	for(uint i = 0; i < problem.mirror_typesToObjects.size(); i++){
	    o<<i<<" := :";
	    for(uint j = 0; j < problem.mirror_typesToObjects[i].size(); j++){
		o<<problem.mirror_typesToObjects[i][j]<<", ";
	    }
	    o<<endl;
	}
	
	o<<"\n\nObject of types :: "<<endl;//<<problem.mirror_typesToTypes<<endl;

	for(uint i = 0; i < problem.mirror_objectsToTypes.size(); i++){
	    o<<i<<" := :";
	    for(uint j = 0; j < problem.mirror_objectsToTypes[i].size(); j++){
		o<<problem.mirror_objectsToTypes[i][j]<<", ";
	    }
	    o<<endl;
	}
	
	
	o<<"*********************************************************************"<<endl;
	
	
	return o;
    }
}
