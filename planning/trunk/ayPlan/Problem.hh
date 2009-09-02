// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef PROBLEM_HH
#define PROBLEM_HH

#include "Domain.hh"

namespace Planning
{
    /*PDDL problem (instance of \member{domain}).*/
    class Problem
    {
    public:
	friend ostream& operator<<(ostream& , const Problem& );

	Problem();

	/* Get all the objects associated with \argument{Types} and
	 * place them in the \argument{Constants}.*/
	Constants& getObjects(const Types&, Constants&) ;
	
	/*******************************************************
	 *
	 * A problem is uniquely identified by its \member{name} and is
	 * associated with a domain with the name \member{domainName}.
	 *
	 *
	 *******************************************************/
    
	/* Problems have a string identifier.*/
	std::string name;
    
	/* Name of the domain this is an instance of. Should be equal to
	 * (\member{domain}.\local{name})*/
	std::string domainName;

	/* Generates \member{goal} from the
	 * \argument{SignedPredicates}. Ensures that all the elements
	 * in \argument{SignedPredicates} are indeed
	 * \class{Proposition}s.*/
	void setGoal(const SignedPredicates&);
	
	/* Setup \member{startingState}. */
	void setStartingState(const SignedPredicates&);
	
	/*Argument is assigned to \member{parsed_Objects}*/
	void setObjects(TypeOfSymbols*);

	/* (IGNORED) A "serial" (I.e. -- sequence of actions) plan
	 * must be no longer than this length.*/
	uint serialPlanLength;

	/* (IGNORED) A "parallel" (more than one action can be executed at a given
	 * time, provided their pre- and post-conditions don't collide)
	 * plan must be no longer than this length.*/
	uint parallelPlanLength;
	
	/* \class{Domain} of which the problem is an instance.*/
	Domain domain;


	/*******************************************************
	 *
	 * Objects, how they relate to types, and how types relate to
	 * them.
	 *
	 *******************************************************/
	
	/* Objects (an object "id" is its index in \member{objects}).*/
	Constants objects;

	/* LHS : string, RHS : int. RHS is the index of LHS in \member{objects}*/
	MapStringToInt objectIndex;

	/* Maps a type (as integer; see \member{domain.types}) to
	 * objects (as integer; see \member{objects}).*/
	MapIntToInts typesToObjects;
	
	/* Maps an object (as integer; see \member{objects}) to types
	 * (as integer; see \member{domain.types}).*/
	MapIntToInts objectsToTypes;

	/* Compact \member{typesToObjects} mirror.*/
	vector<vector<uint> > mirror_typesToObjects;
	
	/* Compact \member{objectsToTypes} mirror.*/
	vector<vector<uint> > mirror_objectsToTypes;

	
	
	/*Configure \member{objects}, \member{objectIndex}, and
	 * \member{typesToObjects} from \member{parsed_Objects}.*/
	void configureObjects();

	/* (O -- object, T -- type) O1 may be \subseteq T2 may be
	 * \subseteq T3. Hence O1 \subseteq T3. We want that reasoning
	 * applied to \member{typesToObjects}. NOTE(1):: Call before
	 * \member{unwindObjects()}. NOTE(2):: Configures
	 * \member{objectsToTypes}.*/
	void unwindObjects();



	/*Initialised all member mirrors: \member{mirror_objectsToTypes}, and
	 * \member{mirror_typesToObjects}.*/
	void initialiseMirrors();
	

	/* A PDDL (integer) function can be seen as a mapping from a
	 * proposition to a \type{uint}.*/
	typedef tr1::unordered_map<Proposition<>, uint, boost::hash<Proposition<> > > MapPropositionToUint;
	
	/* PDDL allows one to associate an integer with a ground
	 * symbol. The map is parsed from the description of the
	 * starting-state in the "problem" specification.*/
	MapPropositionToUint startingStateFunctionEvaluation;
	
	/*Accessing private members.*/
	const TypeOfSymbols& getObjects() const;
	const Goal& getGoal() const;
	const StartingState& getStartingState() const;
    private:
	/*******************************************************
	 *
	 * PDDL specification components as parsed.
	 *
	 *
	 *******************************************************/

	
	/* LHS \typename{Types}, RHS should dynamic_cast to
	 * \typename{Constant} symbols.*/
	TypeOfSymbols* parsed_Objects;

	
	/* Goal for this problem is given by a sequence (order unimportant)
	 * of propositional facts that have to be true.*/
	Goal goal;
	
	/* What is the starting state? -- i.e., What is true in the
	 * starting state?*/
	StartingState startingState;
    };
    
    ostream& operator<<(ostream& o, const Problem&);
}

#endif


/*

  \begin{quote}

  Given the existence as uttered forth in the public works of Puncher
  and Wattman of a personal God quaquaquaqua with white beard
  quaquaquaqua outside time without extension who from the heights of
  divine apathia divine athambia divine aphasia loves us dearly with
  some exceptions for reasons unknown but time will tell and suffers
  like the divine Miranda with those who for reasons unknown but time
  will tell are plunged in torment plunged in fire whose fire flames
  if that continues and who can doubt it will fire the firmament that
  is to say blast hell to heaven so blue still and calm so calm with a
  calm which even though intermittent is better than nothing but not
  so fast and considering what is more that as a result of the labors
  left unfinished crowned by the Acacacacademy of Anthropopopometry of
  Essy-in-Possy of Testew and Cunard it is established beyond all
  doubt all other doubt than that which clings to the labors of men
  that as a result of the labors unfinished of Testew and Cunard it is
  established as hereinafter but not so fast for reasons unknown that
  as a result of the public works of Puncher and Wattman it is
  established beyond all doubt that in view of the labors of Fartov
  and Belcher left unfinished for reasons unknown of Testew and Cunard
  left unfinished it is established what many deny that man in Possy
  of Testew and Cunard that man in Essy that man in short that man in
  brief in spite of the strides of alimentation and defecation wastes
  and pines wastes and pines and concurrently simultaneously what is
  more for reasons unknown in spite of the strides of physical culture
  the practice of sports such as tennis football running cycling
  swimming flying floating riding gliding conating camogie skating
  tennis of all kinds dying flying sports of all sorts autumn summer
  winter winter tennis of all kinds hockey of all sorts penicilline
  and succedanea in a word I resume flying gliding golf over nine and
  eighteen holes tennis of all sorts in a word for reasons unknown in
  Feckham Peckham Fulham Clapham namely concurrently simultaneously
  what is more for reasons unknown but time will tell fades away I
  resume Fullham Clapham in a word the dead loss per head since the
  death of Bishop Berkeley being to the tune of one inch four ounce
  per head approximately by and large more or less to the nearest
  decimal good measure round figures stark naked in the stockinged
  feet in Connemara in a word for reasons unknown no matter what
  matter the facts are there and considering what is more much more
  grave that in the light of the labors lost of Steinweg and Peterman
  it appears what is more much more grave that in the light the light
  the light of the labors lost of Steinweg and Peterman that in the
  plains in the mountains by the seas by the rivers running water
  running fire the air is the same and then the earth namely the air
  and then the earth in the great cold the great dark the air and the
  earth abode of stones in the great cold alas alas in the year of
  their Lord six hundred and something the air the earth the sea the
  earth abode of stones in the great deeps the great cold on sea on
  land and in the air I resume for reasons unknown in spite of the
  tennis the facts are there but time will tell I resume alas alas on
  on in short in fine on on abode of stones who can doubt it I resume
  but not so fast I resume the skull fading fading fading and
  concurrently simultaneously what is more for reasons unknown in
  spite of the tennis on on the beard the flames the tears the stones
  so blue so calm alas alas on on the skull the skull the skull the
  skull in Connemara in spite of the tennis the labors abandoned left
  unfinished graver still abode of stones in a word I resume alas alas
  abandoned unfinished the skull the skull in Connemara in spite of
  the tennis the skull alas the stones Cunard (mêlée, final
  vociferations) tennis . . . the stones . . . so calm . . . Cunard
  . . . unfinished . . .
  
  \end{quote}

  Lucky speaks in Samuel Beckett's Waiting for Godot, pp. 44-7, ISBN:
  0-8021-1821-6 / ISBN-13: 978-0-8021-1821-9, 2006.

  (see \url{http://www.groveatlantic.com/grove/bin/wc.dll?groveproc~genauth~56~5215~EXCERPT})
  
 */
