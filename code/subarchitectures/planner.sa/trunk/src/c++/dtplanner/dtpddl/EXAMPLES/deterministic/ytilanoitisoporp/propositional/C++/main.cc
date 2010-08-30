
/*---------------------------------

  Usual suspects C++::1998

  ---------------------------------*/

/*IO, string, files*/
#include<string>
#include<iostream>
#include<iomanip>
#include<sstream>
#include<fstream>

/*Storage*/
#include<vector>
#include<map>
#include<set>

/*Sorting, and set operations*/
#include<algorithm>

/*---------------------------------

  Usual suspects C

  ---------------------------------*/
#include<cstddef>
#include<cstdio>
#include<cmath>
#include<cstdlib>
//#include<cmalloc>
#include<cassert>
#include<cctype>
#include<csignal>
#include<cstdarg>
#include<cstddef>
#include<cstring>

/*---------------------------------

  Usual suspects C++::TR1

  ---------------------------------*/

#include<tr1/unordered_map>
#include<tr1/unordered_set>
//#include<tr1/functional>
//#include<boost/functional/hash.hpp>


/*---------------------------------

  Old fashioned debugging

  ---------------------------------*/
#define DEBUG_LEVEL 1
#define DEBUG_GET_CHAR(Y) {if(Y > DEBUG_LEVEL) {char ch; cin>>ch;} }

#define VERBOSE(X) {cerr<<"INFO :: "<<X<<endl;}
#define VERBOSER(Y, X) {if(Y > DEBUG_LEVEL)cerr<<"INFO ("<<Y<<") :: "<<X;}

#define UNRECOVERABLE_ERROR(X) {cerr<<"UNRECOVERABLE ERROR :: "<<X;assert(0);exit(0);}

using namespace std;

/*Number of objects in the problem being generated.*/
uint objectCount = 0;

/*Set of objects that have to be "on" in the goal.*/
set<uint> goals;

/*Maximum cost of flipping a boolean to true.*/
uint MAX_COST = 10;

namespace Problem
{

    /*File that the problem data is written to.*/
    ofstream problemFile;
    
    void preamble()
    {
	problemFile<<"(define (problem ytilanoitisoporp-"<<time(0)<<" )"<<endl;

	problemFile<<"(:domain ytilanoitisoporp)"<<endl;
    }

    void objects()
    {
	assert(objectCount > 0);

	problemFile<<"(:objects "<<endl;
	for(uint i = 1; i <= objectCount; i++){
	    problemFile<<"o"<<i<<" - boolean"<<endl;
	}

	problemFile<<")"<<endl;
    }

    void init()
    {
	problemFile<<"(:init "<<endl;
    
	for(uint i = 1; i <= objectCount; i++){
	    problemFile<<"(off "<<"o"<<i<<" )"<<endl;
	    problemFile<<"(= (switch-cost o"<<i<<" ) "<<random()%MAX_COST<<" )"<<endl;
	
	    if(i + 1 <= objectCount)
		problemFile<<"(related o"<<i<<" o"<<i+1<<" )"<<endl;
	    else
		problemFile<<"(unrelated o"<<i<<" )"<<endl;
	}
    
	problemFile<<"(= (total-cost) 0)"<<endl;
    
	problemFile<<")"<<endl;
    }

    void goal()
    {
	problemFile<<"(:goal "<<endl;

	problemFile<<"(and "<<endl;
    
	for(uint i = 1; i <= objectCount; i++){
	    if(random()%2){
		problemFile<<"(off "<<"o"<<i<<" )"<<endl;
	    } else {
		goals.insert(i);
		problemFile<<"(on "<<"o"<<i<<" )"<<endl;
	    }
	}
    
	problemFile<<")"<<endl;
    
	problemFile<<")"<<endl;
    }


    void writeProblem()
    {
	ostringstream oss;
	oss<<"problem-"<<objectCount<<".pddl";
	problemFile.open (oss.str().c_str(), ifstream::out);

	/*Usual problem preamble.*/
	preamble();
    
	/*List objects.*/
	objects();

	/*List initial state.*/
	init();

	/*Goal state completely specified.*/
	goal();
    
	/*PDDL junk*/
	problemFile<<" (:metric minimize (total-cost))"<<endl;

	problemFile<<")"<<endl;

	problemFile.close();
    }
}


namespace Domain
{
    
    ofstream domainFile;
    
    void writeDomain()
    {
	ostringstream oss;
	oss<<"domain-"<<objectCount<<".pddl";
	domainFile.open (oss.str().c_str(), ifstream::out);

	domainFile<<"(define (domain ytilanoitisoporp)"<<endl
		  <<"(:requirements :action-costs :typing)"<<endl
	    
		  <<"(:types boolean - object)"<<endl
	    
		  <<"(:predicates (on ?b - boolean)"<<endl
		  <<"(off ?b - boolean)"<<endl
		  <<"(related ?b1 ?b2 - boolean)"<<endl
		  <<"(unrelated ?b - boolean))"<<endl
	
	


		  <<"(:functions (total-cost) - number"<<endl
		  <<"(switch-cost ?b - boolean) - number )"<<endl


		  <<"(:action switch-on"<<endl
		  <<":parameters (?b1 ?b2 - boolean)"<<endl
		  <<":precondition (and (off ?b1) (related ?b1 ?b2) )"<<endl
		  <<":effect (and (not (off ?b1)) "<<endl
		  <<"(off ?b2)"<<endl
		  <<" (not (on ?b2))"<<endl
		  <<"(on ?b1)"<<endl
		  <<"(increase (total-cost) (switch-cost ?b1))"<<endl
		  <<")"<<endl
		  <<")"<<endl

		  <<"(:action unrelated-switch-on"<<endl
		  <<":parameters (?b - boolean)"<<endl
		  <<":precondition (and (off ?b) (unrelated ?b) )"<<endl
		  <<":effect (and (not (off ?b)) "<<endl
		  <<"(on ?b)"<<endl
		  <<"(increase (total-cost) (switch-cost ?b))"<<endl
		  <<")"<<endl
		  <<")"<<endl
	    
		  <<"(:action switch-off"<<endl
		  <<":parameters (?b - boolean)"<<endl
		  <<":precondition (and (on ?b) )"<<endl
		  <<":effect (and (not (on ?b))"<<endl
		  <<"(off ?b)"<<endl
		  <<"(increase (total-cost) (switch-cost ?b))"<<endl
		  <<")"<<endl
		  <<")"<<endl;


	domainFile<<"(:action super-switch"<<endl
		  <<":parameters ()"<<endl
		  <<":precondition ()"<<endl
		  <<":effect (and "<<endl;

	for(uint i = 1; i <= objectCount; i++){
	    if(goals.find(i) != goals.end()){
		domainFile<<"(on o"<<i<<" )"<<endl;
		domainFile<<"(not (off o"<<i<<" ))"<<endl;
	    } else {
		domainFile<<"(not (on o"<<i<<" ))"<<endl;
		domainFile<<"(off o"<<i<<" )"<<endl;
	    }
	    
	}
	
	domainFile<<"(increase (total-cost) 1000))) "<<endl<<")"<<endl;    
	    
	domainFile.close();
    }
}

int main(int argc, char** argv)
{
    if(argc <= 1){
	UNRECOVERABLE_ERROR("Please specify the number of objects."<<endl);
    }

    /*_lame_ code for random seed.*/
    unsigned int seed = time(0);
    srand(seed);
    
    /*Get the number of booleans (fist argument).*/
    istringstream iss(argv[1]);
    iss>>objectCount;
    
    Problem::writeProblem();
    Domain::writeDomain();
    
    return 0;
}
