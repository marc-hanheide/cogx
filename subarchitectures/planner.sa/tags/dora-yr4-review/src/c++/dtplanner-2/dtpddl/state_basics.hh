
/* Copyright (C) 2010 Charles Gretton (charles.gretton@gmail.com)
 *
 * Authorship of this source code was supported by EC FP7-IST grant
 * 215181-CogX.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dear CogX team member :: Please email (charles.gretton@gmail.com)
 * if you make a change to this and commit that change to SVN. In that
 * email, can you please attach the source files you changed as they
 * appeared before you changed them (i.e., fresh out of SVN), and the
 * diff files (*). Alternatively, you could not commit your changes,
 * but rather post me a patch (**) which I will test and then commit
 * if it does not cause any problems under testing.
 *
 * (*) see http://www.gnu.org/software/diffutils/diffutils.html --
 * GNU-09/2009
 *
 * (**) see http://savannah.gnu.org/projects/patch -- GNU-09/2009
 * 
 */


#ifndef STATE_BASICS_HH
#define STATE_BASICS_HH


#include "global.hh"
#include "stl__typed_thing.hh"
#include "stl__deref_tools.hh"
#include "planning_types_enum.hh"



/*The type of the elements in the bit vector of a \class{State} --
 *Should be an unsigned integer.*/
//#define ELEM_TYPE unsigned char
#define ELEM_TYPE unsigned int

/*Number of bits in each element of the bitvector*/
#define SIZE_ELEM (sizeof(ELEM_TYPE) << 3)

#define CEIL(X) ( X % SIZE_ELEM )?( ( X / SIZE_ELEM ) + 1):(X / SIZE_ELEM)
#define FLOOR(X) X / SIZE_ELEM
#define REMAINDER(X) X % SIZE_ELEM

namespace Planning
{
    class Boolean_State;
    class Integer_State;
    class Float_State;
    class Markov_Decision_Process_State;
    class Partially_Observable_Markov_Decision_Process_State;

    /* (see \module{planning_state}) */
    class State;
    
    namespace State_Formula
    {
        class Literal;
        class Disjunctive_Clause;
        class Conjunctive_Normal_Form_Formula;
        class Satisfaction_Listener;
        
        typedef CXX__deref__shared_ptr<basic_type> Satisfaction_Listener__Pointer;
        typedef std::vector<Satisfaction_Listener__Pointer> List__Listeners;
        typedef std::set<Satisfaction_Listener__Pointer> Listeners;
        
        typedef CXX__deref__shared_ptr<Literal> Literal__Pointer; 
        typedef std::set<Literal__Pointer > Literals;       
        typedef std::vector<Literal__Pointer > List__Literals;
        
        typedef CXX__deref__shared_ptr<Disjunctive_Clause> Disjunctive_Clause__Pointer; 
        typedef std::set<Disjunctive_Clause__Pointer > Disjunctive_Clauses;       
        typedef std::vector<Disjunctive_Clause__Pointer > List__Disjunctive_Clauses;     
            
        typedef CXX__deref__shared_ptr<Conjunctive_Normal_Form_Formula> Conjunctive_Normal_Form_Formula__Pointer; 
        typedef std::set<Conjunctive_Normal_Form_Formula__Pointer > Conjunctive_Normal_Form_Formulae;       
        typedef std::vector< Conjunctive_Normal_Form_Formula__Pointer> List__Conjunctive_Normal_Form_Formula;   
    }
    
}

namespace std
{    
    /* (see \module{boolean_state.hh}) */
    std::ostream& operator<<(std::ostream&, const Planning::Boolean_State&);
    
    /* (see \module{float_state.hh}) */
    std::ostream& operator<<(std::ostream&, const Planning::Float_State&);

    /* (see \module{integer_state.hh}) */
    std::ostream& operator<<(std::ostream&, const Planning::Integer_State&);
    
    /* (see \module{markov_decision_process_state.hh}) */
    std::ostream& operator<<(ostream&, const Planning::Markov_Decision_Process_State&);
    
    /* (see \module{planning_state.hh}) */
    std::ostream& operator<<(ostream&, const Planning::State&);

    /* (see \module{boolean_state.hh}) */
    std::size_t hash_value(const Planning::Boolean_State&);

    /* (see \module{integer_state.hh}) */
    std::size_t hash_value(const Planning::Integer_State&);

    /* (see \module{float_state.hh}) */
    std::size_t hash_value(const Planning::Float_State&);

    /* (see \module{markov_decision_process_state.hh}) */
    std::size_t hash_value(const Planning::Markov_Decision_Process_State&);
    
    /* (see \module{planning_state.hh}) */
    std::size_t hash_value(const Planning::State&);
}

namespace std
{
    std::ostream& operator<<(std::ostream&
                             , const Planning::State_Formula::Conjunctive_Normal_Form_Formula&);
    std::ostream& operator<<(std::ostream&
                             , const Planning::State_Formula::Disjunctive_Clause&);
    std::ostream& operator<<(std::ostream&
                             , const Planning::State_Formula::Literal&);
    
}

#endif

/* ...in many of my outside activities I'd taken a tremendous interest
 * in development.. I said: it would be a dream for me. And they
 * discovered then that you had to be an American citizen. So in three
 * days they somehow made me an American citizen. I had to study the
 * booklet all weekend; But anyway it all happened. ... I virtually
 * had my pyjamas engraved with: "President of the World Bank" ... Tom
 * Clausen ---i.e, Alden Winship ("Tom") Clausen--- took the job. And
 * frankly this was very lucky for me.
 *
 *  -- James David Wolfensohn, 9th President of the World Bank, from
 *  July 1, 1995 until June 30, 2005. Talking on the Aljezeera program
 *  "One on One" posted Saturday, July 17, 2010 13:13 Mecca time,
 *  10:13 GMT
 */
