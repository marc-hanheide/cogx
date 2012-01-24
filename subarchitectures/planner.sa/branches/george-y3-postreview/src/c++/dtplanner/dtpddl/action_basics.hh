
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

#ifndef ACTION_BASICS_HH
#define ACTION_BASICS_HH

#include "state_basics.hh"

namespace Planning
{
    class Probabilistic_State_Transformation;
    class State_Transformation;
    class Simple_Int_Transformation;
    class Simple_Double_Transformation;
    
    typedef CXX__deref__shared_ptr<Probabilistic_State_Transformation> Probabilistic_State_Transformation__Pointer; 
    typedef std::set< Probabilistic_State_Transformation__Pointer > Probabilistic_State_Transformations;       
    typedef std::vector< Probabilistic_State_Transformation__Pointer > List__Probabilistic_State_Transformation;
    
    typedef CXX__deref__shared_ptr<State_Transformation> State_Transformation__Pointer; 
    typedef std::set< State_Transformation__Pointer > State_Transformations;       
    typedef std::vector< State_Transformation__Pointer > List__State_Transformation;
    
    typedef CXX__deref__shared_ptr<Simple_Int_Transformation> Simple_Int_Transformation__Pointer; 
    typedef std::set< Simple_Int_Transformation__Pointer > Simple_Int_Transformations;       
    typedef std::vector< Simple_Int_Transformation__Pointer > List__Simple_Int_Transformation;
    
    typedef CXX__deref__shared_ptr<Simple_Double_Transformation> Simple_Double_Transformation__Pointer; 
    typedef std::set< Simple_Double_Transformation__Pointer > Simple_Double_Transformations;       
    typedef std::vector< Simple_Double_Transformation__Pointer > List__Simple_Double_Transformation;
}

namespace Planning
{
    class Action_Literal;
    
    typedef CXX__deref__shared_ptr<Action_Literal> Action_Literal__Pointer; 
    typedef std::set< Action_Literal__Pointer > Action_Literals;       
    typedef std::vector< Action_Literal__Pointer > List__Action_Literals;

    
    class Action_Disjunctive_Clause;
    
    typedef CXX__deref__shared_ptr<Action_Disjunctive_Clause> Action_Disjunctive_Clause__Pointer; 
    typedef std::set< Action_Disjunctive_Clause__Pointer > Action_Disjunctive_Clauses;       
    typedef std::vector< Action_Disjunctive_Clause__Pointer > List__Action_Disjunctive_Clauses;

    class Action_Conjunctive_Normal_Form_Formula;
    
    typedef CXX__deref__shared_ptr<Action_Conjunctive_Normal_Form_Formula> Action_Conjunctive_Normal_Form_Formula__Pointer; 
    typedef std::set< Action_Conjunctive_Normal_Form_Formula__Pointer > Action_Conjunctive_Normal_Form_Formulae;       
    typedef std::vector< Action_Conjunctive_Normal_Form_Formula__Pointer > List__Action_Conjunctive_Normal_Form_Formulae;
}

namespace std
{
    std::ostream& operator<<(std::ostream&
                             , const Planning::State_Transformation&);
    std::ostream& operator<<(std::ostream&
                             , const Planning::Probabilistic_State_Transformation&);
    std::ostream& operator<<(std::ostream&
                             , const Planning::Simple_Int_Transformation&);
    std::ostream& operator<<(std::ostream&
                             , const Planning::Simple_Double_Transformation&);
    std::ostream& operator<<(std::ostream&
                             , const Planning::State_Transformation&);
    
    std::ostream& operator<<(std::ostream&
                             , const Planning::Action_Literal&);
    std::ostream& operator<<(std::ostream&
                             , const Planning::Action_Disjunctive_Clause&);
    std::ostream& operator<<(std::ostream&
                             , const Planning::Action_Conjunctive_Normal_Form_Formula&);
}


#endif

/* Did he fall?
 * 
 * By his body's known weight of eleven-stone-and-four-pounds in
 * avoirdupois measure, as certified by the graduated machine for
 * periodical selfweighing in the premises of Francis Froedman,
 * pharmaceutical chemist of 19 Fredrick street, north, on the last
 * feast of the Ascension, to wit, the twelfth day of May of the
 * bissextile year one-thousand-nine-hundred-and-four of the christian
 * era, (jewish era five-thousand-six-hundred-and-sixty-four,
 * mohammadan era one-thousand-three-hundred-and-twentytwo), golden
 * number 5, epact 13, solar cycle 9, dominical letters C B, Roman
 * indication 2, Julian period 6617, MXMIV.
 *
 *  -- James Joyce, Ulysses (Ed. 1922)
 */
