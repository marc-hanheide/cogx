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

#ifndef BASIC_ACTION_HH
#define BASIC_ACTION_HH

#include "state_basics.hh"

namespace Planning
{
    
    /* We do not keep "transformation" (i.e., a PDDL action)
     * preconditions in the transformation, but rather store those
     * separately as CNF formulae. When a formula is true at some
     * given state, then \class{Satisfaction_Listener} that are
     * registered with that formula get notified (see
     * \module{state_formula.hh}). */
    class State_Transformation
    {
    public:
        State_Transformation(bool compulsory = false);
        
        virtual State& operator()(const State&) = 0;

        void report__newly_satisfied(State&);
        void report__newly_unsatisfied(State&);
        
        bool is_compulsory() const;
        void set__compulsory(bool);
    private:
        bool compulsory;
    };

    
    class STRIPS_Action : public State_Transformation
    {
    public:
        State& operator()(const State&);
        void add__add(uint);
        void add__delete(uint);
        
    protected:
        std::vector<uint> add_list;
        std::vector<uint> delete_list;
    };
    

//     class Basic_Functional_Action
//     {
//     public:
        
//     };

//     class Deterministic_Action 
//     {
//     };
}



#endif
