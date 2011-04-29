
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


#ifndef PLANNING_OBSERVATION_HH
#define PLANNING_OBSERVATION_HH

#include "observation_basics.hh"

#include "boolean_state.hh"
#include "probability_during_expansion__state.hh"

namespace Planning
{

    class Observational_State :
        public Boolean_State,
        public Probability_During_Expansion_State
    {
    public:
        Observational_State(uint number_of_perceptual_propositions = 0);

        void take__observations__from( Observational_State*);
        void reset__observations();
        uint count__observations() const;
        std::stack<const Observation*>& get__observations();
        const Observation* pop__observation();
        void push__observation(const Observation*);
        
        void take__probabilistic_observations__from( Observational_State*);
        void reset__probabilistic_observations();
        std::stack<const Probabilistic_Observation*>& get__probabilistic_observations();
        uint count__probabilistic_observations() const;
        const Probabilistic_Observation* pop__probabilistic_observation();
        void push__probabilistic_observation(const Probabilistic_Observation*);
        
    private:
        
        void replace__observations(std::stack<const Observation*>&);
        void replace__probabilistic_observations(std::stack<const Probabilistic_Observation*>&);
        
        /*Pending observations. All such observations are compulsory.*/
        std::stack<const Observation*> observations;
        
        /*Pending probabilistic observations. All such observations are compulsory.*/
        std::stack<const Probabilistic_Observation*> probabilistic_observations;


        
        
//         uint get__id() const;
//         void set__id(uint);
//     private:
//         /* Observations ID.*/
//         uint id;
    };

    std::size_t hash_value(const Observational_State&);
    
    /*Observaton pointers.*/
    typedef std::tr1::
    unordered_set<Observational_State*
                  , /*state_hash*/deref_hash<Observational_State>
                  ,  deref_equal_to<Observational_State> > Set_Of_Observational_State_Pointers;

    /*Observaton pointers.*/
    typedef std::set<Observational_State*
                     ,  deref_less<Observational_State> > 
    Non_Hashed_Observational_State_Pointers;
    
}

#endif
