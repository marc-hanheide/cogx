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


#ifndef PARTIALLY_OBSERVABLE_MARKOV_DECISION_PROCESS_STATE_HH
#define PARTIALLY_OBSERVABLE_MARKOV_DECISION_PROCESS_STATE_HH


#include <boost/numeric/ublas/blas.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>

/*Value of states can be reported from this module.*/
#include "gmres.hh"

#include "markov_decision_process_state.hh"

#include "planning_observation.hh"

namespace Planning
{
    
    class Partially_Observable_Markov_Decision_Process_State;
}

namespace std
{
    std::size_t hash_value(const Planning::Partially_Observable_Markov_Decision_Process_State&);
}


namespace Planning
{
    
//     class Partially_Observable_Markov_Decision_Process_State;
    
    std::size_t hash_value(const Planning::Partially_Observable_Markov_Decision_Process_State& in);

    class Partially_Observable_Markov_Decision_Process_State :
        public Expandable
    {
    public:
        /* A POMDP state is useless iff no rewards (negative or
         * positive) can be received in any future from the belief.*/
        bool useless() const;
        
        Partially_Observable_Markov_Decision_Process_State();
        Partially_Observable_Markov_Decision_Process_State(const Partially_Observable_Markov_Decision_Process_State&) = delete;

        Partially_Observable_Markov_Decision_Process_State&
        operator=(const Partially_Observable_Markov_Decision_Process_State&) = delete;
        
        typedef Markov_Decision_Process_State MDP_State;
        typedef Partially_Observable_Markov_Decision_Process_State POMDP_State;

        typedef std::pair< MDP_State*, double> Belief_Atom;
        typedef std::vector<Belief_Atom> Belief_State;
        
        typedef std::map< MDP_State*, double> Searchable_Belief_State;
        
        typedef std::map<Observational_State*, Searchable_Belief_State> Observation_to_Belief;
        typedef std::map<uint,  Observation_to_Belief> Action__to__Observation_to_Belief;

        typedef std::map<uint, std::map<Observational_State*, double> > Normalisation_Factors;
        
        bool operator==(const POMDP_State&) const;
        bool operator<(const POMDP_State&) const;
	std::size_t hash_value() const;

        /* Tests if the state has been expanded or not by examining
         * the size of \member{action_based_successor_driver}. If no
         * actions can be applied at this belief-state, then we
         * suppose it is not expanded. For the moment, because we are
         * dealing, supposedly, with the infinite horizon case, I
         * think this is a reasonable assumption.*/
        bool unexpanded() const;

#ifdef LAO_STAR
        bool get__expansion_attempted() const;
        void set__expansion_attempted();
#endif
        
        void accept_values(boost::numeric::ublas::vector<double>& values);
        bool accept_values(boost::numeric::ublas::compressed_vector< double >& values);
        
        double get__expected_value() const;
        void set__expected_value(double) ;
        
        uint get__index() const;
        void set__index(uint) ;
        
        void initialise__prescribed_action_index();
        uint get__prescribed_action() const;
        
        const std::vector<Observational_State*>& get_observations_at_prescribed_action() const;
        const std::vector<double>& get_observation_probabilities_at_prescribed_action() const;
        const std::vector<POMDP_State*>& get_successors_at_prescribed_action() const;
        
        double get__expected_reward() const;
        
        void add__belief_atom( MDP_State*, double);
        
        const Belief_State& get__belief_state() const;
        
        void push__successor(uint action_index
                             , Observational_State*
                             , POMDP_State*
                             , double);

        const std::vector<Observational_State*>& get__possible_observations_given_action(uint action_index) const;
        POMDP_State* get__successor(uint action_index
                                    , Observational_State*);

        
        const std::vector<POMDP_State*>& get__successors(uint action_index);
        std::vector<POMDP_State*> get__successors();

        const std::vector<uint>& get__action_based_successor_driver() const;
        const std::vector< std::vector<Observational_State*> >& get__observation_based_successor_driver() const;
        const std::vector< std::vector<double> >& get__observation_probabilities() const;
        const std::vector<std::vector< Partially_Observable_Markov_Decision_Process_State*> >& get__successors_under_actions() const;
    private:
        std::vector<uint> action_based_successor_driver;

        /* For each \member{action_based_successor_driver}, we store
         * the observation based successor drivers.*/
        std::vector< std::vector<Observational_State*> > observation_based_successor_driver;
        
        /* Probability of receiving entries in
         * \member{observation_based_successor_driver}.*/
        std::vector< std::vector<double> > observation_probabilities;
        
        std::vector<
            std::vector< Partially_Observable_Markov_Decision_Process_State*> > successors;
        
        /*Expected reward of this POMDP state.*/
        double expected_reward;
        
        /*Expected value of this POMDP state.*/
        double expected_value;

        Belief_State belief_State;

        /*State identifier.*/
        uint index;

        /*Policy identifier.*/
        uint prescribed_action_index;
        
#ifdef LAO_STAR
        /* Have we attempted to expand this state?*/
        bool expansion_attempted;
#endif
    };


    
    
    /*State pointers.*/
    typedef std::tr1::
    unordered_set<Partially_Observable_Markov_Decision_Process_State*
                  , /*state_hash*/deref_hash<Partially_Observable_Markov_Decision_Process_State>
                  ,  deref_equal_to<Partially_Observable_Markov_Decision_Process_State> >
    Set_Of_POMDP_State_Pointers;
    
    typedef Partially_Observable_Markov_Decision_Process_State::POMDP_State POMDP_State;
}

namespace std
{
    std::ostream& operator<<(std::ostream&, const Planning::Partially_Observable_Markov_Decision_Process_State&);
    std::ostream& operator<<(std::ostream&, const Planning::Partially_Observable_Markov_Decision_Process_State::Action__to__Observation_to_Belief&);
}


#endif

/* Bama: Its like when I'm right I'm right, when I'm wrong I could
 *       been right, so I'm still right 'cause I could'a' been
 *       wrong.. you know.. and I'm sorry cause I could be wrong right
 *       now; I could be wrong.. but if I'm right...
 *
 * Marcus: I told you don't shoot nobody; And the first thing you do
 *         when we walk in this motherfucker is shoot somebody!
 *
 * Bama: Cause that's what I do.. I kill motherfuckers!.. you know that.
 *
 * -- Dialogue from the 2005 film "Get Rich or Die Tryin'".
 */
