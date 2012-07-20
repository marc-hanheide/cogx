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


#include "policy_iteration_over_information_state_space.hh"

using namespace Planning;

using boost::numeric::ublas::matrix;
using boost::numeric::ublas::permutation_matrix;
using boost::numeric::ublas::zero_vector;
using boost::numeric::ublas::identity_matrix;


template<class T>
bool matrix_inversion__LU_factorisation (const boost::numeric::ublas::matrix<T>& input,
                                         boost::numeric::ublas::matrix<T>& inverse) {

    
    using boost::numeric::ublas::matrix;
    using boost::numeric::ublas::permutation_matrix;
    using boost::numeric::ublas::zero_vector;
    using boost::numeric::ublas::identity_matrix;
    
    using boost::numeric::ublas::lu_factorize;
     
    matrix<T> A(input);
    permutation_matrix<std::size_t> pm(A.size1());
     
    bool result = static_cast<bool>(lu_factorize(A,pm));
    if( result != 0 ) return false;
     
    inverse.assign(boost::numeric::ublas::identity_matrix<T>(A.size1()));
     
    lu_substitute(A, pm, inverse);
     
    return true;
}

Policy_Iteration::Policy_Iteration(Set_Of_POMDP_State_Pointers& states, 
                                   double sink_state_penalty,
                                   double discount_factor)
    :states(states),
     sink_state_penalty(sink_state_penalty), 
     discount_factor(discount_factor)
{
}

boost::numeric::ublas::matrix<double> Policy_Iteration::get_transition_matrix2()
{    
    //boost::numeric::ublas::matrix<double> transition_matrix(dimension, dimension);
    boost::numeric::ublas::matrix<double> transition_matrix(dimension, dimension);
    transition_matrix.assign(identity_matrix<double>(dimension));
//     transition_matrix.assign(boost::numeric::ublas::zero_matrix<double>(dimension, dimension));
//     transition_matrix -= transition_matrix;/*Crappy way of getting a zeroed matrix.*/

    for(auto _state = states.begin()
            ; _state != states.end()
            ; _state++){
        auto state = *_state;
        auto starting_index = state->get__index();
        
        /*Fringe states go to the sink.*/
        if(state->unexpanded()){
            transition_matrix(starting_index, dimension-1) -= discount_factor;
            
//             if(starting_index != (dimension-1)){
//                 transition_matrix(starting_index, dimension-1) = 0.0 - discount_factor;
//             } else {
//                 transition_matrix(starting_index, dimension-1) = 1.0 - discount_factor;
//             }
//             transition_matrix(starting_index, dimension-1) = 1.0;
            continue;
        }
        
        
        
        auto& probabilities = state->get_observation_probabilities_at_prescribed_action();
        auto& successors = state->get_successors_at_prescribed_action();
        
        assert(successors.size() == probabilities.size());

        for(uint successor_index = 0; successor_index < successors.size(); successor_index++){

            transition_matrix(starting_index, successors[successor_index]->get__index())
                -=  discount_factor * probabilities[successor_index];
            
//             transition_matrix
//                 (starting_index,
//                  successors[successor_index]->get__index())
//                 = probabilities[successor_index];
        }

    }

    /*Sink state transition probabilities.*/
//     transition_matrix(dimension-1, dimension-1) = 1.0;
    transition_matrix(dimension-1, dimension-1) -= discount_factor;

//     cerr<<transition_matrix<<std::endl;
    
    return std::move(transition_matrix);
}

boost::numeric::ublas::matrix<double> Policy_Iteration::get_transition_matrix()
{    
    //boost::numeric::ublas::matrix<double> transition_matrix(dimension, dimension);
    boost::numeric::ublas::matrix<double> transition_matrix(dimension, dimension);
    transition_matrix.assign(boost::numeric::ublas::zero_matrix<double>(dimension, dimension));
//     transition_matrix -= transition_matrix;/*Crappy way of getting a zeroed matrix.*/

    for(auto _state = states.begin()
            ; _state != states.end()
            ; _state++){
        auto state = *_state;
        auto starting_index = state->get__index();
        
        /*Fringe states go to the sink.*/
        if(state->unexpanded()){
            transition_matrix(starting_index, dimension-1) = 1.0;
            continue;
        }
        
        
        
        auto& probabilities = state->get_observation_probabilities_at_prescribed_action();
        auto& successors = state->get_successors_at_prescribed_action();
        
        assert(successors.size() == probabilities.size());

        for(uint successor_index = 0; successor_index < successors.size(); successor_index++){
            transition_matrix
                (starting_index,
                 successors[successor_index]->get__index())
                = probabilities[successor_index];
        }

    }

    /*Sink state transition probabilities.*/
    transition_matrix(dimension-1, dimension-1) = 1.0;
        
    return std::move(transition_matrix);
}

boost::numeric::ublas::vector<double> Policy_Iteration::get_reward_vector()
{
    
    boost::numeric::ublas::vector<double> reward_vector(dimension);
    
    for(auto _state = states.begin()
            ; _state != states.end()
            ; _state++){
        auto state = *_state;
        auto index = state->get__index();


        assert(index < reward_vector.size());
        reward_vector[index] = state->get__expected_reward();
    }

    assert(dimension == states.size() + 1);
    assert(dimension == reward_vector.size());
    assert(states.size() < dimension);
//     reward_vector[states.size()] = 0.0;
    reward_vector[states.size()] = sink_state_penalty;
    
    return std::move(reward_vector);
}

void Policy_Iteration::press_greedy_policy()
{
//     std::cerr<<value_vector;
    
    for(auto _state = states.begin()
            ; _state != states.end()
            ; _state++){
        auto state = *_state;
        auto starting_index = state->get__index();

        state->accept_values(value_vector);
    }
}

/*Child's play, but should do for the moment.*/
void Policy_Iteration::operator()()
{
    /*adding a sink state that spins on zero reward.*/
    dimension = states.size() + 1;
    
//     state_transition_matrix = get_transition_matrix();

    if(instantanious_reward_vector.size() != dimension){
        instantanious_reward_vector
            = get_reward_vector();
    }
    
    
    value_vector
        = boost::numeric::ublas::vector<double>(zero_vector<double>(dimension));

 
    boost::numeric::ublas::matrix<double> INVERSE__transition__matrix(dimension, dimension);
     
    /* INVERTING (I - A) */  boost::numeric::ublas::matrix<double> transition__matrix = get_transition_matrix2();// (dimension, dimension);
//     /* INVERTING (I - A) */ transition__matrix.assign(identity_matrix<double>(dimension));
//     /* INVERTING (I - A) */ transition__matrix -= (discount_factor * state_transition_matrix);

//     auto some = get_transition_matrix2();
//     cerr<<state_transition_matrix<<std::endl<<std::endl;
//     cerr<<some<<std::endl<<std::endl;
//     cerr<<transition__matrix<<std::endl<<std::endl;
//     cerr<<some - transition__matrix<<std::endl<<std::endl;
//     exit(0);
    
    
    
    VERBOSER(10501, "Solving system of equations "<<std::endl);
    /* INVERTING  */ if(!matrix_inversion__LU_factorisation(transition__matrix, INVERSE__transition__matrix)){
        /* INVERTING  */     UNRECOVERABLE_ERROR("Unable to take inverse of state-transition matrix.");
        /* INVERTING  */ }
    INTERACTIVE_VERBOSER(true, 10501, "Done solving system of equations "<<std::endl);

    VERBOSER(200, "Matrix is :: "<<transition__matrix<<std::endl);
    VERBOSER(200, "Inverse matrix is :: "<<INVERSE__transition__matrix<<std::endl);
    
    /* INVERSE (I - A) */ //invert(tmp);

    /* \MEMBER{value_vector} ---- (I - A)^{-1} * R */
    value_vector.assign(prod(INVERSE__transition__matrix, instantanious_reward_vector));

    
//     cerr<<transition__matrix<<std::endl<<std::endl;
//     cerr<<value_vector<<std::endl<<std::endl;

//     exit(0);
    
    press_greedy_policy();
    
    VERBOSER(2000, "Instantanious reward vector is :: "<<instantanious_reward_vector<<std::endl);
    
    VERBOSER(200, "Value vector is :: "<<value_vector<<std::endl);
}

