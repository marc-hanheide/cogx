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

#include "policy_iteration_over_information_state_space__GMRES.hh"

using namespace Planning;

Policy_Iteration__GMRES::
Policy_Iteration__GMRES(Set_Of_POMDP_State_Pointers& states, 
                        double sink_state_penalty,
                        double discount_factor)
    :states(states),
     sink_state_penalty(sink_state_penalty),
     discount_factor(discount_factor),
     old__state_space_size(0),
     converged(false)
{
}

void Policy_Iteration__GMRES::configure_transition_matrix()
{    
    using boost::numeric::ublas::identity_matrix;
    
    if(old__state_space_size != dimension){
        state_transition_matrix = GMRES::Matrix(dimension, dimension);
    }
    
    state_transition_matrix.assign(identity_matrix<double>(dimension));
    
    for(auto _state = states.begin()
            ; _state != states.end()
            ; _state++){
        auto state = *_state;
        auto starting_index = state->get__index();
        
        /*Fringe states go to the sink.*/
        if(state->unexpanded()){
#ifdef LAO_STAR
            state_transition_matrix(starting_index, dimension-1) -= discount_factor;
#else
            state_transition_matrix(starting_index, starting_index) -= discount_factor;
#endif
            
            continue;
        }
        
        auto& probabilities = state->get_observation_probabilities_at_prescribed_action();
        auto& successors = state->get_successors_at_prescribed_action();
        
        assert(successors.size() == probabilities.size());

        for(uint successor_index = 0; successor_index < successors.size(); successor_index++){
            state_transition_matrix(starting_index, successors[successor_index]->get__index())
                -= discount_factor * probabilities[successor_index];
//             state_transition_matrix(starting_index, successors[successor_index]->get__index())
//                 = (starting_index == 
//                    successors[successor_index]->get__index())
//                 ?(-1.0 * discount_factor * probabilities[successor_index])
//                 :(1.0 - discount_factor * probabilities[successor_index]);
        }
    }

//     cerr<<state_transition_matrix<<std::endl;
    
    
//     /*Sink state transition probabilities.*/
    assert(state_transition_matrix(dimension-1, dimension-1) == 1.0);
    state_transition_matrix(dimension-1, dimension-1) -= discount_factor;
    
//     state_transition_matrix(dimension-1, dimension-1) = 0.0;
}

void Policy_Iteration__GMRES::configure_reward_vector()
{
    
    if(old__state_space_size != dimension){
        instantanious_reward_vector = GMRES::Vector(dimension);
    }
    
    
    for(auto _state = states.begin()
            ; _state != states.end()
            ; _state++){
        auto state = *_state;
        auto index = state->get__index();

#ifdef LAO_STAR
        if(state->unexpanded()){
            
            auto& belief = state->get__belief_state();
            double expected_value = 0.0;
            for(auto _atom = belief.begin()
                    ; _atom != belief.end()
                    ; _atom++){
                double probability = _atom->second;
                
                //assert(dynamic_cast<Planning::State*>(_atom->first));
                
                expected_value += probability * _atom->first->get__value();
            }
            
            instantanious_reward_vector[index] = expected_value;
            continue;
        }
#endif
        

        assert(index < instantanious_reward_vector.size());
        instantanious_reward_vector[index] = state->get__expected_reward();
    }

    assert(dimension == states.size() + 1);
    assert(dimension == instantanious_reward_vector.size());
    assert(states.size() < dimension);
    assert(states.size() < instantanious_reward_vector.size());
    
#ifdef LAO_STAR 
    instantanious_reward_vector[states.size()] = 0.0;
#else
    instantanious_reward_vector[states.size()] = sink_state_penalty;
#endif
    
//     instantanious_reward_vector[states.size()] = 0.0;
}

void Policy_Iteration__GMRES::press_greedy_policy()
{
    bool convergence = true;
    for(auto _state = states.begin()
            ; _state != states.end()
            ; _state++){
        auto state = *_state;
        auto starting_index = state->get__index();

        state->accept_values(*value_vector);
    }
}

void Policy_Iteration__GMRES::reset__converged()
{
    converged = false;
}




// /*Child's play, but should do for the moment.*/
// void Policy_Iteration__GMRES::operator()()
// {
//     if(converged){
//         WARNING("Converged!");
//         return;
//     }
    
//     /*adding a sink state that spins on zero reward.*/
//     dimension = states.size() + 1;
    
    
//     configure_transition_matrix();
//     configure_reward_vector();

//     INTERACTIVE_VERBOSER(true, 14000, "Given rewards :: "<<instantanious_reward_vector);

//     INTERACTIVE_VERBOSER(true, 14000, "Given transitions :: "<<state_transition_matrix);
    
    
//     gmres.set_matrix(&state_transition_matrix);
//     gmres.set_vector(&instantanious_reward_vector);
//     gmres();

    
//     value_vector = gmres.get_answer();
// //     cerr<<state_transition_matrix<<std::endl<<std::endl;
// //     cerr<<*value_vector<<std::endl<<std::endl;
// //     exit(0);

    

//     if((old__state_space_size == dimension)){
//         bool values_unchanged = false;
//         for(uint i = 0;i < dimension; i++){
//             assert(i < old__value_vector.size());
//             assert(i < (*value_vector).size());
//             if(old__value_vector(i) != (*value_vector)(i)){
//                 values_unchanged = true;
//                 break;
//             }
//         }
        
//         if(!values_unchanged){
//             converged = true;
//             return ;
//         } else {
//             old__state_space_size = dimension;
//             old__value_vector = *value_vector;
//         }
//     } else {
//         old__state_space_size = dimension;
//         old__value_vector = *value_vector;
//     }
    
    
    
//     press_greedy_policy();
// }

bool Policy_Iteration__GMRES::operator()()
{
    if(converged){
        WARNING("Converged!");
        return false;
    }
    
    /*adding a sink state that spins on zero reward.*/
    dimension = states.size() + 1;
    
    
    configure_transition_matrix();
    configure_reward_vector();

    INTERACTIVE_VERBOSER(true, 14000, "Given rewards :: "<<instantanious_reward_vector);

    INTERACTIVE_VERBOSER(true, 14000, "Given transitions :: "<<state_transition_matrix);
    
    
    gmres.set_matrix(&state_transition_matrix);
    gmres.set_vector(&instantanious_reward_vector);
    gmres();

    
    value_vector = gmres.get_answer();
//     cerr<<state_transition_matrix<<std::endl<<std::endl;
//     cerr<<*value_vector<<std::endl<<std::endl;
//     exit(0);

    
    Are_Doubles_Close are_Doubles_Close(1e-5);
    if((old__state_space_size == dimension)){
        bool values_unchanged = true;
        for(uint i = 0;i < dimension; i++){
            assert(i < old__value_vector.size());
            assert(i < (*value_vector).size());
            if(!are_Doubles_Close(old__value_vector(i),
                                  (*value_vector)(i))){//old__value_vector(i) != (*value_vector)(i)){
                values_unchanged = false;
                break;
            }
        }
        
        if(values_unchanged){
            converged = true;
            return false;
        } else {
            old__state_space_size = dimension;
            old__value_vector = *value_vector;
        }
    } else {
        old__state_space_size = dimension;
        old__value_vector = *value_vector;
    }
    
    
    
    press_greedy_policy();

    return true;
}

/* "The only sexiest lady in the whole wide world... let's have sex!"
 *
 * Sexy Sexy Lover (extended rap), Modern Talking, May 17, 1999. This
 * song is a testament to the deep and complex thoughtfullness and
 * depth of the deep deep thoughtful and complex message that
 * epitomizes Eurodance -- Like a Verdi Falstaff on steriods and
 * speed. */
