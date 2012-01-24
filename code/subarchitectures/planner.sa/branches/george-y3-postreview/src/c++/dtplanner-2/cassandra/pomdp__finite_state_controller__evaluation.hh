/* Copyright (C) 2009 Charles Gretton (charles.gretton@gmail.com)
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


#ifndef POMDP__FINITE_STATE_CONTROLLER__EVALUATION_HH
#define POMDP__FINITE_STATE_CONTROLLER__EVALUATION_HH

#include "utilities.hh"
// #include "cassandra_POMDP__parser.hh"

#include "pomdp__finite_state_controller.hh"

#include <boost/numeric/ublas/blas.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>


// using namespace boost::numeric::ublas;
// using boost::numeric::ublas::vector;
// using boost::numeric::ublas::lu_factorize;



namespace POMDP
{
//     using std::tr1::shared_ptr;
    
//     using boost::numeric::ublas::vector;
    
    /*Entry $n$ give the probability of being in the $n$th state*/
    typedef boost::numeric::ublas::vector<double> Belief_State;

    namespace Solving
    {
        using boost::numeric::ublas::matrix;
        using boost::numeric::ublas::permutation_matrix;
        using boost::numeric::ublas::zero_vector;
        using boost::numeric::ublas::identity_matrix;
        
        class Finite_State_Controller__Evaluator
        {
        public:            
            friend std::ostream& operator<<(std::ostream&,
                                            const Finite_State_Controller__Evaluator&);

            Finite_State_Controller__Evaluator(const FSC&);
            
            /*Value at a belief-state*/
            double operator()(const Belief_State&);
            double operator()(const Belief_State&, int node_index);

            const boost::numeric::ublas::vector<double> get__value_vector() const;

            /* Get the expected reward of executing the policy
             * associated with the FSC node at \argument{node_index}
             * given the state has associated index
             * \member{state_index}.*/
            double get__expected_reward(int state_index, int node_index) const;
            
            /*Compute the value of each controller node at each problem state*/
            void operator()();
        private:
            matrix<double> get_transition_matrix();
            boost::numeric::ublas::vector<double> get_reward_vector();
            
            /* Value for each state-node pair*/
            boost::numeric::ublas::vector<double> value_vector;
            
            /*Number of problem states*/
            int state_count;
            
            /* Number of controller nodes*/
            int node_count;

            /*Dimension of the transition matrix*/
            int matrix_dimension;

            /* Node at which to begin controller exeuction given last
             * call to \member{double operator()(const
             * Belief_State&)}.*/
            int optimal_starting_node;
            
            /*Controller*/
            const FSC& fsc;
        };
        
        std::ostream& operator<<(std::ostream&,
                                 const Finite_State_Controller__Evaluator&);
        
        typedef Finite_State_Controller__Evaluator FSC__Evaluator;
        
    }
    
}


#endif
