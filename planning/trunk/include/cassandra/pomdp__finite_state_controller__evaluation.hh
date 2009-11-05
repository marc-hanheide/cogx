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
