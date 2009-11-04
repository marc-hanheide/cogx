#ifndef POMDP__FINITE_STATE_CONTROLLER__EVALUATION_HH
#define POMDP__FINITE_STATE_CONTROLLER__EVALUATION_HH

#include "utilities.hh"
#include "cassandra_POMDP__parser.hh"

#include <boost/numeric/ublas/blas.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>


using namespace boost::numeric::ublas;
using boost::numeric::ublas::vector;
using boost::numeric::ublas::lu_factorize;



namespace POMDP
{
    using std::tr1::shared_ptr;
//     using std::vector;
    using POMDP::Parsing::Problem_Data;
    
    using namespace boost::numeric::ublas;
    using boost::numeric::ublas::vector;
    
    /*Entry $n$ give the probability of being in the $n$th state*/
    typedef vector<double> Belief_State;

    class Finite_State_Controller;
    
    class Finite_State_Controller__Index_Management
    {
    public:
        Finite_State_Controller__Index_Management(const Finite_State_Controller& fsc):fsc(fsc){};

        /* Gives the numeric index associated with a state (at
         * \argument{state_index}) and a node (at
         * \argument{node_index}). */
        int compute_index__state_node(int state_index, int node_index) const;
    private:
        const Finite_State_Controller& fsc;
    };
        
    typedef Finite_State_Controller__Index_Management FSC__Index_Management;
        
    class Finite_State_Controller
    {
    public:
        /*\argument{problem_Data} is the
         * POMDP. \argument{number_of_nodes} gives the number of
         * controller nodes.*/
        Finite_State_Controller(shared_ptr<Problem_Data> problem_Data, uint number_of_nodes = 5);

        /* Add space for entries in
         * \member{node_transition_probabilities},
         * \member{action_execution_probabilities}.*/
        void zero_initialise();
        void zero_initialise__node_transition_probabilities();
        void zero_initialise__action_execution_probabilities();
        
        void randomize();
        
        /* starting-node -> action -> observation -> successor-node -> double*/
        vector<vector<vector<vector< double > > > >  node_transition_probabilities;

        /* starting-node -> action -> double*/
        vector<vector< double > > action_execution_probabilities;
        
        uint count_nodes() const;
        
        /*Assigned problem*/
        shared_ptr<Problem_Data> problem_Data;

        /*Number of controller nodes*/
        uint number_of_nodes;

        Finite_State_Controller__Index_Management fsc__Index_Management;
    };

    typedef Finite_State_Controller FSC;
    
    namespace Solving
    {

        class Finite_State_Controller__Randomizer
        {
        public:
            void operator()(FSC&);

        private:
            void randomize__node_transition_probabilities(FSC&);
            void randomize__action_execution_probabilities(FSC&);
        };

        typedef Finite_State_Controller__Randomizer FSC__Randomizer;
        
        class Finite_State_Controller__Evaluator
        {
        public:            
            friend std::ostream& operator<<(std::ostream&,
                                            const Finite_State_Controller__Evaluator&);

            Finite_State_Controller__Evaluator(const FSC&);
            
            /*Value at a belief-state*/
            double operator()(const Belief_State&);
    
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

            /*Controller*/
            const FSC& fsc;
        };
        
        std::ostream& operator<<(std::ostream&,
                                 const Finite_State_Controller__Evaluator&);
        
        typedef Finite_State_Controller__Evaluator FSC__Evaluator;
        
        class Finite_State_Controller__Node_Improvement
        {
        public:
        private:
        };
        
        typedef Finite_State_Controller__Randomizer FSC__Node_Improvement;
        
        class Finite_State_Controller__Improvement
        {   
        public:
            /* For each node of \argument{finite_State_Controller},
             * improve the controller at that node according to
             * \class{Finite_State_Controller__Node_Improvement}*/
            bool operator()(){};
        private:
        };
        
        typedef Finite_State_Controller__Randomizer FSC__Improvement;
        
    }
    
}


#endif
