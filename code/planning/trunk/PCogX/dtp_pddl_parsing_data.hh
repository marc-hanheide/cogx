#ifndef DTP_PDDL_PARSING_DATA_HH
#define DTP_PDDL_PARSING_DATA_HH



#include "global.hh"
#include "planning_symbols.hh"

namespace Planning
{
    namespace Parsing
    {   
        
        class Domain_Data;
        class Problem_Data;
        
        /* Setup the typedef for the datastructure to be used by the
         * parser when parsing domain definitions. (see
         * \class{Domain_Data} from \module{dtp_pddl_parsing_data}) */ 
        typedef Domain_Data Domain_Stack;
        
        /* Setup the typedef for the datastructure to be used by the
         * parser when parsing domain definitions. (see
         * \class{Problem_Data} from \module{dtp_pddl_parsing_data}) */
        typedef Problem_Data Problem_Stack;

        
        class Domain_Data
        {
        public:
            friend class Problem_Data;

            const std::string& get__domain_name() const {return domain_name;};
            
            /* The contents of \member{constants} is treated as domain
             * constants from hereon -- unless a further commitment is
             * made. */
            void commit__constants();
            
            /* The contents of \member{types} is treated as domain
             * types from hereon -- unless a further commitment is
             * made. */
            void commit__types();
            
            void report__domain_name(const std::string& str);//{domain_name = str;};
            void report__action_name(const std::string& str);//{action_name = str;};
            void report__perception_name(const std::string& str);//{perception_name = str;};
            void report__predicate_name(const std::string& str);//{predicate_name = str;};
            void report__function_name(const std::string& str);//{function_name = str;};
            
            void add__requirement(const std::string& str);//{domain_requirements.push_back(str);};
            void add__type(const std::string& str);//{types.push_back(str);};
            void add__constant(const std::string& str);//{constants.push_back(str);};
            void add__variable(const std::string& str);//{variables.push_back(str);};

            void add__number(const std::string& str);
            
            void add__domain_predicates();
            
            void add__domain_functions();

            void add__negative_effect();
            
            void add__negative_precondition();
            
            void add__predicate_precondition();


            
            void combine__action_signature();
            
            void combine__perception_signature();

            
            void combine__arguments_description_component();
            
            void combine__predicate_description_component();

            void combine__function_component();

            
            void combine__function_description_component();

            void combine__action_description();
            
            void combine__perception_description();
            
            void reinterpret__predicate_as_perception_action_precondition();
            
            void set_function_range__number();//{function_Range = Function::s_number;};
            void set_function_range__int();//{function_Range = Function::s_int;};
            void set_function_range__double();//{function_Range = Function::s_double;};
        private:
            /* Action symbol that acts as the precondition for a
             * perception (i.e., part of a POMDP observation) -- (see
             * \member{reinterpret__predicate_as_perception_action_precondition}).*/
            Predicate perception_action_precondition;
            
            /* Last number (double, not mixed with a string) parsed
             * (see \member{add__number}).*/
            double number;
            
            /* Captures action name and arguments.*/
            Predicate action_signature;
            
            /* Captures perception name and arguments.*/
            Predicate perception_signature;
            
            /* Preconditions of an action being parse.*/
            Predicates action_precondition;
            
            /* Predicates that occure for the parsed domain (i.e.,
             * parsed from PDDL "(:predicates ... )" bit ).*/
            Predicates domain_predicates;

            /* Functions that occure for the parsed domain (i.e.,
             * parsed from PDDL "(:functions ... )" bit ).*/
            Functions domain_functions;

            /* List of predicates parsed (see \member{TODO}).*/
            Predicates predicates;
            
            /* List of functions parsed (see \member{TODO}).*/
            Functions functions;
            
            //typedef Function::Range Function_Range;
            Function::Range function_Range;
            
            /* Name of action that was last parsed, and has not been
             * used yet to create an \class{Action}.*/
            std::string action_name;

            /* Name of domain, if we are indeed parsing a domain
             * description.*/
            std::string domain_name;

            /* Name of perception (POMDP observation variable) that
             * was last parsed. */
            std::string perception_name;
            
            /* Name of action/predicate that was last parsed, and has not been
             * used yet to create an \class{Predicate}.*/
            std::string predicate_name;
            
            /* Name of action that was last parsed, and has not been
             * used yet to create an \class{Function}.*/
            std::string function_name;
            
            /* List of types that have been parsed and not used yet
             * (i.e., as part of an argument list -- see
             * \member{arguments_Description})*/
            Variables variables;

            /* List of types that have been parsed and not used yet
             * (either in a descrption of the domain types, or
             * otherwise as part of an argument list -- see
             * \member{arguments_Description})*/
            Types types;

            /* List of constants parsed that has not been used yet
             * (either in a descrption of the domain constants, or
             * otherwise in an argument list -- see
             * \member{arguments_Description}).*/
            Constants constants;

            /* PDDL constants (names) available in the domain (i.e.,
             * parsed from PDDL "(:constants ... )" bit ).*/
            Constants domain_constants;

            /* PDDL types (names) availabe in the domain (i.e.,
             * parsed from PDDL "(:types ... )" bit ).*/
            Types domain_types;

            /* PDDL domain requirements (i.e., parsed from PDDL
             * "(:requirements ... )" bit ).*/
            Requirements domain_requirements;

            /* Description -- as parsed so far -- of a the arguments
             * to a symbol (predicat, function, or action).*/
            Arguments_Description arguments_Description;
        public:
            typedef decltype(Domain_Data::domain_name) Identifier;
        };

        class Problem_Data
        {
        public:
            Problem_Data(std::shared_ptr<Domain_Data>& domain_Data);

            const std::string get__problem_name() const {return problem_name;};
            const std::shared_ptr<Domain_Data>& get__domain_Data() const {return domain_Data;};
            
            void reset__domain_Data(std::shared_ptr<Domain_Data>& in__domain_Data);
            void reset__domain_Data(const std::string& domain_name);
            
        private:
            /* Each problem has a name. The
             * \member{domain_Data::domain_name} and
             * \member{problem_name} uniquely identify a problem.*/
            std::string problem_name;

            /*Problem is an instance of a domain.*/
            std::shared_ptr<Domain_Data> domain_Data;
        public:
            
            typedef std::tr1::tuple<Domain_Data::Identifier
                                    , decltype(Problem_Data::problem_name)> Identifier;
        };
        
        /* Each domain is given a unique name. Here we provide a
         * candidate datastructure to get the (parse) data associated
         * with a particular domain.*/
        typedef std::map<Domain_Stack::Identifier
                         , std::shared_ptr<Domain_Stack> > Domain_Name__To__Domain_Data;

        
        
        /* Each problem is identified uniquely in terms of its name,
         * and the domain to which it is associated. Here we provide a
         * candidate datastructure to get the (parse) data associated
         * with a particular problem.*/
        typedef std::map<Problem_Stack::Identifier
                         , std::shared_ptr<Problem_Stack> > Domain_And_Problem_Names__To__Problem_Data;
        
        /* Last problem parsed (if nothing has yet been parsed then
         * assume 0 == problem_stack.use_count()).*/
        extern std::shared_ptr<Problem_Stack> problem_Stack;

        /* Last domain parsed (if nothing has yet been parsed then
         * assume 0 == problem_stack.use_count()).*/
        extern std::shared_ptr<Domain_Stack> domain_Stack;

        /* Applications collection of parsed planning domains (see
         * \module{dtp_pddl_parsing_data.hh} \class{Domain_Data}).*/
        extern Domain_Name__To__Domain_Data domains;
        
        /* Applications collection of parsed planning problems (see
         * \module{dtp_pddl_parsing_data.hh} \class{Problem_Data}).*/
        extern Domain_And_Problem_Names__To__Problem_Data problems;
        
    }
}


#endif
