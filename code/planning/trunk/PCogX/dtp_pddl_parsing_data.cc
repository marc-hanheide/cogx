#include "dtp_pddl_parsing_data.hh"

namespace Planning{
    namespace Parsing
    {
        std::shared_ptr<Problem_Stack> problem_Stack;
        std::shared_ptr<Domain_Stack> domain_Stack;
        
        Domain_Name__To__Domain_Data domains;
        Domain_And_Problem_Names__To__Problem_Data problems;
    }
}

using namespace Planning::Parsing;

void Domain_Data::commit__constants()
{
    domain_constants = std::move(constants);
    constants = Constants();
}
            
void Domain_Data::commit__types()
{
    domain_types = std::move(types);
    types = Types();
}

void Domain_Data::add__domain_predicates()
{
    domain_predicates = std::move(predicates);
    predicates = Predicates();
}
            
void Domain_Data::add__domain_functions()
{
    domain_functions = std::move(functions);
    functions = Functions();
}
            
void Domain_Data::report__domain_name(const std::string& str){domain_name = str;}
void Domain_Data::report__action_name(const std::string& str){action_name = str;}
void Domain_Data::report__perception_name(const std::string& str){perception_name = str;}
void Domain_Data::report__predicate_name(const std::string& str){predicate_name = str;}
void Domain_Data::report__function_name(const std::string& str){function_name = str;}
            
void Domain_Data::add__requirement(const std::string& str){domain_requirements.push_back(str);}
void Domain_Data::add__type(const std::string& str){types.push_back(str);}
void Domain_Data::add__constant(const std::string& str){constants.push_back(str);}
void Domain_Data::add__variable(const std::string& str){variables.push_back(str);}

void Domain_Data::add__number(const std::string& str)
{
    std::istringstream iss(str);
    iss>>number;
    VERBOSER(1, number);
}



void Domain_Data::add__negative_effect()
{
    QUERY_WARNING(1 == predicates.size(),
                  "Negation applied to too many terms. We only support negation"<<std::endl
                  <<"single proposition or predicate at this stage.");
                
    Predicate predicate = std::move(predicates[0]);
    predicates = Predicates();
    predicate.sign = false;

    /*UNIMPLEMENTED -- add the \local{predicate} to the action effects.*/
}
            
            
void Domain_Data::add__negative_precondition(){
    QUERY_WARNING(1 == predicates.size(),
                  "Negation applied to too many terms. We only support negation"<<std::endl
                  <<"single proposition or predicate at this stage.");

    Predicate predicate = std::move(predicates[0]);
    predicates = Predicates();
    predicate.sign = false;
    action_precondition.push_back(std::move(predicate));
}
            
void Domain_Data::add__predicate_precondition(){
    QUERY_WARNING(!predicates.size(),
                  "Adding precondition without having parsed a predicate.");
    assert(predicates.size());
                
    action_precondition.push_back(std::move(predicates[0]));
    predicates = Predicates();
}
            
void Domain_Data::combine__action_signature(){
    action_signature = Predicate(std::move(action_name),
                                 true,
                                 std::move(arguments_Description));
                
    action_name = std::string("");
    arguments_Description = Arguments_Description();
}
            
void Domain_Data::combine__perception_signature()
{
    perception_signature = Predicate(std::move(perception_name),
                                     true,
                                     std::move(arguments_Description));
                
    perception_name = std::string("");
    arguments_Description = Arguments_Description();
}
                

void Domain_Data::combine__arguments_description_component(){

    /* Arguments are either made up of variables or constants.*/
    if(variables.size()){
        VERBOSER(1, "-VARIABLES-> "<<variables.size()<<" :: "<<types.size()<<" :: "<<variables<<" :: "<<types);
        arguments_Description
            .push_back(Typed_Variables_Or_Constants(Typed_Variables(std::move(variables), std::move(types)), Constants()));
    } else {
        assert(constants.size() > 0);

        VERBOSER(1, "-CONSTANTS-> "<<constants);
                    
        arguments_Description
            .push_back(Typed_Variables_Or_Constants(Typed_Variables(Variables(), Types()), std::move(constants)));
    }
                
                
    variables = Variables();
    types = Types();
    constants = Constants();
}
            
void Domain_Data::combine__predicate_description_component()
{
    predicates.insert(Predicate(std::move(predicate_name),
                  true,
                  std::move(arguments_Description)));

    assert(predicates.size());
    VERBOSER(1, predicates[predicates.size() - 1]);

    
    predicate_name = std::string("");
    arguments_Description = Arguments_Description();
}
            

void Domain_Data::combine__function_component()
{
    /*UNIMPLEMENTED -- Lookup the range of the function in
     * the list of functions associated with the domain
     * being parsed. */
    functions.insert(Function(std::move(function_name),
                              true,
                              std::move(arguments_Description)));

                
    assert(functions.size());
    VERBOSER(1, functions[functions.size() - 1]);
    
    function_name = std::string("");
    arguments_Description = Arguments_Description();
}

            
void Domain_Data::combine__function_description_component()
{
    functions.insert(Function(std::move(function_name),
                              true,
                              std::move(arguments_Description),
                              function_Range));
                
    assert(functions.size());
    VERBOSER(1, functions[functions.size() - 1]);
    
    function_name = std::string("");
    arguments_Description = Arguments_Description();
}

void Domain_Data::combine__action_description()
{
    VERBOSER(1, "UNIMPLEMENTED");
}
            
void Domain_Data::combine__perception_description()
{
    VERBOSER(1, "UNIMPLEMENTED");
}
            
void Domain_Data::reinterpret__predicate_as_perception_action_precondition()
{
    assert(1 == predicates.size());
    perception_action_precondition = std::move(predicates[predicates.size() - 1]);
    predicates = Predicates();
}
            
void Domain_Data::set_function_range__number(){function_Range = Function::s_number;}
void Domain_Data::set_function_range__int(){function_Range = Function::s_int;}
void Domain_Data::set_function_range__double(){function_Range = Function::s_double;}



Problem_Data::Problem_Data(std::shared_ptr<Domain_Data>& domain_Data)
    :domain_Data(domain_Data)
{}

void Problem_Data::reset__domain_Data(std::shared_ptr<Domain_Data>& in__domain_Data)
{
    domain_Data = in__domain_Data;
}

void Problem_Data::reset__domain_Data(const std::string& domain_name)
{
    if(domain_name == domain_Data->domain_name){
        WARNING("No change occurred when re-specifying problem associated to a domain.");
        return;
    }
                
    auto domains_iterator = Planning::Parsing::domains.find(domain_name);
    if(domains_iterator != Planning::Parsing::domains.end()){
        reset__domain_Data(domains_iterator->second);
    }
}
