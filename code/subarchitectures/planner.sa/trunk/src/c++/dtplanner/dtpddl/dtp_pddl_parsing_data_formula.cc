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
#include "dtp_pddl_parsing_data_formula.hh"

using namespace Planning::Parsing;


void Formula_Data::report__enter_parsing_initial_state()
{
    parsing_initial_state = true;
}

void Formula_Data::report__exit_parsing_initial_state()
{
    parsing_initial_state = false;
}


Formula_Data::Formula_Data():
    formula_parsing_level(0),
    skip_next____report__formula(false),
    last_number_parsed_was_double(false),
    parsing_initial_state(false)
{
}

void Formula_Data::report__constant_in_formula()
{
    report__dive();
    
    if(subformulae.find(formula_parsing_level) == subformulae.end()){
        VERBOSER(111, "Preparing space to add constant at level ::"<<(formula_parsing_level)<<std::endl);
        subformulae[formula_parsing_level] = Planning::Formula::Subformulae();
        VERBOSER(111, ":: DONE ::"<<std::endl);
    }
    
    assert(subformulae.find(formula_parsing_level) != subformulae.end());
    assert(argument_List.size());
    subformulae[formula_parsing_level].push_back(argument_List.back());
    argument_List = Argument_List();
    
    report__emerge();
}

void Formula_Data::report__object_in_formula()
{
    report__dive();
    
    if(subformulae.find(formula_parsing_level) == subformulae.end()){
        VERBOSER(111, "Preparing space to add variable at level ::"<<(formula_parsing_level)<<std::endl);
        subformulae[formula_parsing_level] = Planning::Formula::Subformulae();
        VERBOSER(111, ":: DONE ::"<<std::endl);
    }
    
    assert(subformulae.find(formula_parsing_level) != subformulae.end());
    assert(argument_List.size());
    assert(argument_List.back().test_cast<Variable>());
    subformulae[formula_parsing_level].push_back(argument_List.back());
    variables.insert(argument_List.back().do_cast_and_copy<Planning::Variable>());
    argument_List = Argument_List();
    
    report__emerge();
}

void Formula_Data::report__number_in_formula()
{
    report__dive();
    
    if(subformulae.find(formula_parsing_level) == subformulae.end()){
        VERBOSER(111, "Preparing space to add number at level ::"<<(formula_parsing_level)<<std::endl);
        subformulae[formula_parsing_level] = Planning::Formula::Subformulae();
        VERBOSER(111, ":: DONE ::"<<std::endl);
    }
    
    auto probability = last_number__double;
    if(!last_number_parsed_was_double){
        probability = static_cast<double>(last_number__int);
    } 
    
    VERBOSER(111, "At level :: "<<formula_parsing_level<<" Got number :: "<<probability<<" in action effect.");
    
    NEW_object_referenced_WRAPPED_deref_POINTER
        (Planning::Formula::Number
         , number
         , probability);
    
    assert(subformulae.find(formula_parsing_level) != subformulae.end());
    subformulae[formula_parsing_level].push_back(number);
    
    report__emerge();
}

void Formula_Data::report__probabilistic_formula()
{
    formula_type.push(probabilistic_effect);
    VERBOSER(111, "(probabilistic NUM (... ) NUM (...)...) at stack element :: "
             <<formula_type.size()<<std::endl);
}

void Formula_Data::report__increase_formula()
{
    formula_type.push(increase);

    VERBOSER(2000, "Got an (increase (... ) NUM) at stack element :: "
             <<formula_type.size()<<std::endl);
}

void Formula_Data::report__decrease_formula()
{
    formula_type.push(decrease);

    VERBOSER(2000, "Got an (decrease (... ) NUM) at stack element :: "
             <<formula_type.size()<<std::endl);
}

void Formula_Data::report__assign_formula()
{
    formula_type.push(assign);

    VERBOSER(2000, "Got an (assign (... ) NUM) at stack element :: "
             <<formula_type.size()<<std::endl);
}


void  Formula_Data::report__equality_formula()
{
    if(parsing_initial_state){
        report__assign_formula();
    } else {
        formula_type.push(equality_test);
        
        VERBOSER(2000, "Got an (= (... ) NUM) at stack element :: "
                 <<formula_type.size()<<std::endl);
    }
}


void Formula_Data::report__skip_next____report__formula()
{
    VERBOSER(41,"Skipping on :: "<<formula_parsing_level);
    skip_next____report__formula = true;
}

void Formula_Data::report__dive(){;
    formula_parsing_level++;
    assert(formula_parsing_level>=0);
    VERBOSER(111, "Got a parenthesis '(' while parsing formula."<<std::endl
             <<"formula_parsing_level is now :: "<<formula_parsing_level<<std::endl)
}
void Formula_Data::report__emerge(){
    formula_parsing_level--;
    assert(formula_parsing_level>=0);
    VERBOSER(111, "Got a parenthesis ')' while parsing formula."
             <<"formula_parsing_level is now :: "<<formula_parsing_level<<std::endl);
}
            

void Formula_Data::report__empty_formula()
{
    VERBOSER(111, "Parsed empty formula.");
    formula_type.push(vacuous);
}

void Formula_Data::report__if_formula()
{
    formula_type.push(material_implication);
}

void Formula_Data::report__not_formula()
{
    formula_type.push(negation);
}

void Formula_Data::report__and_formula()
{
    VERBOSER(22, "Parsing ::CONJUNCT::AND:: at level :: "<<formula_parsing_level<<std::endl);
    formula_type.push(conjunction);
}

void Formula_Data::report__or_formula()
{
    formula_type.push(disjunction);
}

void Formula_Data::report__exists_formula()
{
    formula_type.push(exists);
}

void Formula_Data::report__forall_formula()
{
    formula_type.push(forall);
}

void Formula_Data::report__conditional_effect_formula()
{
    formula_type.push(conditional_effect);
}

CXX__deref__shared_ptr<basic_type>
Formula_Data::complete__probabilistic_formula()
{
    assert(subformulae[formula_parsing_level+1].size());
    
    {/*BEGIN -- In case of parsing or authorship errors.*/
        
        std::ostringstream oss;
        for(auto p = subformulae[formula_parsing_level+1].begin()
                ; p != subformulae[formula_parsing_level+1].end()
                ; p++){
            oss<<*p<<" ";
        }
        std::string str = oss.str();
        
        QUERY_UNRECOVERABLE_ERROR(
            subformulae[formula_parsing_level+1].size() % 2,/*Index starting at 0*/
            "While parsing (:probabilistic num (formula) num (formula)) element."<<std::endl
            <<"Expecting something of the above form, but got :: "<<str<<std::endl
            <<"Which has :: "<<subformulae[formula_parsing_level+1].size()<<" elements."<<std::endl);
    }/*END -- In case of parsing or authorship errors.*/

    Planning::Formula::numbers__vector probabilities;
    Planning::Formula::Subformulae associated_formula;
    
    for(int i = 0
            ; i < subformulae[formula_parsing_level+1].size()
            ; i++){
        auto tmp = subformulae[formula_parsing_level+1][i]; 
        if(!(i%2)){

            QUERY_UNRECOVERABLE_ERROR(
                !tmp.test_cast<Planning::Formula::Number>()
                , "Expecting a number, but got :: "<<tmp<<std::endl);
            probabilities.push_back(tmp.do_cast_and_copy<Planning::Formula::Number>());
        } else {
            associated_formula.push_back(tmp);
        }
    }

    
    NEW_object_referenced_WRAPPED_deref_POINTER
        (Planning::Formula::Probabilistic
         , tmp
         , associated_formula
         , probabilities);

    return tmp;
}


CXX__deref__shared_ptr<basic_type>
Formula_Data::complete__quantified_formula(int quantifier)
{
   
    QUERY_UNRECOVERABLE_ERROR(
        !stack_of__Typed_Arguments.size(),
        "Parsing a quantifier symbol -- e.g. (forall (?a - TYPE ) (formula)) --"<<std::endl
        <<"but no variables -- e.g. (?a - TYPE) -- seem to have been specified.");
    
    
    /*Specification of arguments that were quantified.*/
    auto __quantified_variables = stack_of__Typed_Arguments.top();
    stack_of__Typed_Arguments.pop();/*Using some parsed variable arguments.*/



    QUERY_UNRECOVERABLE_ERROR(
        subformulae.find(formula_parsing_level + 1) == subformulae.end(),
        "Got a quantified varaibles :: "<<__quantified_variables<<" ::  without a subformula.");
    QUERY_UNRECOVERABLE_ERROR(
        subformulae[formula_parsing_level + 1].size() != 1,
        "Got a quantified variables :: "<<__quantified_variables<<" :: without a coherent subformula.");
    
    Planning::Argument_List _quantified_variables
        = Planning::get__symbols(__quantified_variables);
    
//     Planning::Argument_Types variables_types
//         = Planning::get__types(__quantified_variables);

    /* Set of variables that were quantified. Converting vector of
     * pointers to variables to vector of variables. Then storing the
     * result as a set of variables \local{_variables}.*/
    Variables _variables;
    {
        std::vector<Variable> quantified_variables(_quantified_variables.size());
        
        auto i = 0;
        for(auto var = _quantified_variables.begin()
                ; var != _quantified_variables.end()
                ; var++){
            assert(var->test_cast<Variable>());
            assert(i < quantified_variables.size());
            quantified_variables[i++] = var->do_cast_and_copy<Variable>();
        }
        
        /* Variable symbols that this quantifier is talking about.*/
         _variables = Variables(quantified_variables.begin(),
                                quantified_variables.end());
    }

    /* We are making a derived predicate for the quantifier here. The
     * name of that predicate will be given in terms of the quantified
     * formula.*/
    assert(subformulae.find(formula_parsing_level + 1) != subformulae.end());
    assert(subformulae[formula_parsing_level + 1].size());
    std::ostringstream oss;
    oss<<"DERIVED-PREDICATE-"
       <<subformulae[formula_parsing_level + 1][0].get()->get__type_name()
       <<"-"
       <<quantifier
       <<"-"
       <<subformulae[formula_parsing_level + 1][0].get()->get__id();
    NEW_object_referenced_WRAPPED(Planning::Predicate_Name,
                                  name,
                                  oss.str());
    
    for(auto variable = _variables.begin()
            ; variable != _variables.end()
            ; variable++){
        /*This no longer occurs free in the subformula.*/
        this->variables.erase(*variable);
    }
    
    Typed_Arguments derived_predicatex_arguments;
    /*For each variable that was parsed in a subformula.*/
    for(auto variable = this->variables.begin()
            ; variable != this->variables.end()
            ; variable++){
        auto some_types = find__type_of_variable(*variable);

        QUERY_UNRECOVERABLE_ERROR(
            !some_types.size(),
            "Could not find types for variable :: "<<*variable<<std::endl
            <<"While parsing quantifier for :: "<<subformulae[formula_parsing_level + 1].back()<<std::endl);
        
        NEW_object_referenced_WRAPPED_deref_POINTER
            (Planning::Variable, new_variable, variable->get__name());
        Planning::get__symbols(derived_predicatex_arguments).push_back(new_variable);
        Planning::get__types(derived_predicatex_arguments).push_back(some_types);
    }
    
    NEW_object_referenced_WRAPPED_deref_POINTER
        (Planning::Derived_Predicate
         , new_derived_predicate
         , name
         , derived_predicatex_arguments
         , __quantified_variables
         , _variables
         , subformulae[formula_parsing_level + 1].back()
         , quantifier
//          , -1
         );
    
    VERBOSER(41, "GOT :: "<<new_derived_predicate<<std::endl);

    derived_Predicates__artificial
        .insert(*dynamic_cast<Planning::Derived_Predicate*>
                (new_derived_predicate.get().get()));
    
    return new_derived_predicate; 
}

CXX__deref__shared_ptr<basic_type>
Formula_Data::complete__forall_formula()
{
    return complete__quantified_formula(Planning::enum_types::forall);
}

CXX__deref__shared_ptr<basic_type>
Formula_Data::complete__exists_formula()
{
    return complete__quantified_formula(Planning::enum_types::exists);
}










// template<typename PROPOSITION_SYMBOL, typename PREDICATE_SYMBOL>
// report__formula_atomic_symbol<PROPOSITION_SYMBOL, PREDICATE_SYMBOL>()


void  Formula_Data::report__formula_action()
{
    report__formula_atomic_symbol
        <Planning::Formula::Action_Proposition
        , Planning::Formula::Action_Predicate
        , Planning::Action_Name>
        (action_Name);
}

void  Formula_Data::report__formula_percept()
{
    report__formula_atomic_symbol
        <Planning::Formula::Observational_Proposition
        , Planning::Formula::Observational_Predicate
        , Planning::Percept_Name>
        (percept_Name);
}

void  Formula_Data::report__formula_perceptual_function()
{
    report__formula_atomic_symbol
        <Planning::Formula::Perceptual_Ground_Function
        , Planning::Formula::Perceptual_Function
        , Planning::Perceptual_Function_Name>
        (perceptual_Function_Name);
}

void  Formula_Data::report__formula_state_function()
{
    report__formula_atomic_symbol
        <Planning::Formula::State_Ground_Function
        , Planning::Formula::State_Function
        , Planning::State_Function_Name>
        (state_Function_Name);
}

void  Formula_Data::report__formula_predicate()
{
    report__formula_atomic_symbol
        <Planning::Formula::State_Proposition
        , Planning::Formula::State_Predicate
        , Planning::Predicate_Name>
        (predicate_Name);
}

void Formula_Data::report__formula(const std::string& str)
{
    if(skip_next____report__formula){
        VERBOSER(41,"Skipping off :: "<<formula_parsing_level<<" :: "<<str<<std::endl);
        skip_next____report__formula = false;
        return;
    }
    
    
    QUERY_UNRECOVERABLE_ERROR(!formula_type.size(), "Failed to get formula type from :: "
                              <<formula_parsing_level<<" :: "<<str<<std::endl
                              <<"Could be a misspelling, or I stuffed up the operator set"<<std::endl
                              <<"supported by my parser.");
    
    VERBOSER(31, "Got :: "<<formula_type.top()<<std::endl
             <<"From :: "<<str<<std::endl);

    QUERY_UNRECOVERABLE_ERROR
        (formula_parsing_level < 0,
         "We are parsing a formula at level :: "<<formula_parsing_level<<" :: "
         <<str<<" :: I can't accept a negative parse level!"<<std::endl);
    
    
    if(subformulae.find(formula_parsing_level) == subformulae.end()){
        subformulae[formula_parsing_level] = Planning::Formula::Subformulae();
    }
            
    switch(formula_type.top()){
        case vacuous:
        {
            VERBOSER(25, "VACUOUS");
            
            NEW_object_referenced_WRAPPED_deref_POINTER
                (Planning::Formula::Vacuous
                 , tmp
                 , static_cast<void*>(0));
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
            break;
        case forall:
        {
            VERBOSER(25, "FORALL");
            auto tmp = complete__forall_formula();
            subformulae[formula_parsing_level].push_back(tmp);
        }
            break;
        case exists:
        {
            auto tmp = complete__exists_formula();
            subformulae[formula_parsing_level].push_back(tmp);
        }
            break;            
        case disjunction:
        {
            VERBOSER(25, "disjunction");

            check__exists_parsed_subformulae(formula_parsing_level + 1);
    
            NEW_object_referenced_WRAPPED_deref_POINTER
                (Planning::Formula::Disjunction
                 , tmp
                 , subformulae[formula_parsing_level+1]);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
            break;
        case conjunction:
        {
            VERBOSER(25, "conjunction");

            check__exists_parsed_subformulae(formula_parsing_level + 1);
            
            
            NEW_object_referenced_WRAPPED_deref_POINTER
                (Planning::Formula::Conjunction
                 , tmp
                 , subformulae[formula_parsing_level + 1]);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
        break;
        case negation:
        {
            VERBOSER(25, "negation");
            check__exists_parsed_subformulae(formula_parsing_level + 1);
            
            assert(subformulae[formula_parsing_level+1].size() == 1);
            NEW_object_referenced_WRAPPED_deref_POINTER
                (Planning::Formula::Negation
                 , tmp
                 , subformulae[formula_parsing_level+1][0]);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
            break;
        case material_implication:
        {
            VERBOSER(25, "material_implication");
            check__exists_parsed_subformulae(formula_parsing_level + 1);
            check__cardinality_constraint_on_subformulae_at_index(2, formula_parsing_level + 1);
            
            NEW_object_referenced_WRAPPED_deref_POINTER
                (Planning::Formula::Negation
                 , tmp1
                 , subformulae[formula_parsing_level + 1][0]);
            
            subformulae[formula_parsing_level + 1][0] = tmp1;
            
            NEW_object_referenced_WRAPPED_deref_POINTER
                (Planning::Formula::Disjunction
                 , tmp
                 , subformulae[formula_parsing_level + 1]);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
        break;
        case increase:
        {
            check__exists_parsed_subformulae(formula_parsing_level + 1);
            check__cardinality_constraint_on_subformulae_at_index
                (2, formula_parsing_level+1);
            
            
            
            auto subs = subformulae[formula_parsing_level+1].begin();
            auto evaluation_expression_LHS = *subs;
            subs++;
            auto evaluation_expression_RHS = *subs;

            
            NEW_object_referenced_WRAPPED_deref_POINTER
                (Planning::Formula::Increase
                 , tmp
                 , evaluation_expression_LHS
                 , evaluation_expression_RHS);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
        break;
        case decrease:
        {
            check__exists_parsed_subformulae(formula_parsing_level + 1);
            check__cardinality_constraint_on_subformulae_at_index
                (2, formula_parsing_level+1);
            
            auto subs = subformulae[formula_parsing_level+1].begin();
            auto evaluation_expression_LHS = *subs;
            subs++;
            auto evaluation_expression_RHS = *subs;

            
            NEW_object_referenced_WRAPPED_deref_POINTER
                (Planning::Formula::Decrease
                 , tmp
                 , evaluation_expression_LHS
                 , evaluation_expression_RHS);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
        break;
        case assign:
        {
            check__exists_parsed_subformulae(formula_parsing_level + 1);
            check__cardinality_constraint_on_subformulae_at_index
                (2, formula_parsing_level+1);
            
            auto subs = subformulae[formula_parsing_level+1].begin();
            auto evaluation_expression_LHS = *subs;
            subs++;
            auto evaluation_expression_RHS = *subs;

            
            NEW_object_referenced_WRAPPED_deref_POINTER
                (Planning::Formula::Assign
                 , tmp
                 , evaluation_expression_LHS
                 , evaluation_expression_RHS);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
        break;
        case equality_test:
        {
            UNRECOVERABLE_ERROR("unimplemented.");
        }
        break;
        case conditional_effect:
        {
            /* (when (boolean-condition/i.e., precondition)
             * (effect-formula)). As with LAMA, this is to be later
             * treated as an action that is necessarily triggered by
             * the parent effect/action.*/

            
            check__exists_parsed_subformulae(formula_parsing_level + 1);
            check__cardinality_constraint_on_subformulae_at_index
                (2, formula_parsing_level+1);


            auto components = subformulae[formula_parsing_level+1].begin();
            auto precondition_LHS = *components;
            components++;
            auto effect_RHS = *components;            
            
            NEW_object_referenced_WRAPPED_deref_POINTER
                (Planning::Formula::Conditional_Effect
                 , tmp
                 , precondition_LHS
                 , effect_RHS);
            
            subformulae[formula_parsing_level].push_back(tmp);
            
        }
        break;
        case probabilistic_effect:
        {
            auto tmp = complete__probabilistic_formula();
            subformulae[formula_parsing_level].push_back(tmp);
        }
        break;
        default:
            UNRECOVERABLE_ERROR("Unable to parse formula.");
            break;
    }

    /*RESET subformulae.*/
    subformulae[formula_parsing_level + 1] = Planning::Formula::Subformulae();

    if(formula_parsing_level == 1){
        assert(subformulae.find(formula_parsing_level) != subformulae.end());
        assert(subformulae[formula_parsing_level].size());
        VERBOSER(111, "FORMULA :: "<<*subformulae[formula_parsing_level].begin()<<std::endl);
    }
    
    formula_type.pop();
}




void Formula_Data::commit__argument_types()
{
    /*New variables*/
    for(auto i = argument_List.begin(); i != argument_List.end(); i++){
        std::tr1::get<0>(typed_Arguments).push_back(*i);
    }
    
    NEW_object_referenced_WRAPPED(Planning::Type, object_type, "object");
    Types object_types;object_types.insert(object_type);
    
    /*And their types.*/
    for(auto i = argument_List.begin(); i != argument_List.end(); i++){

        //         auto q = i->get().get();
        //         dynamic_cast<Variable*>(q);
        
        if(i->test_cast<Variable>()){//dynamic_cast<Variable*>(i->get())){
            if(types_of_types.size()){    
                std::tr1::get<1>(typed_Arguments).push_back(types_of_types);
            } else { 
                std::tr1::get<1>(typed_Arguments).push_back(object_types);
            }
        } else {
            /* I may-as-well suppose all constant arguments to a
             *  predicate symbol have the type "object".*/
            std::tr1::get<1>(typed_Arguments).push_back(object_types);
        }   
    }
    
    
    argument_List = Argument_List ();
    types_of_types = Types();
}


void Formula_Data::add__number (const std::string& str)
{
    VERBOSER(111, "Parsing a number :: "<<str<<std::endl);
    
    std::istringstream iss(str);
    if(last_number_parsed_was_double){
        iss>>last_number__double;
        VERBOSER(101, "This was a double with value :: "<<last_number__double<<std::endl);
    } else  {
        iss>>last_number__int;
        VERBOSER(101, "This was an int with value :: "<<last_number__int<<std::endl);
    }
}

void Formula_Data::report__parsing_real_number()
{
    last_number_parsed_was_double = true;
}


void Formula_Data::report__parsing_integer_number()
{
    last_number_parsed_was_double = false;
}

void Formula_Data::add__variable_argument(const std::string& str)
{
    NEW_object_referenced_WRAPPED_deref_POINTER(Planning::Variable, variable, str);
    
    argument_List.push_back(variable);
}

void Formula_Data::add__constant_argument(const std::string& str)
{
    NEW_object_referenced_WRAPPED_deref_POINTER(Planning::Constant, constant, str);

    argument_List.push_back(constant);
}


void Formula_Data::report__perceptual_function_name(const std::string& str)
{
    NEW_object_referenced_WRAPPED(Planning::Perceptual_Function_Name, tmp, str);
    perceptual_Function_Name = tmp;
}

void Formula_Data::report__percept_name(const std::string& str)
{
    NEW_object_referenced_WRAPPED(Planning::Percept_Name, tmp, str);
    percept_Name = tmp;
}

void Formula_Data::report__state_function_name(const std::string& str)
{
    NEW_object_referenced_WRAPPED(Planning::State_Function_Name, tmp, str);
    state_Function_Name = tmp;
}


void Formula_Data::report__predicate_name(const std::string& str)
{
    NEW_object_referenced_WRAPPED(Planning::Predicate_Name, tmp, str);
    predicate_Name = tmp;
}

void Formula_Data::stack__typed_Arguments()
{
    VERBOSER(41, "Pushing back quantified arguments :: "<<typed_Arguments);
    stack_of__Typed_Arguments.push(typed_Arguments);
    typed_Arguments = Typed_Arguments();
}

bool Formula_Data::check__exists_parsed_subformulae(int index) const
{
    QUERY_UNRECOVERABLE_ERROR
        (subformulae.find(index) == subformulae.end(),
         "Parsing an expression that requires a subformula :: "
         <<"at level :: "<<formula_parsing_level<<std::endl);
    
    return subformulae.find(index) != subformulae.end();
}

bool Formula_Data::check__cardinality_constraint_on_subformulae_at_index
(int count, int index) const
{
    check__exists_parsed_subformulae(index);
    
    if(subformulae.find(index)->second.size() != count){
        std::ostringstream oss;
        for(auto f = subformulae.find(formula_parsing_level+1)->second.begin()
                ; f != subformulae.find(formula_parsing_level+1)->second.end()
                ; f++){
            oss<<*f;
        }
        
        std::string str = oss.str();
        
        UNRECOVERABLE_ERROR("Expecting :: "<<count
                            <<" subformula for domain element\n"
                            <<"but got :: "<<subformulae.find(formula_parsing_level+1)->second.size()<<std::endl
                            <<"These were :: "<<str<<std::endl);
        
        return false;
    }/* END DEBUG */
    
    return true;
}
