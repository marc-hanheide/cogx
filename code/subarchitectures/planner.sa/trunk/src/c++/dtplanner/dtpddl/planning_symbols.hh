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
#ifndef PLANNING_SYMBOLS_HH
#define PLANNING_SYMBOLS_HH

#include "global.hh"

#include "stl__typed_thing.hh"
#include "planning_types_enum.hh"

#define PRINTING public: std::ostream& operator<<(std::ostream&) const
#define PRINTING_ PRINTING;

namespace Planning
{
    WRAPPED_STRING__WITH_DECLARATIONS(enum_types::type, Type, PRINTING_);
    WRAPPED_STRING__WITH_DECLARATIONS(enum_types::variable, Variable, PRINTING_);
    WRAPPED_STRING__WITH_DECLARATIONS(enum_types::constant, Constant, PRINTING_);
    WRAPPED_STRING__WITH_DECLARATIONS(enum_types::requirement, Requirement, PRINTING_);


    
    typedef std::set<Constant> Constants;
    typedef std::set<Variable> Variables;
    
    
    typedef std::vector<Variable> Variable_List;
    
    typedef std::set<Requirement> Requirements;
    
    typedef std::set<Type> Types;


    typedef std::map<Planning::Variable, Planning::Constant> Assignment;
    
    typedef std::map<Constant, Types > Constants_Description;
    
    typedef std::vector<Types> Argument_Types;
    typedef /*basic_types__vector*/ std::vector<CXX__deref__shared_ptr__visitable<basic_type> > Argument_List;

    typedef basic_types Arguments;
    
    typedef std::tr1::tuple<Argument_List,  Argument_Types > Typed_Arguments;
    const Argument_List& get__symbols(const Typed_Arguments&in);
    const Argument_Types& get__types(const Typed_Arguments&in);
    Argument_List& get__symbols( Typed_Arguments&in);
    Argument_Types& get__types( Typed_Arguments&in);
    
    typedef std::vector<Constant> Constant_Arguments;

    WRAPPED_STRING__WITH_DECLARATIONS(enum_types::predicate_name, Predicate_Name, PRINTING_);
    WRAPPED_STRING(enum_types::percept_name, Percept_Name);
    WRAPPED_STRING(enum_types::observation_name, Observation_Name);
    WRAPPED_STRING(enum_types::state_function_name, State_Function_Name);
    WRAPPED_STRING(enum_types::perceptual_function_name, Perceptual_Function_Name);
    WRAPPED_STRING(enum_types::domain_name, Domain_Name);
    WRAPPED_STRING(enum_types::problem_name, Problem_Name);
    WRAPPED_STRING(enum_types::action_name, Action_Name);


    template<int ID_VAL, typename NAMING_TYPE>
    class First_Order_Symbol_Description
        : public type_wrapper<ID_VAL
                              , NAMING_TYPE
                              , Typed_Arguments
                              >
    {PRINTING;
    public:
        
        typedef type_wrapper<ID_VAL, NAMING_TYPE, Typed_Arguments> Parent;
    
        
        NAMING_TYPE get__name() const;
        Typed_Arguments get__arguments() const;
    };

    class Predicate_Description
        : public First_Order_Symbol_Description<enum_types::predicate_description
                                                , Predicate_Name> {};
    
    class Percept_Description
        : public First_Order_Symbol_Description<enum_types::percept_description
                                                , Percept_Name> {};

    typedef std::set<Predicate_Description> Predicate_Descriptions ;
    typedef std::set<Percept_Description> Percept_Descriptions ;

    
    template<int ID_VAL, typename NAMING_TYPE>
    class Typed_First_Order_Symbol_Description
        : public type_wrapper<ID_VAL
                              , NAMING_TYPE
                              , Typed_Arguments
                              , Types
                              >
    {PRINTING;
    public:
        typedef type_wrapper<ID_VAL
            , NAMING_TYPE
            , Typed_Arguments
            , Types> Parent;
        
        NAMING_TYPE get__name() const;
        Typed_Arguments get__arguments() const;
        Types get__range_description() const;
    };

    
    class State_Function_Description
        : public Typed_First_Order_Symbol_Description<enum_types::state_function_description
                                                      , State_Function_Name> {};
    
    class Perceptual_Function_Description
        : public Typed_First_Order_Symbol_Description<enum_types::perceptual_function_description
                                                      , Perceptual_Function_Name> {};
    
    typedef std::set<State_Function_Description> State_Function_Descriptions ;
    typedef std::set<Perceptual_Function_Description> Perceptual_Function_Descriptions ;
    
}

#include "planning_symbols__TEMPLATES.hh"

namespace std
{
    ostream& operator<<(ostream&, const Planning::Typed_Arguments&);
}


#endif
