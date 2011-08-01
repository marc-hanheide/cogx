/* Copyright (C) 2010 Charles Gretton (charles.gretton@gmail.com)
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
#ifndef PLANNING_DERIVED_PREDICATE__TEMPLATES_HH
#define PLANNING_DERIVED_PREDICATE__TEMPLATES_HH

namespace Planning
{
    template<int ID_VAL, typename NAMING_TYPE>
    NAMING_TYPE First_Order_Derived_Symbol_Header<ID_VAL,NAMING_TYPE>
    ::get__name() const
    {return std::tr1::get<0>(this->contents());}

    template<int ID_VAL, typename NAMING_TYPE>
    Planning::Typed_Arguments First_Order_Derived_Symbol_Header<ID_VAL,NAMING_TYPE>
    ::get__arguments() const 
    {return std::tr1::get<1>(this->contents());}

    
    template<int ID_VAL, typename NAMING_TYPE>
    ostream& First_Order_Derived_Symbol_Header<ID_VAL,NAMING_TYPE>
    ::operator<<(ostream& o) const           
    {
        auto name = std::tr1::get<0>(Parent::contents());
        o<<"("<<name<<" ";

        auto arguments = std::tr1::get<1>(Parent::contents());;

        auto argument_List = std::tr1::get<0>(arguments);
        auto argument_Types = std::tr1::get<1>(arguments);

        assert(argument_List.size() == argument_Types.size());
    
    
        auto argument = argument_List.begin();
        auto type = argument_Types.begin();
        for(; argument != argument_List.end(); argument++, type++){
            o<<*argument<<" - "<<*type<<" ";
        }
        o<<")";
    
        return o;                                            
    } 

    template<int ID_VAL, typename NAMING_TYPE>
    NAMING_TYPE
    First_Order_Derived_Symbol<ID_VAL,NAMING_TYPE>::get__name() const {return std::tr1::get<0>(this->Parent::contents());}
    
    template<int ID_VAL, typename NAMING_TYPE>
    Planning::Typed_Arguments
    First_Order_Derived_Symbol<ID_VAL,NAMING_TYPE>::get__arguments() const {return std::tr1::get<1>(this->Parent::contents());}
    
    template<int ID_VAL, typename NAMING_TYPE>
    Planning::Typed_Arguments
    First_Order_Derived_Symbol<ID_VAL,NAMING_TYPE>::get__quantified_symbols() const {return std::tr1::get<2>(this->Parent::contents());}
    
    template<int ID_VAL, typename NAMING_TYPE>
    Planning::Variables
    First_Order_Derived_Symbol<ID_VAL,NAMING_TYPE>::get__variables() const {return std::tr1::get<3>(this->Parent::contents());}
    
    template<int ID_VAL, typename NAMING_TYPE>
    Formula::Subformula
    First_Order_Derived_Symbol<ID_VAL,NAMING_TYPE>::get__formula() const {return std::tr1::get<4>(this->Parent::contents());}
    
    template<int ID_VAL, typename NAMING_TYPE>
    void
    First_Order_Derived_Symbol<ID_VAL,NAMING_TYPE>::alter__formula(Formula::Subformula subformula)
    {
        std::tr1::get<4>(this->_contents()) = subformula;
    }
    
    template<int ID_VAL, typename NAMING_TYPE>
    int First_Order_Derived_Symbol<ID_VAL,NAMING_TYPE>::get__type() const {return std::tr1::get<5>(this->Parent::contents());}

    template<int ID_VAL, typename NAMING_TYPE>
    std::string First_Order_Derived_Symbol<ID_VAL,NAMING_TYPE>::get__description() const
    {         
        std::ostringstream oss;
    
        oss<<"(:derived ";

        oss<<"("<<get__name()<<" ";

        auto variables = get__symbols(get__arguments());
        auto variables_types = get__types(get__arguments());

        assert(variables.size() == variables_types.size());
        for(uint i = 0; i < variables.size(); i++){

            oss<<variables[i]<<" - (either ";

            for(auto type = variables_types[i].begin()
                    ; type != variables_types[i].end()
                    ; type++){
                oss<<*type<<" ";
            }
        
            oss<<")  ";
        }
    
    
        oss<<" )"<<std::endl;

        switch(get__type()){
            case Planning::enum_types::forall :
            {
                oss<<"(forall (";

            
                auto variables = get__symbols(get__quantified_symbols());
                auto variables_types = get__types(get__quantified_symbols());

                assert(variables.size() == variables_types.size());
                for(uint i = 0; i < variables.size(); i++){

                    oss<<variables[i]<<" - (either ";

                    for(auto type = variables_types[i].begin()
                            ; type != variables_types[i].end()
                            ; type++){
                        oss<<*type<<" ";
                    }
        
                    oss<<")  ";
                }
            
                oss<<")"<<std::endl;
            }
            break;
            case Planning::enum_types::exists :
            {
                oss<<"(exists (";

            
                auto variables = get__symbols(get__quantified_symbols());
                auto variables_types = get__types(get__quantified_symbols());

                assert(variables.size() == variables_types.size());
                for(uint i = 0; i < variables.size(); i++){

                    oss<<variables[i]<<" - (either ";

                    for(auto type = variables_types[i].begin()
                            ; type != variables_types[i].end()
                            ; type++){
                        oss<<*type<<" ";
                    }
        
                    oss<<")  ";
                }
            
                oss<<")"<<std::endl;
            }
            break;
        }
    
    
    
        oss<<get__formula()<<std::endl;

    
        switch(get__type()){
            case Planning::enum_types::forall :
            {
                oss<<")";
            }
            break;
            case Planning::enum_types::exists :
            {
                oss<<")";
            }
            break;
        }
    
    
    
        oss<<")"<<std::endl;

        return oss.str();
    }


    template<int ID_VAL, typename NAMING_TYPE>
    ostream& First_Order_Derived_Symbol<ID_VAL,NAMING_TYPE>::operator<<(ostream&o) const           
    {
        QUERY_UNRECOVERABLE_ERROR(
            !Parent::is_configured(),
            "TRYING TO PRINT UNCONFIGURED PREDICATE."<<std::endl);
    
        auto arguments = get__arguments();
        auto untyped_arguments = std::tr1::get<0>(arguments);
    
        o<<"("<<get__name()/*DERIVED_SYMBOL"<<Parent::get__id()*/<<" ";
        Planning::Formula::Printing::operator<<(o, untyped_arguments);
        o<<" )"<<std::endl;
    
        return o;                                            
    }

    

}


#endif
