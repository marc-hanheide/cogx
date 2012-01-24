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
#ifndef PLANNING_SYMBOLS__TEMPLATES_HH
#define PLANNING_SYMBOLS__TEMPLATES_HH


namespace Planning
{
    
    template<int ID_VAL, typename NAMING_TYPE >
    NAMING_TYPE 
    Planning
    ::Typed_First_Order_Symbol_Description<ID_VAL
                                     , NAMING_TYPE>
    ::get__name() const
    {
        return std::tr1::get<0>(Parent::contents());
    }
    
    template<int ID_VAL, typename NAMING_TYPE >
    NAMING_TYPE 
    Planning
    ::First_Order_Symbol_Description<ID_VAL
                                     , NAMING_TYPE>
    ::get__name() const
    {
        return std::tr1::get<0>(Parent::contents());
    }
    
    template<int ID_VAL, typename NAMING_TYPE >
    Typed_Arguments
    Planning
    ::First_Order_Symbol_Description<ID_VAL
                                     , NAMING_TYPE>
    ::get__arguments() const
    {
        return std::tr1::get<1>(Parent::contents());
    }
    
    template<int ID_VAL, typename NAMING_TYPE >
    Typed_Arguments
    Planning
    ::Typed_First_Order_Symbol_Description<ID_VAL
                                     , NAMING_TYPE>
    ::get__arguments() const
    {
        return std::tr1::get<1>(Parent::contents());
    }
    
    template<int ID_VAL, typename NAMING_TYPE >
    Types
    Planning
    ::Typed_First_Order_Symbol_Description<ID_VAL
                                     , NAMING_TYPE>
    ::get__range_description() const
    {
        return std::tr1::get<2>(Parent::contents());
    }
    
    
    template<int ID_VAL, typename NAMING_TYPE >
    std::ostream& Planning
    ::First_Order_Symbol_Description<ID_VAL
                                     , NAMING_TYPE>
    ::operator<<(std::ostream& o) const
    {

        auto name = std::tr1::get<0>(Parent::contents());
        o<<"("<<name<<" ";

        auto arguments = std::tr1::get<1>(Parent::contents());;

        o<<arguments;
        
        o<<")";
    
        return o;
    }
    
    template<int ID_VAL, typename NAMING_TYPE >
    std::ostream& Planning
    ::Typed_First_Order_Symbol_Description<ID_VAL
                                     , NAMING_TYPE>
    ::operator<<(std::ostream& o) const
    {    
        auto name = std::tr1::get<0>(Parent::contents());
        o<<"("<<name<<" ";

        auto arguments = std::tr1::get<1>(Parent::contents());

        o<<arguments;
        
        o<<")";

        auto types = std::tr1::get<2>(Parent::contents());

        o<<" - "<<types<<" ";
    
        return o;
    }
}


#endif
