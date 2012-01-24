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

#include "dtp_pddl_parsing_data_domain.hh"

using namespace Planning::Parsing;

std::ostream& std::operator<<(std::ostream& o, const Planning::Parsing::Domain_Data& data)
{
    using std::endl;
    
    /*First print the domain preamble.*/
    o<<"(define (domain "<<data.get__domain_Name()<<")"<<endl;

    
    o<<"(:requirements "<<endl;
    for(auto requirement_string = data.domain_requirements.begin()
            ; requirement_string != data.domain_requirements.end()
            ; requirement_string++){
        o<<*requirement_string<<endl;
    }
    o<<" ) "<<endl;
    
    o<<")"<<endl;

    
    o<<"(:types "<<endl;
    for(auto d_type = data.types_description.begin()
            ; d_type != data.types_description.end()
            ; d_type++){

        if(d_type->second.size() == 0) continue;
        
        for(auto r_type = d_type->second.begin()
                ; r_type != d_type->second.end()
                ; r_type++){
            o<<d_type->first<<" - "<<*r_type<<endl;
        }

        o<<endl;
    }
    o<<" ) "<<endl;

    
    o<<"(:constants "<<endl;
    for(auto constant = data.constants_Description.begin()
            ; constant != data.constants_Description.end()
            ; constant++){

        QUERY_UNRECOVERABLE_ERROR(!constant->second.size(),
                                  "Constant :: "<<constant->first<<std::endl
                                  <<"was given without a type specification."<<std::endl);
        
        o<<constant->first<<" - (either ";
        
        for(auto r_type = constant->second.begin()
                ; r_type != constant->second.end()
                ; r_type++){
            o<<*r_type<<" ";
        }
        
        o<<")"<<endl;
    }
    o<<" ) "<<endl;    


    o<<"(:predicates "<<endl;
    for(auto predicate = data.predicate_Descriptions.begin()
            ; predicate != data.predicate_Descriptions.end()
            ; predicate++){

        o<<*predicate<<std::endl;
    }
    o<<" ) "<<endl;    

    o<<"(:percepts "<<endl;
    for(auto percept = data.percept_Descriptions.begin()
            ; percept != data.percept_Descriptions.end()
            ; percept++){

        o<<*percept<<std::endl;
    }
    o<<" ) "<<endl;    

    o<<"(:s-functions "<<endl;
    for(auto function = data.state_Function_Descriptions.begin()
            ; function != data.state_Function_Descriptions.end()
            ; function++){

        o<<*function<<std::endl;
    }
    o<<" ) "<<endl;    

    o<<"(:o-functions "<<endl;
    for(auto function = data.perceptual_Function_Descriptions.begin()
            ; function != data.perceptual_Function_Descriptions.end()
            ; function++){

        o<<*function<<std::endl;
    }
    o<<" ) "<<endl;
    
    
    /* Print descriptions for each derived predicate that we generated. */
    for(auto derived_Predicate = data.derived_Predicates.begin()
            ; derived_Predicate != data.derived_Predicates.end()
            ; derived_Predicate++){
        o<<derived_Predicate->get__description()<<std::endl;
    }
    

    /* Print descriptions for each derived predicate that we generated. */
    for(auto derived_Predicate = data.derived_Predicates__artificial.begin()
            ; derived_Predicate != data.derived_Predicates__artificial.end()
            ; derived_Predicate++){
        o<<derived_Predicate->get__description()<<std::endl;
    }
    
    
    for(auto schema = data.action_Schemas.begin()
            ; schema != data.action_Schemas.end()
            ; schema++){
        o<<*schema<<std::endl;
    }
    
    for(auto schema = data.observation_Schemas.begin()
            ; schema != data.observation_Schemas.end()
            ; schema++){
        o<<*schema<<std::endl;
    }
    
    
    
    o<<")"<<endl;

    
    
    return o;
}
