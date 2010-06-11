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
#include "dtp_pddl_parsing_interface.hh"


std::string Planning::Parsing::pddl_preprocessor(const std::string& file_name)
{
    std::string result = "";
    
    char ch;
    
    FILE * file = 0;
    file = fopen(file_name.c_str(), "r");
    {if(!file) UNRECOVERABLE_ERROR("Cannot open file :: "<<file_name);}
    
    {if(ferror(file)) UNRECOVERABLE_ERROR("Cannot open file :: "<<file_name);}
    
    while((ch = getc(file)) != EOF){
        {if(ferror(file)) UNRECOVERABLE_ERROR("reading from :: "<<file_name);}
        
        if(ch == ';'){
            if((ch = getc(file)) == EOF) {
                {if(ferror(file)) UNRECOVERABLE_ERROR("reading from :: "<<file_name)}
                
                break;
            }
            

            if(ch == ';'){
                ch = getc(file);
                {if(ferror(file)) UNRECOVERABLE_ERROR("reading from :: "<<file_name)}
                
                while(ch != EOF && ch != '\n'){
                    ch = getc(file);
                    {if(ferror(file)) UNRECOVERABLE_ERROR("reading from :: "<<file_name)}
                }
                if(ch == '\n') result += "\n";
            } else {
                result += ";";
                result += ch;
            }
        } else {   
            result += ch;
        }
    }
    
    fclose(file);

    return std::move(result);
}

