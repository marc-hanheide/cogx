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


#ifndef ACTION_EXECUTABILITY__STATE_HH
#define ACTION_EXECUTABILITY__STATE_HH


#include "boolean__satisfaction_status_management.hh"
#include "unsigned_integer__status_management.hh"


namespace Planning
{
    
    class Action_Executability__State
    {
    public:
        Action_Executability__State(uint actions_count = 0);
        
        const Boolean__Satisfaction_Status_Management& get__transformation__satisfaction_status() const;
        const Unsigned_Integer__Satisfaction_Status_Management& get__transformation__count_status() const;
        Boolean__Satisfaction_Status_Management& get__transformation__satisfaction_status() ;
        Unsigned_Integer__Satisfaction_Status_Management& get__transformation__count_status() ;
        
    private:
//         /* What actions are executable in this state? This includes
//          * both optional and compulsory transformations.*/
//         Boolean__Satisfaction_Status_Management actions__excitability_status;

        Boolean__Satisfaction_Status_Management transformation__satisfaction_status;
        Unsigned_Integer__Satisfaction_Status_Management transformation__count_status;
    };
    
}


#endif
