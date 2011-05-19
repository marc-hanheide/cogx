
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

#ifndef OBSERVATION_BASICS_HH
#define OBSERVATION_BASICS_HH

#include "state_basics.hh"
#include "action_basics.hh"


namespace Planning
{
    class Observational_State;
    
    class Observation;
    
    typedef CXX__deref__shared_ptr<Observation> Observation__Pointer; 
    typedef std::set< Observation__Pointer > Observations;       
    typedef std::vector< Observation__Pointer > List__Observation;
    
    class Probabilistic_Observation;
    
    typedef CXX__deref__shared_ptr<Probabilistic_Observation> Probabilistic_Observation__Pointer; 
    typedef std::set< Probabilistic_Observation__Pointer > Probabilistic_Observations;       
    typedef std::vector< Probabilistic_Observation__Pointer > List__Probabilistic_Observation;
}


namespace std
{
    std::ostream& operator<<(std::ostream&
                             , const Planning::Observation&);
    std::ostream& operator<<(std::ostream&
                             , const Planning::Probabilistic_Observation&);
    
    /* (see \module{planning_observation.hh}) */
    std::size_t hash_value(const Planning::Observational_State&);
    
}


#endif
