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
#include "global.hh"

using std::ostringstream;
using std::endl;
using std::string;



Are_Doubles_Close::Are_Doubles_Close(double epsilon)
    :epsilon(epsilon)
{
}

#include "boost/test/floating_point_comparison.hpp"

bool Are_Doubles_Close::operator()(double number1, double number2) const
{
    assert(epsilon > 0.0);
    
    if(number1 == 0.0 && number2 != 0.0){
        VERBOSER(150, "number 1 :: "<<number1<<"");
        return boost::test_tools::tt_detail::fpt_abs(number2) < epsilon;
    }

    if(number2 == 0.0 && number1 != 0.0){
        return boost::test_tools::tt_detail::fpt_abs(number1) < epsilon;
    }
    
    
    auto diff = boost::test_tools::tt_detail::fpt_abs( number1 - number2 );
    auto d1   = boost::test_tools::tt_detail::safe_fpt_division( diff, boost::test_tools::tt_detail::fpt_abs( number1 ) );
    auto d2   = boost::test_tools::tt_detail::safe_fpt_division( diff, boost::test_tools::tt_detail::fpt_abs( number2 ) );

    VERBOSER(150, "diff "<<diff);
    VERBOSER(150, "d1 "<<d1);
    VERBOSER(150, "d2 "<<d2);
    
    return ((d1 <= (epsilon) && d2 <= (epsilon)));
}

bool is_admissible_probability(double number)
{
    Are_Doubles_Close are_Doubles_Close(1e-9);

    assert(are_Doubles_Close(0.0, 0.0));
    assert(!are_Doubles_Close(1.0, 0.0));
    assert(are_Doubles_Close(1e-200, 0.0));
    
    bool answer =
        (number <= 1.0 &&
         number >= 0.0) ||
        are_Doubles_Close(number, 1.0) ||
        are_Doubles_Close(number, 0.0);

    return answer;
}
