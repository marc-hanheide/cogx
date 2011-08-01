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

#include "gmres.hh"

GMRES::GMRES()
    :A(0),
     b(0)
{
}


void GMRES::set_matrix(Matrix* in)
{
    this->A = in;
}

void GMRES::set_vector(Vector* in)
{
    this->b = in;
}


GMRES::Vector* GMRES::get_answer()
{
    return &x;
}

    
void GMRES::operator()()
{
    assert(A);
    assert(b);
    
    INTERACTIVE_VERBOSER(true, 900, "Making linear operator :: "<<std::endl);
    LinOp< Matrix > T(*A);  
    INTERACTIVE_VERBOSER(true, 900, "Making preconditioner :: "<<std::endl);
    DiagonalPreconditioner< Matrix > prec(*A);
    INTERACTIVE_VERBOSER(true, 900, "Initialising answer :: "<<std::endl);
    x = *b;
    
    INTERACTIVE_VERBOSER(true, 900, "Solving system :: "<<std::endl);
#ifndef NDEBUG
    disable_type_check<bool>::value = true;
#endif
    gmres_restarts< Matrix >(T, x, *b, prec, A->size1()-1, 0, 1.0e-8);

//     gmres_short< Matrix >(T, x, *b, prec, 20, 3*A->size1(), 1.0e-8);
//     gmres_restarts< Matrix >(T, x, *b, prec, 2, 3*A->size1()/2, 1.0e-8);
#ifndef NDEBUG
    disable_type_check<bool>::value = false;
#endif

//   cerr<<x;
  
//   gmres_short< Matrix >(T, x, *b, prec, 20, 3*A->size1(), 1.0e-8);
//   gmres_restarts< Matrix >(T, x, *b, prec, 20, 3*A->size1()/20, 1.0e-8);
  
  //gmres_short< Matrix >(T, x, *b, prec, 20, 3*A->size1(), 1.0e-8);
}
