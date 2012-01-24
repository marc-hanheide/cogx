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

#ifndef GMRES_HH
#define GMRES_HH

#include "global.hh"

#include "LINALG/lin_op.hpp"
#include "LINALG/orthogonal.hpp"
#include "LINALG/precond.hpp"

class GMRES
{
public:

    GMRES();
    
    typedef boost::numeric::ublas::compressed_vector< double > Vector;
    typedef boost::numeric::ublas::compressed_matrix< double, boost::numeric::ublas::column_major > Matrix;
//     typedef boost::numeric::ublas::matrix< double, boost::numeric::ublas::column_major > mat_dense;
//     typedef boost::numeric::ublas::banded_matrix< double, boost::numeric::ublas::column_major > band_matr;

    /*Configure A in Ax=b */
    void set_matrix(Matrix* A);

    /* Configure b in Ax=b */
    void set_vector(Vector* b);

    /*Answer is x from Ax=b */
    Vector* get_answer();

    /*Compute x in Ax=b given A and b. */
    void operator()();
private:
    Matrix* A;
    Vector* b;
    Vector x;
};

#endif
