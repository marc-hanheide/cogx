/**
 * $Id$
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */


#include "PoseCv.hh"


namespace P 
{



/***************************** PoseCv ******************************
 * Constructor/Destructor
 */
PoseCv::PoseCv()
{
  R = cvCreateMat( 3, 3, CV_32F );
  t = cvCreateMat( 3, 1, CV_32F );
  n = cvCreateMat( 3, 1, CV_32F );
}

PoseCv::~PoseCv()
{
  cvReleaseMat(&R);
  cvReleaseMat(&t);
  cvReleaseMat(&n);
}

/**************************** PRIVATE *************************************/



/**************************** PUBLIC *************************************/

} // --THE END--



