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
  R = cvCreateMat( 3, 3, CV_64F );
  t = cvCreateMat( 3, 1, CV_64F );
  n = cvCreateMat( 3, 1, CV_64F );
}

PoseCv::~PoseCv()
{
  cvReleaseMat(&R);
  cvReleaseMat(&t);
  cvReleaseMat(&n);
}

/**************************** PRIVATE *************************************/



/**************************** PUBLIC *************************************/
void InitializePoseCv(PoseCv &pose)
{
  cvmSet(pose.R,0,0,1); cvmSet(pose.R,0,1,0); cvmSet(pose.R,0,2,0);
  cvmSet(pose.R,1,0,0); cvmSet(pose.R,1,1,1); cvmSet(pose.R,1,2,0);
  cvmSet(pose.R,2,0,0); cvmSet(pose.R,2,1,0); cvmSet(pose.R,2,2,1);

  cvmSet(pose.t,0,0,0);
  cvmSet(pose.t,1,0,0);
  cvmSet(pose.t,2,0,0);

  cvmSet(pose.n,0,0,1);
  cvmSet(pose.n,1,0,0);
  cvmSet(pose.n,2,0,0);

   
}

void DeletePoseCv(Array<PoseCv*> &ps)
{
  for (unsigned i=0; i<ps.Size(); i++)
    delete ps[i];
  ps.Clear();
}


} // --THE END--



