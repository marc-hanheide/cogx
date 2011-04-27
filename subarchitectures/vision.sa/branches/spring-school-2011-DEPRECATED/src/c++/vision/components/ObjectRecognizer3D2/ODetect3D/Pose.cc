/**
 * $Id$
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */


#include "Pose.hh"


namespace P 
{



/***************************** Pose ******************************
 * Constructor/Destructor
 */
Pose::Pose()
{
  t.create(3,1,0.);
  R.identity(3);
}

Pose::Pose(Matrix &RR, Matrix &tt)
{
  t = tt;
  R = RR;
}

Pose::Pose(Matrix &RR, Matrix &tt, Matrix &nn)
{
  t = tt;
  R = RR;
  n = nn;
}

Pose::~Pose()
{
}

/**************************** PRIVATE *************************************/



/**************************** PUBLIC *************************************/

} // --THE END--



