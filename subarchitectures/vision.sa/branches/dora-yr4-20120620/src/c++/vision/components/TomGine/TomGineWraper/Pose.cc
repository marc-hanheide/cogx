/**
 * @file Pose.cc
 * @author Richtsfeld, Prankl
 * @date March 2011
 * @version 0.1
 * @brief Pose class.
 */

#include "Pose.hh"

namespace TGThread
{

/***************************** Pose ******************************
 * Constructor/Destructor
 */
Pose::Pose()
{
}

Pose::Pose(bool init)
{
  if (init)
  {
    InitializePose(*this);
  }
}

Pose::Pose(const cv::Mat& matR, const cv::Mat& matt)
{
  R = matR;
  t = matt;
}

Pose::Pose(const Pose &pose)
{
  CopyPose((Pose&)pose, *this);
}


Pose::~Pose()
{
}

/**************************** PRIVATE *************************************/



/**************************** PUBLIC *************************************/
/**
 * copy operator
 */
Pose& Pose::operator=(const Pose &pose)
{
  CopyPose((Pose&)pose, *this); 
  return *this;
}


/**
 * Print a pose to a stream
 */
std::ostream& operator<<(std::ostream &os, const Pose &pose)
{
  //if (pose.empty()) 
  //  return os<<'['<<']'<<'\n'<<'['<<']'<<'\n';

  const double *R = pose.R.ptr<double>(0);
  const double *t = pose.t.ptr<double>(0);

  return os<<'['<< R[0]<<' '<<R[1]<<' '<<R[2]<<';'
                << R[3]<<' '<<R[4]<<' '<<R[5]<<';'
                << R[6]<<' '<<R[7]<<' '<<R[8]<< ']'
           <<'\n'
           <<'['<< t[0]<<' '<<t[1]<<' '<<t[2]<<']'
           <<'\n';
}

/**
 * Read a vector from a stream.
 */
std::istream& operator>>(std::istream &is, Pose &pose) throw(std::runtime_error)
{
  if (pose.empty()) pose.create();

  double *R = pose.R.ptr<double>(0);
  double *t = pose.t.ptr<double>(0);
  char c;
  is >> c;
  if(c == '[')
  {
    is>>R[0]>>R[1]>>R[2]>>c;
    is>>R[3]>>R[4]>>R[5]>>c;
    is>>R[6]>>R[7]>>R[8]>>c;
    if(c == ']')
    {
      is >> c; 
      if(c == '[')
      {
        is>>t[0]>>t[1]>>t[2]>>c;
        if(c != ']')
          throw std::runtime_error("Pose.cc: error: ']' expected");
      }
      else
        throw std::runtime_error("Pose.cc: error: '[' expected");

    }
    else
      throw std::runtime_error("Pose.cc: error: ']' expected");
  }
  else
  {
    throw std::runtime_error("Pose.cc: error: '[' expected");
  }
  return is;
}


/**
 * Compute projection matrix
 */
void Pose::Pose2ProjMat(double R[9], double t[3], double C[9], double P[12])
{
  double tmpC[12], tmpRt[16];

  cv::Mat matC = cv::Mat(3, 4, CV_64F, tmpC);
  cv::Mat matRt = cv::Mat(4, 4, CV_64F, tmpRt);
  cv::Mat matP = cv::Mat(3, 4, CV_64F, P);

  tmpC[0]=C[0], tmpC[1]=C[1], tmpC[2]=C[2],  tmpC[3]=0.; 
  tmpC[4]=C[3], tmpC[5]=C[4], tmpC[6]=C[5],  tmpC[7]=0.; 
  tmpC[8]=C[6], tmpC[9]=C[7], tmpC[10]=C[8], tmpC[11]=0.; 

  tmpRt[0]=R[0], tmpRt[1]=R[1], tmpRt[2]=R[2],  tmpRt[3]=t[0]; 
  tmpRt[4]=R[3], tmpRt[5]=R[4], tmpRt[6]=R[5],  tmpRt[7]=t[1]; 
  tmpRt[8]=R[6], tmpRt[9]=R[7], tmpRt[10]=R[8], tmpRt[11]=t[2];
  tmpRt[12]=0.,  tmpRt[13]=0.,  tmpRt[14]=0.,   tmpRt[15]=1.;
 
  matP = matC.mul(matRt);
}



} // --THE END--



