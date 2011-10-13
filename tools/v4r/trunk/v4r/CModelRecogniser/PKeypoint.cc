/**
 * $Id$
 */


#include "PKeypoint.hh"




namespace P 
{

unsigned PKeypoint::nbcnt = 0;
unsigned PKeypoint::nb2cnt = 0;


/************************************************************************************
 * Constructor/Destructor
 */
PKeypoint::PKeypoint()
 : id(UINT_MAX), nb(0),nb2(0), pt(0.,0.), size(1), angle(0), response(0), fw(0), bw(0)
{
}

PKeypoint::PKeypoint(const cv::KeyPoint &k)
 : id(k.class_id), nb(0),nb2(0), pt(k.pt.x,k.pt.y), size(k.size), angle(k.angle), response(k.response), fw(0), bw(0)
{
}

/**
 * copy constructor
 */
PKeypoint::PKeypoint(const PKeypoint &k)
 : id(k.id), nb(k.nb), nb2(k.nb2), pt(k.pt), size(k.size), angle(k.angle), response(k.response), pos(k.pos), fw(0), bw(0)
{
}

PKeypoint::PKeypoint(cv::Point2d _pt, double _size, double _angle, double _response, unsigned _id)
 : id(_id), nb(0), nb2(0), pt(_pt), size(_size), angle(_angle), response(_response), fw(0), bw(0)
{
}

PKeypoint::~PKeypoint()
{
  ReleaseLinks();
  ReleaseFW();
  ReleaseBW();
}


// ****************************************************************************************


/**
 * vote for an object center
 */
void PKeypoint::Vote(const PKeypoint &model, const PKeypoint &key, const cv::Point2d &center, cv::Point2d &vote, double &dAngle, double &dScale)
{
  cv::Point2d v;

  dScale = key.size/model.size;
  dAngle = PMath::ScaleAngle_mpi_pi((-key.angle+model.angle)*M_PI/180.);  // opencv keypoints are in [°]

  v = center - model.pt;
  PVec::Rotate2(&v.x, dAngle, &vote.x);

  vote *= dScale;
  vote += key.pt;
}

/**
 * Draw a keypoint
 */
void PKeypoint::Draw(cv::Mat &img, const PKeypoint &key, const cv::Scalar& col)
{
  cv::Point center( cvRound(key.pt.x), cvRound(key.pt.y) );
  int radius = cvRound(key.size/2);
  cv::circle(img, center, radius, col, 1, 8, 0);

  float srcAngleRad = key.angle*(float)CV_PI/180.f;
  cv::Point orient(cvRound(cos(srcAngleRad)*radius), cvRound(sin(srcAngleRad)*radius));
  cv::line( img, center, center+orient, col, 1);
}

/**
 * Convert PKeypoints to opencv KeyPoints
 */
void PKeypoint::ConvertToCv(const std::vector<cv::Ptr<PKeypoint> > &keys, std::vector<cv::KeyPoint> &cvKeys)
{
  cvKeys.resize(keys.size());

  for (unsigned i=0; i<keys.size(); i++)
  {
    const PKeypoint &key = *keys[i];
    cvKeys[i] = cv::KeyPoint(key.pt.x, key.pt.y, key.size, key.angle, key.response,0,key.id);
  }
}

/**
 * Convert opencv KeyPoints to PKeypoints
 */
void PKeypoint::ConvertFromCv(const std::vector<cv::KeyPoint> &cvKeys, std::vector<cv::Ptr<PKeypoint> > &keys)
{
  keys.resize(cvKeys.size());

  for (unsigned i=0; i<cvKeys.size(); i++)
  {
    keys[i] = new PKeypoint(cvKeys[i]);
  }

}

/**
 * copy a std::vector of keypoints
 */
void PKeypoint::Copy(const std::vector<cv::Ptr<PKeypoint> > &src, std::vector<cv::Ptr<PKeypoint> > &dst)
{
  dst.resize(src.size());

  for (unsigned i=0; i<src.size(); i++)
  {
    dst[i] = new PKeypoint(*src[i]);
  }
}



/**
 * Print
 */
std::ostream& operator<<(std::ostream &os, const PKeypoint &k)
{
  os << '['<<k.id << ' ' << k.pt.x << ' ' << k.pt.y << ' ' << k.size << ' ' << k.angle << ' ' << k.response << ']';

  return os;
}

/**
 * Read
 */
std::istream& operator>>(std::istream &is, PKeypoint &k)
{
  char c;
  unsigned tmp;

  is >> c;
  
  if (c == '[')
  {
    is >> k.id >> k.pt.x >> k.pt.y >> k.size >> k.angle >> k.response;

    is>>c;
    if(c != ']')
      throw std::runtime_error ("PKeypoint::operator>>: Error reading PKeypoint: ']' expected"); 
  }
  else
  {
    throw std::runtime_error ("PKeypoint::operator>>: Error reading PKeypoint: '[' expected");
  }

  return is;
}



}












