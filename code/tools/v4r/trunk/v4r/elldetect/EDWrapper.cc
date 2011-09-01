/**
 * $Id$
 */


#include "EDWrapper.hh"

#define DEBUG


namespace RTE 
{

static const bool CmpEllID(const RTE::CzEllipse *a, const RTE::CzEllipse* b)
{
  return (a->iMarkerID < b->iMarkerID);
}



/************************************************************************************
 * Constructor/Destructor
 */

EDWrapper::EDWrapper(Parameter p)
{ 
  SetParameter(p);
}

EDWrapper::~EDWrapper()
{
}

/**
 * InitEllDetect
 */
void EDWrapper::InitEllDetect()
{
  RTE::CzEllipseDetection::sEllipseDetectionConfig *config = elldetect.GetConfig();

  if (param.ellSubPx) config->eMethod = RTE::CzEllipseDetection::ELL_CONFIGMETHOD_DUAL_ELLIPSE;
  else config->eMethod = RTE::CzEllipseDetection::ELL_CONFIGMETHOD_HYPOTHESES;

  config->iMinEllipseDiameter = param.iMinEllipseDiameter;
  config->iMaxEllipseDiameter = param.iMaxEllipseDiameter;
  config->dRadiusInOut = param.dRadiusInOut;
  config->iMinContrast = param.iMinContrast;
  config->dMaxContrastDev = param.dMaxContrastDev;
  config->dMaxQuotient = param.dMaxQuotient;
  config->dMinSupport = param.dMinSupport;
  config->dMaxHypFiterror = param.dMaxHypFiterror;
  config->dMaxFiterror = param.dMaxFiterror;
  config->iSobelApertureSize = param.iSobelApertureSize;
  config->iCannyLow = param.iCannyLow;
  config->iCannyHigh = param.iCannyHigh;
  config->bGaussSmooth = param.bGaussSmooth;
  config->iLinkingGrowCount = param.iLinkingGrowCount;
  config->dDualEllipseMinGradient = param.dDualEllipseMinGradient;
  config->iDualEllipseBorder = param.iDualEllipseBorder;
  if (param.eMarkerCode==0) config->eMarkerCode = RTE::CzEllipseDetection::ELL_MARKERCODE_12BIT;
  else if (param.eMarkerCode==1) config->eMarkerCode = RTE::CzEllipseDetection::ELL_MARKERCODE_14BIT;
  else if (param.eMarkerCode==2) config->eMarkerCode = RTE::CzEllipseDetection::ELL_MARKERCODE_20BIT;
  config->dMarkerQuotientOuterInner = param.dMarkerQuotientOuterInner;
  config->dMarkerDestInnerRadius = param.dMarkerDestInnerRadius;
  config->iMarkerRingWidth = param.iMarkerRingWidth;
  config->dMarkerMinContrast = param.dMarkerMinContrast;
}



/**
 * DetectWhiteElls
 */
void EDWrapper::DetectWhiteElls(IplImage *img, vector<RTE::CzEllipse> &ells)
{
  ells.clear();

  elldetect.Clear();
  elldetect.GetConfig()->bEllipseWhite = true;
  elldetect.GetConfig()->bDecodeMarker = param.decodeWhite;
  elldetect.FindEllipses(img);

  ells.resize(elldetect.GetEllipsesCount());
  for (unsigned i=0; i<ells.size(); i++)
    ells[i] = *elldetect.GetEllipse(i);
}

/**
 * DetectBlackElls
 */
void EDWrapper::DetectBlackElls(IplImage *img, vector<RTE::CzEllipse> &ells)
{
  ells.clear();

  elldetect.Clear();
  elldetect.GetConfig()->bEllipseWhite = false;
  elldetect.GetConfig()->bDecodeMarker = param.decodeBlack;
  elldetect.FindEllipses(img);

  ells.resize(elldetect.GetEllipsesCount());
  for (unsigned i=0; i<ells.size(); i++)
    ells[i] = *elldetect.GetEllipse(i);
}

/**
 * DetectRings
 */
void EDWrapper::DetectRings(vector<RTE::CzEllipse> &white, vector<RTE::CzEllipse> &black, vector<pair<unsigned, unsigned> > &rings)
{
  double sqrDist, minSqrDist;
  unsigned idx;

  rings.clear();

  for (unsigned i=0; i<white.size(); i++)
  {
    idx = UINT_MAX;
    minSqrDist = DBL_MAX;
    for (unsigned j=0; j<black.size(); j++)
    {
      if (white[i].dA < black[j].dA && white[i].dB < black[j].dB)
      {
        sqrDist = DistanceSqr2(&white[i].dX, &black[j].dX);
        if (sqrDist < minSqrDist)
        {
          minSqrDist = sqrDist;
          idx = j;
        }
      }
    }
    if (idx!=UINT_MAX && sqrt(minSqrDist) < param.thrRing)
      rings.push_back(pair<unsigned,unsigned>(i,idx));
  }
}






/******************************* PUBLIC **************************************
 * Detect ellipses
 * depending on the parameter ellipses with a white center, ellipses with a black center, and
 * black rings on a white center are detected
 * @param img gray scale image
 * @param rings indices to <inner_white,outer_black> ellipses
 */
void EDWrapper::Detect(const cv::Mat &img, vector<RTE::CzEllipse> &whiteElls,
                       vector<RTE::CzEllipse> &blackElls, vector<pair<unsigned, unsigned> > &rings)
{
  if( img.type() != CV_8U ) cv::cvtColor( img, grayImage, CV_BGR2GRAY );
  else grayImage = img;

  IplImage iplImage = grayImage;

  if (param.whiteElls) DetectWhiteElls(&iplImage, whiteElls);
  if (param.blackElls) DetectBlackElls(&iplImage, blackElls);
  if (param.detRings) DetectRings(whiteElls, blackElls, rings);

  #ifdef DEBUG
  cout<<"--"<<endl;
  cout<<"Num. of black ellipses: "<<blackElls.size()<<endl;
  cout<<"Num. of white ellipses: "<<whiteElls.size()<<endl;
  cout<<"Num. of rings: "<<rings.size()<<endl;
  if (!dbg.empty())
  {
    DrawEllipses(dbg, whiteElls, CV_RGB(255,0,0), param.decodeWhite);
    DrawEllipses(dbg, blackElls, CV_RGB(0,255,0), param.decodeBlack);
    DrawRings(dbg, whiteElls, blackElls, rings, CV_RGB(0,0,255));
  }
  #endif
}


/**
 * SetParameter
 */
void EDWrapper::SetParameter(Parameter &p)
{
  param = p;
  InitEllDetect();
}


/**
 * Draw ellipses
 */
void EDWrapper::DrawEllipses(cv::Mat &img, vector<RTE::CzEllipse> &ells, cv::Scalar color, bool printcode)
{
  for (int i=0; i<ells.size(); i++)
  {
    RTE::CzEllipse &ell = ells[i];
    DrawEll(img, ell, color);
		
		if (printcode) 
    {
			std::ostringstream sin;
			sin << ell.iMarkerID;
			std::string val = sin.str();
			cv::putText(img, val.c_str(), cvPoint(ell.dX + 18,ell.dY+18), cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(0.0));
			cv::putText(img, val.c_str(), cvPoint(ell.dX + 22,ell.dY+18), cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(0.0));
			cv::putText(img, val.c_str(), cvPoint(ell.dX + 22,ell.dY+22), cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(0.0));
			cv::putText(img, val.c_str(), cvPoint(ell.dX + 18,ell.dY+22), cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(0.0));
			cv::putText(img, val.c_str(), cvPoint(ell.dX + 20,ell.dY+20), cv::FONT_HERSHEY_DUPLEX, .8, CV_RGB(255,255,255));
		 }
  }
}

/**
 * Draw ellipses
 */
void EDWrapper::DrawRings(cv::Mat &img, vector<RTE::CzEllipse> &whiteElls,
        vector<RTE::CzEllipse> &blackElls, vector<pair<unsigned, unsigned> > &rings, cv::Scalar color)
{
  for (unsigned i=0; i<rings.size(); i++)
    DrawRing(img, whiteElls[rings[i].first], blackElls[rings[i].second], color);
}

/**
 * DrawEll
 */
void EDWrapper::DrawEll(cv::Mat &img, RTE::CzEllipse &ell, cv::Scalar color)
{
  cv::ellipse(img, cv::Point((int)(ell.dX+.5),(int)(ell.dY+.5)), cv::Size((int)(ell.dA+.5), (int)(ell.dB+.5)), ell.dPhi*180./M_PI, 0., 360., color, 1, CV_AA, 0);
}

/**
 * DrawEll
 */
void EDWrapper::DrawRing(cv::Mat &img, RTE::CzEllipse &iell, RTE::CzEllipse &oell, cv::Scalar color)
{
  cv::ellipse(img, cv::Point((int)(iell.dX+.5),(int)(iell.dY+.5)), cv::Size((int)(iell.dA+.5), (int)(iell.dB+.5)), iell.dPhi*180./M_PI, 0., 360., color, 1, CV_AA, 0);
  cv::ellipse(img, cv::Point((int)(oell.dX+.5),(int)(oell.dY+.5)), cv::Size((int)(oell.dA+.5), (int)(oell.dB+.5)), oell.dPhi*180./M_PI, 0., 360., color, 1, CV_AA, 0);
}


}












