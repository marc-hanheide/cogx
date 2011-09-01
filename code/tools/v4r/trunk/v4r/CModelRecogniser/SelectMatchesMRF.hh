/**
 * $Id$
 */

#ifndef P_SELECT_MATCHES_MRF_HH
#define P_SELECT_MATCHES_MRF_HH

#include <opencv2/highgui/highgui.hpp>
#include "v4r/PGeometry/PHomography.hh"
#include "v4r/PMath/PVector.hh"
#include "v4r/PMath/PMath.hh"
#include "PKeypoint.hh"
#include "PKeypointMRF.hh"



namespace P
{


class SelectMatchesMRF
{
public:
  class Parameter
  {
  public:
    int width, height;
    int numIter;            //3  //2
    double sigmaMotDiff;    //20 //5..10
    double baseCost;    
    Parameter(int w=640, int h=480, int iter=2, double sigma=5., double cost=5) 
      : width(w), height(h), numIter(iter), sigmaMotDiff(sigma), baseCost(cost) {}
  };

private:
  double INV_SQR_SIGMA2;

  void InitKeypoints(const vector<cv::KeyPoint> &queryKeys, vector<PKeypointMRF> &mrfKeys);
  void InitKeypoints(const vector<cv::Ptr<PKeypoint> > &queryKeys, vector<PKeypointMRF> &mrfKeys);
  void InitCosts(const vector<vector<cv::DMatch> > &matches, vector<PKeypointMRF> &mrfKeys);
  void CreateGraph(vector<PKeypointMRF> &keys);
  void Msg(PKeypointMRF &key, vector<PKeypointMRF> &keys, 
        const vector<cv::Ptr<PKeypoint> > &trainKeys, const vector<vector<cv::DMatch> > &matches);




public:
  Parameter param;
  cv::Mat dbg;
  SelectMatchesMRF(Parameter p=Parameter());
  ~SelectMatchesMRF();
  void Operate(const vector<cv::Ptr<PKeypoint> > &queryKeys, 
        const vector<cv::Ptr<PKeypoint> > &trainKeys, const vector<vector<cv::DMatch> > &matches, 
        vector<int> &selected);
  void Operate(const vector<cv::KeyPoint> &queryKeys, const vector<cv::KeyPoint> &trainKeys, 
        const vector<vector<cv::DMatch> > &matches, vector<int> &selected);
};


/*********************** INLINE METHODES **************************/

}

#endif

