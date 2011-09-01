/**
 * $Id$
 */


#include "CModelHandler.hh"




namespace P 
{



/************************************************************************************
 * Constructor/Destructor
 */

CModelHandler::CModelHandler()
{ 
}

CModelHandler::~CModelHandler()
{
}


/**
 * Compute view ray
 */
void CModelHandler::ComputeViewRay(Pose &pose, cv::Point3d &objCenter, cv::Point3d &vr)
{
  double pos[3];
  Pose invPose;
  InvPose(pose,invPose);

  pos[0]=0., pos[1]=0., pos[2]=0.;
  PMat::MulAdd3( invPose.R.ptr<double>(), pos, invPose.t.ptr<double>(), &vr.x);
  vr -= objCenter;
  if (!PMath::IsZero(vr.x) && !PMath::IsZero(vr.y) && !PMath::IsZero(vr.z))
    PVec::Normalise3(&vr.x,&vr.x);
}







/***************************************************************************************/

/**
 * Update the view indexing histogram
 */
void CModelHandler::RenewProbSphere(const cv::Mat &cam, const cv::Mat &distCoeffs, CModel &model)
{
  model.viewHist = new SphereHistogram(2); 

  ConfValues predProb(cam, distCoeffs);
  
  cv::Point3d vr;

  for (unsigned i=0; i<model.views.size(); i++)
  {
    ComputeViewRay(model.views[i]->pose, model.center, vr);

    model.viewHist->InsertMax(i,vr, predProb);
  }
}



/** 
 * Save model file
 */
void CModelHandler::Save(const string &filename, CModel &model)
{
  cout<<"Save to file: "<<filename<<"...";

  ofstream out(filename.c_str());
  model.save(out);
  out.close();

  cout<<"ok"<<endl;
}

/**
 * Load model file
 */
bool CModelHandler::Load(const string &filename, CModel &model)
{
  cout<<"Load from file: "<<filename<<"...";

  ifstream in(filename.c_str());

  if (in.is_open())
  {
    model.load(in);
    in.close();

    cout<<"ok"<<endl;
    return true;
  }

  cout<<"failed"<<endl;
  return false;
}

/** 
 * Save model file
 */
void CModelHandler::Save(const string &filename, cv::Ptr<CModel> &model)
{
  cout<<"Save to file: "<<filename<<"...";

  ofstream out(filename.c_str());
  model->save(out);
  out.close();

  cout<<"ok"<<endl;
}

/**
 * Load model file
 */
bool CModelHandler::Load(const string &filename, cv::Ptr<CModel> &model)
{
  cout<<"Load from file: "<<filename<<"...";

  ifstream in(filename.c_str());

  if (in.is_open())
  {
    if (model.empty()) model = new CModel();

    model->load(in);
    in.close();

    cout<<"ok"<<endl;
    return true;
  }
  
  cout << "failed"<<endl;
  return false;
}

}












