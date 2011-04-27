/**
 * @author Alper Aydemir 
 * @date September 2010 
 */

#include <highgui.h>
#include <VideoUtils.h>
#include "ARTagRecognizer.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <VisionData.hpp>
/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ARTagRecognizer();
  }
}

// Convenience
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <ctime>

#include <AR/ar.h>
#include <Matrix33.h>
std::string sfloat(double f, int precision=6)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << f;
  return out.str();
}

double fclocks()
{
  return 1.0 * clock() / CLOCKS_PER_SEC;
}
// --------------------------


namespace cast
{

  using namespace std;
  using namespace VisionData;
  using namespace cogx;
  using namespace Math;

  void ARTagRecognizer::configure(const map<string,string> & _config)
  {
    log("configuring..");
    map<string,string>::const_iterator it;

    if((it = _config.find("--videoname")) != _config.end())
    {
      videoServerName = it->second;
    }

    if((it = _config.find("--camid")) != _config.end())
    {
      istringstream istr(it->second);
      istr >> camId;
    }

    // sanity checks: Have all important things be configured? Is the
    // configuration consistent?
    if(videoServerName.empty())
      throw runtime_error(exceptionMessage(__HERE__, "no video server name given"));

    it = _config.find("--tagpath");
    if (it != _config.end()) {
      tagpath = it->second;
    }
    log("Path to tag data %s", tagpath.c_str());

    string cfg;
    it = _config.find("--cfgfilename");
    if (it != _config.end()) {
      cfg = it->second;
    }
    marker_width = 160.0;
    thresh = 100;                                                                         
    count = 0;
    dummy = true;
    log("Path to tag cfg file %s", (tagpath + cfg).c_str());
    LoadPatterns(tagpath + cfg);
  }

  void ARTagRecognizer::LoadPatterns(std::string filename){
    string line;  
    //open file
    ifstream file(filename.c_str());
    if (!file.good()){
      log("Could not open file, returning without doing anything.");
      return;
    }
    object temp;
    std::string number;
    temp.id = -1;
    while(getline(file,line)){
      //first line is label
      temp.label = line;
      //second filename for pattern
      getline(file,line);
      temp.filename = tagpath + line;

      //next 4 lines are the tf matrix rows
      // can't do it in a loop because Pose3 has members for each matrix element
      getline(file,line);
      istringstream istr(line);
      Pose3 O;

      istr >> number; 
      O.rot.m00 = atof(number.c_str());
      istr >> number; 
      O.rot.m01 = atof(number.c_str());
      istr >> number; 
      O.rot.m02 = atof(number.c_str());


      getline(file,line);
      istringstream istr2(line);

      istr2 >> number; 
      O.rot.m10 = atof(number.c_str());
      istr2 >> number; 
      O.rot.m11 = atof(number.c_str());
      istr2 >> number; 
      O.rot.m12 = atof(number.c_str());



      getline(file,line);
      istringstream istr3(line);
      istr3 >> number; 
      O.rot.m20 = atof(number.c_str());
      istr3 >> number; 
      O.rot.m21 = atof(number.c_str());
      istr3 >> number; 
      O.rot.m22 = atof(number.c_str());

      getline(file,line);
      istringstream istr4(line);
      istr4 >> number; 
      O.pos.x = atof(number.c_str());
      istr4 >> number; 
      O.pos.y = atof(number.c_str());
      istr4 >> number; 
      O.pos.z = atof(number.c_str());
      temp.trans = O;

      taggedObjects.push_back(temp); 

    }

    for (unsigned int i = 0; i < taggedObjects.size(); i++){
      log("loaded %s at %s", taggedObjects[i].label.c_str(), taggedObjects[i].filename.c_str());
      printPose(taggedObjects[i].trans);
    }


  }
  void ARTagRecognizer::start()
  {
    log("starting...");
    init();
    // get connection to the video server
    videoServer = getIceServer<Video::VideoInterface>(videoServerName);

    // register our client interface to allow the video server pushing images
    Video::VideoClientInterfacePtr servant = new VideoClientI(this);
    registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);
    addChangeFilter(createLocalTypeFilter<VisionData::ARTagCommand>(cdl::ADD),
	new MemberFunctionChangeReceiver<ARTagRecognizer>(this,
	  &ARTagRecognizer::newARTagCommand));


  }
  bool isprev_trans = false;

  void setRot(double p11,double p12,double p13,
      	       double p21, double p22, double p23,
	       double p31, double p32, double p33, Pose3 &p){
  
 	  p.rot.m00 = p11;
	  p.rot.m01 = p12;
	  p.rot.m02 = p13;

	  p.rot.m10 = p21;
	  p.rot.m11 = p22;
	  p.rot.m12 = p23;

	  p.rot.m20 = p31;
	  p.rot.m21 = p32;
	  p.rot.m22 = p33;
  }
  void ARTagRecognizer::newARTagCommand(const cast::cdl::WorkingMemoryChange &objID) {

    VisionData::ARTagCommandPtr obj =
      getMemoryEntry<VisionData::ARTagCommand>(objID.address);
   log("got new ARTagCommand"); 
    //TODO find the object id with corresponding label
    // call getMarker if it exists write to WM with the same address
    // if it doesn't same thing.

    std::string label = obj->label;
    Pose3 p = getMarkerPosition(label);
    
    if (p.pos.x != 1E40){
      //load fake visual object
      Post3DObjectPtr obj3D = new VisionData::Post3DObject;
      log("adding fake visual object command: %s",label.c_str());
      printPose(p);
      obj3D->label = label;
      if(dummy){
	if(label == "metalbox"){
	  p.pos.x = -1.19;
	  p.pos.y = -5.32;
	  p.pos.z = 0.90;
	  setRot(0,0,-1.1,
	         0,1.4,0,
		 1,0,0,p);
	  obj3D->pose = p;
	}
	else if(label == "table1" || label == "table12"){
	  p.pos.x = 1.36;
	  p.pos.y = 0.39;
	  p.pos.z = 0.20;
	  setRot(1,0,0,
	      0,1,0,
	      0,0,1,p);
	  obj3D->pose = p;
	  obj3D->label = "table1";
	}
	else if(label == "table2"){
	  p.pos.x = -1.23;
	  p.pos.y = -4.39;
	  p.pos.z = 0.25;
	  setRot(1,0,0,
	      0,1,0,
	      0,0,1,p);
	  obj3D->pose = p;
	}
	else if(label == "shelves"){
	  p.pos.x = 0.42;
	  p.pos.y = -2.79;
	  p.pos.z = 1.3;
	  setRot(0,-1,0,
	      1,0,0,
	      0,0,1,p);
	  obj3D->pose = p;	}
      }
      else{
	obj3D->pose = p;
      }
      addToWorkingMemory(newDataID(),obj3D);
      sleep(1);
      log("overwriting request.");
      overwriteWorkingMemory(objID.address,obj);
    }
    else{
      log("object %s not detected overwriting request.",label.c_str());
      overwriteWorkingMemory(objID.address,obj);
    }
  }
  /*
     char           *patt_name      = "/home/cogx/projects/kthsys/subarchitectures/vision.sa/config/ARData/Data/patt.hiro";                                                                    
     int             patt_id;                                                                                              

     int             marker_width     = 80.0;
     double          marker_center[2] = {0.0, 0.0};                                                                        
     double          marker_trans[3][4];                                                                                   

     int             xsize, ysize;                                                                                         
     int             thresh = 100;                                                                         
     int             count = 0;                                                                                            
   */
  /* set up the video format globals */                                                                                 

  char           *cparam_name    = "/home/cogx/projects/kthsys/subarchitectures/vision.sa/config/ARData/Data/camera_para.dat";                                                              


  void ARTagRecognizer::init(){
    ARParam  wparam;

    /* set the initial camera parameters */
    if( arParamLoad(cparam_name, 1, &wparam) < 0 ) {
      printf("Camera parameter load error !!\n");
      exit(0);
    }
    arParamChangeSize( &wparam, 640, 480, &cparam );
    arInitCparam( &cparam );
    printf("*** Camera Parameter ***\n");
    arParamDisp( &cparam );
    log("lala");
    for (unsigned int i = 0; i < taggedObjects.size(); i++){
      /* load pattern file */
      if( (taggedObjects[i].id =arLoadPatt(taggedObjects[i].filename.c_str())) < 0)
      {
	printf ("Pattern file load error !! \n");
	exit(0);
      }
      log("loaded pattern for: %s, id: %d",taggedObjects[i].label.c_str(), taggedObjects[i].id);

    }
  }

  void ARTagRecognizer::printPose(Pose3 pose){

    printf("%3.2f , %3.2f, %3.2f \n",pose.rot.m00, pose.rot.m01, pose.rot.m02);
    printf("%3.2f , %3.2f, %3.2f \n",pose.rot.m10, pose.rot.m11, pose.rot.m12);
    printf("%3.2f , %3.2f, %3.2f \n",pose.rot.m20, pose.rot.m21, pose.rot.m22);
    printf("pos: %3.2f , %3.2f, %3.2f \n", pose.pos.x, pose.pos.y, pose.pos.z);

  }
  Pose3 ARTagRecognizer::getMarkerPosition(std::string label)
  {
    int targetId = -1;
    for(unsigned int i =0; i < taggedObjects.size(); i++){
      if (taggedObjects[i].label == label)
	targetId = i;
    }
    log("requested object %s had id: %d", label.c_str(), targetId);
    if(targetId == -1){
      log("this object does not exists! ignoring..");
    }
    Pose3 ret;
    ret.pos.x = 1E40;
    ARMarkerInfo    *marker_info;
    int             marker_num;
    int i,k;
    double  marker_trans[3][4];                                                                       
    double marker_center[2] = {0.0, 0.0};                                                                        
    videoServer->getImage(camId,m_image);
    log("Image received.");
    IplImage *iplImage = convertImageToIpl(m_image);
    ARUint8 *dataPtr = (unsigned char*)iplImage->imageData;
    // detect the markers in the video frame
    if(arDetectMarker(dataPtr, thresh, 
	  &marker_info, &marker_num) < 0 ) {
      exit(0);
    }
    k = -1;
    for( i = 0; i < marker_num; i++ ) {
      log("Found A pattern with id: %d",marker_info[i]);
      for (unsigned int j = 0; j < taggedObjects.size(); j++){
	if( marker_info[i].id  == targetId) {
	  //  you've found a pattern 
	  printf("Found pattern with id: %d \n",taggedObjects[j].id);
	  double inv_trans[3][4];

	  if( arGetTransMat(&marker_info[i], marker_center, marker_width, marker_trans) < 0 ){
	    log("transmat failed!!!");
	  }
	  /*else{
	    if( arGetTransMatCont(&marker_info[i], marker_trans, marker_center, marker_width, marker_trans) < 0 ) return;
	  }*/

	  //Calculate its position and pose and put in the WM
	  Pose3 P, A, B, O;
	  P = m_image.camPars.pose;
	  printPose(P);
	  A.rot.m00 = marker_trans[0][0];
	  A.rot.m01 = marker_trans[0][1];
	  A.rot.m02 = marker_trans[0][2];

	  A.rot.m10 = marker_trans[1][0];
	  A.rot.m11 = marker_trans[1][1];
	  A.rot.m12 = marker_trans[1][2];

	  A.rot.m20 = marker_trans[2][0];
	  A.rot.m21 = marker_trans[2][1];
	  A.rot.m22 = marker_trans[2][2];

	  A.pos.x = marker_trans[0][3]/1000.0;
	  A.pos.y = marker_trans[1][3]/1000.0;
	  A.pos.z = marker_trans[2][3]/1000.0;

	  A.pos.x += taggedObjects[i].trans.pos.x;
	  A.pos.y += taggedObjects[i].trans.pos.y;
	  A.pos.z += taggedObjects[i].trans.pos.z;

	  printf("object pose A: %3.2f, %3.2f, %3.2f \n", A.pos.x,A.pos.y,A.pos.z);

	  Math::transform(P,A,B);

	  mult(B.rot,taggedObjects[j].trans.rot,B.rot);
	  printf("object pose B: %3.2f, %3.2f, %3.2f \n", B.pos.x,B.pos.y,B.pos.z);
	  ret = B;
	  cvReleaseImage(&iplImage);
	}
	else{
	//  log("object %s not found",label.c_str());
  cvReleaseImage(&iplImage);
	}
      }
    }
    return ret;
  }
      void ARTagRecognizer::runComponent()
    {
      log("I am running!");
      sleep(10);
      while(isRunning())
      {
	//getMarkerPosition();
	sleepComponent(500);
      }
    }

  } // namespace

