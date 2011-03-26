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

    sendSelfCommand = false;
    if((it = _config.find("--send-self-command")) != _config.end())
    {
    sendSelfCommand = true;
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
    marker_width = 80.0;
    thresh = 100;                                                                         
    count = 0;
    dummy = true;
    log("Path to tag cfg file %s", (tagpath + cfg).c_str());
    LoadPatterns(tagpath + cfg);

    it = _config.find("--pre-known-object-file");
        if (it != _config.end()) {
          cfg = it->second;
          LoadPreKnown(tagpath + cfg);
        }

  }

  void ARTagRecognizer::LoadPreKnown(std::string filename){
    string line;
    //open file
    ifstream file(filename.c_str());
    if (!file.good()){
      log("Could not open file, returning without doing anything.");
      return;
    }
    objectInfo temp;
    int i = 0;
    while(getline(file,line)){
      //first line is label
      string label = line;

      getline(file,line);
      istringstream istr(line);
      istr >> temp.fixedPosition.x;
      istr >> temp.fixedPosition.y;
      istr >> temp.fixedPosition.z;

      getline(file,line);
      istringstream istr2(line);
      istr2 >> temp.boxDimensions.x;
      istr2 >> temp.boxDimensions.y;
      istr2 >> temp.boxDimensions.z;


      m_preKnownObjects[i] = temp;
      i++;
    }
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
    log("Getting video server name %s", videoServerName.c_str());
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

	    //Try detection for all ARTags belonging to this object
	    for(unsigned int targetId =0; targetId < taggedObjects.size(); targetId++){
	      if (taggedObjects[targetId].label == label)
	      {
		if (m_preKnownObjects.count(targetId) == 0) {
		  log("Warning: nothing preloaded for %i(%s)", targetId, label.c_str());
		}
		else if (taggedObjects[targetId].wmid != "") {
		  log("ARTag already detected, skipping...");
		}
		else {
		  Pose3 p; 
		  p.pos.x = 0;
		  p.pos.y = 0;
		  p.pos.z = 0;
		  setIdentity(p);

		  if (isMarkerPresent(targetId)){
		    VisionData::VisualObjectPtr newVisualObject =
		      new VisionData::VisualObject;

		    newVisualObject->pose = p;
		    newVisualObject->pose.pos = m_preKnownObjects[targetId].fixedPosition;
		    newVisualObject->detectionConfidence = 1.0;

		    newVisualObject->model = new VisionData::GeometryModel;
		    VisionData::Vertex vert;
		    vert.pos = m_preKnownObjects[targetId].fixedPosition;
		    newVisualObject->model->vertices.push_back(vert);

		    newVisualObject->identLabels.push_back(label);
		    newVisualObject->identLabels.push_back("unknown");
		    newVisualObject->identDistrib.push_back(1.0);
		    newVisualObject->identDistrib.push_back(0.0);

		    string dataID = newDataID();
		    addToWorkingMemory(dataID,newVisualObject);
		    taggedObjects[targetId].wmid = dataID;
		  }
		  else{
		    log("object %s not detected overwriting request.",label.c_str());
		    overwriteWorkingMemory(objID.address,obj);
		  }
		}
	      }
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

  char           *cparam_name    = "/home/cogx/avs-yr3/subarchitectures/vision.sa/config/ARdata/camera_para.dat";                                                              


  void ARTagRecognizer::init(){
    ARParam  wparam;

     //set the initial camera parameters 
    if( arParamLoad(cparam_name, 1, &wparam) < 0 ) {
      log("Camera parameter load error !!\n");
      exit(1);
    }
    arParamChangeSize( &wparam, 640, 480, &cparam );
    arInitCparam( &cparam );
    log("*** Camera Parameter ***\n");
    arParamDisp( &cparam );
    log("lala");
    for (unsigned int i = 0; i < taggedObjects.size(); i++){
      /* load pattern file */
      if( (taggedObjects[i].id =arLoadPatt(taggedObjects[i].filename.c_str())) < 0)
      {
	log("Pattern file load error !! \n");
	exit(1);
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
  bool ARTagRecognizer::isMarkerPresent(int targetId)
  {    log("requested object %s had id: %d", taggedObjects[targetId].label.c_str(), targetId);
    if(targetId == -1){
      log("this object does not exists! ignoring..");
    }
    Pose3 ret;
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
    log("Found %d markers", marker_num);
    k = -1;
    for( i = 0; i < marker_num; i++ ) {
      log("Found A pattern with id: %d",marker_info[i]);
	if( marker_info[i].id  == taggedObjects[targetId].id) {
	  //  you've found a pattern 
	  log("Found pattern with id: %d \n",taggedObjects[targetId].id);
	  cvReleaseImage(&iplImage);
	  return true;
	  /*	  double inv_trans[3][4];

	  if( arGetTransMat(&marker_info[i], marker_center, marker_width, marker_trans) < 0 ){
	    log("transmat failed!!!");
	  }
	  else{
	    if( arGetTransMatCont(&marker_info[i], marker_trans, marker_center, marker_width, marker_trans) < 0 ) return;
	  }

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

	  mult(B.rot,taggedObjects[targetId].trans.rot,B.rot);
	  printf("object pose B: %3.2f, %3.2f, %3.2f \n", B.pos.x,B.pos.y,B.pos.z);
	  ret = B; */
	}
	else{
	  log("object %s not found",taggedObjects[targetId].label.c_str());
  cvReleaseImage(&iplImage);
  return false;
	}
    }
    return false;
  }
      void ARTagRecognizer::runComponent()
    {
      log("I am running!");
      sleep(10);
      while(isRunning())
      {
	if(sendSelfCommand){
	  log("Sending command to self");
	  for (unsigned i=0; i < taggedObjects.size(); i++){
	  VisionData::ARTagCommandPtr cmd = new VisionData::ARTagCommand;
	  cmd->label = taggedObjects[i].label;
	  addToWorkingMemory(newDataID(), cmd);
	  }
	}
	sleepComponent(500);
      }
    }

  } // namespace

