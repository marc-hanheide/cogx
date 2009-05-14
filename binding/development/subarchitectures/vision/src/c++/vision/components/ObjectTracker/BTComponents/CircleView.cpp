/** @file CircleView.cpp
 *  @brief A circle view.
 *
 *  @author Somboon Hongeng
 *  @date march 2007
 *  @bug No known bugs.
 */
#include "CircleView.h"
#include "ViewTracker.h"
#include <vision/components/common/SystemUtils/Common.h>
#include <vision/components/common/VisualizationUtils/Display.h>

using namespace Common;
using namespace Display;
using namespace std;

const int CircleView::DOFNUM;

CircleView::CircleView(ViewTracker *parent, int viewId, 
		       int objId, Vision::ObjType _type, 
		       TrackAux::ObjShape _shape,
		       map<string, string> config) 
    : ObjView()
{    
    m_pViewTracker = parent;
    m_view = viewId;
    m_id = objId;
    m_type = _type;
    m_shape = _shape;
    
    initStateParameters(config);
}

CircleView::~CircleView() {
    
}

void CircleView::initRadius(int newRadius)
{
    // when radius is initialized by roi, we do not scale
    // allows radius to vary (-5, +5) pixels
    m_radius[0] = std::max(1, newRadius - 5);
    m_radius[1] = newRadius + 5;
    m_dofrange[2][0] = m_radius[0]; 
    m_dofrange[2][1] = m_radius[1];
}

void CircleView::ResampleAlgorithm(float myoldstate[][DOFNUM], 
				   float oldsampleweight[], 
				   int oldparticlenum, 
				   float mynewstate[][DOFNUM], 
				   int newparticlenum) 
{
    float *myaccweigth = new float [oldparticlenum]; 
    TrackAux::AccumulateProb(oldsampleweight, myaccweigth, oldparticlenum); 
    for(int i=0;i<newparticlenum;i++) {
	float randomchoice = (float)TrackAux::urandom()*myaccweigth[oldparticlenum-1];
	
	int choiceidx;
	try {
	    choiceidx = TrackAux::bsearch(myaccweigth,oldparticlenum,randomchoice); 
	}
	catch (...) {
	  throw user_error(__HERE__, "View::ResampleAlgorithm out-of-bound error");
	}
	
	for (int j=0; j<DOFNUM; j++) 
	    mynewstate[i][j] = myoldstate[choiceidx][j];
    }
    delete []myaccweigth;
}


void CircleView::initstate(float* initstates)
{
    initUsingBackprojectWeight(this->NUM_STATES) ; 
    for(int i=0; i< this->NUM_STATES; i++) 
	m_weight[i] = 1.0f; 
}


void CircleView::initStateParameters(map<string,string> &config) 
{
    if (m_pViewTracker->bTmpimages_allocated==false) 
	throw user_error(__HERE__, "Images must be allocated first");

    ResolutionType resType = m_pViewTracker->resolutionType();
    float scale = (resType==RES_QVGA)? 0.5 : 1;
    
    map<string,string>::iterator iter;
    if ((iter=config.find("R_MIN")) != config.end())
      m_radius[0] = scale*strtod(iter->second.c_str(), NULL);
    else
      throw user_error(__HERE__, "R_MIN is not specified");
    
    if ((iter=config.find("R_MAX")) != config.end())
      m_radius[1] = scale*strtod(iter->second.c_str(), NULL);
    else
      throw user_error(__HERE__, "R_MAX is not specified");

    
    m_dofrange[0][0] = 0.;				   
    m_dofrange[0][1] = (float) (m_pViewTracker->imgsize.width);
    m_dofrange[1][0] = 0.;				   
    m_dofrange[1][1] = (float) (m_pViewTracker->imgsize.height);
    m_dofrange[2][0] = m_radius[0]; 
    m_dofrange[2][1] = m_radius[1];
    
    //m_dofvar[0] = m_dofvar[1] = 20.; m_dofvar[2] = 5.;
    m_dofvar[0] = m_dofvar[1] = scale*10.; m_dofvar[2] = scale*5.;
    
    for (int i=0; i<this->DOFNUM; i++)
	m_beststate[i]=0.;
    m_bestweight = 0.;
}


void CircleView::drawAllSamples(IplImage *img) 
{
    for (int j=0; j<NUM_STATES; j++) {
	int x = cvRound(m_allstate[j][0]);
	int y = cvRound(m_allstate[j][1]);
	int r = cvRound(m_allstate[j][2]);      
	cvCircle(img, cvPoint(x,y), r,CV_RGB(0,255,126),1);
    }  
}

void CircleView::drawASample(IplImage *img) 
{
    int x = cvRound(m_beststate[0]);
    int y = cvRound(m_beststate[1]);
    int r = cvRound(m_beststate[2]);
    CvScalar color = getRandomColor(m_id);
    cvCircle(img,cvPoint(x,y),r, color, 2);
}


float CircleView::colorCost() {
    return Evaluate_backProjection(m_beststate);
}


float CircleView::edgeCost() {
    return Evaluate_distimg(m_beststate);
}

float CircleView::averageBackProjection(float state[], bool bCompletePose) {

    if(!m_bhistvalid) 
	return 0.0;
    
    // set bbx
    int x = cvRound(state[0]/this->DOWNSAMPLE) ; 
    int y = cvRound(state[1]/this->DOWNSAMPLE) ; 
    int r = cvRound(state[2]/this->DOWNSAMPLE) ; 

    if (r==0)	
	r=1; 

    CvPoint pt1 = cvPoint( std::max(0,x-r), 
			   std::max(0,y-r) );
    CvPoint pt2 =cvPoint( std::min(m_bsize.width,x+r), 
			  std::min(m_bsize.height,y+r) ); 
    IplROI roi = {0, pt1.x, pt1.y, pt2.x-pt1.x, pt2.y-pt1.y} ; 
        
    m_backprojectImage->roi = &roi; 
    float meanvalue = cvMean(m_backprojectImage) ; 
    m_backprojectImage->roi = NULL;  
    return meanvalue ; 
}

float CircleView::Evaluate_distimg(float state[])
{
    // Access to data of distimg. No need to release memory.
    IplImage *distanceImage = m_pViewTracker->getDistImg();
    if (distanceImage == NULL) 
      throw user_error(__HERE__, "image null");  
    float* data;
    int step;
    cvGetRawData(distanceImage, (uchar**)&data, &step);
    step /= sizeof(data[0]);

    float totalcost = 0. ; 
    for(int i=0; i<this->NUM_DIRS; i++) 
    {
	int x=  cvRound((m_orient[i].x*state[2])+state[0]) ; 
	int y=  cvRound((m_orient[i].y*state[2])+state[1]) ; 
	if (m_pViewTracker->is_in_ROI(x,y)) 
	{
	    float* rowdata = data + y*step;
	    float distvalue = (float)fabs(rowdata[x]);
	    distvalue = std::min((float)(this->DISTTHRESH), distvalue); 
	    
	    //totalcost += angleconfidence>0.7?distvalue:10; 
	    totalcost += distvalue;//+ 10*(1-angleconfidence) ; 
	    //           distcost      anglecost
	}
	else 
	    totalcost += this->DISTTHRESH;
    }
    return (totalcost/this->NUM_DIRS);
}


void CircleView::get2DGroundPoint(float &x, float &y) {  
    x = m_beststate[0];
    y = (m_beststate[1] + m_beststate[2]);
}


void CircleView::getROI(CvBox2D &roi, float &outWeight) {
    roi.center.x = m_beststate[0];
    roi.center.y = m_beststate[1];
    roi.size.width = 2.0*m_beststate[2];
    roi.size.height = 2.0*m_beststate[2];
    outWeight = m_bestweight;
}


// initCondition is true only when ViewTracker is initially created.
void CircleView::detect(IplImage *resizedImg, bool initCondition) 
{
    if (m_bhistvalid == false) {
	user_printf(__HERE__, "Invalid histogram\n");
	return;
    }

    if (m_bImagesAllocated == false) {
	allocateImages(cvGetSize(resizedImg));
	m_bImagesAllocated = true;
    }
    
    CalBack(resizedImg, m_backprojectImage, m_hist) ; 

    float mean_size = (m_radius[0]+m_radius[1])/2;

    computePotentialImg(mean_size);

    bool bReinitialize=false;
    if ((GetDetectionState() == UNINITIALIZED) ||
	(GetDetectionState() == DISAPPEARED) || //always track disaapeared hand 
	(initCondition == true))
	bReinitialize = true;

    if (bReinitialize == true) {
	initstate(NULL) ;
	CalculateWeights();
    }
    else {
	Resample(); 
	DSystem();  
	CalculateWeights();
    }

    //currently support only mode 0, the most likely
    OutputTESTING(0) ; 
    
    // Need reinitialization?
    if (!is_reliable()) {
	m_missingCounter++; //check colorCost() edgeCost()
	SetDetectionState(TEMPORARY_LOST);
    }
    else {	
	//m_missingCounter--;
	m_missingCounter = 0;
	SetDetectionState(RELIABLE);
    }

    m_missingCounter = std::max(0, m_missingCounter);
    if (m_missingCounter > Vision::MAX_MISSING_COUNT) { 
	m_missingCounter = 0;
	SetDetectionState(DISAPPEARED);
    }
    
}

void CircleView::initUsingBackprojectWeight(int numStates) 
{
    user_assert(m_bhistvalid, __HERE__, "CircleView:: histogram not valid") ; 
    
    CvMat  *weightM  = cvCreateMat(m_bsize.height,m_bsize.width,CV_32F);
    CvMat  *accweightM  = cvCreateMat(m_bsize.height,m_bsize.width,CV_32F);
    cvZero(accweightM);
    cvConvert(m_weightImage,weightM) ; 
    TrackAux::AccumulateProb(weightM->data.fl, accweightM->data.fl, 
			     m_bsize.width*m_bsize.height); 
    
    float choicestate[this->DOFNUM];
    for(int i=0; i<numStates; i++) 
    {
	try {
	    float randomchoice = TrackAux::urandom()*accweightM->data.fl[m_bsize.width*m_bsize.height-1];
	    int choiceidx = TrackAux::bsearch(accweightM->data.fl,m_bsize.width*m_bsize.height,randomchoice);
	    float randomR = TrackAux::grandom((m_dofrange[2][0] + m_dofrange[2][1])/2, m_dofvar[2]);
	    randomR = std::min(std::max(randomR, m_dofrange[2][0]), m_dofrange[2][1]);//  make it within dofrange
	    
	    choicestate[0] = (choiceidx%m_bsize.width)*this->DOWNSAMPLE;
	    choicestate[1] = (choiceidx/m_bsize.width)*this->DOWNSAMPLE;
	    choicestate[2] = randomR; 
	    for (int j=0; j<DOFNUM; j++) 
		m_allstate[i][j] = choicestate[j];
	}
	catch(...) {
	    cout << "error in CircleView::initUsingBackprojectWeight\n";
	}
    }

    cvReleaseMat(&weightM);
    cvReleaseMat(&accweightM); 
}

void CircleView::CalculateWeights() {    
    double sum = 0.0;
    for (int i=0; i<this->NUM_STATES; i++) {
	m_weight[i] = Evaluate_combined(m_allstate[i]) ;
	sum += m_weight[i];
    }    
    if (sum == 0.0)
	throw user_error(__HERE__, "Total weight cannot be zero");
    for (int i=0; i<this->NUM_STATES; i++) 
	m_weight[i] /= (float)sum;
}

float CircleView::Evaluate_combined(float state[]) 
{    
    float edgematching_cost = Evaluate_distimg(state);
    float resultweight = (float)exp(-edgematching_cost);
    
    float colorDistValue = Evaluate_backProjection(state) ; 
    resultweight *= (float)exp(-colorDistValue);      

    return resultweight ; 
}

void CircleView::Resample() 
{
    user_assert(m_bhistvalid, __HERE__, "invalid histogram");
    float * weight1 = new float[this->NUM_STATES] ; 
    for(int i=0; i<this->NUM_STATES; i++)
	weight1[i] = m_weight[i];       
    ResampleAlgorithm(m_allstate, weight1, NUM_STATES, m_oldstate, NUM_STATES);
    
    delete []weight1 ; 
}

void CircleView::DSystem() 
{  
    for(int i=0;i<this->NUM_STATES-1;i++) {
	for(int j=0;j<this->DOFNUM;j++) {
	    float newstatevalue = (float)TrackAux::grandom(m_oldstate[i][j], m_dofvar[j]); 
	    m_allstate[i][j] = std::min(m_dofrange[j][1], 
					std::max(m_dofrange[j][0], newstatevalue));
	}
    }
    // Include the previous best state to test
    for(int j=0;j<this->DOFNUM;j++) 
	m_allstate[this->NUM_STATES-1][j] = m_beststate[j];    
}

void CircleView::OutputTESTING(int method) 
{  
    m_bestweight=0.;
    if(method == 0) {
	float maxweight = 0 ; 
	int maxidx = 0; 
	for (int i=0; i<NUM_STATES; i++) {
	    if(m_weight[i] > maxweight) {
		maxweight = m_weight[i] ; 
		maxidx = i ; 
	    }
	}
	for (int i=0; i<DOFNUM; i++) {
	    m_beststate[i] = m_allstate[maxidx][i];
	}
	m_bestweight = maxweight ; 
    }
    else if(method == 1) { 
	for(int i=0; i<this->DOFNUM; i++) 
	    m_beststate[i] = 0; 
	float weightsum = 0;
	for(int i=0; i<this->NUM_STATES; i++) {
	    for(int j=0;j<this->DOFNUM;j++) {
		m_beststate[j] += m_allstate[i][j]*m_weight[i];
	    }
	    weightsum += m_weight[i]; 
	    if (weightsum == 0.)
		throw user_error(__HERE__, "weightsum cannot be 0");
	    for(int j=0;j<this->DOFNUM;j++) {		
		m_beststate[j] /= weightsum*this->NUM_STATES; 
	    }
	}
	m_bestweight = weightsum/this->NUM_STATES ; 
    }
}
