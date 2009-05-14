/** @file RectView.cpp
 *  @brief A rectangle view.
 *
 *  @author Somboon Hongeng
 *  @date march 2007
 *  @bug No known bugs.
 */
#include "RectView.h"
#include "ViewTracker.h"
#include <vision/components/common/SystemUtils/Common.h>
#include <vision/components/common/VisualizationUtils/Display.h>

//#define SHOW_MASK
//#define SHOW_CALBACK

using namespace Common;
using namespace Display;
using namespace std;

const int RectView::DOFNUM_BOX;

RectView::RectView(ViewTracker *parent, int viewId, 
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

    m_mask = NULL;

    initStateParameters(config);
    
    m_storage = cvCreateMemStorage(0); 

    for (int i=0; i<this->NUM_DIRS; i++) 
	m_dirWeights[i] = 0.0;

    m_vContours.clear();
}

RectView::~RectView() {
    if (m_mask!=NULL)
	cvReleaseImage(&m_mask);

    cvReleaseMemStorage(&m_storage);
}


void RectView::compBoxOutlineTemplates(int c_x, int c_y, 
				       int c_w, int c_h, int c_d) 
{  
    int d = c_d;

    CvPoint2D32f delta, upP, lowP;  // up and low are from viewer's pov    
    delta.x = m_orient[d].x*c_w;   // cos  1
    delta.y = m_orient[d].y*c_w;  // sin  0    
    upP.x = (float)c_x + delta.y;   // point p
    upP.y = (float)c_y - delta.x;
    lowP.x = (float)c_x - delta.y;  // point q
    lowP.y = (float)c_y + delta.x;
    
    float dh = (float)c_h*2/this->NUM_PTS;
    delta.x = m_orient[d].x*dh;   // cos 1
    delta.y = m_orient[d].y*dh;  // sin 0
    
    int halfSamples = (int)(this->NUM_PTS/2);
    
    for(int i=0; i<halfSamples; i++) 
    {   //height adjusting	
	m_boxOutlines[0][i].x = cvRound(lowP.x + delta.x*i); //h*cos
	m_boxOutlines[0][i].y = cvRound(lowP.y + delta.y*i); //
	m_boxOutlines[1][i].x = cvRound(upP.x + delta.x*i);
	m_boxOutlines[1][i].y = cvRound(upP.y + delta.y*i);
    }
    
    for(int i=0; i<halfSamples; i++) 
    {   //height adjusting	
	m_boxOutlines[0][i+halfSamples].x = cvRound(lowP.x - delta.x*i); //h*cos
	m_boxOutlines[0][i+halfSamples].y = cvRound(lowP.y - delta.y*i); //
	m_boxOutlines[1][i+halfSamples].x = cvRound(upP.x - delta.x*i);
	m_boxOutlines[1][i+halfSamples].y = cvRound(upP.y - delta.y*i);
	//cvCircle(img,cvPoint(outline[i].x, outline[i].y), 2, color_r,1); 
    }
}


void RectView::drawBoxOutline(IplImage *img, float *state, CvScalar color) 
{
    CvPoint2D32f boxpts[4];
    Vector3D pts[4];
    
    computeBBoxPoints(state, boxpts);
    alignPointsOnBBox(boxpts, pts);
    
    cvLine(img, cvPoint((int)pts[0].x(), (int)pts[0].y()), cvPoint((int)pts[1].x(), (int)pts[1].y()), color, 2);
    cvLine(img, cvPoint((int)pts[1].x(), (int)pts[1].y()), cvPoint((int)pts[3].x(), (int)pts[3].y()), color, 2);
    cvLine(img, cvPoint((int)pts[3].x(), (int)pts[3].y()), cvPoint((int)pts[2].x(), (int)pts[2].y()), color, 2);
    cvLine(img, cvPoint((int)pts[2].x(), (int)pts[2].y()), cvPoint((int)pts[0].x(), (int)pts[0].y()), color, 2);
}

void RectView::alignPointsOnBBox(CvPoint2D32f boxpts[], Vector3D pts[])
{
    int minYidx1=-1, minYidx2=-1, minYidx3=-1, minYidx4=-1; 
    
    //float minX = std::numeric_limits<float>::max(); 
    float minY = std::numeric_limits<float>::max(); 
    //float maxX = std::numeric_limits<float>::min(); 
    //float maxY = std::numeric_limits<float>::min(); 
    
    for (int i=0; i<4; i++) {
	if (boxpts[i].y < minY) {
	    minY = boxpts[i].y;
	    minYidx1 = i;
	}
    }
    minY = std::numeric_limits<float>::max(); 
    for (int i=0; i<4; i++) {
	if (i!= minYidx1)
	    if (boxpts[i].y < minY) {
		minY = boxpts[i].y;
		minYidx2 = i;
	    }
    }
    // now sort pnts according to X
    if (boxpts[minYidx1].x < boxpts[minYidx2].x) {
	pts[0].set(boxpts[minYidx1].x, boxpts[minYidx1].y, 0.0);
	pts[1].set(boxpts[minYidx2].x, boxpts[minYidx2].y, 0.0); 
    }
    else {
	pts[0].set(boxpts[minYidx2].x, boxpts[minYidx2].y, 0.0);
	pts[1].set(boxpts[minYidx1].x, boxpts[minYidx1].y, 0.0); 
    }
    
    for (int i=0; i<4; i++) {
	if ((i!= minYidx1) && (i!=minYidx2)) {
	    minYidx3 = i;
	    break;
	}
    }
    for (int i=0; i<4; i++) {
	if ((i!= minYidx1) && (i!=minYidx2) && (i!=minYidx3)) {
	    minYidx4 = i;
	    break;
	}
    }
    
    
    // now sort pnts according to X
    if (boxpts[minYidx3].x < boxpts[minYidx4].x) {
	pts[2].set(boxpts[minYidx3].x, boxpts[minYidx3].y, 0.0);
	pts[3].set(boxpts[minYidx4].x, boxpts[minYidx4].y, 0.0); 
    }
    else {
	pts[2].set(boxpts[minYidx4].x, boxpts[minYidx4].y, 0.0);
	pts[3].set(boxpts[minYidx3].x, boxpts[minYidx3].y, 0.0); 
    }
}


void RectView::computeBBoxPoints(float state[],
				 CvPoint2D32f boxpts[]) {
  
    CvPoint2D32f upP, lowP;  // up and low are from viewer's pov
    
    float c_x, c_y, c_w, c_h;
    int c_d;
    
    c_x = state[0]; c_y = state[1];
    c_w = state[2]; c_h = state[3]; 
    c_d = (int)state[4];
    
    float x_cos = m_orient[c_d].x * c_w;   // cos  1
    float y_sin = m_orient[c_d].y * c_w;  // sin  0
    
    upP.x = c_x + y_sin;   // point p
    upP.y = c_y - x_cos;
    lowP.x = c_x - y_sin;  // point q
    lowP.y = c_y + x_cos;
    
    
    float h_cos = m_orient[c_d].x * c_h;  
    float h_sin = m_orient[c_d].y * c_h ;
    
    
    // bottom right0, left1
    boxpts[0].x = cvRound(lowP.x + h_cos); //h*cos
    boxpts[0].y = cvRound(lowP.y + h_sin); //
    
    boxpts[1].x = cvRound(upP.x + h_cos);
    boxpts[1].y = cvRound(upP.y + h_sin);
  
    // top right0, left1
    boxpts[2].x = cvRound(lowP.x - h_cos); //h*cos
    boxpts[2].y = cvRound(lowP.y - h_sin); //
    
    boxpts[3].x = cvRound(upP.x - h_cos);
    boxpts[3].y = cvRound(upP.y - h_sin);    
}

//  BUG: num of contours misht be more than MAX_NUM_CONTOURS  
void RectView::computeCC(IplImage *resizedImg) 
{
    // 1) thres and save as maskImg
     float salienceMin = 50;
     float salienceMax = 255;
    
    CvSize bpSize = cvGetSize(m_backprojectImage);
    IplImage *dummy = cvCreateImage(bpSize,8,1);
    cvZero(m_mask);
    cvInRangeS( m_backprojectImage, cvScalar(salienceMin),
		cvScalar(salienceMax), m_mask );
    cvDilate(m_mask, dummy, NULL, 2);
    cvErode(dummy, m_mask, NULL, 2);
    cvReleaseImage(&dummy);
    
#ifdef SHOW_MASK
    cvNamedWindow("thres img", 1);
    cvShowImage("thres img", m_mask);
    cvWaitKey();
#endif

    
    CvSeq* contour=NULL;    
    // int total = cvFindContours(m_mask, m_storage, &contour, 
// 			       sizeof(CvContour), 
// 			       CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    //CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    
        
    dummy = cvCloneImage(resizedImg);


    m_vContours.clear();
    for( ; contour != 0; contour = contour->h_next ) {
	
	CvSeq* result = cvApproxPoly(contour, sizeof(CvContour), m_storage, 
			      CV_POLY_APPROX_DP, 
			      cvContourPerimeter(contour)*0.01, 0 );
    
	if ((fabs(cvContourArea(result, CV_WHOLE_SEQ)) > 500) &&
	    (result->total >= 6)){

	    
#if 0
	    CvBox2D box = cvMinAreaRect2(result);
	    drawMinAreaRectBox(dummy, box);
#else      
	    CvBox2D box = cvFitEllipse2(result);
	    //drawEllipseBox(dummy, box);
#endif
	    
	    //contours[contourId] = box;
	    if (validateBox(box)) 
	      m_vContours.push_back(box);

	    
	} // if the area of the region > 1000
	
	
	if (result!=NULL)
	    cvClearSeq(result); // return memory to storage
    } // for all contours
    
    
    
    if (contour!=NULL) cvClearSeq(contour);
    
    cvReleaseImage(&dummy);
}


void RectView::computeDirectionWeights(void) 
{
    int gaussSize = (int)(this->NUM_DIRS/2);
    double gaussMask[gaussSize]; 
    
    for (int k=0; k< this->NUM_DIRS; k++)
	m_dirWeights[k] = 0;
    
    float box_angle;
    float ang_std;
    

    for (unsigned i=0; i<m_vContours.size(); i++) {
      
      CvBox2D box = m_vContours[i];

	computeBoxAngles(box, box_angle, ang_std);
	int step_width = (int)(360/this->NUM_DIRS);
	int std_steps = (int)ceil(ang_std/step_width);
	
	if ( isinf(step_width) || isnan(step_width) ||
	     isinf(std_steps) || isnan(std_steps) )
	  throw user_error(__HERE__, "Invalid numbers");

	float strength = (box.size.width*box.size.height)/10;
	
	computeGaussianWeight(gaussMask, gaussSize, std_steps);
    
	int dirIdx = degree_to_dirIdx(box_angle, step_width) -1 ;
	dirIdx = adjust_dirIdx(dirIdx,0);
    
	m_dirWeights[dirIdx] +=  strength*gaussMask[0];
	
// 	cout << "[RectView::computeDirectionWeights] " << endl;
// 	cout << "Best dirIdx " << dirIdx << ", strength " << m_dirWeights[dirIdx] << endl;

// 	for (int k=1; k<gaussSize; k++) {
// 	    int nb_dirIdx = adjust_dirIdx(dirIdx,k);
// 	    m_dirWeights[nb_dirIdx] += strength*gaussMask[k];
	    
// 	    nb_dirIdx = adjust_dirIdx(dirIdx,-k);
// 	    m_dirWeights[nb_dirIdx] += strength*gaussMask[k];
// 	    //armDirWeights[k] += w*def.depth;
// 	}


	// upto +90 and -90
	for (int k=1; k<gaussSize-9; k++)
	{
	    int nb_dirIdx = adjust_dirIdx_0_18(dirIdx,k);
	    m_dirWeights[nb_dirIdx] += strength*gaussMask[k];
	    
	    nb_dirIdx = adjust_dirIdx_0_18(dirIdx,-k);
	    m_dirWeights[nb_dirIdx] += strength*gaussMask[k];
	    //armDirWeights[k] += w*def.depth;
	}
    }
}


double RectView::computeAngle(CvPoint2D32f &p0, CvPoint2D32f &p1, CvPoint2D32f &p2) {
    double a[] = {p1.x - p0.x, p2.x - p0.x};
    double b[] = {p1.y - p0.y, p2.y - p0.y};
    
    // p0p1 angle
    double ang1 = cvFastArctan(b[0], a[0] );

    // p0p2 angle
    double ang2 = cvFastArctan(b[1], a[1] );

    double ang_diff = compute_angle_difference(ang1, ang2);
    
    if (ang_diff == 0) {
//       cout << p0.x << "," << p0.y << endl;
//       cout << p1.x << "," << p1.y << endl;
//       cout << p2.x << "," << p2.y << endl;
//       cout << "ang1 " << ang1 << endl;
//       cout << "ang2 " << ang2 << endl;
//       cout << "ang_diff " << ang_diff << endl;
//       throw user_error(__HERE__, "angdiff is not valid");
      
      ang_diff = 5; // for now, use this default when things go wrong
    }

    return ang_diff/2.0;
}



bool RectView::validateBox(CvBox2D ell_box) {
  CvSize bpSize = cvGetSize(m_backprojectImage);
  if (ell_box.center.x>0 && ell_box.center.x<bpSize.width &&
      ell_box.center.y>0 && ell_box.center.y<bpSize.height &&
      ell_box.size.width>0 && ell_box.size.width<bpSize.width &&
      ell_box.size.height>0 && ell_box.size.height<bpSize.width)
    return true;
  else 
    return false;
}

void RectView::computeBoxAngles(CvBox2D ell_box, 
				float &angle_in_deg, float &std_angle) 
{  
    //float angle_rad = (float) (ell_box.angle*TrackAux::PI)/180;

    // Get corners and center
    CvPoint2D32f corners[4];
    cvBoxPoints(ell_box, corners);
    CvPoint2D32f ctr = cvPoint2D32f(ell_box.center.x, ell_box.center.y);
    

    double ang_std1 = 0.3*computeAngle(ctr, corners[0], corners[1]);
    double ang_std2 = 0.3*computeAngle(ctr, corners[1], corners[2]);
    
    angle_in_deg = ell_box.angle; 
    std_angle = (float)std::min(ang_std1, ang_std2);    


    // convert angle from (upright-0, upright360) to  0<---*--->180 format
    if ((angle_in_deg >= 90) && (angle_in_deg < 180))
	angle_in_deg += 180;
    else if ((angle_in_deg >= 180) && (angle_in_deg < 270))
	angle_in_deg -= 180;
    
    if ((angle_in_deg >=0) && (angle_in_deg < 90))
	angle_in_deg += 90;
    else 
	angle_in_deg -= 270;

#ifdef LOG
    cout << "[RectView::computeBoxAngles] ell.angle: " << angle_in_deg
	 << ", ell.ang_std: " << std_angle << endl;
#endif
}


void RectView::computeBoxAngles_orig(CvBox2D ell_box, float &angle_in_deg, float &std_angle) 
{  
    float angle_rad = (float) (ell_box.angle*TrackAux::PI)/180;
    float unit_cos = (float)cos(angle_rad);
    float unit_sin = (float)sin(angle_rad); 
    
    // shift width
    CvPoint p, q;
    p.x =  (int) (ell_box.center.x + (ell_box.size.height/2)*unit_sin);
    p.y =  (int) (ell_box.center.y - (ell_box.size.height/2)*unit_cos);
    q.x =  (int) (ell_box.center.x - (ell_box.size.height/2)*unit_sin);
    q.y =  (int) (ell_box.center.y + (ell_box.size.height/2)*unit_cos);
    
    // shift height
    float d_h = ell_box.size.width/2;
    float dh_sin = d_h * unit_sin;
    float dh_cos = d_h * unit_cos;
    
    CvPoint corner1, corner2, ctr;
    corner1.x = p.x + (int)dh_cos;
    corner1.y = p.y + (int)dh_sin;  
    corner2.x = q.x + (int)dh_cos;
    corner2.y = q.y + (int)dh_sin;
    ctr.x = (int)ell_box.center.x;
    ctr.y = (int)ell_box.center.y;
    
    double a[] = {corner1.x - ctr.x, corner2.x - ctr.x};
    double b[] = {corner1.y - ctr.y, corner2.y - ctr.y};
    double ang1 = cvFastArctan((double) b[0], (double) a[0] );
    double ang2 = cvFastArctan((double) b[1], (double) a[1] );
    double ang_diff1 = compute_angle_difference(ell_box.angle, ang1);
    double ang_diff2 = compute_angle_difference(ell_box.angle, ang2);
    double ang_std = (MIN(ang_diff1, ang_diff2))/3;
    
    angle_in_deg = ell_box.angle; 
    std_angle = ang_std;    

    // convert angle from (upright-0, upright360) to  0<---*--->180 format
    if ((angle_in_deg >= 90) && (angle_in_deg < 180))
	angle_in_deg += 180;
    else if ((angle_in_deg >= 180) && (angle_in_deg < 270))
	angle_in_deg -= 180;
    
    if ((angle_in_deg >=0) && (angle_in_deg < 90))
	angle_in_deg += 90;
    else 
	angle_in_deg -= 270;

#ifdef LOG
    cout << "[RectView::computeBoxAngles] ell.angle: " << angle_in_deg
	 << ", ell.ang_std: " << std_angle << endl;
#endif
}


void RectView::computeGaussianWeight(double gaussMask[], 
				     int total_steps, int std_steps) 
{
  if (std_steps == 0)
    throw user_error(__HERE__, "std_steps cannot be 0");

    for (int i=0; i<total_steps; i++) {
	double dist = (i*i/(2*std_steps*std_steps));
	gaussMask[i] = (exp(-dist))/std_steps;      
    }
}


int RectView::degree_to_dirIdx(double angle_in_degrees, double step_width) {
  if (step_width == 0.0)
    throw user_error(__HERE__, "step_width cannot be 0");
    int idx  = (int)ceil(angle_in_degrees/step_width);
    return(idx);
}


int RectView::adjust_dirIdx_0_18(int dirIdx, int adjustment) 
{
    int idx=0;
    dirIdx += adjustment;
    if (dirIdx > 17)
	idx = dirIdx - 18;
    else if (dirIdx < 0)
	idx = dirIdx + 18;
    else 
	idx = dirIdx;    
    return(idx);
}

int RectView::adjust_dirIdx(int dirIdx, int adjustment) 
{
    int idx=0;
    dirIdx += adjustment;
    if (dirIdx > this->NUM_DIRS-1)
	idx = dirIdx - this->NUM_DIRS;
    else if (dirIdx < 0)
	idx = dirIdx + this->NUM_DIRS;
    else 
	idx = dirIdx;    
    return(idx);
}

double RectView::compute_angle_difference(double ang1, double ang2) 
{
  double ang_diff = fabs(ang1 - ang2);
  if (ang_diff > 180)
      ang_diff -= 180;
  return ang_diff;
}


void RectView::normalize_dirWeights(void) 
{
#ifdef LOG
  cout << "[RectView::normalize_dirWeights] " << endl;
#endif

    double sum=0.;
    for (int k=0; k< this->NUM_DIRS; k++)
	sum += m_dirWeights[k];    
    for (int k=0; k< this->NUM_DIRS; k++) {
	m_dirWeights[k] /= sum;

#ifdef LOG
	cout << "(" << k 
	     << "," << m_dirWeights[k] 
	     << ") ";
#endif
    }

#ifdef LOG
    cout << endl;
#endif
}


void RectView::ResampleAlgorithm_BOX(float myoldstate[][DOFNUM_BOX], 
				     float oldsampleweight[], 
				     int oldparticlenum, 
				     float mynewstate[][DOFNUM_BOX], 
				     int newparticlenum) {
    float *myaccweigth = new float [oldparticlenum]; 
    TrackAux::AccumulateProb(oldsampleweight, myaccweigth, oldparticlenum); 
    for(int i=0;i<newparticlenum;i++) {
	float randomchoice = (float) TrackAux::urandom() * myaccweigth[oldparticlenum-1];
	int choiceidx = TrackAux::bsearch(myaccweigth, oldparticlenum, randomchoice); 
	memcpy(mynewstate[i], myoldstate[choiceidx], this->DOFNUM_BOX*sizeof(float));
    }
    delete []myaccweigth;
}


//////////// Interface Implementation /////////////

void RectView::allocateImages(CvSize _bsize) {
    if (m_bImagesAllocated)
	return;
    
    m_bsize.width = _bsize.width; 
    m_bsize.height= _bsize.height;
    
    m_backprojectImage = cvCreateImage(m_bsize,8,1) ; 
    m_weightImage = cvCreateImage(m_bsize,8,1) ;
    m_mask = cvCreateImage(m_bsize,8,1); 

    m_bImagesAllocated = true;
}




//  To do: Should be initialized from database/config file/motion detection
void RectView::initStateParameters(map<string,string> &config) 
{
    if (m_pViewTracker->bTmpimages_allocated==false) 
	throw user_error(__HERE__, "Images must be allocated first");
    
    ResolutionType resType = m_pViewTracker->resolutionType();
    float scale = (resType==RES_QVGA)? 0.5 : 1;
    
    map<string,string>::iterator iter;
    if ((iter=config.find("W_MIN")) != config.end())
      m_ellipseRadius[0][0] = scale * strtod(iter->second.c_str(), NULL);
    else
      throw user_error(__HERE__, "W_MIN is not specified");
    
    if ((iter=config.find("W_MAX")) != config.end())
      m_ellipseRadius[0][1] = scale * strtod(iter->second.c_str(), NULL);
    else
      throw user_error(__HERE__, "W_MAX is not specified");
        
    if ((iter=config.find("H_MIN")) != config.end())
      m_ellipseRadius[1][0] = scale * strtod(iter->second.c_str(), NULL);
    else
      throw user_error(__HERE__, "H_MIN is not specified");

    if ((iter=config.find("H_MAX")) != config.end())
      m_ellipseRadius[1][1] = scale * strtod(iter->second.c_str(), NULL);
    else
      throw user_error(__HERE__, "H_MAX is not specified");

    
    // box (lab)
    m_dofrange_box[0][0] = 0.;				   
    m_dofrange_box[0][1] = (float) (m_pViewTracker->imgsize.width);
    m_dofrange_box[1][0] = 0.;				   
    m_dofrange_box[1][1] = (float) (m_pViewTracker->imgsize.height);
    m_dofrange_box[2][0] = m_ellipseRadius[0][0];  
    m_dofrange_box[2][1] = m_ellipseRadius[0][1];  
    m_dofrange_box[3][0] = m_ellipseRadius[1][0];  
    m_dofrange_box[3][1] = m_ellipseRadius[1][1];  
    m_dofrange_box[4][0] = 0;   
    m_dofrange_box[4][1] = scale*35;  

    m_dofvar_box[0] = m_dofvar_box[1] = scale*10.; 
    m_dofvar_box[2] = m_dofvar_box[3] = scale*5;
    m_dofvar_box[4] = scale*4;
    
    
    for (int i=0; i<this->DOFNUM_BOX; i++)
	m_beststate_box[i]=0.;
    m_bestweight = 0.;
}


void RectView::drawAllSamples(IplImage *img) {
    for (int j=0; j<NUM_STATES; j++) {
	int x = cvRound(m_allstate_box[j][0]);
	int y = cvRound(m_allstate_box[j][1]);
	int r = cvRound( (m_allstate_box[j][2] + m_allstate_box[j][3])/2 );      
	cvCircle(img, cvPoint(x,y), r,CV_RGB(0,255,126),1);
    }  
}

void RectView::drawASample(IplImage *img) 
{
    CvScalar color = getRandomColor(m_id);
    float tmpstate[5] = {m_beststate_box[0],
			 m_beststate_box[1],
			 m_beststate_box[2],
			 m_beststate_box[3],
			 m_beststate_box[4]};		
    drawBoxOutline(img, tmpstate, color);
}



float RectView::colorCost() {
    return Evaluate_backProjection(m_beststate_box);
}

float RectView::edgeCost() {
    return 0.;
}


float RectView::averageBackProjection(float state[], bool bCompletePose) 
{
    if(!m_bhistvalid) 
	return 0.0;

    if (bCompletePose == true) {

	int x = cvRound(state[0]/this->DOWNSAMPLE) ; 
	int y = cvRound(state[1]/this->DOWNSAMPLE) ; 
	int w = cvRound(state[2]/this->DOWNSAMPLE) ; 
	int h = cvRound(state[3]/this->DOWNSAMPLE) ; 
	int d = cvRound(state[4]);
	
	float m[6];
	CvMat map_mat = cvMat(2,3,CV_32F,m);
	IplImage *tmpimg = cvCreateImage(cvGetSize(m_backprojectImage), 8, 1); 
	cv2DRotationMatrix(cvPoint2D32f(state[0],state[1]), 10*(9-d), 1, &map_mat);
	cvWarpAffine(m_backprojectImage, tmpimg, &map_mat); 
	
	CvPoint pt1 = cvPoint(MAX(0,x-w),MAX(0,y-h));
	CvPoint pt2 =cvPoint( std::min(m_bsize.width,x+w), 
			      std::min(m_bsize.height,y+h) ) ; 
	IplROI roi = {0, pt1.x, pt1.y, pt2.x-pt1.x, pt2.y-pt1.y} ; 
	
	tmpimg->roi = &roi; 
	float meanvalue = cvMean(tmpimg) ; 
	tmpimg->roi = NULL;
	
	cvReleaseImage(&tmpimg); 
	return meanvalue ; 
    }
    else {
	int x = cvRound(state[0]/this->DOWNSAMPLE) ; 
	int y = cvRound(state[1]/this->DOWNSAMPLE) ; 
	int r = cvRound(state[2]/this->DOWNSAMPLE) ; 
	CvPoint pt1 = cvPoint( std::max(0,x-r), std::max(0,y-r) );
	CvPoint pt2 =cvPoint( std::min(m_bsize.width,x+r), std::min(m_bsize.height,y+r) ); 
	IplROI roi = {0, pt1.x, pt1.y, pt2.x-pt1.x, pt2.y-pt1.y} ; 
	
	m_backprojectImage->roi = &roi; 
	
	float meanvalue = cvMean(m_backprojectImage) ; 
	
	m_backprojectImage->roi = NULL;  
	return meanvalue ; 
    }
    
}


float RectView::Evaluate_distimg(float state[]) {
    // Access to data of distimg. No need to release memory.
    IplImage *distanceImage = m_pViewTracker->getDistImg();
    if (distanceImage == NULL) 
	throw user_error(__HERE__, "image null");  
    float* data;
    int step;
    cvGetRawData(distanceImage, (uchar**)&data, &step);
    step /= sizeof(data[0]);
    
    // create m_boxOutlines
    compBoxOutlineTemplates((int)state[0], (int)state[1], 
			    (int)state[2], (int)state[3], (int)state[4]);
    float totalcost = 0. ; 
    int numBoxContours = 2; 
    for (int i=0; i<numBoxContours;  i++) 
    {
	for(int j=0; j<this->NUM_PTS; j++) 
	{
	    int x = m_boxOutlines[i][j].x;
	    int y = m_boxOutlines[i][j].y;
	    if (m_pViewTracker->is_in_ROI(x,y)) 
	    {
		float* rowdata = data + y*step;
		float distvalue = (float)fabs(rowdata[x]);
		distvalue = std::min((float)(this->DISTTHRESH), distvalue); 
		totalcost += distvalue;
	    }
	    else 
		totalcost += this->DISTTHRESH;
	}  
    } 
    return (totalcost/(this->NUM_PTS*numBoxContours));
}

float RectView::Evaluate_combined(float state[]) {
    float edgematching_cost = Evaluate_distimg(state);
    float resultweight = (float)exp(-edgematching_cost);
    
    float colorDistValue = Evaluate_backProjection(state) ; 
    resultweight *= (float)exp(-colorDistValue);      
    
    return resultweight ; 
}

void RectView::get2DGroundPoint(float &x, float &y) {
    x = m_beststate_box[0];
    y = m_beststate_box[1] + m_beststate_box[3];
}

void RectView::getROI(CvBox2D &roi, float &outWeight) {
    roi.center.x = m_beststate_box[0];
    roi.center.y = m_beststate_box[1];
    roi.size.width = 2.0*m_beststate_box[2];
    roi.size.height = 2.0*m_beststate_box[3];    
    outWeight = this->m_bestweight;
}

void RectView::detect(IplImage *resizedImg, bool initCondition) 
{

    if (m_bhistvalid == false)
	return;

    //m_bReinitialize = initCondition || m_bReinitialize;
    
    if (m_bImagesAllocated == false) {
	allocateImages(cvGetSize(resizedImg));
	m_bImagesAllocated = true;
    }
    
    CalBack(resizedImg, m_backprojectImage, m_hist) ;

    

#ifdef SHOW_CALBACK
    cvNamedWindow("calback",1);
    cvShowImage("calback", m_backprojectImage);
    cvWaitKey();
#endif      

    computeCC(resizedImg);
    computeDirectionWeights();
    normalize_dirWeights();
    
    float mean_size = (m_ellipseRadius[0][0]+ m_ellipseRadius[0][1])/2; 
    computePotentialImg(mean_size); 


#ifdef SHOW_WEIGHT
    cvNamedWindow("weight img", 1);
    cvShowImage("weight img", m_weightImage);
    cvWaitKey();
#endif   

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
    

    if (!is_reliable()) {
	m_missingCounter++;
	SetDetectionState(TEMPORARY_LOST);
    }
    else  {
	//m_missingCounter--;
	m_missingCounter = 0;
	SetDetectionState(RELIABLE);
    }

    m_missingCounter = std::max(0, m_missingCounter);

    if (m_missingCounter > Vision::MAX_MISSING_COUNT) {	
	SetDetectionState(DISAPPEARED);
	m_missingCounter = 0;
    }
}

void RectView::initstate(float * initstate) 
{
#ifdef LOG
  cout << "[RectView::initstat] " << endl;
#endif    

    initUsingBackprojectWeight(this->NUM_STATES) ; 
    for(int i=0; i< this->NUM_STATES; i++) 
	m_weight[i] = 1.0f; 
}

void RectView::initUsingBackprojectWeight(int numStates) 
{
    user_assert(m_bhistvalid, __HERE__,
		"RectView:: histogram not valid") ; 
    
    CvMat  *weightM  = cvCreateMat(m_bsize.height,m_bsize.width,CV_32F);
    CvMat  *accweightM  = cvCreateMat(m_bsize.height,m_bsize.width,CV_32F);
    cvZero(accweightM);
    cvConvert(m_weightImage,weightM) ; 
    TrackAux::AccumulateProb(weightM->data.fl, accweightM->data.fl, 
			     m_bsize.width*m_bsize.height); 
    

    CvMat *dirM =  cvCreateMat(1, this->NUM_DIRS, CV_32F);
    CvMat  *accDirM  = cvCreateMat(1, this->NUM_DIRS, CV_32F);
    cvZero(accDirM);
    cvInitMatHeader(dirM, 1, this->NUM_DIRS, CV_32F, &m_dirWeights);
    TrackAux::AccumulateProb(dirM->data.fl, accDirM->data.fl, this->NUM_DIRS) ;
    
    float choicestate_box[this->DOFNUM_BOX];
    for(int i=0; i<numStates; i++) 
    {
	bool bDone = false;
	int loop = 1;
	int MAXLOOP = 3000;
	while (!bDone) 
	{
	    float randomchoice = TrackAux::urandom() * accweightM->data.fl[m_bsize.width * m_bsize.height-1];
	    int choiceidx = TrackAux::bsearch(accweightM->data.fl, m_bsize.width * m_bsize.height, randomchoice);
	    
	    float randomRx = TrackAux::grandom( (m_dofrange_box[2][0] + m_dofrange_box[2][1])/2, m_dofvar_box[2]);
	    randomRx = std::min( std::max(randomRx, m_dofrange_box[2][0]), m_dofrange_box[2][1]); 
	    
	    float randomRy = TrackAux::grandom( (m_dofrange_box[3][0] + m_dofrange_box[3][1])/2, m_dofvar_box[3]);
	    randomRy = std::min( std::max(randomRy, m_dofrange_box[3][0]), m_dofrange_box[3][1]); 
	    
	    
	    // add random directions
	    float dirRandomchoice = TrackAux::urandom()*accDirM->data.fl[this->NUM_DIRS-1];
	    int dirChoiceidx = TrackAux::bsearch(accDirM->data.fl, 
						 this->NUM_DIRS, 
						 dirRandomchoice);
	    
	    choicestate_box[0] = (choiceidx%m_bsize.width)*this->DOWNSAMPLE;
	    choicestate_box[1] = (choiceidx/m_bsize.width)*this->DOWNSAMPLE;
	    choicestate_box[2] = randomRx; 
	    choicestate_box[3] = randomRy;
	    choicestate_box[4] = dirChoiceidx;
	    // choicestate_box[4] = assignPrincipalAxis(choicestate_box[0], choicestate_box[1]);
	    
	    if ((averageBackProjection(choicestate_box, false) > 0.5*m_maxWeight) ||
		(loop > MAXLOOP))
		bDone = true;
	    loop++;
	}
	
	// To do: avoid memcpy 
	memcpy(m_allstate_box[i], choicestate_box, this->DOFNUM_BOX*sizeof(float));
    }
    
    cvReleaseMat(&dirM);
    cvReleaseMat(&accDirM);

    cvReleaseMat(&weightM);
    cvReleaseMat(&accweightM); 
}


void RectView::CalculateWeights() 
{
    double sum = 0.0;
    for (int i=0; i<this->NUM_STATES; i++) {
	m_weight[i] = Evaluate_combined(m_allstate_box[i]) ;
	sum += m_weight[i];
    }
    
    for (int i=0; i<this->NUM_STATES; i++) 
	m_weight[i] /= (float)sum;
}

void RectView::Resample() 
{
    user_assert(m_bhistvalid, 
		__HERE__, "RectView::Resample: invalid histogram") ; 
    
    float * weight1 = new float[this->NUM_STATES] ; 
    for(int i=0; i<this->NUM_STATES; i++)
	weight1[i] = m_weight[i]; 

    ResampleAlgorithm_BOX(m_allstate_box, weight1, this->NUM_STATES, 
			  m_oldstate_box, this->NUM_STATES) ; 
    delete []weight1 ; 
}

void RectView::DSystem() 
{
    for(int i=0;i<this->NUM_STATES-1;i++) {
	for(int j=0;j<this->DOFNUM_BOX;j++) {
	    float newstatevalue = (float)TrackAux::grandom(m_oldstate_box[i][j], m_dofvar_box[j]); 
	    m_allstate_box[i][j] = std::min(m_dofrange_box[j][1], 
					    std::max(m_dofrange_box[j][0], newstatevalue));
	}
    }
    // Include the previous best state to test
    for(int j=0;j<this->DOFNUM_BOX;j++) 
	m_allstate_box[this->NUM_STATES-1][j] = m_beststate_box[j];    
}

void RectView::OutputTESTING(int method) 
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
	for (int i=0; i<DOFNUM_BOX; i++)
	    m_beststate_box[i] = m_allstate_box[maxidx][i];
	m_bestweight = maxweight ; 
    }
    else if(method == 1) 
	throw user_error(__HERE__, "unsupport output method");
}
