/**
 * @file TomGineThread.cc
 * @author Richtsfeld, Prankl
 * @date March 2011
 * @version 0.1
 * @brief Wraper for the tomGine, implementing a threaded 3d render window.
 */


#include "TomGineThread.hh"

namespace TGThread 
{

/**
 * @brief RGBValue of point clouds, accessable as float or long value.
 */
typedef union
{
  struct
  {
    unsigned char b;  // Blue channel
    unsigned char g;  // Green channel
    unsigned char r;  // Red channel
    unsigned char a;  // Alpha channel
  };
  float float_value;
  long long_value;
} RGBValue;

/**
 * ThreadDrawing
 */
void* ThreadDrawing(void* c)
{
  TomGineThread *tt = (TomGineThread*)c;

  cout<<"INIT 3D RENDERING"<<endl;    // then wie can use 3d rendering
  TomGine::tgEngine render(tt->width,tt->height, 10.0, 0.01, "TomGine Render Engine", true);

  bool end=false;
  std::vector<blortGLWindow::Event> eventlist;
  tt->tgTex = new TomGine::tgTexture();

  while (!end && !tt->stopTomGineThread)
  {
    pthread_mutex_lock(&tt->dataMutex);
    tt->Draw3D(render);

    render.Update(eventlist);
    pthread_mutex_unlock(&tt->dataMutex);

    usleep(10000);

    pthread_mutex_lock(&tt->dataMutex);
    end = tt->KeyHandler(eventlist);
    pthread_mutex_unlock(&tt->dataMutex);
  }

  delete tt->tgTex;

  return((void *)0);
}



/********************** TomGineThread ************************/
/**
 * Constructor
 */
TomGineThread::TomGineThread(int w, int h)
  : width(w), height(h), mode(1), stopTomGineThread(false), 
    drawImage(false), draw3D(true), drawPointCloud(true), drawVPointCloud(true), drawLabels(false), useProbLevel(false), showCoordinateFrame(false)
{
  pthread_mutex_init(&dataMutex,NULL);
  pthread_create(&thread, NULL, ThreadDrawing, this);
  InitializePose(camPose);
  camPoseChanged=false;
  rotCenter[0]=0.; 
  rotCenter[1]=0.; 
  rotCenter[2]=0.;
}

/**
 * Destructor
 */
TomGineThread::~TomGineThread()
{
  stopTomGineThread=true;
  pthread_join(thread,NULL);
  pthread_mutex_destroy(&dataMutex);
}

/**
 * KeyHandler
 */
bool TomGineThread::KeyHandler(std::vector<blortGLWindow::Event> &events)
{
  for (unsigned i=0; i<events.size(); i++)
  {
    if(events[i].type == blortGLWindow::TMGL_Press)
    {
      if (events[i].input == blortGLWindow::TMGL_Escape)
        return true;
      else if (events[i].input == blortGLWindow::TMGL_0)
        mode=0;
      else if (events[i].input == blortGLWindow::TMGL_1)
        mode=1;
      else if (events[i].input == blortGLWindow::TMGL_2)
        mode=2;
      else if (events[i].input == blortGLWindow::TMGL_3)
        mode=3;
      else if (events[i].input == blortGLWindow::TMGL_4)
        mode=4;
      else if (events[i].input == blortGLWindow::TMGL_5)
        mode=5;
      else if (events[i].input == blortGLWindow::TMGL_6)
        mode=6;
      else if (events[i].input == blortGLWindow::TMGL_7)
        mode=7;
      else if (events[i].input == blortGLWindow::TMGL_8)
        mode=8;
      else if (events[i].input == blortGLWindow::TMGL_c)
        Clear();
      else if (events[i].input == blortGLWindow::TMGL_i)
        drawImage = !drawImage;
      else if (events[i].input == blortGLWindow::TMGL_d)
        draw3D = !draw3D;
      else if (events[i].input == blortGLWindow::TMGL_p)
        drawPointCloud = !drawPointCloud;
      else if (events[i].input == blortGLWindow::TMGL_l)
        drawLabels = !drawLabels;
      else if (events[i].input == blortGLWindow::TMGL_u)
        useProbLevel = !useProbLevel;
      else if (events[i].input == blortGLWindow::TMGL_q)
        return true;
    }
  }
  events.clear();
  return false;
}

/**
 * Set tgCamera
 */
void TomGineThread::SetTGCamera(unsigned width, unsigned height, double zFar, double zNear, 
                                Pose &pose, cv::Mat &intrinsic, TomGine::tgCamera &tgCam)
{
  if (pose.empty() || intrinsic.empty())
    return;

  TomGine::tgCamera::Parameter param;

  param.width = width;
  param.height = height;
  param.zFar = zFar;
  param.zNear = zNear;

  // Instrinsic parameters:
  // entries of the camera matrix
  cv::Mat cam = intrinsic;
  if (cam.type() != CV_64F) intrinsic.convertTo(cam, CV_64F);
  double *d = cam.ptr<double>(0);
  param.fx = d[0];
  param.fy = d[4];
  param.cx = d[2];
  param.cy = d[5];

  // radial distortion parameters
  param.k1 = 0.;
  param.k2 = 0.;
  param.k3 = 0.;
  // tangential distortion parameters
  param.p1 = 0.;
  param.p2 = 0.;

  // extrinsic parameters: 3D pose of camera w.r.t. world
  Pose invPose;
  InvPose(pose,invPose);

  cv::Mat R = invPose.R;
  if (R.type() != CV_32F) invPose.R.convertTo(R, CV_32F);
  param.rot = mat3(R.ptr<float>(0));
  d = invPose.t.ptr<double>(0);
  param.pos = vec3(d[0],d[1],d[2]);

  tgCam.Load(param);
}

/**
 * DrawPointCloud
 */
void TomGineThread::DrawPointCloud()
{
  glDisable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);                                    // TODO Gehört eigentlich wo anders hin!!!
  bool haveCol=false;
  bool haveColCloud = true;
  uchar *d;

  if (mCloud.empty())
    return;
  if (cCloud.empty())
    haveColCloud = false;
  
//   if (c
  if (mCloud.rows==img.rows && mCloud.cols==img.cols)
  {
    haveCol=true;
    d = img.ptr<uchar>(0);
  }

  if (!haveCol)
    glColor4ub(255,0,0,255);

  glBegin(GL_POINTS);
  for (int v = 0; v < (int)mCloud.rows; ++v)
  {
    for (int u = 0; u < (int)mCloud.cols; ++u)
    {
      cv::Point3f &pt = mCloud(v,u);
      cv::Point3f &col = cCloud(v,u);
     if (pt.x == pt.x && pt.y==pt.y && pt.z==pt.z)
      {
        if (haveCol)
        {
//           glColor4ub(d[2],d[1],d[0],255);
          glVertex3f(pt.x, pt.y, pt.z);
        }
        else if (haveColCloud)
        {
          glColor4ub((int) col.x, (int) col.y, (int) col.z, 255);
                glVertex3f(pt.x, pt.y, pt.z);
        }
        else
          glVertex3f(pt.x, pt.y, pt.z);
      }
      if (haveCol) d+=3;
    }
  }
  glEnd( );
}

/**
 * @brief DrawVPointCloud
 */
void TomGineThread::DrawVPointCloud()
{
  glDisable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);                                    // TODO Gehört eigentlich wo anders hin!!!
  uchar *d;

  RGBValue color;
  
  if(!vCloud.empty())
  {
    glBegin(GL_POINTS);
    for (int v = 0; v < (int)vCloud.rows; ++v)
    {
      for (int u = 0; u < (int)vCloud.cols; ++u)
      {
        cv::Vec4f &pt = vCloud(v,u);
        if (pt[0] == pt[0] && pt[1]==pt[1] && pt[2]==pt[2])
        {
          color.float_value = pt[3];
          glColor4ub((int) color.b, (int) color.g, (int) color.r, 255);
          glVertex3f(pt[0], pt[1], pt[2]);
        }
      }
    glEnd( );
    }  
  }
  else return;
  
  if(!vClouds.empty())
  {
    glBegin(GL_POINTS);
    for (int v = 0; v < (int)vClouds.rows; ++v)
    {
      for (int u = 0; u < (int)vClouds.cols; ++u)
      {
        cv::Vec4f &pt = vClouds(v,u);
        if (pt[0] == pt[0] && pt[1]==pt[1] && pt[2]==pt[2])
        {
          color.float_value = pt[3];
          glColor4ub((int) color.b, (int) color.g, (int) color.r, 255);
          glVertex3f(pt[0], pt[1], pt[2]);
        }
      }
    }
    glEnd( );
  } 
  else return;
}


/**
 * @brief  DrawPoints3D
 */
void TomGineThread::DrawPoints3D()
{
  glDisable(GL_LIGHTING);
  glEnable(GL_POINT_SMOOTH);

  glBegin(GL_POINTS);
  for (unsigned i=0; i<points3D.size(); i++)
  {
//   printf("TomGineThread::DrawPoints3D: point: %4.3f / %4.3f / %4.3f\n", points3D[i].x, points3D[i].y, points3D[i].z);
    glPointSize(sizePoints3D[i]);
    glColor3f(colPoints3D[i].x,colPoints3D[i].y,colPoints3D[i].z);
    glVertex3f(points3D[i].x, points3D[i].y, points3D[i].z);
  }
  glEnd( );
}

/**
 * @brief DrawLines3D
 */
void TomGineThread::DrawLines3D()
{
  glDisable(GL_LIGHTING);
//   glEnable(GL_POINT_SMOOTH);

  for (unsigned i=0; i<lines3D.size(); i++)
  {
    if(useProbLevel)
    {
      if(lineProbs3D[i] > 0.95)
      {
        glLineWidth(lineProbs3D[i]*5.0f);
        glBegin(GL_LINES);
        glColor3f(lineCols3D[i][2],lineCols3D[i][1],lineCols3D[i][0]);
        glVertex3f(lines3D[i].first[0],lines3D[i].first[1],lines3D[i].first[2]);
        glVertex3f(lines3D[i].second[0],lines3D[i].second[1],lines3D[i].second[2]);
        glEnd( );
      }
    }
    else
    {
      glLineWidth(lineProbs3D[i]*5.0f);
      glBegin(GL_LINES);
      glColor3f(lineCols3D[i][2],lineCols3D[i][1],lineCols3D[i][0]);
      glVertex3f(lines3D[i].first[0],lines3D[i].first[1],lines3D[i].first[2]);
      glVertex3f(lines3D[i].second[0],lines3D[i].second[1],lines3D[i].second[2]);
      glEnd( );
    }
  }    
}


/**
 * DrawLabels3D
 */
void TomGineThread::DrawLabels3D(TomGine::tgEngine &render)
{
  printf("TomGineThread::DrawLabels3D: This function wont work!\n");
//   for(unsigned i=0; i<vLabel.size(); i++)
//   {
//     if(vLabel[i].render)
//     {
//       printf("Render vLabel!\n");
//       vLabel[i].lLabel.SetFont("./comic.ttf");
//       vLabel[i].lLabel.AddText(vLabel[i].lL.c_str());
//       TomGine::tgPose p;
//       p.t.x = vLabel[i].lP.x;
//       p.t.y = vLabel[i].lP.y;
//       p.t.z = vLabel[i].lP.z;
//       vLabel[i].lLabel.SetPose(p);
//      
// 
// //       vLabel[i].lLabelS.SetFont("./comic.ttf");
// //       vLabel[i].lLabelS.AddText(vLabel[i].lLS.c_str());
// //       p.t.x = vLabel[i].lPS.x;
// //       p.t.y = vLabel[i].lPS.y;
// //       p.t.z = vLabel[i].lPS.z;
// //       vLabel[i].lLabelS.SetPose(p);
// // 
// //       vLabel[i].lLabelE.SetFont("./comic.ttf");
// //       vLabel[i].lLabelE.AddText(vLabel[i].lLE.c_str());
// //       p.t.x = vLabel[i].lPE.x;
// //       p.t.y = vLabel[i].lPE.y;
// //       p.t.z = vLabel[i].lPE.z;
// //       vLabel[i].lLabelE.SetPose(p);
//       
//       vLabel[i].render = false;
//     }
//     vLabel[i].lLabel.Draw();
// //     vLabel[i].lLabelS.Draw();
// //     vLabel[i].lLabelE.Draw();
//   }

//   for(unsigned i=0; i<vLabel.size(); i++)
//   {
//     TomGine::tgPose p;
//     p.t.x = 0.0;
//     p.t.y = 0.0;
//     p.t.z = 0.0;
//     render.PrintText2D("Andison", vec2(10, 10));
//   }
}
  
  
/**
 * @brief Draw3D
 */
void TomGineThread::Draw3D(TomGine::tgEngine &render)
{
  glClearColor(0.0, 0.0, 0.0, 1.);
  
  // set camera pose...
  if (camPoseChanged && !intrinsic.empty())
  {
    SetTGCamera(width,height, 10.0, 0.01, camPose, intrinsic, tgCam);
    render.SetCamera(tgCam);
    render.UpdateCameraViews(tgCam);
    render.SetCenterOfRotation(rotCenter[0], rotCenter[1], rotCenter[2]);
    camPoseChanged=false;
  }

  // set background imgae
  if (drawImage && !img.empty()) 
    render.LoadBackgroundImage((unsigned char*)img.ptr<uchar>(0), img.cols, img.rows, GL_BGR, true);
  else
    render.UnloadBackgroundImage();

  // draw ...
  if (draw3D)         // draw general data
  {
    DrawPoints3D();
    DrawLines3D();
  }
  if (drawPointCloud) // draw point cloud
  {
    DrawPointCloud();
  }
  if (drawVPointCloud)
  {
    DrawVPointCloud();
  }
  if (drawLabels)
  {
    DrawLabels3D(render);
  }
}





/***************************** PUBLIC *****************************/

/**
 * @brief Set intrinsic camera parameter
 */
void TomGineThread::SetParameter(cv::Mat &_intrinsic)
{
  pthread_mutex_lock(&dataMutex);    
  _intrinsic.copyTo(intrinsic);
  pthread_mutex_unlock(&dataMutex);
}

/**
 * @brief Set camera (viewing direction)
 */
void TomGineThread::SetCamera(cv::Mat &R, cv::Mat &t, cv::Vec3d &_rotCenter)
{
  pthread_mutex_lock(&dataMutex);
  R.copyTo(camPose.R);
  t.copyTo(camPose.t);
  rotCenter = _rotCenter; 
  camPoseChanged = true;
  pthread_mutex_unlock(&dataMutex);
}

/**
 * @brief Set rotation center
 */
void TomGineThread::SetRotationCenter(cv::Vec3d &_rotCenter)
{
  pthread_mutex_lock(&dataMutex);
  rotCenter = _rotCenter; 
  camPoseChanged = true;
  pthread_mutex_unlock(&dataMutex);
}

/**
 * @brief SetShapeModel (TomGine-model)
 */
void TomGineThread::SetShapeModel(TomGine::tgModel &tgmodel)
{
  pthread_mutex_lock(&dataMutex);
  object = tgmodel;
  pthread_mutex_unlock(&dataMutex);
}


/**
 * @brief Set image
 */
void TomGineThread::SetImage(cv::Mat &_img)
{
  if (_img.channels() != 3)
    return;

  pthread_mutex_lock(&dataMutex);
  _img.copyTo(img);
  pthread_mutex_unlock(&dataMutex);
}

/**
 * @brief Set coordinate frame
 */
void TomGineThread::SetCoordinateFrame(double size)
{
  showCoordinateFrame = true;
  coordinateFrameSize = size;
}


/**
 * @brief AddPoint3D
 */
void TomGineThread::AddPoint3D(double x, double y, double z, uchar r, uchar g, uchar b, double size)
{
  pthread_mutex_lock(&dataMutex);
  points3D.push_back(cv::Point3f((float)x,(float)y,(float)z));
  colPoints3D.push_back(cv::Point3f((float)r/255.,(float)g/255.,(float)b/255.));
  sizePoints3D.push_back((float)size);
  pthread_mutex_unlock(&dataMutex);
}

/**
 * @brief AddLine3D
 */
void TomGineThread::AddLine3D(double x1, double y1, double z1, double x2, double y2, double z2, uchar r, uchar g, uchar b, 
			      float probability, string lLabel, string lLabelS, string lLabelE)
{
  pthread_mutex_lock(&dataMutex);
  lines3D.push_back(make_pair<cv::Vec3f,cv::Vec3f>(cv::Vec3f((float)x1,(float)y1,(float)z1), 
                                                   cv::Vec3f((float)x2,(float)y2,(float)z2)));
  lineCols3D.push_back(cv::Vec3f((float)r/255.,(float)g/255.,(float)b/255.));
  lineProbs3D.push_back(probability);
 
  pthread_mutex_unlock(&dataMutex);
}

/**
 * @brief AddLine3D
 */
void TomGineThread::AddLine3D(cv::Point3d p0, cv::Point3d p1, float probability, string lLabel, string lLabelS, string lLabelE, uchar r, uchar g, uchar b)
{
  pthread_mutex_lock(&dataMutex);
  lines3D.push_back(make_pair<cv::Vec3f,cv::Vec3f>(cv::Vec3f((float)p0.x,(float)p0.y,(float)p0.z), 
                                                   cv::Vec3f((float)p1.x,(float)p1.y,(float)p1.z)));
  lineCols3D.push_back(cv::Vec3f((float)r/255.,(float)g/255.,(float)b/255.));
  lineProbs3D.push_back(probability);
  
  Label l;
  l.render = true;
  l.lL = lLabel;
  l.lLS = lLabelS;
  l.lLE = lLabelE;
  l.lP = p0+p1;
  l.lP.x = l.lP.x/2.;
  l.lP.y = l.lP.y/2.;
  l.lP.z = l.lP.z/2.;
  l.lPS = p0;
  l.lPE = p1;
  vLabel.push_back(l);
 
  pthread_mutex_unlock(&dataMutex);
}

/**
 * @brief Add complete graph, consisting of links between two points with a probability value.
 * @param first Vector with first points of links
 * @param second Vector with second points of links
 * @param probability Vector with probability of link
 * @param r Red (0-255)
 * @param g Green (0-255)
 * @param b Blue (0-255)
 */
void TomGineThread::AddGraphModel(std::vector<cv::Point3d> first, std::vector<cv::Point3d> second, std::vector<double> probability, 
				   std::vector<std::string> link, std::vector<std::string> node_0, std::vector<std::string> node_1,
				   uchar r, uchar g, uchar b)
{
  if(first.size() != second.size() || first.size() != probability.size() || first.size() != link.size() ||
     first.size() != node_0.size() || first.size() != node_1.size())
  {
    printf("TomGineThread::DrawGraphModel: Error: Invalid field size!\n");
    return;
  }
  
  for(unsigned i=0; i< first.size(); i++)
    AddLine3D(first[i], second[i], (float) probability[i], link[i], node_0[i], node_1[i], r, g, b);
}

/**
 * @brief SetPointCloud
 */
void TomGineThread::SetPointCloud(cv::Mat_<cv::Point3f> &matCloud, cv::Mat_<cv::Point3f> &colCloud)
{
  printf("TomGineThread::SetPointCloud: Antiquated!\n");
  pthread_mutex_lock(&dataMutex);
  matCloud.copyTo(mCloud);
  colCloud.copyTo(cCloud);
  pthread_mutex_unlock(&dataMutex);
}

/**
 * @brief AddPointCloud
 * @param vecCloud Cloud of points in openCV vector format.
 */
void TomGineThread::AddPointCloud(cv::Mat_<cv::Vec4f> &matCloud)
{
  pthread_mutex_lock(&dataMutex);

  unsigned vCloudSize = 0;
  if(!vCloud.empty()) 
    vCloudSize = vCloud.cols;
  unsigned cloudSize = matCloud.cols*matCloud.rows + vCloudSize;
  
  cv::Mat_<cv::Vec4f> newCloud;
  newCloud = cv::Mat_<cv::Vec4f>(1, cloudSize);
  unsigned z=0;
  for(unsigned idx=0; idx < matCloud.cols*matCloud.rows; idx++, z++)
    newCloud(0, idx) = (matCloud(0, idx)); 
  
  if(!vCloud.empty())
    for(unsigned idx=0; idx < vCloud.cols; idx++)
      newCloud(0, z+idx) = (vCloud(0, idx)); 
  
  newCloud.copyTo(vCloud);
  pthread_mutex_unlock(&dataMutex);
}

/**
 * @brief AddPointCloud
 * @param vecCloud Cloud of points in openCV vector format.
 */
void TomGineThread::AddPointCloud(std::vector<cv::Vec4f> &vecCloud)
{
  pthread_mutex_lock(&dataMutex);

  unsigned vCloudSize = 0;
  if(!vCloud.empty()) 
    vCloudSize = vCloud.cols;
  unsigned cloudSize = vecCloud.size() + vCloudSize;
  
  cv::Mat_<cv::Vec4f> newCloud;
  newCloud = cv::Mat_<cv::Vec4f>(1, cloudSize);
  unsigned z=0;
  for(unsigned idx=0; idx < vecCloud.size(); idx++, z++)
    newCloud(0, idx) = (vecCloud[idx]); 
  
  if(!vCloud.empty())
    for(unsigned idx=0; idx < vCloud.cols; idx++)
      newCloud(0, z+idx) = (vCloud(0, idx)); 
  
  newCloud.copyTo(vCloud);
  pthread_mutex_unlock(&dataMutex);
}

/**
 * @brief Set several point clouds in cv::Vec4f format.
 * @param vecClouds Point clouds with Vec4f format
 */
void TomGineThread::AddPointClouds(vector< cv::Mat_<cv::Vec4f> > &vecClouds)
{
  unsigned cloudSize = 0;
  for(unsigned i=0; i < vecClouds.size(); i++)
    cloudSize += vecClouds[i].cols*vecClouds[i].rows;

  pthread_mutex_lock(&dataMutex);
  cv::Mat_<cv::Vec4f> newCloud;
  newCloud = cv::Mat_<cv::Vec4f>(1, cloudSize);
  
  for(unsigned i=0, z=0; i < vecClouds.size(); i++)
    for(unsigned idx=0; idx < vecClouds[i].cols*vecClouds[i].rows; idx++, z++)
      newCloud(0, z) = (vecClouds[i](0, idx));
    
  newCloud.copyTo(vClouds);
  pthread_mutex_unlock(&dataMutex);
}

/**
 * @brief Add a convex hulls. (Matrix with one row of points!)
 * @param vecHull Convex hull with Vec4f point format (x, y, z, rgb)
 */
void TomGineThread::AddConvexHull(cv::Mat_<cv::Vec4f> &vecHull)
{
  RGBValue color;
  if(vecHull.rows != 1)
  {
    printf("TomGineThread::AddConvexHull: Warning: Convex hull represented as matrix with more than one row!\n");
    return;
  }
  for(unsigned j=0; j < vecHull.cols-1; j++)
  {
    // How to access each point?
    cv::Vec4f s = vecHull(0, j);
    cv::Vec4f e = vecHull(0, j+1);
    color.float_value = s[3];
    AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], color.b, color.g, color.r);
  }
  // add line from first to last matrix element
  cv::Vec4f s = vecHull(0, 0);
  cv::Vec4f e = vecHull(0, vecHull.cols-1);
  color.float_value = s[3];
  AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], color.b, color.g, color.r);
}

/**
 * @brief Add a convex hulls. (Matrix with one row of points!)
 * @param vecHull Convex hull with Vec4f point format (x, y, z, rgb)
 */
void TomGineThread::AddConvexHull(std::vector<cv::Vec4f> &vecHull)
{  
  if(vecHull.size() < 3)
  {
    printf("TomGineThread::AddConvexHull: Warning: Convex hull with less than three points.!\n");
    return;
  }
  
  RGBValue color;
  for(unsigned j=0; j < vecHull.size()-1; j++)
  {
    // How to access each point?
    cv::Vec4f s = vecHull[j];
    cv::Vec4f e = vecHull[j+1];
    color.float_value = s[3];
    AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], color.r, color.g, color.b);
  }
  // add line from first to last matrix element
  cv::Vec4f s = vecHull[0];
  cv::Vec4f e = vecHull[vecHull.size()-1];
  color.float_value = s[3];
  AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], color.r, color.g, color.b);
}


/**
 * @brief Add several convex hulls. (Matrix with one row of points!)
 * @param vecHulls Convex hulls with Vec4f point format (x, y, z, rgb)
 */
void TomGineThread::AddConvexHulls(vector< cv::Mat_<cv::Vec4f> > &vecHulls)
{
  RGBValue color;
  for(unsigned i=0; i < vecHulls.size(); i++)
  {
    if(vecHulls[i].rows != 1)
    {
      printf("TomGineThread::AddConvexHulls: Warning: Convex hull represented as matrix with more than one row!\n");
      return;
    }
    for(unsigned j=0; j < vecHulls[i].cols-1; j++)
    {
      // How to access each point?
      cv::Vec4f s = vecHulls[i](0, j);
      cv::Vec4f e = vecHulls[i](0, j+1);
      color.float_value = s[3];
      AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], color.b, color.g, color.r);
    }
    // add line from first to last matrix element
    cv::Vec4f s = vecHulls[i](0, 0);
    cv::Vec4f e = vecHulls[i](0, vecHulls[i].cols-1);
    color.float_value = s[3];
    AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], color.b, color.g, color.r);
  }
}

/**
 * @brief Add a hull prism as a Space of Interest (SOI)..
 * @param vecHull Hull points (first half bottom, then top) in Vec4f point format (x, y, z, rgb)
 */
void TomGineThread::AddHullPrism(std::vector<cv::Vec4f> &vecHull)                                           /// TODO Color is now set to white!!!
{  
  if(vecHull.size() < 3)
  {
    printf("TomGineThread::AddHullPrism: Warning: Hull prism with less than three points.!\n");
    return;
  }
  
  RGBValue color;   
//   color.float_value = vecHull[0][3];
  color.b = 255;
  color.g = 255;
  color.r = 255;
      
  int top = vecHull.size();
  int bottom = top/2; 
  for(unsigned j=0; j < bottom; j++)
  {
    int i = j+1; 
    if(i >= bottom) i=0;
    cv::Vec4f s = vecHull[j];
    cv::Vec4f e = vecHull[i];
//     color.float_value = s[3];
    AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], color.b, color.g, color.r, 0.2);
  }
  for(unsigned j=bottom; j < top; j++)
  {
    int i = j+1; 
    if(i >= top) i=bottom;
    cv::Vec4f s = vecHull[j];
    cv::Vec4f e = vecHull[i];
//     color.float_value = s[3];
    AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], color.b, color.g, color.r, 0.2);
  }
  for(unsigned j=0; j < bottom; j++)
  {
    int i = j + bottom; 
    cv::Vec4f s = vecHull[j];
    cv::Vec4f e = vecHull[i];
//     color.float_value = s[3];
    AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], color.b, color.g, color.r, 0.2);
  }
}


/**
 * @brief Clear the TomGine Thread
 */
void TomGineThread::Clear()
{
  pthread_mutex_lock(&dataMutex);
  points3D.clear();
  colPoints3D.clear();
  sizePoints3D.clear();
  lines3D.clear();
  lineCols3D.clear();
  lineProbs3D.clear();
  polygons3D.clear();
  colPolygons3D.clear();
  filledPolygon3D.clear();
  vLabel.clear();
  mCloud.release();
  cCloud.release();
  vCloud.release();
  vClouds.release();
  pthread_mutex_unlock(&dataMutex);
  
  // set coordinate frame
  if(showCoordinateFrame)
  {
    AddLine3D(0, 0, 0, 0.1, 0, 0, 255, 0, 0, coordinateFrameSize);
    AddLine3D(0, 0, 0, 0, 0.1, 0, 0, 255, 0, coordinateFrameSize);
    AddLine3D(0, 0, 0, 0, 0, 0.1, 0, 0, 255, coordinateFrameSize);
  }
}



}

