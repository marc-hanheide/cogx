/**
 * @file TomGineThread.cc
 * @author Richtsfeld, Prankl
 * @date March 2011
 * @version 0.1
 * @brief Wraper for the tomGine, implementing a threaded 3d render window.
 */


#include "TomGineThread.hh"

namespace P 
{


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
    drawImage(false), draw3D(true), drawPointCloud(true), drawLabels(false)
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
void TomGineThread::DrawPointCloud(cv::Mat_<cv::Vec3f> &pc)
{
  glDisable(GL_LIGHTING);
  bool haveCol=false;
  uchar *d;

  if (pc.empty())
    return;
  if (pc.rows==img.rows && pc.cols==img.cols)
  {
    haveCol=true;
    d = img.ptr<uchar>(0);
  }

  if (!haveCol)
    glColor4ub(0,0,0,255);

  glBegin(GL_POINTS);
  for (int v = 0; v < (int)pc.rows; ++v)
  {
    for (int u = 0; u < (int)pc.cols; ++u)
    {
      cv::Vec3f &pt = pc(v,u);
      if (pt[0]==pt[0] && pt[1]==pt[1] && pt[2]==pt[2])
      {
        if (haveCol)
        {
          glColor4ub(d[2],d[1],d[0],255);
          glVertex3f(pt[0], pt[1], pt[2]);
        }
        else
          glVertex3f(pt[0], pt[1], pt[2]);
      }
      if (haveCol) d+=3;
    }
  }
  glEnd( );
}

/**
 * DrawPoints3D
 */
void TomGineThread::DrawPoints3D()
{
  glDisable(GL_LIGHTING);
  glEnable(GL_POINT_SMOOTH);

  glBegin(GL_POINTS);
  for (unsigned i=0; i<points3D.size(); i++)
  {
    glPointSize(sizePoints3D[i]);
    glColor3f(colPoints3D[i][2],colPoints3D[i][1],colPoints3D[i][0]);
    glVertex3f(points3D[i][0],points3D[i][1],points3D[i][2]);
  }
  glEnd( );
}

/**
 * DrawLines3D
 */
void TomGineThread::DrawLines3D()
{
  glDisable(GL_LIGHTING);
//   glEnable(GL_POINT_SMOOTH);

  for (unsigned i=0; i<lines3D.size(); i++)
  {
    if(drawLabels)
    {
      if(lineProbs3D[i] > 0.9)
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
//   for(unsigned i=0; i<vLabel.size(); i++)
//   {
//     if(vLabel[i].render)
//     {
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
//     render.PrintText3D("Logitech", vec3(1., 1., 1.));
//   }
}
  
  
/**
 * Draw3D
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
    DrawPointCloud(mCloud);
  }
  if (drawLabels)
  {
    DrawLabels3D(render);
  }
}





/***************************** PUBLIC *****************************/

/**
 * Set intrinsic camera parameter
 */
void TomGineThread::SetParameter(cv::Mat &_intrinsic)
{
  pthread_mutex_lock(&dataMutex);    
  _intrinsic.copyTo(intrinsic);
  pthread_mutex_unlock(&dataMutex);
}

/**
 * Set camera (viewing direction)
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
 * Set rotation center
 */
void TomGineThread::SetRotationCenter(cv::Vec3d &_rotCenter)
{
  pthread_mutex_lock(&dataMutex);
  rotCenter = _rotCenter; 
  camPoseChanged = true;
  pthread_mutex_unlock(&dataMutex);
}

/**
 * SetShapeModel (TomGine-model)
 */
void TomGineThread::SetShapeModel(TomGine::tgModel &tgmodel)
{
  pthread_mutex_lock(&dataMutex);
  object = tgmodel;
  pthread_mutex_unlock(&dataMutex);
}

/**
 * SetPointCloud
 */
void TomGineThread::SetPointCloud(cv::Mat_<cv::Vec3f> &matCloud)
{
  pthread_mutex_lock(&dataMutex);
  matCloud.copyTo(mCloud);
  pthread_mutex_unlock(&dataMutex);
}

/**
 * Set image
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
 * AddPoint3D
 */
void TomGineThread::AddPoint3D(double x, double y, double z, uchar r, uchar g, uchar b, double size)
{
  pthread_mutex_lock(&dataMutex);
  points3D.push_back(cv::Vec3f((float)x,(float)y,(float)z));
  colPoints3D.push_back(cv::Vec3f((float)r/255.,(float)g/255.,(float)b/255.));
  sizePoints3D.push_back((float)size);
  pthread_mutex_unlock(&dataMutex);
}

/**
 * AddLine3D
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
 * AddLine3D
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
void TomGineThread::DrawGraphModel(std::vector<cv::Point3d> first, std::vector<cv::Point3d> second, std::vector<double> probability, 
				   std::vector<std::string> link, std::vector<std::string> node_0, std::vector<std::string> node_1,
				   uchar r, uchar g, uchar b)
{
  if(first.size() != second.size() || first.size() != probability.size() || first.size() != link.size() ||
     first.size() != node_0.size() || first.size() != node_1.size())
  {
    printf("TomGineThread::DrawGraphModel: Error: Invalid field size!\n");
    return;
  }
  
//   for(unsigned i=0; i< first.size(); i++)
//     printf("TomGineThread: %s - %s - %s\n", link[i].c_str(), node_0[i].c_str(), node_1[i].c_str());
  
  for(unsigned i=0; i< first.size(); i++)
    AddLine3D(first[i], second[i], (float) probability[i], link[i], node_0[i], node_1[i], r, g, b);
//     AddLine3D(first[i], second[i], (float) probability[i], "a", "b", "c", r, g, b);
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
  pthread_mutex_unlock(&dataMutex);
}



}

