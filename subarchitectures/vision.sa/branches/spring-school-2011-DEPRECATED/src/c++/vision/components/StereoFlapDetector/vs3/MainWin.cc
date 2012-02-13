/**
 * $Id: MainWin.cc,v 1.39 2007/07/27 17:01:25 mxz Exp mxz $
 *
 * TODO: after loading new image, gestalt_id needs new bounds
 * TODO: make "detail" _and_ "show id" checkbox
 * TODO: avoid multiple updateGL (SelectGestalt etc.)
 */

#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <QMenuBar>
#include <QMenu>
#include <QFileDialog>
#include <QStatusBar>
#include <QValidator>
#include <QLabel>
#include <QMouseEvent>
#include <QEvent>
#include <Q3VBox>
#include <Q3HBox>
#include "Math.hh"
#include "Draw.hh"
#include "OpenCvImgSeqVideo.hh"
#include "OpenCvLiveVideo.hh"
#include "VisionCore.hh"
#include "FormJunctions.hh"
#include "FormArcJunctions.hh"
#include "FormConvexArcGroups.hh"
#include "FormSegments.hh"
#include "Ellipse.hh"
#include "Line.hh"
#include "Closure.hh"
#include "MainWin.hh"
#include "HCF.hh"

namespace Z
{

const int DRAW_IMAGE = 0;
const int DRAW_LIGHT_IMAGE = 1;
const int DRAW_DARK_IMAGE = 2;
const int DRAW_WHITE = 3;
const int DRAW_BLACK = 4;
const int DRAW_3D = 5;

// default runtime limit for one pass of image processing
const int RUNTIME_DEFAULT = 100;

// Set where to draw to: main or info draw area.
extern void SetActiveDrawArea(QGLWidget *da);

// global main window pointer
MainWin *MainWin::main_win = 0;

static Video *video = 0;
// the core vision system object
static VisionCore *vcore = 0;
class Opts
{
public:
  VideoType vid_type;
  list<string> images;
  string config_name;
  Opts() : vid_type(VIDEO_TYPE_IMG_SEQ) {}
};
static Opts opts;

void DisplayUsage(const char *progname)
{
  printf("%s [-h] [-c configfile] [-b] [-f] image\n"
      "  -h  display this usage\n"
      "  -c  specify config file\n"
      "  -b  draw all overlays in black (generate high constrast snapshots)\n"
      "  -f  draw all overlays fat (generate high contrast snapshots)\n"
      "  image  image in any format\n", progname);
}

//---------- MainWin -----------------------------------------------------------

/**
 * Constructor.
 * Sets up menus and widgets.
 */
MainWin::MainWin(int argc, char **argv)
{
  // set global pointer
  main_win = this;
  ParseOptions(argc, argv);
  switch(opts.vid_type)
  {
    case VIDEO_TYPE_IMG_SEQ:
      video = new OpenCvImgSeqVideo();
      break;
    case VIDEO_TYPE_LIVE:
      video = new OpenCvLiveVideo(320, 240);
      break;
    default:
      throw Except(__HERE__, "no valid video type specified");
      break;
  }
  vcore = new VisionCore(opts.config_name);
  BuildMenu();
  BuildContent();
  zoom = 1.;
  if(!opts.images.empty())
  {
    for(list<string>::iterator i = opts.images.begin(); i != opts.images.end();
        i++)
      video->AddFrame(*i);
    HaveNewImage();
  }
  else if(video->IsLive())
    HaveLiveVideo();
}

/**
 * Everything has an end...
 */
MainWin::~MainWin()
{
  delete vcore;
  delete video;
}

/**
 * Parse command line options, namely given images.
 */
void MainWin::ParseOptions(int argc, char **argv)
{
  int c;
  while(1)
  {
    c = getopt(argc, argv, "c:lbfh");
    if(c == -1)
      break;
    switch(c)
    {
      case 'h':
        DisplayUsage(argv[0]);
        break;
      case 'c':
        if(optarg)
          opts.config_name = optarg;
        break;
      case 'l':
        opts.vid_type = VIDEO_TYPE_LIVE;
        break;
      case 'b':
        SetBlackWhite(true);
        break;
      case 'f':
        SetFatLines(true);
        break;
      default:
        fprintf(stderr, "getopt returned character code 0%o\n", c);
    }
  }
  // remaining argument is image filename to load
  while(optind < argc)
    opts.images.push_back(argv[optind++]);
}

/**
 * Construct menu and set up callbacks.
 */
void MainWin::BuildMenu()
{
  QMenu *fileMenu = menuBar()->addMenu("&File");

  QAction *quitAct = new QAction(tr("&Quit"), this);
  quitAct->setShortcut(tr("Ctrl+Q"));
  connect(quitAct, SIGNAL(triggered()), this, SLOT(close()));
  fileMenu->addAction(quitAct);

  QMenu *imageMenu = menuBar()->addMenu("&Image");

  QAction *loadAct = new QAction(tr("&Load image ..."), this);
  loadAct->setShortcut(tr("Ctrl+L"));
  connect(loadAct, SIGNAL(triggered()), this, SLOT(LoadImage()));
  imageMenu->addAction(loadAct);

  QAction *nextAct = new QAction(tr("&Next"), this);
  nextAct->setShortcut(tr("Ctrl+N"));
  connect(nextAct, SIGNAL(triggered()), this, SLOT(NextImage()));
  imageMenu->addAction(nextAct);

  QAction *procAct = new QAction(tr("&Process"), this);
  procAct->setShortcut(tr("Ctrl+P"));
  connect(procAct, SIGNAL(triggered()), this, SLOT(ProcessImage()));
  imageMenu->addAction(procAct);

  QAction *clearAct = new QAction(tr("Clear results"), this);
  connect(clearAct, SIGNAL(triggered()), this, SLOT(ClearResults()));
  imageMenu->addAction(clearAct);

  QAction *snapAct = new QAction(tr("&Snapshot"), this);
  snapAct->setShortcut(tr("Ctrl+S"));
  connect(snapAct, SIGNAL(triggered()), this, SLOT(SaveImage()));
  imageMenu->addAction(snapAct);

  QAction *runAct = new QAction(tr("Run"), this);
  connect(runAct, SIGNAL(triggered()), this, SLOT(Run()));
  imageMenu->addAction(runAct);

  QAction *rewindAct = new QAction(tr("Rewind"), this);
  connect(rewindAct, SIGNAL(triggered()), this, SLOT(Rewind()));
  imageMenu->addAction(rewindAct);

  QAction *gestAct = new QAction(tr("&Gestalts ..."), this);
  gestAct->setShortcut(tr("Ctrl+G"));
  connect(gestAct, SIGNAL(triggered()), this, SLOT(OpenGestaltWin()));
  imageMenu->addAction(gestAct);

  QAction *princAct = new QAction(tr("Princ&iples ..."), this);
  princAct->setShortcut(tr("Ctrl+I"));
  connect(princAct, SIGNAL(triggered()), this, SLOT(OpenPrincipleWin()));
  imageMenu->addAction(princAct);

  QAction *groundAct = new QAction(tr("Load Ground&truth ..."), this);
  groundAct->setShortcut(tr("Ctrl+T"));
  connect(groundAct, SIGNAL(triggered()), this, SLOT(OpenEvalFile()));
  imageMenu->addAction(groundAct);

  /*QMenu *zoom = new QMenu(this);
  menuBar()->insertItem("&Zoom", zoom);
  zoom->insertItem("Original Size", this, SLOT(NoZoom()), CTRL+Key_0);
  zoom->insertItem("Half", this, SLOT(HalfZoom()), CTRL+Key_Less);
  zoom->insertItem("Double", this, SLOT(DoubleZoom()), CTRL+Key_Greater);*/

  QMenu *helpMenu = menuBar()->addMenu("&Help");

  QAction *aboutAct = new QAction(tr("About"), this);
  aboutAct->setShortcut(tr("F1"));
  connect(aboutAct, SIGNAL(triggered()), this, SLOT(DisplayAbout()));
  helpMenu->addAction(aboutAct);
}

/**
 * Fill the window with stuff.
 */
void MainWin::BuildContent()
{
  gestalt_win = 0;
  draw_area = new MainDrawArea(this, this);
  // if we have started the system already (i.e. loadad an image)
  if(video->Width() > 0 && video->Height() > 0)
    draw_area->setFixedSize(video->Width(), video->Height());
  setCentralWidget(draw_area);
  gestalt_win = new GestaltWin(this);
  princ_win = new PrincipleWin(this);
  eval_win = new EvalWin(this);
  idle_timer = new QTimer(this);
  connect(idle_timer, SIGNAL(timeout()), this, SLOT(ProcessNextImage()));
}

void MainWin::HaveLiveVideo()
{
  setWindowTitle(QString(video->FrameName().c_str()));
  draw_area->setFixedSize(video->Width(), video->Height());
  Run();
}

void MainWin::HaveNewImage()
{
  vcore->NewImage(video->CurrentFramePtr());
  setWindowTitle(QString(video->FrameName().c_str()));
  if(draw_area->width() != video->Width() ||
     draw_area->height() != video->Height())
    draw_area->setFixedSize(video->Width(), video->Height());
  draw_area->updateGL();
}

void MainWin::LoadImage()
{
  char cwd[1024];
  assert(getcwd(cwd, 1024) != NULL);
  QStringList files = QFileDialog::getOpenFileNames(
                          this,
                          "Choose a file",
                          cwd,
                          "Images (*.png *.xpm *.jpg *.jpeg *.ppm *.bmp)");
  if(!files.isEmpty())
  {
    QStringList::Iterator it = files.begin();
    video->ClearFrames();
    while(it != files.end())
    {
      video->AddFrame(string((*it).toAscii()));
      ++it;
    }
    HaveNewImage();
  }
}

void MainWin::NextImage()
{
  video->Forward();
  HaveNewImage();
}

void MainWin::Rewind()
{
  video->MoveToStart();
  HaveNewImage();
}

/**
 * Process the current image.
 * Incremental or (old-style) non-incremental
 * TODO: make incremental processing interruptible
 */
void MainWin::ProcessImage()
{
  if(gestalt_win->incremental->isChecked())
  {
    int time_ms;
    if(gestalt_win->time_input->text().isEmpty())
      time_ms = RUNTIME_DEFAULT;
    else
      time_ms = gestalt_win->time_input->text().toInt();
    vcore->ProcessImage(time_ms);
  }
  else
  {
    // no time limit = non-incremental
    vcore->ProcessImage();
  }
  draw_area->updateGL();
  gestalt_win->UpdateContent();
  princ_win->UpdateContent();
}

void MainWin::ClearResults()
{
  vcore->ClearGestalts();
}

void MainWin::SaveImage()
{
  static int i = 0;
  char filename[256];
  snprintf(filename, 256, "out%03d.ppm", i++);
  SetActiveDrawArea(draw_area);
  SaveDrawArea(string(filename));
}

void MainWin::ProcessNextImage()
{
  double runtime;
  video->Forward();
  HaveNewImage();
  ProcessImage();
  runtime = vcore->RunTime();
  PrintStatus("processing time: %.3f s (%.3f Hz)", runtime,
      (runtime != 0. ? 1./runtime : 0.));
}

void MainWin::Run()
{
  idle_timer->start(0);
}

void MainWin::DisplayAbout()
{
  printf("this is Vision System 2\n");
}

void MainWin::OpenGestaltWin()
{
  gestalt_win->show();
}

void MainWin::OpenPrincipleWin()
{
  princ_win->show();
}

void MainWin::OpenEvalFile()
{
  char cwd[1024];
  assert(getcwd(cwd, 1024) != NULL);
  QString filename = QFileDialog::getOpenFileName(
                          this,
                          "Choose a file",
                          cwd,
                          "Groundtruth files (*.gt)");
  if(!filename.isEmpty())
  {
    eval_win->LoadGT(filename.toAscii());
    eval_win->show();
  }
}

void MainWin::NoZoom()
{
}

void MainWin::HalfZoom()
{
}

void MainWin::DoubleZoom()
{
}

void MainWin::MysteriousFunction(int mouse_x, int mouse_y)
{
}

void MainWin::DrawVoteImg()
{
  const bool use_color = false;
  VoteImage *vi =
  //((FormJunctions*)vcore->principles[GestaltPrinciple::FORM_JUNCTIONS])->vote_img;
  //((FormConvexArcGroups*)vcore->principles[GestaltPrinciple::FORM_CONVEX_ARC_GROUPS])->vote_img;
  ((FormArcJunctions*)vcore->principles[GestaltPrinciple::FORM_ARC_JUNCTIONS])->vote_img;
  if(vi == 0)
  {
    // no vote image yet
    return;
  }
  for(int x = 0; x < vi->width; x++)
    for(int y = 0; y < vi->height; y++)
    {
      VoteImage::Elem *el = vi->Pixel(x, y);
      while(el != 0)
      {
        unsigned vtype = el->id%8;
        switch(vtype)
        {
          case VOTE_TS:
          case VOTE_NLS:
          case VOTE_NRS:
            DrawPoint2D(x, y, (use_color ? RGBColor::magenta : RGBColor::black));
            break;
          case VOTE_TE:
          case VOTE_NLE:
          case VOTE_NRE:
            DrawPoint2D(x, y, (use_color ? RGBColor::cyan : RGBColor::black));
            break;
          default:
            DrawPoint2D(x, y, (use_color ? RGBColor::white : RGBColor::black));
        }
        el = el->next;
      }
    }
}

//---------- MainDrawArea ------------------------------------------------------

MainDrawArea::MainDrawArea(QWidget *parent, MainWin *main_w)
  : QGLWidget(parent), main_win(main_w)
{
  cam_trans[0] = cam_trans[1] = cam_trans[2] = 0.;
  cam_rot[0] = cam_rot[1] = 0.;
  mouse_x = mouse_y = 0;
  mouse_butt = Qt::NoButton;
  depth_step = 50.;
}

void MainDrawArea::mouseMoveEvent(QMouseEvent *ev)
{
  if(main_win->gestalt_win->draw_image->currentIndex() != DRAW_3D)
  {
    if(ev->x() > 0 && ev->x() < video->Width() &&
       ev->y() > 0 && ev->y() < video->Height())
    {
      RGBColor col = vcore->Pixel(ev->x(), ev->y());
      PrintStatus("pos: %3d %3d RGB: %3d %3d %3d", ev->x(), ev->y(),
          col.r, col.g, col.b);
    }
  }
  else
  {
    double trans_scale = 1., rot_scale = 1.;
    int delta_x = ev->x() - mouse_x;
    int delta_y = ev->y() - mouse_y;

    if(mouse_butt & Qt::LeftButton)
    {
      if(mouse_butt & Qt::SHIFT)
      {
        cam_trans[0] += ((GLfloat)delta_x)/trans_scale;
        cam_trans[1] -= ((GLfloat)delta_y)/trans_scale;
      }
      else
      {
        cam_rot[0] += (GLfloat)delta_x/rot_scale;
        cam_rot[1] += (GLfloat)delta_y/rot_scale;
      }
    }
    else if(mouse_butt & Qt::MidButton)
    {
      cam_trans[2] -= ((GLfloat)delta_y)/trans_scale;
    }
    else if(mouse_butt & Qt::RightButton)
    {
      depth_step -= ((GLfloat)delta_y)/trans_scale;
      depth_step = max(depth_step, 0.);
    }
    mouse_x = ev->x();
    mouse_y = ev->y();
    updateGL();
  }
}

/**
 * Pick gestalt under mouse pointer.
 * If more then one gestalts are at (x,y) the first one is returned.
 * If the mouse is clicked over the same position repeatedly, the second,
 * third, etc. is returned.
 */
void MainDrawArea::mousePressEvent(QMouseEvent *ev)
{
  static unsigned prev_picked = UNDEF_ID;
  static int prev_x = INT_MAX, prev_y = INT_MAX;

  mouse_x = ev->x();
  mouse_y = ev->y();
  mouse_butt = ev->buttons();

  if(main_win->gestalt_win->debug->isChecked())
  {
    main_win->MysteriousFunction(ev->x(), ev->y());
  }
  else
  {
    if(!main_win->gestalt_win->gestalt_list->selectedItems().isEmpty())
    {
      Gestalt::Type type =
        (Gestalt::Type)main_win->gestalt_win->gestalt_list->currentRow() - 1;
      bool mask = main_win->gestalt_win->mask->isChecked();
      int x = ev->x(), y = ev->y();
      if(x != prev_x || y != prev_y)
        prev_picked = UNDEF_ID;
      // check if gestalt is at x,y
      unsigned i = vcore->PickGestaltAt(type, x, y, prev_picked, mask);
      if(i != UNDEF_ID)
      {
        main_win->gestalt_win->gestalt_id->setValue(i);
        main_win->gestalt_win->gestalt_rank->setValue(vcore->Gestalts(type, i)->Rank());
        prev_picked = i;
        prev_x = x;
        prev_y = y;
      }
      else // check again in small surrounding area
      {
        bool found = false;
        int a_end = min(video->Width()-1, x+1);
        int b_end = min(video->Height()-1, y+1);
        for(int a = max(0, x-1); a <= a_end && !found; a++)
          for(int b = max(0, y-1); b <= b_end && !found; b++)
          {
            i = vcore->PickGestaltAt(type, a, b, prev_picked, mask);
            if(i != UNDEF_ID)
            {
              main_win->gestalt_win->gestalt_id->setValue(i);
              main_win->gestalt_win->gestalt_rank->setValue(vcore->Gestalts(type, i)->Rank());
              found = true;
              prev_picked = i;
              prev_x = x;
              prev_y = y;
            }
          }
      }
    }
  }
}

void MainDrawArea::initializeGL()
{
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void MainDrawArea::resizeGL(int w, int h)
{
  glViewport(0, 0, w, h);
  // note: projection is set up in paintGL, depending whether 2D or 3D is
  // painted
}

void MainDrawArea::DrawOverlays()
{
  if(main_win->gestalt_win->draw_gt->isChecked())
    main_win->eval_win->DrawGT();

  if(main_win->gestalt_win->draw_edges->isChecked())
    vcore->DrawGestalts(Gestalt::SEGMENT, 0);

  if(main_win->gestalt_win->draw_vote_img->isChecked())
    main_win->DrawVoteImg();

  QListViewItem *selected_type =
    main_win->gestalt_win->gestalt_list->selectedItem();
  if(selected_type != 0)
  {
    unsigned detail = (unsigned)main_win->gestalt_win->detail->value();
    bool mask = main_win->gestalt_win->mask->isChecked();
    double thr;
    if(main_win->gestalt_win->thr_input->text().isEmpty())
      thr = -HUGE;
    else
      thr = main_win->gestalt_win->thr_input->text().toDouble();
    Gestalt::Type type = Gestalt::EnumType(selected_type->text(0).ascii());
    if(main_win->gestalt_win->select_all_of_type->isChecked())
    {
      for(unsigned i = 0; i < vcore->NumGestalts(type); i++)
        if(vcore->Gestalts(type, i)->sig >= thr)
          if(!mask || (mask && vcore->Gestalts(type, i)->IsUnmasked()))
            vcore->DrawGestalt(type, i, detail);
    }
    else
    {
      unsigned rank = (unsigned)main_win->gestalt_win->gestalt_rank->value();
      unsigned i;
      if(rank < vcore->NumGestalts(type))
      {
        if(main_win->gestalt_win->select_up_to->isChecked())
        {
          for(i = 0; i <= rank; i++)
          {
            Gestalt *gst = vcore->RankedGestalts(type, i);
            if(gst->sig >= thr)
              if(!mask || (mask && gst->IsUnmasked()))
                vcore->DrawGestalt(type, gst->ID(), detail);
          }
        }
        else
        {
          Gestalt *gst = vcore->RankedGestalts(type, rank);
          if(gst->sig >= thr)
            if(!mask || (mask && gst->IsUnmasked()))
              vcore->DrawGestalt(type, gst->ID(), detail);
        }
      }
    }
  }
}

/**
 * Draw image (normal, light, dark) or just white or black background and
 * overlay Gestalts
 */
void MainDrawArea::paint2D()
{
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glScaled(1., -1., 1);  // y points down
  gluOrtho2D(0, width(), 0, height());
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelZoom(1.0, -1.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  // Render all primitives at integer positions. Avoids gaps when drawing lines
  // pixel per pixel.
  glTranslatef(0.375, 0.375, 0.0);

  switch(main_win->gestalt_win->draw_image->currentIndex())
  {
    case DRAW_IMAGE:
      SetDrawImagesSolid();
      vcore->DrawImage();
      break;
    case DRAW_LIGHT_IMAGE:
      SetDrawImagesLight();
      vcore->DrawImage();
      break;
    case DRAW_DARK_IMAGE:
      SetDrawImagesDark();
      vcore->DrawImage();
      break;
    case DRAW_WHITE:
      SetClearImagesWhite();
      glClear(GL_COLOR_BUFFER_BIT);
      break;
    case DRAW_BLACK:
      SetClearImagesBlack();
      glClear(GL_COLOR_BUFFER_BIT);
      break;
    default:
      break;
  }
  DrawOverlays();
}

/**
 * Draw surfaces (closures) in pseudo 3D.
 * Use label = depth to layer surfaces.
 */
void MainDrawArea::paint3D()
{
  double dist = max((double)video->Width(), (double)video->Height());

  glDisable(GL_BLEND);
  glClearDepth(1.0);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glClearColor(1.0, 1.0, 1.0, 1.);
  glShadeModel(GL_SMOOTH);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60., (double)width()/(double)height(), 0.001, 10000.);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslated(cam_trans[0], cam_trans[1], cam_trans[2]);
  gluLookAt(0., 0., -dist,  0., 0., 0.,  0., -1., 0.);
  glRotated(-cam_rot[0], 0., 1., 0.);

  // move so that y-axis around which we rotate is in center of image
  glTranslated(
      -(double)video->Width()/2., -(double)video->Height()/2., 0.);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  DrawCoordFrame();
  DrawImageFrame();
  DrawOverlays();
  DrawSurfaces();
  //DrawLines();
}

void MainDrawArea::DrawZAxis(char axis)
{
  static const double l = 50.;

  glBegin(GL_LINES);
  glVertex3d(0., 0., 0.);
  glVertex3d(0., 0., l);
  glEnd();
  glRasterPos3d(0., 0., 1.5*l);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, axis);
}

void MainDrawArea::DrawCoordFrame()
{
  glColor3d(0.5, 0.5, 0.5);
  glPushMatrix();
  DrawZAxis('z');
  glRotated(-90, 1., 0., 0.);
  DrawZAxis('y');
  glRotated(90, 0., 1., 0.);
  DrawZAxis('x');
  glPopMatrix();
}

void MainDrawArea::DrawImageFrame()
{
  const double l = 10.;
  const double w = (double)video->Width();
  const double h = (double)video->Height();

  glColor3d(0.5, 0.5, 0.5);
  glBegin(GL_LINE_LOOP);
  glVertex3d(0., 0., 0.);
  glVertex3d(w, 0., 0.);
  glVertex3d(w, h, 0.);
  glVertex3d(0., h, 0.);
  glEnd();
  // draw "photo edges" at all depth levels
  glColor3d(0.5, 0.5, 0.5);
  for(int i = MinDepth(); i <= MaxDepth(); i++)
  {
    glBegin(GL_LINE_STRIP);
    glVertex3d(0., l , i*depth_step);
    glVertex3d(0., 0., i*depth_step);
    glVertex3d(l , 0., i*depth_step);
    glEnd();
    glBegin(GL_LINE_STRIP);
    glVertex3d(w - l, 0 , i*depth_step);
    glVertex3d(w    , 0., i*depth_step);
    glVertex3d(w    , l , i*depth_step);
    glEnd();
    glBegin(GL_LINE_STRIP);
    glVertex3d(w    , h - l , i*depth_step);
    glVertex3d(w    , h     , i*depth_step);
    glVertex3d(w - l, h     , i*depth_step);
    glEnd();
    glBegin(GL_LINE_STRIP);
    glVertex3d(l, h     , i*depth_step);
    glVertex3d(0, h     , i*depth_step);
    glVertex3d(0, h - l , i*depth_step);
    glEnd();
  }
}

void MainDrawArea::DrawLines()
{
  for(unsigned i = 0; i < NumLines(vcore); i++)
  {
    double depth;
    Line *line = Lines(vcore, i);
    // if label is valid draw in white
    if(line->label != UNDEF_DEPTH)
    {
      glColor3d(1., 1., 1.);
      depth = depth_step*line->label;
    }
    // else if depth is UNDEF (label is UNCOMMITTED) draw in red
    else
    {
      glColor3d(1., 0., 0.);
      depth = 0.;
    }
    glBegin(GL_LINES);
    glVertex3d(line->point[START].x, line->point[START].y, depth);
    glVertex3d(line->point[END].x, line->point[END].y, depth);
    glEnd();
  }
}

void MainDrawArea::DrawSurfaces()
{
  // HACK: problems with getting OpenGL to draw proper depth
  // -> do it myself
  for(int d = MaxDepth(); d >= MinDepth(); d--)
    for(unsigned i = 0; i < NumClosures(vcore); i++)
      if(Closures(vcore, i)->label == d)
        DrawSurface(i);
}

/**
 * TODO: maybe have depth range fixed and depth_step accordingly)
 */
void MainDrawArea::DrawSurface(unsigned i)
{
  double depth;
  Closure *clos = Closures(vcore, i);
  // if label is valid draw in grey
  if(clos->label != UNDEF_DEPTH)
  {
    // grey value depends on depth: min depth = black, max depth = light grey
    double g = 
      0.7*(double)(clos->label - MinDepth())/ (double)(MaxDepth() - MinDepth());
    glColor3d(g, g, g);
    depth = depth_step*clos->label;
  }
  // else if depth is UNDEF (label is UNCOMMITTED) draw in red
  else
  {
    glColor3d(1., 0., 0.);
    depth = 0.;
  }
  glBegin(GL_LINE_LOOP);
  for(unsigned i = 0; i < clos->lines.Size(); i++)
  {
    Vector2 v = clos->GetVertex(i);
    glVertex3d(v.x, v.y, depth);
  }
  glEnd();
}

void MainDrawArea::paintGL()
{
  SetActiveDrawArea(this);

  if(main_win->gestalt_win->draw_image->currentIndex() != DRAW_3D)
    paint2D();
  else
    paint3D();
}

//---------- InfoDrawArea ------------------------------------------------------

InfoDrawArea::InfoDrawArea(QWidget *parent, MainWin *main_w)
  : QGLWidget(parent), main_win(main_w)
{
}

void InfoDrawArea::initializeGL()
{
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void InfoDrawArea::resizeGL(int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0, 1, 0, 1);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelZoom(1.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void InfoDrawArea::paintGL()
{
  SetActiveDrawArea(this);
  glClear(GL_COLOR_BUFFER_BIT);
  QListViewItem *selected_type =
    main_win->gestalt_win->gestalt_list->selectedItem();
  if(selected_type != 0)
  {
    Gestalt::Type type = Gestalt::EnumType(selected_type->text(0).ascii());
    if(!main_win->gestalt_win->select_all_of_type->isChecked())
      vcore->DrawGestaltInfo(type,
          (unsigned)main_win->gestalt_win->gestalt_id->value());
  }
}

//---------- GestaltWin --------------------------------------------------------

GestaltWin::GestaltWin(MainWin *main_w)
  : QDialog(main_w), main_win(main_w)
{
  Q3VBox *vb = new Q3VBox(this);

  incremental = new QCheckBox("Incremental", vb);
  incremental->setChecked(true);

  debug = new QCheckBox("Nasty mysterious debug option", vb);
  debug->setChecked(false);

  gestalt_list = new GestaltList(vb);
  FillGestaltList();
  /**
   * Update the number of gestalts of each type when the mouse enters the list.
   */
  connect(gestalt_list, SIGNAL(selectionChanged(QListViewItem*)),
      this, SLOT(SelectGestaltType(QListViewItem*)));
  connect(gestalt_list, SIGNAL(selectionChanged()),
      main_win->draw_area, SLOT(updateGL()));

  Q3HBox *hb = new Q3HBox(vb);
  Q3HBox *hb2 = new Q3HBox(vb);
  Q3HBox *hb3 = new Q3HBox(vb);
  Q3HBox *hb4 = new Q3HBox(vb);

  draw_image = new QComboBox(hb);
  draw_image->insertItem("Image", DRAW_IMAGE);
  draw_image->insertItem("Light Image", DRAW_LIGHT_IMAGE);
  draw_image->insertItem("Dark Image", DRAW_DARK_IMAGE);
  draw_image->insertItem("White", DRAW_WHITE);
  draw_image->insertItem("Black", DRAW_BLACK);
  draw_image->insertItem("3D", DRAW_3D);
  connect(draw_image, SIGNAL(activated(int)),
      main_win->draw_area, SLOT(updateGL()));

  draw_edges = new QCheckBox("Edges", hb);
  draw_edges->setChecked(true);
  connect(draw_edges, SIGNAL(toggled(bool)),
      main_win->draw_area, SLOT(updateGL()));

  draw_vote_img = new QCheckBox("Votes", hb);
  draw_vote_img->setChecked(false);
  connect(draw_vote_img, SIGNAL(toggled(bool)),
      main_win->draw_area, SLOT(updateGL()));

  draw_gt = new QCheckBox("GT", hb);
  draw_gt->setChecked(false);
  connect(draw_gt, SIGNAL(toggled(bool)),
      main_win->draw_area, SLOT(updateGL()));

  select_all_of_type = new QCheckBox("All", hb2);
  select_all_of_type->setDisabled(true);
  select_all_of_type->setChecked(false);
  connect(select_all_of_type, SIGNAL(toggled(bool)),
      main_win->draw_area, SLOT(updateGL()));
  connect(select_all_of_type, SIGNAL(toggled(bool)),
      this, SLOT(SelectAllOfType(bool)));

  select_up_to = new QCheckBox("Up to", hb2);
  select_up_to->setDisabled(true);
  select_up_to->setChecked(false);
  connect(select_up_to, SIGNAL(toggled(bool)),
      main_win->draw_area, SLOT(updateGL()));
  connect(select_up_to, SIGNAL(toggled(bool)),
      this, SLOT(SelectUpTo(bool)));

  QLabel *rank_label = new QLabel("Rank", hb2);
  rank_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  gestalt_rank = new QSpinBox(hb2);
  gestalt_rank->setDisabled(true);
  connect(gestalt_rank, SIGNAL(valueChanged(int)),
      main_win->draw_area, SLOT(updateGL()));
  connect(gestalt_rank, SIGNAL(valueChanged(int)),
      this, SLOT(SelectGestaltRank(int)));

  QLabel *id_label = new QLabel("ID", hb2);
  id_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  gestalt_id = new QSpinBox(hb2);
  gestalt_id->setDisabled(true);
  connect(gestalt_id, SIGNAL(valueChanged(int)),
      main_win->draw_area, SLOT(updateGL()));
  connect(gestalt_id, SIGNAL(valueChanged(int)),
      this, SLOT(SelectGestaltID(int)));

  QLabel *detail_label = new QLabel("Detail", hb3);
  detail_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  detail = new QSpinBox(hb3);
  connect(detail, SIGNAL(valueChanged(int)),
      main_win->draw_area, SLOT(updateGL()));

  QLabel *thr_label = new QLabel("Thresh.", hb3);
  thr_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  thr_input = new QLineEdit(hb3);
  connect(thr_input, SIGNAL(returnPressed()),
      main_win->draw_area, SLOT(updateGL()));

  mask = new QCheckBox("Mask", hb3);
  mask->setChecked(false);
  connect(mask, SIGNAL(toggled(bool)), main_win->draw_area, SLOT(updateGL()));

  QLabel *time_label = new QLabel("runtime [ms]", hb4);
  time_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  time_input = new QLineEdit(hb4);
  time_input->setValidator(new QIntValidator(0, 1000000, this));
  QString str;
  str.setNum(RUNTIME_DEFAULT);
  time_input->setText(str);

  gestalt_info = new Q3TextEdit(vb);
  gestalt_info->setTextFormat(PlainText);
  gestalt_info->setReadOnly(TRUE);
 
  draw_area = new InfoDrawArea(vb, main_win);
  // TODO: more flexible size management
  draw_area->setMinimumSize(100, 100);

  vb->adjustSize();
  setCaption("Gestalts");
}

void GestaltWin::FillGestaltList()
{
  gestalt_list->setColumnCount(2);
  gestalt_list->setColumnCount(Gestalt::MAX_TYPE + 1);

  QTableWidgetItem *gestaltsHeaderItem = new QTableWidgetItem(tr("Gestalts"));
  QTableWidgetItem *numberHeaderItem = new QTableWidgetItem(tr("Number"));
  numberHeaderItem->setTextAlignment(Qt::AlignRight);
  gestalt_list->setItem(0, 0, gestaltsHeaderItem);
  gestalt_list->setItem(0, 1, numberHeaderItem);

  for(int i = 0; i < Gestalt::MAX_TYPE; i++)
  {
    QString type, number;
    type = Gestalt::TypeName((Gestalt::Type)i);
    number.setNum(vcore->gestalts[i].Size());
    QTableWidgetItem *typeItem = new QTableWidgetItem(tr(type));
    QTableWidgetItem *numberItem = new QTableWidgetItem(tr(number));
    // note: row 0 is the table header
    gestalt_list->setItem(i + 1, 0, typeItem);
    gestalt_list->setItem(i + 1, 1, numberItem);
  }
}

/**
 * Update the number of gestalts of each type after processing etc.
 */
void GestaltList::UpdateGestaltList()
{
  for(int i = 0; i < Gestalt::MAX_TYPE; i++)
  {
    QString number;
    number.setNum(vcore->gestalts[i].Size());
    it.current()->setText(1, number);
    // note: row 0 is the table header
    gestalt_list->item(i + 1, 1)->setText(number);
  }
}

void GestaltWin::SelectGestaltType()
{
  select_all_of_type->setEnabled(true);
  select_up_to->setEnabled(true);
  gestalt_rank->setEnabled(true);
  gestalt_id->setEnabled(true);
  UpdateRanges();
  gestalt_rank->setValue(0);
  if(!select_all_of_type->isChecked())
    SelectGestaltRank((int)gestalt_rank->value());
}

void GestaltWin::SelectGestaltID(int id)
{
  gestalt_info->clear();
  QListViewItem *selected_type =
    main_win->gestalt_win->gestalt_list->selectedItem();
  if(selected_type != 0)
  {
    Gestalt::Type type = Gestalt::EnumType(selected_type->text(0).ascii());
    if(id >= 0)
    {
      gestalt_info->append(vcore->Gestalts(type, id)->GetInfo());
      gestalt_rank->setValue(vcore->Gestalts(type, id)->Rank());
      draw_area->updateGL();
    }
  }
}

void GestaltWin::SelectGestaltRank(int rank)
{
  gestalt_info->clear();
  QListViewItem *selected_type =
    main_win->gestalt_win->gestalt_list->selectedItem();
  if(selected_type != 0)
  {
    Gestalt::Type type = Gestalt::EnumType(selected_type->text(0).ascii());
    if(rank >= 0)
    {
      gestalt_info->append(vcore->RankedGestalts(type, rank)->GetInfo());
      gestalt_id->setValue(vcore->RankedGestalts(type, rank)->ID());
      draw_area->updateGL();
    }
  }
}

void GestaltWin::SelectAllOfType(bool is_checked)
{
  if(is_checked)
  {
    gestalt_id->setDisabled(true);
    gestalt_rank->setDisabled(true);
    select_up_to->setDisabled(true);
    gestalt_info->clear();
    draw_area->updateGL();
  }
  else
  {
    gestalt_id->setDisabled(false);
    gestalt_rank->setDisabled(false);
    select_up_to->setDisabled(false);
    SelectGestaltID((int)gestalt_id->value());
  }
}

void GestaltWin::SelectUpTo(bool is_checked)
{
  if(is_checked)
  {
    gestalt_info->clear();
    draw_area->updateGL();
  }
  else
  {
    SelectGestaltID((int)gestalt_id->value());
  }
}

void GestaltWin::UpdateRanges()
{
  QListViewItem *selected = gestalt_list->selectedItem();
  if(selected != 0)
  {
    Gestalt::Type type = Gestalt::EnumType(selected->text(0).ascii());
    unsigned n = vcore->NumGestalts(type);
    gestalt_id->setMaxValue(n-1);
    gestalt_rank->setMaxValue(n-1);
  }
}

void GestaltWin::UpdateContent()
{
  SelectGestaltID((int)gestalt_id->value());
  UpdateGestaltList();
  UpdateRanges();
  draw_area->updateGL();
}

//---------- PrincipleWin ------------------------------------------------------

PrincipleWin::PrincipleWin(MainWin *main_w)
  : QDialog(main_w), main_win(main_w)
{
  Q3VBox *vb = new Q3VBox(this);
  principle_list = new PrincipleList(vb);
  /**
   * Update entries when the mouse enters the list.
   */
  principle_list->setSizePolicy(QSizePolicy(QSizePolicy::Expanding,
        QSizePolicy::Expanding));
  vb->adjustSize();
  setCaption("Gestalt Principles");
}

void PrincipleList::UpdateContent()
{
  UpdatePrincipleList();
}

void PrincipleWin::FillPrincipleList()
{
  principle_list->setColumnCount(2);
  // principles + header + total
  principle_list->setColumnCount(GestaltPrinciple::MAX_TYPE + 2);

  QTableWidgetItem *principleHeaderItem = new QTableWidgetItem(tr("Principles"));
  QTableWidgetItem *timesHeaderItem = new QTableWidgetItem(tr("Time [s]"));
  timesHeaderItem->setTextAlignment(Qt::AlignRight);
  principles_list->setItem(0, 0, principlesHeaderItem);
  principles_list->setItem(0, 1, timesHeaderItem);

  QTableWidgetItem *totalFooterItem = new QTableWidgetItem(tr("total"));
  QTableWidgetItem *timesFooterItem = new QTableWidgetItem(tr("---"));
  timesFooterItem->setTextAlignment(Qt::AlignRight);
  principles_list->setItem(GestaltPrinciple::MAX_TYPE + 1, 0, totalFooterItem);
  principles_list->setItem(GestaltPrinciple::MAX_TYPE + 1, 1, timesFooterItem);

  for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
  {
    if(vcore->principles[i] != 0)
    {
      QString type = GestaltPrinciple::TypeName((GestaltPrinciple::Type)i);
      QTableWidgetItem *princItem = new QTableWidgetItem(type);
      QTableWidgetItem *timeItem = new QTableWidgetItem("---"); 
      timeItem->setTextAlignment(Qt::AlignRight);
      if(vcore->IsEnabledGestaltPrinciple((GestaltPrinciple::Type)i))
      {
        princItem->setEnabled(true);
        timeItem->setEnabled(true);
      }
      else
      {
        princItem->setEnabled(false);
        timeItem->setEnabled(false);
      }
      // note: row 0 is the table header
      principle_list->setItem(i + 1, 0, princItem);
      principle_list->setItem(i + 1, 1, timeItem);
    }
  }
  UpdatePrincipleList();
}

void PrincipleWin::UpdatePrincipleList()
{
  QString time;
  double total = 0.;
  Q3ListViewItemIterator it(this);
  for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
  {
    if(vcore->principles[i] != 0)
    {
      time.setNum(vcore->principles[i]->RunTime(), 'f', 6);
      it.current()->setText(1, time);
      ++it;
      total += vcore->principles[i]->RunTime();
    }
  }
  time.setNum(total, 'f', 6);
  lastItem()->setText(1, time);
}

//---------- EvalWin -----------------------------------------------------------

EvalWin::EvalWin(MainWin *main_w)
  : QDialog(main_w), main_win(main_w)
{
  Q3VBox *vb = new Q3VBox(this);
  vb->resize(300,300);
  Q3HBox *hb = new Q3HBox(vb);

  QLabel *img_label = new QLabel("Image #", hb);
  img_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  img_id = new QSpinBox(hb);
  img_id->setMinValue(-1);
  img_id->setMaxValue(-1);
  img_id->setValue(-1);
  connect(img_id, SIGNAL(valueChanged(int)), this, SLOT(SelectImgID(int)));

  process_but = new QPushButton("Process", hb);
  connect(process_but, SIGNAL(clicked()), this, SLOT(Process()));
 
  all_but = new QPushButton("All", hb);
  connect(all_but, SIGNAL(clicked()), this, SLOT(ProcessAll()));

  snap_but = new QPushButton("Snapshot", hb);
  connect(snap_but, SIGNAL(clicked()), this, SLOT(SnapshotAllGT()));

  gt_info = new Q3TextEdit(vb);
  gt_info->setTextFormat(PlainText);
  gt_info->setReadOnly(TRUE);
  setCaption("Evaluation");
}

bool EvalWin::ReadImageName(ConfigFile &gt_file, char *img_name)
{
  string str;
  if(gt_file.GetLine(str))
  {
    sscanf(str.c_str(), "%s", img_name);
    return true;
  }
  return false;
}

bool EvalWin::ReadNumEllipses(ConfigFile &gt_file, int &n)
{
  string str;
  if(gt_file.GetLine(str))
  {
    sscanf(str.c_str(), "%d", &n);
    return true;
  }
  return false;
}

bool EvalWin::ReadEllipse(ConfigFile &gt_file, EllPar &e)
{
  string str;
  if(gt_file.GetLine(str))
  {
    sscanf(str.c_str(), "%lf %lf %lf %lf %lf", &e.x, &e.y, &e.a, &e.b, &e.phi);
    return true;
  }
  return false;
}

void EvalWin::LoadGT(const char *file_name)
{
  char img_name[1024];
  int i, n;
  EllPar ell;
  EvalImage img;
  gt_filename = file_name;
  ConfigFile gt_file(gt_filename);
  ClearEvalData();
  while(ReadImageName(gt_file, img_name))
  {
    img.name = img_name;
    img.ells.clear();
    ReadNumEllipses(gt_file, n);
    for(i = 0; i < n; i++)
    {
      ReadEllipse(gt_file, ell);
      img.ells.push_back(ell);
    }
    imgs.push_back(img);
    stat_imgs.push_back(StatImage(img));
  }
  if(imgs.size() > 0)
  {
    img_id->setMinValue(0);
    img_id->setMaxValue(imgs.size() - 1);
  }
  else
  {
    img_id->setMinValue(-1);
    img_id->setMaxValue(-1);
  }
  SelectImgID(img_id->value());
}

void EvalWin::ClearEvalData()
{
  imgs.clear();
  tp_ids.clear();
}

void EvalWin::SelectImgID(int id)
{
  if(id >= 0)
  {
    char buf[1024];
    gt_info->clear();
    for(unsigned i = 0; i < imgs[id].ells.size(); i++)
    {
      snprintf(buf, 1024, "%.2f %.2f %.2f %.2f %.4f", imgs[id].ells[i].x,
          imgs[id].ells[i].y, imgs[id].ells[i].a, imgs[id].ells[i].b,
          imgs[id].ells[i].phi);
      gt_info->append(buf);
    }
    tp_ids.clear();
    video->ClearFrames();
    video->AddFrame(imgs[id].name.c_str());
    main_win->HaveNewImage();
  }
}

void EvalWin::DrawGT()
{
  int id = img_id->value();
  if(id >= 0)
  {
    for(unsigned i = 0; i < imgs[id].ells.size(); i++)
    {
      DrawEllipse2D(imgs[id].ells[i].x,
          imgs[id].ells[i].y, imgs[id].ells[i].a, imgs[id].ells[i].b,
          imgs[id].ells[i].phi, RGBColor::blue);
    }
  }
  for(unsigned i = 0; i < tp_ids.size(); i++)
  {
    Ellipse *e = Ellipses(vcore, tp_ids[i]);
    DrawEllipse2D(e->x, e->y, e->a, e->b, e->phi, RGBColor::red);
  }
}

void EvalWin::SnapshotAllGT()
{
  string out_name;
  string::size_type p;
  for(unsigned i = 0; i < imgs.size(); i++)
  {
    img_id->setValue(i);
    vcore->ProcessImage();
    main_win->draw_area->updateGL();
    p = imgs[i].name.find_last_of('.');
    if(p == string::npos)
    {
      fprintf(stderr, "* %s is not a valid image filename\n",
          imgs[i].name.c_str());
      continue;
    }
    out_name.assign(imgs[i].name, 0, p);
    out_name.append("_gt.ppm");
    SetActiveDrawArea(main_win->draw_area);
    SaveDrawArea(out_name);
  }
}

/**
 * evaluate current image
 */
void EvalWin::Process()
{
  /* uncomment this to collect statistics
  int id = img_id->value();
  OpenStatFile();
  stat_imgs[id].CollectStatistics(stat_file);
  CloseStatFile();
  return;
  */
  vcore->ProcessImage();
  tp_ids.clear();
  for(unsigned i = 0; i < NumEllipses(vcore); i++)
    if(IsTruePositive(i))
      tp_ids.push_back(i);
  main_win->draw_area->updateGL();
}

void EvalWin::ProcessAll(const string &conv, const string &arcfit,
    const string &search, const string &crit)
{
  struct timespec start, end;
  sum_tp = 0;
  sum_fn = 0;
  sum_gt = 0;
  sum_ap = 0;
  sum_time = 0.;
  min_time = HUGE;
  max_time = -HUGE;
  vcore->GetConfig().items["ARC_FIT_METHOD"] = arcfit;
  vcore->GetConfig().items["CONVEXARCS_STRONG_CONVEXITY"] = conv;
  vcore->GetConfig().items["CONVEXARCS_SEARCH_METHOD"] = search;
  vcore->GetConfig().items["CONVEXARCS_GROUP_CRITERION"] = crit;
  printf("doing %s %s %s %s\n", arcfit.c_str(), conv.c_str(), search.c_str(),
      crit.c_str());
  for(unsigned i = 0; i < imgs.size(); i++)
  {
    img_id->setValue(i);
    clock_gettime(CLOCK_REALTIME, &start);
    vcore->ProcessImage();
    clock_gettime(CLOCK_REALTIME, &end);
    imgs[i].time = timespec_diff(&end, &start);
    imgs[i].tp = 0;
    for(unsigned j = 0; j < imgs[i].ells.size(); j++)
      for(unsigned k = 0; k < NumEllipses(vcore); k++)
        if(Match(k, imgs[i].ells[j]))
        {
          imgs[i].tp++;
          break;
        }
    imgs[i].fn = imgs[i].ells.size() - imgs[i].tp;
    imgs[i].ap = NumEllipses(vcore);
    sum_tp += imgs[i].tp;
    sum_fn += imgs[i].fn;
    sum_gt += imgs[i].ells.size();
    sum_ap += imgs[i].ap;
    sum_time += imgs[i].time;
    min_time = fmin(min_time, imgs[i].time);
    max_time = fmax(max_time, imgs[i].time);
  }
  WriteEvalFile();
}

void EvalWin::ProcessAll()
{
  OpenEvalFile();
  ProcessAll("0", "GROW", "EXHAUSTIVE", "COCURV");
  ProcessAll("0", "SPLIT", "EXHAUSTIVE", "COCURV");
  ProcessAll("0", "RANSAC", "EXHAUSTIVE", "COCURV");
  ProcessAll("0", "GROW", "GREEDY", "COCURV");
  ProcessAll("0", "SPLIT", "GREEDY", "COCURV");
  ProcessAll("0", "RANSAC", "GREEDY", "COCURV");
  ProcessAll("0", "GROW", "GREEDY", "YUEN");
  ProcessAll("0", "SPLIT", "GREEDY", "YUEN");
  ProcessAll("0", "RANSAC", "GREEDY", "YUEN");
  ProcessAll("0", "GROW", "GREEDY", "ELL");
  ProcessAll("0", "SPLIT", "GREEDY", "ELL");
  ProcessAll("0", "RANSAC", "GREEDY", "ELL");
  ProcessAll("1", "GROW", "EXHAUSTIVE", "COCURV");
  ProcessAll("1", "SPLIT", "EXHAUSTIVE", "COCURV");
  ProcessAll("1", "RANSAC", "EXHAUSTIVE", "COCURV");
  ProcessAll("1", "GROW", "GREEDY", "COCURV");
  ProcessAll("1", "SPLIT", "GREEDY", "COCURV");
  ProcessAll("1", "RANSAC", "GREEDY", "COCURV");
  ProcessAll("1", "GROW", "GREEDY", "YUEN");
  ProcessAll("1", "SPLIT", "GREEDY", "YUEN");
  ProcessAll("1", "RANSAC", "GREEDY", "YUEN");
  ProcessAll("1", "GROW", "GREEDY", "ELL");
  ProcessAll("1", "SPLIT", "GREEDY", "ELL");
  ProcessAll("1", "RANSAC", "GREEDY", "ELL");
  CloseEvalFile();
}

void EvalWin::CollectStatistics()
{
  OpenStatFile();
  // process whatever image is just loaded
  vcore->ProcessImage();
  // and collect statistics for all ground truth ellipses
  for(unsigned i = 0; i < stat_imgs.size(); i++)
    stat_imgs[i].CollectStatistics(stat_file);
  CloseStatFile();
}

void EvalWin::DrawStat()
{
  int id = img_id->value();
  if(id >= 0)
  {
    for(unsigned i = 0; i < stat_imgs[id].ells.size(); i++)
      stat_imgs[id].ells[i].Draw();
  }
}

/**
 * Note: the definition of a match (less than 10% deviation of axes etc.) is
 * quite arbitrary.
 */
bool EvalWin::Match(unsigned found_ell, EllPar &gt_ell)
{
  double r = fmax(gt_ell.a*0.10, 3.);
  Vector2 c_d(Ellipses(vcore, found_ell)->x, Ellipses(vcore, found_ell)->y);
  Vector2 c_t(gt_ell.x, gt_ell.y);
  if(Distance(c_d, c_t) <= r)
    if(fabs(Ellipses(vcore, found_ell)->a - gt_ell.a) <= fmax(gt_ell.a*0.1, 3.))
      if(fabs(Ellipses(vcore, found_ell)->b - gt_ell.b) <= fmax(gt_ell.b*0.1, 3.))
      {
        // if almost circular, ignore angle
        if(gt_ell.b/gt_ell.a > 0.9)
          return true;
        else if(fabs(DiffAngle_mpi_pi(Ellipses(vcore, found_ell)->phi, gt_ell.phi))
              <= M_PI/8.)
          return true;
      }
  return false;
}

bool EvalWin::IsTruePositive(unsigned found_ell)
{
  int id = img_id->value();
  for(unsigned i = 0; i < imgs[id].ells.size(); i++)
    if(Match(found_ell, imgs[id].ells[i]))
      return true;
  return false;
}

void EvalWin::OpenEvalFile()
{
  string filename ;
  string::size_type p = gt_filename.find_last_of('.');
  if(p == string::npos)
  {
    fprintf(stderr, "* %s is not a valid groundtruth filename\n",
        gt_filename.c_str());
    return;
  }
  filename.assign(gt_filename, 0, p);
  filename.append(".eval");
  eval_file = fopen(filename.c_str(), "w");
  assert(eval_file != 0);
  filename.assign(gt_filename, 0, p);
  filename.append(".csv");
  tab_file = fopen(filename.c_str(), "w");
  assert(tab_file != 0);
  filename.assign(gt_filename, 0, p);
  filename.append(".plot");
  plot_file = fopen(filename.c_str(), "w");
  assert(plot_file != 0);
}

void EvalWin::CloseEvalFile()
{
  fclose(plot_file);
  fclose(tab_file);
  fclose(eval_file);
}

/**
 * TODO: write data per parameter
 */
void EvalWin::WriteEvalFile()
{
  fprintf(eval_file, "# ARC_FIT_METHOD              %s\n",
      vcore->GetConfig().GetValueString("ARC_FIT_METHOD").c_str());
  fprintf(eval_file, "# CONVEXARCS_STRONG_CONVEXITY %s\n",
      vcore->GetConfig().GetValueString("CONVEXARCS_STRONG_CONVEXITY").c_str());
  fprintf(eval_file, "# CONVEXARCS_SEARCH_METHOD    %s\n",
      vcore->GetConfig().GetValueString("CONVEXARCS_SEARCH_METHOD").c_str());
  fprintf(eval_file, "# CONVEXARCS_GROUP_CRITERION  %s\n",
      vcore->GetConfig().GetValueString("CONVEXARCS_GROUP_CRITERION").c_str());
  fprintf(eval_file, "# img  TP  GT  total  time\n");
  for(unsigned i = 0; i < imgs.size(); i++)
  {
    fprintf(eval_file, "%3u  %3d %3d %5d   %6.3f\n", i, imgs[i].tp,
        (int)imgs[i].ells.size(), imgs[i].ap, imgs[i].time);
  }
  fprintf(eval_file, "# sum: TP  GT  total  min     max     avg time\n");
  fprintf(eval_file,"     %3d %3d %5d   %6.3f  %6.3f  %6.3f\n",
      sum_tp, sum_gt, sum_ap, min_time, max_time, sum_time/(double)imgs.size());
  fprintf(eval_file,"     %2.f%% \n",
      100.*(double)sum_tp/(double)sum_gt);
  fprintf(eval_file, "#---------------------------------------\n\n");

  fprintf(tab_file,
      "%-6s   %d   %-10s   %-6s   %3.f   %6.3f   %6.3f   %6.3f\n",
      vcore->GetConfig().GetValueString("ARC_FIT_METHOD").c_str(),
      vcore->GetConfig().GetValueInt("CONVEXARCS_STRONG_CONVEXITY"),
      vcore->GetConfig().GetValueString("CONVEXARCS_SEARCH_METHOD").c_str(),
      vcore->GetConfig().GetValueString("CONVEXARCS_GROUP_CRITERION").c_str(),
      100.*(double)sum_tp/(double)sum_gt, min_time, max_time,
      sum_time/(double)imgs.size());

  fprintf(plot_file, "%6.3f %3.f  # %s %s %s\n", sum_time/(double)imgs.size(),
      100.*(double)sum_tp/(double)sum_gt,
      vcore->GetConfig().GetValueString("ARC_FIT_METHOD").c_str(),
      vcore->GetConfig().GetValueString("CONVEXARCS_SEARCH_METHOD").c_str(),
      vcore->GetConfig().GetValueString("CONVEXARCS_GROUP_CRITERION").c_str());
}

void EvalWin::OpenStatFile()
{
  stat_file = fopen("ell_stat.txt","w");
  assert(stat_file != 0);
  fprintf(stat_file, "# c .. ellipse circumference\n");
  fprintf(stat_file, "# l .. total pixels counted on circumference\n");
  fprintf(stat_file, "# k .. supporting pixels (=edgels)\n");
  fprintf(stat_file, "# p_e .. edgel probability\n");
  fprintf(stat_file, "# l*p_e .. expected number of supporting edgels\n");
  fprintf(stat_file, "#          under Binomial distribution\n");
  fprintf(stat_file, "#     c         l        k      p_e       p_e*l\n\n");
}

void EvalWin::CloseStatFile()
{
  fclose(stat_file);
  stat_file = 0;
}

/**
 * Transform a point from image to ellipse co-ordinates.
 */
Vector2 EvalWin::StatEll::TransformToEllipse(const Vector2 &p)
{
  return Rotate(p - Vector2(x, y), -phi);
}

/**
 * Transform a point from ellipse to image co-ordinates.
 */
Vector2 EvalWin::StatEll::TransformFromEllipse(const Vector2 &p)
{
  return Rotate(p, phi) + Vector2(x, y);
}

double EvalWin::StatEll::Distance(const Vector2 &p)
{
  return DistanceCentAxPar(TransformToEllipse(p));
}

/**
 * Approximation of the shortest absolute distance of a point to a centered,
 * axis-parallel ellipse.
 */
double EvalWin::StatEll::DistanceCentAxPar(const Vector2 &p)
{
  double x2, y2, n, d;
  if(IsZero(p.x) && IsZero(p.y))
    return b;
  x2 = Sqr(p.x);
  y2 = Sqr(p.y);
  n = fabs(x2/pow(a,2) + y2/pow(b,2) - 1.);
  d = 2.*sqrt(x2/pow(a,4) + y2/pow(b,4));
  return n/d;
}

/**
 * Tangent vector at given ellipse point.
 * Always points counterclockwise.
 */
Vector2 EvalWin::StatEll::Tangent(const Vector2 &p)
{
  return Rotate(TangentCentAxPar(TransformToEllipse(p)), phi);
}

/**
 * Tangent vector at given ellipse point, for centered, axis-parallel ellipse.
 * Always points counterclockwise.
 */
Vector2 EvalWin::StatEll::TangentCentAxPar(const Vector2 &p)
{
  double t = atan2(a*p.y, b*p.x);
  return Vector2(-a*sin(t), b*cos(t));
}

static int neighbours[8][2] =
  {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};

/**
 * Find optimal neighbour of (x,y) in directions from a to b.
 * Result is returned in x_o, y_o. Optimal distance d_o.
 */
void EvalWin::StatEll::OptimalNeighbour(int x, int y, int a, int b, 
    int &x_o, int &y_o, double &d_o)
{
  int x_n, y_n, i = a;
  double d;
  bool cont = true;
  x_o = INT_MAX;
  y_o = INT_MAX;
  d_o = HUGE;
  while(cont)
  {
    if(i == b)
      cont = false;
    x_n = x + neighbours[i][0];
    y_n = y + neighbours[i][1];
    d = Distance(Vector2(x_n, y_n));
    if(d < d_o)
    {
      d_o = d;
      x_o = x_n;
      y_o = y_n;
    }
    i = ScaleIntAngle_0_8(i + 1);
  }
}

/**
 * TODO: beautify this
 */
void EvalWin::StatEll::BuildContourString()
{
  int x, y, x_s, y_s, x_o, y_o, i, cnt = 0;
  double d, d_o;
  // estimated start point of contour
  Vector2 p = TransformFromEllipse(Vector2(a, 0.));
  x = (int)round(p.x);
  y = (int)round(p.y);
  d = Distance(Vector2(x, y));
  // check if any neighbour is nearer to ellipse
  OptimalNeighbour(x, y, 0, 7, x_o, y_o, d_o);
  if(d < d_o)
  {
    d_o = d;
    x_o = x;
    y_o = y;
  }
  contour.Clear();
  contour.PushBack(ContourPoint(x_o, y_o));
  // this is now the real start point
  x_s = x = x_o;
  y_s = y = y_o;
  while(true)
  {
    // TODO: dont need Normalise here
    Vector2 t = Normalise(Tangent(Vector2(x, y)));
    // direction of tangent in [0..8[
    // TODO: shouln't i subtract an offset here of 22.5 deg?
    i = (int)round(ScaleAngle_0_2pi(atan2(t.y, t.x))*4./M_PI);
    // we want next pixel in main direction plus its two neighbours
    OptimalNeighbour(x, y, ScaleIntAngle_0_8(i-1), ScaleIntAngle_0_8(i+1),
        x_o, y_o, d_o);
    if(x_o == x_s && y_o == y_s)
      break;
    if(cnt++ > 1000)
    {
      //printf("*** endless loop!\n");
      break;
    }
    else
    {
      contour.PushBack(ContourPoint(x_o, y_o));
      x = x_o;
      y = y_o;
    }
  }
}

void EvalWin::StatEll::FillContourString()
{
  sup_edgels = 0;
  for(unsigned i = 0; i < contour.Size(); i++)
  {
    if(FormSegments::edge_img->IsInside(contour[i].x, contour[i].y))
      if(FormSegments::edge_img->Occupied(contour[i].x, contour[i].y))
      {
        contour[i].supports = true;
        sup_edgels++;
      }
  }
}

void EvalWin::StatEll::Draw()
{
  for(unsigned i = 0; i < contour.Size(); i++)
    DrawPoint2D(contour[i].x, contour[i].y, RGBColor::blue);
}

EvalWin::StatImage::StatImage(EvalWin::EvalImage &ei)
{
  name = ei.name;
  k_avg = 0;
  ells.resize(ei.ells.size());
  for(unsigned i = 0; i < ei.ells.size(); i++)
  {
    ells[i].x = ei.ells[i].x;
    ells[i].y = ei.ells[i].y;
    ells[i].a = ei.ells[i].a;
    ells[i].b = ei.ells[i].b;
    ells[i].phi = ei.ells[i].phi;
  }
}

void EvalWin::StatImage::CollectStatistics(FILE *stat_file)
{
  FILE *contourfile = fopen("contour.txt", "w");
  k_avg = 0.;
  for(unsigned i = 0; i < ells.size(); i++)
  {
    ells[i].BuildContourString();
    ells[i].FillContourString();
    /* HACK: coment out for now
    fprintf(stat_file, "%9.0f %9d %9d %9.6f %9.0f\n",
       EllipseCircumference(ells[i].a, ells[i].b),
       ells[i].contour.Size(), ells[i].sup_edgels,
       vcore->p_e,
       vcore->p_e*(double)ells[i].contour.Size());*/
  }
  for(unsigned i = 0; i < ells.size(); i++)
  {
    for(unsigned j = 0; j < ells[i].contour.Size(); j++)
      if(ells[i].contour[j].supports)
        fprintf(contourfile, "1\n");
      else
        fprintf(contourfile, "0\n");
    fprintf(contourfile, "\n\n");
  }
  fclose(contourfile);
}

}
