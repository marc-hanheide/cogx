/**
 * $Id: MainWin.hh,v 1.21 2007/03/25 21:35:57 mxz Exp mxz $
 */

#ifndef MAIN_WIN_HH
#define MAIN_WIN_HH

#include <vector>
#include <QMainWindow>
#include <QDialog>
#include <QTableWidget>
#include <QSpinBox>
#include <QCheckBox>
#include <QTextEdit>
#include <QLineEdit>
#include <QComboBox>
#include <QSlider>
#include <QPushButton>
#include <QTimer>
#include <QGLWidget>
#include <QMouseEvent>
#include <QEvent>
#include "Namespace.hh"
#include "ConfigFile.hh"
#include "Array.hh"
#include "Vector2.hh"

namespace Z
{

class MainWin;

/**
 * Main drawing area, for image and gestalts.
 * Note: origin in top left corner, y pointing downwards.
 */
class MainDrawArea : public QGLWidget
{
Q_OBJECT
private:
  MainWin *main_win;
  double cam_trans[3];
  double cam_rot[2];
  int mouse_x, mouse_y;
  Qt::MouseButtons mouse_butt;
  double depth_step;

  void paint2D();
  void paint3D();
  void DrawOverlays();
  void DrawZAxis(char axis);
  void DrawCoordFrame();
  void DrawImageFrame();
  void DrawLines();
  void DrawSurfaces();
  void DrawSurface(unsigned surf);

protected:
  void initializeGL();
  void paintGL();
  void resizeGL(int w, int h);
  void mousePressEvent(QMouseEvent *ev);
  void mouseMoveEvent(QMouseEvent *ev);
public:
  MainDrawArea(QWidget *parent, MainWin *main_w);
};

/**
 * Drawing area for gestalt info.
 * Note: origin in bottom left corner, y pointing upwards.
 * Any gestalt can draw pictorial info here, e.g. histograms.
 */
class InfoDrawArea : public QGLWidget
{
Q_OBJECT
private:
  MainWin *main_win;
protected:
  void initializeGL();
  void paintGL();
  void resizeGL(int w, int h);
public:
  InfoDrawArea(QWidget *parent, MainWin *main_w);
};

/**
 * Popup window showing various gestalt info.
 */
class GestaltWin : public QDialog
{
Q_OBJECT
private:
  MainWin *main_win;

  void FillGestaltList();
  void UpdateGestaltList();

public:
  QCheckBox *debug;
  QCheckBox *incremental;
  QTableWidget *gestalt_list;
  QComboBox *draw_image;
  QCheckBox *draw_edges;
  QCheckBox *draw_vote_img;
  QCheckBox *draw_gt;
  QCheckBox *select_all_of_type;
  QCheckBox *select_up_to;
  QSpinBox *gestalt_id;
  QSpinBox *gestalt_rank;
  QLineEdit *thr_input;
  QCheckBox *mask;
  QLineEdit *time_input;
  QSpinBox *detail;
  QTextEdit *gestalt_info;
  InfoDrawArea *draw_area;
  QTableWidget *principle_list;

  GestaltWin(MainWin *main_w);

public slots:
  void SelectGestaltType();
  void SelectGestaltID(int id);
  void SelectGestaltRank(int rank);
  void SelectAllOfType(bool is_checked);
  void SelectUpTo(bool is_checked);
  void UpdateRanges();
  void UpdateContent();
};

/**
 * Popup window showing various gestalt principle info.
 */
class PrincipleWin : public QDialog
{
Q_OBJECT
private:
  MainWin *main_win;

  void FillPrincipleList();
  void UpdatePrincipleList();

public:
  QTableWidget *principle_list;

  PrincipleWin(MainWin *main_w);

public slots:
  void UpdateContent();
};

class EvalWin : public QDialog
{
Q_OBJECT
public:
  class EllPar
  {
  public:
    double x, y, a, b, phi;
  };

  class EvalImage
  {
  public:
    string name;  // image name
    vector<EllPar> ells;  // ground truth ellipses
    int tp;       // true positives
    int fn;       // false negatives
    int ap;       // all positives (true plus false positives)
    double time;  // run time in [s]
  };

  class StatEll
  {
  public:
    class ContourPoint
    {
    public:
      int x, y;
      bool supports;
      ContourPoint() {};
      ContourPoint(int xi, int yi)
        : x(xi), y(yi), supports(false) {}
    };
    Array<ContourPoint> contour;
    double x, y, a, b, phi;
    int sup_edgels;

    Vector2 TransformToEllipse(const Vector2 &p);
    Vector2 TransformFromEllipse(const Vector2 &p);
    double Distance(const Vector2 &p);
    double DistanceCentAxPar(const Vector2 &p);
    Vector2 Tangent(const Vector2 &p);
    Vector2 TangentCentAxPar(const Vector2 &p);
    void OptimalNeighbour(int x, int y, int a, int b, int &x_o, int &y_o,
        double &d_o);
    void BuildContourString();
    void FillContourString();
    void Draw();
  };

  class StatImage
  {
  public:
    string name;  // image name
    vector<StatEll> ells;  // ground truth ellipses
    double k_avg;
    StatImage() {}
    StatImage(EvalImage &ei);
    void CollectStatistics(FILE *stat_file);
  };

  MainWin *main_win;

  string gt_filename;
  vector<EvalImage> imgs;
  vector<StatImage> stat_imgs;
  vector<unsigned> tp_ids;  // ids of true positives
  int sum_tp, sum_fn;  // sum of true positive, false negative
  int sum_gt;  // sum of ground truths
  int sum_ap;  // sum of all posotives
  double min_time, max_time, sum_time;
  FILE *eval_file;
  FILE *tab_file;
  FILE *plot_file;
  FILE *stat_file;

  QTextEdit *gt_info;
  QSpinBox *img_id;
  QPushButton *process_but;
  QPushButton *all_but;
  QPushButton *snap_but;

private:
  bool ReadImageName(ConfigFile &gt_file, char *img_name);
  bool ReadNumEllipses(ConfigFile &gt_file, int &n);
  bool ReadEllipse(ConfigFile &gt_file, EllPar &e);
  bool Match(unsigned found_ell, EllPar &gt_ell);
  bool IsTruePositive(unsigned found_ell);

public:
  EvalWin(MainWin *main_w);
  void LoadGT(const char *file_name);
  void OpenEvalFile();
  void CloseEvalFile();
  void OpenStatFile();
  void CloseStatFile();
  void WriteEvalFile();
  void ClearEvalData();

public slots:
  void SelectImgID(int id);
  void DrawGT();
  void DrawStat();
  void Process();
  void ProcessAll();
  void ProcessAll(const string &conv, const string &arcfit,
      const string &search, const string &crit);
  void SnapshotAllGT();
  void CollectStatistics();
};

/*class ConfigWin : public QDialog
{
Q_OBJECT
};*/

/**
 * The applications main window.
 */
class MainWin : public QMainWindow
{
Q_OBJECT
private:
  double zoom;

public:
  static MainWin *main_win;
  MainDrawArea *draw_area;
  GestaltWin *gestalt_win;
  PrincipleWin *princ_win;
  EvalWin *eval_win;
  QTimer *idle_timer;

private:
  void ParseOptions(int argc, char **argv);
  void BuildMenu();
  void BuildContent();

public:
  MainWin(int argc, char **arg);
  ~MainWin();
  void MysteriousFunction(int mouse_x, int mouse_y);
  void DrawVoteImg();
  void HaveNewImage();
  void HaveLiveVideo();

public slots:
  void LoadImage();
  void NextImage();
  void Rewind();
  void ProcessImage();
  void ProcessNextImage();
  void ClearResults();
  void SaveImage();
  void Run();
  void DisplayAbout();
  void OpenGestaltWin();
  void OpenPrincipleWin();
  void OpenEvalFile();
  void NoZoom();
  void HalfZoom();
  void DoubleZoom();
};

}

#endif

