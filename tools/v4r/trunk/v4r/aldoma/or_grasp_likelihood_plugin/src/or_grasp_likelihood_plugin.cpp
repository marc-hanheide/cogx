#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkPoints.h>
#include <vtkGlyph3D.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkCellArray.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkTransformFilter.h>
#include <vtkTransform.h>
#include <vtkCellLocator.h>
#include <vtkGenericCell.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkTriangle.h>

#include <algorithm>

using namespace std;
using namespace OpenRAVE;

//sort tuples of 7 elements (rays, value)
typedef struct
{
  bool
  operator() (std::vector<float> const& a, std::vector<float> const& b)
  {
    return a[6] > b[6];
  }
} sort_rays;

void inline
getPointsActor3D (vtkSmartPointer<vtkPoints> & points, vtkSmartPointer<vtkPolyDataMapper> & spike, double scale = 0.05)
{
  vtkSmartPointer < vtkConeSource > coneSource = vtkSmartPointer<vtkConeSource>::New ();
  coneSource->SetResolution (6);

  vtkSmartPointer < vtkPolyData > poly = vtkSmartPointer<vtkPolyData>::New ();
  vtkSmartPointer < vtkCellArray > conn = vtkSmartPointer<vtkCellArray>::New ();

  poly->SetPoints (points);
  poly->SetVerts (conn);

  vtkSmartPointer < vtkGlyph3D > glyph = vtkSmartPointer<vtkGlyph3D>::New ();
  glyph->SetInput (poly);
  glyph->SetSourceConnection (coneSource->GetOutputPort ());
  glyph->SetScaleFactor (scale);

  spike->SetInputConnection (glyph->GetOutputPort ());
}

void inline
visModels (vtkSmartPointer<vtkPolyDataMapper> mapper1, vtkSmartPointer<vtkPolyDataMapper> mapper2)
{
  vtkSmartPointer < vtkActor > actor = vtkSmartPointer<vtkActor>::New ();
  actor->SetMapper (mapper1);
  actor->GetProperty ()->SetOpacity (1);

  vtkSmartPointer < vtkActor > actor2 = vtkSmartPointer<vtkActor>::New ();
  actor2->SetMapper (mapper2);

  vtkSmartPointer < vtkRenderer > renderer = vtkSmartPointer<vtkRenderer>::New ();
  vtkSmartPointer < vtkRenderWindow > renderWindow = vtkSmartPointer<vtkRenderWindow>::New ();
  renderWindow->AddRenderer (renderer);
  vtkSmartPointer < vtkRenderWindowInteractor > renderWindowInteractor
      = vtkSmartPointer<vtkRenderWindowInteractor>::New ();
  renderWindowInteractor->SetRenderWindow (renderWindow);
  renderer->AddActor (actor);
  renderer->AddActor (actor2);

  actor2->GetProperty ()->SetColor (1, 0, 0);

  renderer->SetBackground (0.1804, 0.5451, 0.3412); // Sea green

  renderWindow->Render ();
  renderWindowInteractor->Start ();
}

void inline
visPointsAndMapper (vtkSmartPointer<vtkPoints> & points, vtkSmartPointer<vtkPolyDataMapper> & mapper)
{
  vtkSmartPointer < vtkPolyDataMapper > spike = vtkSmartPointer<vtkPolyDataMapper>::New ();
  getPointsActor3D (points, spike, 0.010);
  visModels (mapper, spike);
}

float
inline
getIntersectionPointDist (vtkPolyData * polydata, vtkIdType cellId, double * p1, float MAX_DIST)
{
  double A[3], B[3], C[3];
  vtkIdType npts = 0, *ptIds = NULL;
  polydata->GetCellPoints (cellId, npts, ptIds);
  polydata->GetPoint (ptIds[0], A);
  polydata->GetPoint (ptIds[1], B);
  polydata->GetPoint (ptIds[2], C);

  double normal[3];
  vtkTriangle::ComputeNormal (A, B, C, normal);

  double AMinp1[3] = {A[0] - p1[0], A[1] - p1[1], A[2] - p1[2]};
  double uNum = AMinp1[0] * normal[0] + AMinp1[1] * normal[1] + AMinp1[2] * normal[2];
  double uDen = (-p1[0] * normal[0]) + (-p1[1] * normal[1]) + (-p1[2] * normal[2]);
  uNum /= uDen;

  double intersectionV[3];
  intersectionV[0] = p1[0] + uNum * (-p1[0]);
  intersectionV[1] = p1[1] + uNum * (-p1[1]);
  intersectionV[2] = p1[2] + uNum * -p1[2];

  float dist = 0;
  dist = sqrt (intersectionV[0] * intersectionV[0] + intersectionV[1] * intersectionV[1] + intersectionV[2]
      * intersectionV[2]);

  if (vtkTriangle::PointInTriangle (intersectionV, A, B, C, 0.0001))
  {
    dist = sqrt (intersectionV[0] * intersectionV[0] + intersectionV[1] * intersectionV[1] + intersectionV[2]
        * intersectionV[2]);
  }
  else
  {
    dist = MAX_DIST;
  }

  return dist;
}

class ORLikelihoodPlugin : public ProblemInstance
{

private:
  vtkSmartPointer<vtkPolyData> mesh_;

public:
  ORLikelihoodPlugin (EnvironmentBasePtr penv) :
    ProblemInstance (penv)
  {
    __description = "A plugin to compute given contact points the overlap likelihood of a mesh and a grasp.";
    RegisterCommand ("computeLikelihood", boost::bind (&ORLikelihoodPlugin::computeLikelihood, this, _1, _2),
                     "computes");
    RegisterCommand ("computeRaysOverlapLikelihood", boost::bind (&ORLikelihoodPlugin::computeIntersectionAndSortRays,
                                                                  this, _1, _2),
                     "compute for each approach ray the likelihood of the intersection with the model");
    RegisterCommand ("load", boost::bind (&ORLikelihoodPlugin::Load, this, _1, _2), "loads a given file");
  }

  void
  Destroy ()
  {
    RAVELOG_INFO ("problem unloaded from environment\n");
  }

  int
  main (const string& cmd)
  {
    RAVELOG_INFO ("problem initialized cmd; %s\n", cmd.c_str ());
    return 0;
  }

  bool
  computeIntersectionAndSortRays (ostream& sout, istream& sinput)
  {

    std::string env_body_id;
    sinput >> env_body_id;

    std::cout << "body_id:" << env_body_id << std::endl;
    std::cout << "Going to sort approach rays..." << std::endl;

    std::string num;
    int len = 6;
    std::vector<float> ray (len, 0); //6 values for ray, 1 for overlap metric
    std::vector < std::vector<float> > rays_with_values;

    int i = 0;
    while (!sinput.eof ())
    {
      sinput >> num;

      ray[i % len] = atof (num.c_str ());
      ++i;

      if ((i % len) == 0)
      {
        std::vector<float> ray_plus_value (7, 0);
        for (int k = 0; k < ray.size (); k++)
        {
          ray_plus_value[k] = ray[k];
        }
        rays_with_values.push_back (ray_plus_value);
      }
    }

    vtkSmartPointer < vtkPoints > contacts = vtkSmartPointer<vtkPoints>::New ();
    for (int k = 0; k < rays_with_values.size (); k++)
    {
      contacts->InsertNextPoint (rays_with_values[k][0], rays_with_values[k][1], rays_with_values[k][2]);
    }

    vtkSmartPointer < vtkPolyDataMapper > mesh = vtkSmartPointer<vtkPolyDataMapper>::New ();
    mesh->SetInput (mesh_);
    mesh->Update ();

    visPointsAndMapper (contacts, mesh);

    std::cout << "approach rays length:" << rays_with_values.size () << std::endl;

    mesh_->BuildCells ();
    vtkCellLocator * cellLoc = vtkCellLocator::New ();
    cellLoc->SetDataSet (mesh_);
    cellLoc->BuildLocator ();

    vtkIdList * cells = vtkIdList::New ();
    float dist, MIN_DIST;
    double p1[3], p2[3];

    vtkSmartPointer < vtkDoubleArray > weights = vtkSmartPointer<vtkDoubleArray>::New ();
    weights = (vtkDoubleArray *)mesh_->GetPointData ()->GetScalars ();

    double closestPoint[3];
    vtkSmartPointer < vtkGenericCell > cell = vtkSmartPointer<vtkGenericCell>::New ();
    vtkIdType cellId;
    int subId;
    double dist2;

    //go through rays with values, check intersection and add overlap to 7-th value
    for (int k = 0; k < rays_with_values.size (); k++)
    {
      p1[0] = rays_with_values[k][0];
      p1[1] = rays_with_values[k][1];
      p1[2] = rays_with_values[k][2];

      cellLoc->FindClosestPoint (p1, closestPoint, cell, cellId, subId, dist2);
      double p_likelihood = getLikelihood (mesh_, weights, cell, closestPoint);
      cout << "p_likelihood" << p_likelihood << endl;


      /*p1[0] = rays_with_values[k][0];
      p1[1] = rays_with_values[k][1];
      p1[2] = rays_with_values[k][2];

      p2[0] = p1[0] - rays_with_values[k][3] * 50;
      p2[1] = p1[1] - rays_with_values[k][4] * 50;
      p2[2] = p1[2] - rays_with_values[k][5] * 50;

      cellLoc->FindCellsAlongLine (p1, p2, 0.001, cells);
      cout << "Number of cells along line:" << cells->GetNumberOfIds () << endl;

      MIN_DIST = std::numeric_limits<float>::max ();

      for (int id = 0; id < cells->GetNumberOfIds (); id++)
      {
        dist = getIntersectionPointDist (mesh_, cells->GetId (id), p1, std::numeric_limits<float>::max ());
        if (dist < MIN_DIST)
        {
          MIN_DIST = dist;
        }
      }*/

      rays_with_values[k][6] = p_likelihood;
    }

    //sort in place
    std::sort (rays_with_values.begin (), rays_with_values.end (), sort_rays ());

    //return after sort
    for (int k = 0; k < rays_with_values.size (); k++)
    {
      for (int i = 0; i < 6; i++)
      {
        sout << rays_with_values[k][i];

        if (k != (rays_with_values.size () - 1) || i <= 4)
          sout << " ";
      }

      std::cout << "likelihood:" << rays_with_values[k][6] << endl;
    }

    return true;
  }

  double
  getLikelihood (vtkPolyData * mesh_transformed, vtkDoubleArray * weights, vtkGenericCell * cell, double * closestPoint)
  {
    // 1.b) Get points and likelihoods of the points of the cell
    double p1[3], p2[3], p3[3];
    vtkSmartPointer < vtkIdList > ptIds = vtkSmartPointer<vtkIdList>::New ();
    ptIds = cell->GetPointIds ();
    mesh_transformed->GetPoint (ptIds->GetId (0), p1);
    mesh_transformed->GetPoint (ptIds->GetId (1), p2);
    mesh_transformed->GetPoint (ptIds->GetId (2), p3);

    double l1, l2, l3;
    l1 = weights->GetValue (ptIds->GetId (0));
    l2 = weights->GetValue (ptIds->GetId (1));
    l3 = weights->GetValue (ptIds->GetId (2));

    double p1p2[3];
    double p1p3[3];

    for (int ii = 0; ii < 3; ii++)
    {
      p1p2[ii] = p2[ii] - p1[ii];
      p1p3[ii] = p3[ii] - p1[ii];
    }

    double t, u;
    t = u = 0;
    double MinusP0plusClosestPoint[3];
    for (int ii = 0; ii < 3; ii++)
    {
      MinusP0plusClosestPoint[ii] = -p1[ii] + closestPoint[ii];
    }

    double m_p1p3 = p1p3[0] * p1p3[0] + p1p3[1] * p1p3[1] + p1p3[2] * p1p3[2];
    for (int ii = 0; ii < 3; ii++)
    {
      t += (p1p3[ii] / m_p1p3) * MinusP0plusClosestPoint[ii];
    }

    double m_p1p2 = p1p2[0] * p1p2[0] + p1p2[1] * p1p2[1] + p1p2[2] * p1p2[2];

    //(p0 + t*p1p3 - closestPoint) * pinv(p1p2)
    double Txp1p3[3];
    for (int ii = 0; ii < 3; ii++)
      Txp1p3[ii] = p1p3[ii] * t + p1[ii] - closestPoint[ii];

    //Txp1p3 * pinv(p1p2)
    for (int ii = 0; ii < 3; ii++)
      u += (p1p2[ii] / m_p1p2) * Txp1p3[ii];

    //std::cout << l1 << " " << l2 << " " << l3 << "t:" << t << "u:" << u << " sum:" << u + t << std::endl;

    // 1.c) Interpolate
    //p_likelihood = (l1 + l2 + l3) / 3.0;
    //std::cout << "p_likelihood" << p_likelihood << std::endl;
    double p_likelihood = l1 + t * (l2 - l1) + u * (l3 - l1);
    return p_likelihood;
  }

  bool
  computeLikelihood (ostream& sout, istream& sinput)
  {

    std::string env_body_id;
    sinput >> env_body_id;

    //std::cout << "body_id:" << env_body_id << std::endl;

    //get contact points from sinput..
    vtkSmartPointer < vtkPoints > contacts = vtkSmartPointer<vtkPoints>::New ();

    std::string num;
    std::vector<float> triplet (3, 0);

    int i = 0;
    while (!sinput.eof ())
    {
      sinput >> num;
      //std::cout << num << std::endl;

      triplet[i % 3] = atof (num.c_str ());
      ++i;

      if ((i % 3) == 0)
      {
        contacts->InsertNextPoint (triplet[0], triplet[1], triplet[2]);
      }
    }

    //std::cout << "Number of contacts:" << contacts->GetNumberOfPoints () << std::endl;

    //ATT: Contact points are in world coordinates and mesh in camera coordinates... get the appropriate transform
    KinBodyPtr target = GetEnv ()->GetBodyFromEnvironmentId (atoi (env_body_id.c_str ()));

    OpenRAVE::Transform t = target->GetTransform ();
    //std::cout << t.rot << " trans:" << t.trans << std::endl;

    RaveTransformMatrix<float> rot_matrix = OpenRAVE::geometry::matrixFromQuat (t.rot);

    //fstd::cout << "rot:" << rot_matrix << std::endl;
    //visualize mesh and contact points...
    vtkSmartPointer < vtkMatrix4x4 > matrix = vtkSmartPointer<vtkMatrix4x4>::New ();
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
      {
        matrix->SetElement (j, k, rot_matrix.rot (j, k));
      }

    //matrix->Identity();
    matrix->SetElement (0, 3, t.trans[0]);
    matrix->SetElement (1, 3, t.trans[1]);
    matrix->SetElement (2, 3, t.trans[2]);

    vtkSmartPointer < vtkTransform > transform = vtkSmartPointer<vtkTransform>::New ();
    transform->SetMatrix (matrix);

    vtkSmartPointer < vtkTransformFilter > trans = vtkSmartPointer<vtkTransformFilter>::New ();
    trans->SetTransform (transform);
    trans->SetInput (mesh_);
    trans->Update ();

    vtkSmartPointer < vtkPolyDataMapper > mesh = vtkSmartPointer<vtkPolyDataMapper>::New ();
    mesh->SetInputConnection (trans->GetOutputPort ());
    mesh->Update ();

    double ovall_likelihood = 0.0;
    std::vector<double> likelihoods (contacts->GetNumberOfPoints (), 0);

    vtkSmartPointer < vtkPolyData > mesh_transformed = mesh->GetInput ();
    mesh_transformed->BuildCells ();

    //compute overall overlap likelihood

    vtkCellLocator * cellLoc = vtkCellLocator::New ();
    cellLoc->SetDataSet (mesh_transformed);
    cellLoc->BuildLocator ();

    double x[3];
    double closestPoint[3];
    vtkSmartPointer < vtkGenericCell > cell = vtkSmartPointer<vtkGenericCell>::New ();
    vtkIdType cellId;
    int subId;
    double dist2;

    vtkSmartPointer < vtkDoubleArray > weights = vtkSmartPointer<vtkDoubleArray>::New ();
    weights = (vtkDoubleArray *)mesh_transformed->GetPointData ()->GetScalars ();

    for (int i = 0; i < contacts->GetNumberOfPoints (); i++)
    {
      // 1) Get for each point the likelihood
      // 1.a) Get cell
      double * p = contacts->GetPoint (i);
      x[0] = p[0];
      x[1] = p[1];
      x[2] = p[2];
      cellLoc->FindClosestPoint (x, closestPoint, cell, cellId, subId, dist2);

      double p_likelihood = getLikelihood (mesh_transformed, weights, cell, closestPoint);
      //std::cout << "p_likelihood interpolation" << p_likelihood << std::endl;
      likelihoods[i] = p_likelihood;
    }

    //visPointsAndMapper (contacts, mesh);

    //Overall grasp likelihood
    //We have here different strategies to do that:
    // - average of each point
    // - favor grasps where all independent contact points have good likelihoods
    // - etc.

    for (int i = 0; i < likelihoods.size (); i++)
    {
      ovall_likelihood += likelihoods[i];
    }

    ovall_likelihood /= likelihoods.size ();

    std::cout << "Overall likelihood:" << ovall_likelihood << std::endl;
    //put result into sout...
    sout << ovall_likelihood;

    return true;
  }

  bool
  Load (ostream& sout, istream& sinput)
  {
    string filename;
    sinput >> filename;

    mesh_ = vtkSmartPointer<vtkPolyData>::New ();
    vtkSmartPointer < vtkPolyDataReader > reader = vtkSmartPointer<vtkPolyDataReader>::New ();
    reader->SetFileName (filename.c_str ());
    reader->SetOutput (mesh_);

    mesh_->BuildCells ();

    return true;
  }
};

InterfaceBasePtr
CreateInterfaceValidated (InterfaceType type, const std::string& interfacename, std::istream& sinput,
                          EnvironmentBasePtr penv)
{
  if (type == PT_ProblemInstance && interfacename == "orgrasplikelihood")
  {
    return InterfaceBasePtr (new ORLikelihoodPlugin (penv));
  }
  return InterfaceBasePtr ();
}

void
GetPluginAttributesValidated (PLUGININFO& info)
{
  info.interfacenames[PT_ProblemInstance].push_back ("ORGraspLikelihood");
}

OPENRAVE_PLUGIN_API void
DestroyPlugin ()
{
  RAVELOG_INFO ("destroying plugin\n");
}
