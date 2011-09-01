/*
 * sampling.cpp
 *
 *  Created on: Apr 12, 2011
 *      Author: aa
 */

#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkTriangle.h>
#include <vtkMath.h>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <Eigen/StdVector>
#include <pcl/common/common.h>

inline double
uniform_deviate (int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

inline void
randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
                     Eigen::Vector4f& p)
{
  float r1 = uniform_deviate (rand ());
  float r2 = uniform_deviate (rand ());
  float r1sqr = sqrt (r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
  p[3] = 0;
}

inline void
randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p)
{
  float r = uniform_deviate (rand ()) * totalArea;

  std::vector<double>::iterator low = lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
  vtkIdType el = (vtkIdType) (low - cumulativeAreas->begin ());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType *ptIds = NULL;
  polydata->GetCellPoints (el, npts, ptIds);
  polydata->GetPoint (ptIds[0], A);
  polydata->GetPoint (ptIds[1], B);
  polydata->GetPoint (ptIds[2], C);
  randomPointTriangle (A[0], A[1], A[2], B[0], B[1], B[2], C[0], C[1], C[2], p);
}

void
uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples,
                                     pcl::PointCloud<pcl::PointXYZ> & cloud_out)
{
  polydata->BuildCells ();
  vtkSmartPointer < vtkCellArray > cells = polydata->GetPolys ();

  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
  size_t i = 0;
  vtkIdType npts = 0, *ptIds = NULL;
  for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
  {
    polydata->GetPoint (ptIds[0], p1);
    polydata->GetPoint (ptIds[1], p2);
    polydata->GetPoint (ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
    cumulativeAreas[i] = totalArea;
  }

  cloud_out.points.resize (n_samples);
  cloud_out.width = n_samples;
  cloud_out.height = 1;

  for (i = 0; i < n_samples; i++)
  {
    Eigen::Vector4f p;
    randPSurface (polydata, &cumulativeAreas, totalArea, p);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
  }
}

/*void
getVerticesAsPointCloud(vtkSmartPointer<vtkPolyData> polydata, pcl::PointCloud<pcl::PointXYZ> & cloud_out) {
  vtkPoints *points = polydata->GetPoints ();
  cloud_out.points.resize(points->GetNumberOfPoints ());
  cloud_out.width = cloud_out.points.size();
  cloud_out.height = 1;
  cloud_out.is_dense = false;

  for (int i = 0; i < points->GetNumberOfPoints (); i++)
  {
    double p[3];
    points->GetPoint (i, p);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
  }
}*/

void
step_sampling (vtkSmartPointer<vtkPolyData> polydata, float step_,
                                  pcl::PointCloud<pcl::PointXYZ> & cloud_out)
{

  vtkPoints *points = polydata->GetPoints ();
  vtkPoints *extra_points = vtkPoints::New ();

  cerr << polydata->GetNumberOfVerts () << " vertices." << endl;
  cerr << polydata->GetNumberOfLines () << " lines." << endl;
  cerr << polydata->GetNumberOfPolys () << " polys." << endl;
  cerr << polydata->GetNumberOfStrips () << " strips." << endl;

  double minP[3], maxP[3];
  for (int i = 0; i < 3; i++)
  {
    minP[i] = FLT_MAX;
    maxP[i] = -FLT_MAX;
  }

  for (int i = 0; i < points->GetNumberOfPoints (); i++)
  {
    double p[3];
    points->GetPoint (i, p);
    for (int j = 0; j < 3; j++)
    {
      minP[j] = (p[j] < minP[j]) ? p[j] : minP[j];
      maxP[j] = (p[j] > maxP[j]) ? p[j] : maxP[j];
    }
  }
  cerr << "min: " << minP[0] << " " << minP[1] << " " << minP[2] << endl;
  cerr << "max: " << maxP[0] << " " << maxP[1] << " " << maxP[2] << endl;

  double diagonal = sqrt (
                          (maxP[0] - minP[0]) * (maxP[0] - minP[0]) + (maxP[1] - minP[1]) * (maxP[1] - minP[1])
                              + (maxP[2] - minP[2]) * (maxP[2] - minP[2]));
  cerr << "diagonal: " << diagonal << endl;

  double step_arg = step_;
  if (step_arg > 0)
    ;
  else
  {
    cerr << "WARNING: Step number should be bigger than 0!" << endl;
  }

  //double step = diagonal / step_arg;
  double step = step_arg;

  vtkCellArray* polys = polydata->GetPolys ();
  cerr << polys->GetNumberOfCells () << " cells." << endl;
  vtkIdType npts, *pts;
  int n = 0;
  for (polys->InitTraversal (); polys->GetNextCell (npts, pts); n++)
  {
    if (npts != 3)
      cerr << npts << " points in cell " << n << endl;
    else
    {
      // Get the 3 points of the triangle
      double p1[3], p2[3], p3[3];
      points->GetPoint (pts[0], p1);
      points->GetPoint (pts[1], p2);
      points->GetPoint (pts[2], p3);

      double a[3];
      double b[3];
      double c[3];
      double p_med[3];

      double side[3];
      for (int i = 0; i < 3; i++)
        side[i] = 0;

      //norm of the triangle sides
      double ma = 0, mb = 0, mc = 0;

      for (int i = 0; i < 3; i++)
      {
        a[i] = p2[i] - p1[i];
        b[i] = p3[i] - p2[i];
        c[i] = p1[i] - p3[i];
        ma += a[i] * a[i];
        mb += b[i] * b[i];
        mc += c[i] * c[i];
        p_med[i] = (p1[i] + p2[i] + p3[i]) / 3;
        side[0] += a[i] * p_med[i];
        side[1] += b[i] * p_med[i];
        side[2] += c[i] * p_med[i];
      }
      ma = sqrt (ma);
      mb = sqrt (mb);
      mc = sqrt (mc);

      // u is the largest, v the smallest vector (normalized)
      double u[3], v[3], start_point[3];
      double mu, mv;
      int sign;
      if (ma > mb)
      {
        if (ma > mc)
        {
          for (int i = 0; i < 3; i++)
            u[i] = a[i] / ma;
          mu = ma;
          if (mb > mc)
          {
            for (int i = 0; i < 3; i++)
            {
              v[i] = c[i] / mc;
              start_point[i] = p2[i];
            }
            mv = mc;
            sign = -1;
          }
          else
          {
            for (int i = 0; i < 3; i++)
            {
              v[i] = b[i] / mb;
              start_point[i] = p1[i];
            }
            mv = mb;
            sign = 1;
          }
        }
        else
        {
          for (int i = 0; i < 3; i++)
          {
            u[i] = c[i] / mc;
            v[i] = b[i] / mb;
            start_point[i] = p1[i];
          }
          mu = mc;
          mv = mb;
          sign = -1;
        }
      }
      else
      {
        if (ma > mc)
        {
          for (int i = 0; i < 3; i++)
          {
            u[i] = b[i] / mb;
            v[i] = c[i] / mc;
            start_point[i] = p2[i];
          }
          mu = mb;
          mv = mc;
          sign = 1;
        }
        else
        {
          for (int i = 0; i < 3; i++)
            v[i] = a[i] / ma;
          mv = ma;
          if (mb > mc)
          {
            for (int i = 0; i < 3; i++)
            {
              u[i] = b[i] / mb;
              start_point[i] = p3[i];
            }
            mu = mb;
            sign = -1;
          }
          else
          {
            for (int i = 0; i < 3; i++)
            {
              u[i] = c[i] / mc;
              start_point[i] = p3[i];
            }
            mu = mc;
            sign = 1;
          }
        }
      }

      double point[3];

      for (double i = 0.0; i <= mu; i += step)
        for (double j = 0.0; (j <= i * mv / mu) && (j <= mv); j += step)
        {
          for (int k = 0; k < 3; k++)
          {
            point[k] = start_point[k] + sign * (i * u[k] + j * v[k]);
          }

          extra_points->InsertNextPoint (point);
        }
    }
  }

  cerr << extra_points->GetNumberOfPoints () << " created." << endl;
  cloud_out.points.resize (extra_points->GetNumberOfPoints ());
  cloud_out.width = extra_points->GetNumberOfPoints ();
  cloud_out.height = 1;

  for (int i = 0; i < extra_points->GetNumberOfPoints (); i++)
  {
    double p[3];
    extra_points->GetPoint (i, p);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
  }

  std::cout << "Finished inserting points" << std::endl;
}
