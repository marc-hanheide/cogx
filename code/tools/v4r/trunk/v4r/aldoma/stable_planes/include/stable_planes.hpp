/*
 * stable_planes.hpp
 *
 *  Created on: Apr 12, 2011
 *      Author: aa
 */

#ifndef STABLE_PLANES_HPP_
#define STABLE_PLANES_HPP_

#include "cluster_planar_faces.hpp"
#include <boost/filesystem.hpp>
#include <pcl/segmentation/segment_differences.h>

inline std::string
getName (std::string file)
{
  std::vector < std::string > strs;
  boost::split (strs, file, boost::is_any_of ("/"));
  std::string nameWithExt = strs[strs.size () - 1];
  return nameWithExt.substr (0, nameWithExt.find_last_of ("."));
}

inline void
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
}

class Plane
{

public:
  //Eigen::Vector3f normal;
  //Eigen::Vector3f center;
  double normal[3];
  double center[3];
  double area;
  double areaRatio;
  pcl::PointCloud<pcl::PointXYZ>::Ptr support_points_;

  void
  setSupportPoints (pcl::PointCloud<pcl::PointXYZ>::Ptr support_points)
  {
    support_points_ = support_points;
  }
};

inline bool
sortPlanesByArea (const Plane & p1, const Plane & p2)
{
  return p1.area > p2.area;
}

inline bool
sortPlanesByAreaRatio (const Plane & p1, const Plane & p2)
{
  return p1.areaRatio > p2.areaRatio;
}

class stablePlanes {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;
  float THRESHOLD_;
  bool VISUALIZE_;
  double threshold_non_inf_;
  vtkSmartPointer<vtkPolyData> polydata_;
  std::string filename_for_cache_;

  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    stablePlanes() {
      THRESHOLD_ = 0.005;
      VISUALIZE_ = false;
      threshold_non_inf_ = 0.005;
    }

    void
    setFilenameForCache(std::string file) {
      filename_for_cache_ = file;
    }

    void
    setInputCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr data)
    {
      cloud_out = data;
    }

    void
    setInputPolyData (vtkSmartPointer<vtkPolyData> polydata)
    {
      polydata_ = polydata;
    }

    void
    setThresholdForFilteringNonInformativePlanes(double d)
    {
      threshold_non_inf_ = d;
    }

    static void
    transformOnPlane (const Eigen::Vector3f & centroid, const Eigen::Vector3f & normal, Eigen::Matrix4f & transform)
    {
      Eigen::Vector3f A;
      if (normal[2] != 0)
      {
        A[0] = 0.31234;
        A[1] = 0.53424;
        A[2] = (normal.dot (centroid) - normal[0] * 0.31234 - normal[1] * 0.53424) / normal[2];
      }
      else if (normal[1] != 0)
      {
        A[0] = 0.31234;
        A[1] = (normal.dot (centroid) - normal[0] * 0.31234 - normal[2] * 0.53424) / normal[1];
        A[2] = 0.53424;
      }
      else
      {
        A[0] = (normal.dot (centroid) - normal[1] * 0.31234 - normal[2] * 0.53424) / normal[0];
        A[1] = 0.31234;
        A[2] = 0.53424;
      }

      Eigen::Vector3f yPrima = normal * -1.0;
      Eigen::Vector3f xPrima = A - centroid;
      Eigen::Vector3f zPrima = yPrima.cross (xPrima); //cross (*yPrima, *xPrima);
      yPrima.normalize ();
      zPrima.normalize ();
      xPrima.normalize ();

      transform.Identity ();
      transform (3, 0) = 0;
      transform (0, 0) = (xPrima)[0];
      transform (1, 0) = (xPrima)[1];
      transform (2, 0) = (xPrima)[2];
      transform (3, 1) = 0;
      transform (0, 1) = (yPrima)[0];
      transform (1, 1) = (yPrima)[1];
      transform (2, 1) = (yPrima)[2];
      transform (3, 2) = 0;
      transform (0, 2) = (zPrima)[0];
      transform (1, 2) = (zPrima)[1];
      transform (2, 2) = (zPrima)[2];
      transform (0, 3) = (centroid[0]);
      transform (1, 3) = (centroid[1]);
      transform (2, 3) = (centroid[2]);
      transform (3, 3) = 1;

      Eigen::Matrix4f inv;
      inv = transform.inverse ();
      transform = inv;
    }

    double
    polygonArea (pcl::PointCloud<pcl::PointXYZ> polygon)
    {
      //y is null
      double pos = 0, neg = 0;
      for (size_t i = 0; i < (polygon.points.size () - 1); i++)
      {
        pos += polygon.points[i].x * polygon.points[i + 1].z;
        neg += polygon.points[i].z * polygon.points[i + 1].x;
      }

      return fabs ((pos - neg) / 2.0);
    }

    void setThreshold(float thres) {
      THRESHOLD_ = thres;
    }

    void setVisualize(bool vis) {
      VISUALIZE_ = vis;
    }

    static bool getStablePlanesFromFile(std::vector<Plane> & stable_planes, std::string filename_for_cache_) {

      std::ifstream in;
      in.open (filename_for_cache_.c_str (), std::ifstream::in);

      if(in) {
        char linebuf[256];

        while(in.getline (linebuf, 256)) {
          std::string line (linebuf);
          std::vector < std::string > strs;
          boost::split (strs, line, boost::is_any_of (" "));

          Plane p;
          p.normal[0] = atof (strs[0].c_str ());
          p.normal[1] = atof (strs[1].c_str ());
          p.normal[2] = atof (strs[2].c_str ());

          p.center[0] = atof (strs[3].c_str ());
          p.center[1] = atof (strs[4].c_str ());
          p.center[2] = atof (strs[5].c_str ());

          stable_planes.push_back(p);
        }

        in.close ();

        return true;
      }

      return false;
    }

    static bool getStablePlanesFromCache(std::vector<Plane> & stable_planes, std::string filename_for_cache_) {
      std::stringstream path_to_file;
      path_to_file << "./stable_planes/" << filename_for_cache_ << ".txt";
      //std::string file = path_to_file.str();

      //read from file
      std::ifstream in;
      in.open (path_to_file.str ().c_str (), std::ifstream::in);

      if(in) {
        char linebuf[256];

        while(in.getline (linebuf, 256)) {
          std::string line (linebuf);
          std::vector < std::string > strs;
          boost::split (strs, line, boost::is_any_of (" "));

          Plane p;
          p.normal[0] = atof (strs[0].c_str ());
          p.normal[1] = atof (strs[1].c_str ());
          p.normal[2] = atof (strs[2].c_str ());

          p.center[0] = atof (strs[3].c_str ());
          p.center[1] = atof (strs[4].c_str ());
          p.center[2] = atof (strs[5].c_str ());

          stable_planes.push_back(p);
        }

        in.close ();

        return true;
      }

      return false;
    }

    void compute(std::vector<Plane> & stable_planes) {
      clusterPlanarFaces cpf;
      cpf.setInputData (cloud_out);

      if (polydata_ != NULL)
        cpf.setInputPolyData (polydata_);

      std::vector<Cluster *> initial_clusters;
      cpf.cluster (initial_clusters);

      Eigen::Vector3f CoM (0, 0, 0);
      cpf.getCoMFromMesh (CoM);

      //start with biggest cluster
      std::vector<Cluster *> candidates;
      initial_clusters[initial_clusters.size () - 1]->getCandidates (candidates, THRESHOLD_);
      //take the candidates and compute stable planes

      for (size_t i = 0; i < candidates.size (); i++)
      {
        Eigen::Vector3f normal, centroid;
        candidates[i]->getNormalAndCenter (normal, centroid);

        //transform input cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZ> ());
        Eigen::Matrix4f transform;
        transformOnPlane (centroid, normal, transform);
        pcl::transformPointCloud (*cloud_out, *transformed, transform);

        Eigen::Vector4f CoMTrans;
        pcl::compute3DCentroid (*transformed, CoMTrans);

        if (CoMTrans[1] < 0)
        {
          normal *= -1; //flip normal
          transformOnPlane (centroid, normal, transform);
          pcl::transformPointCloud (*cloud_out, *transformed, transform);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr support_points (new pcl::PointCloud<pcl::PointXYZ> ());
        candidates[i]->getPoints (support_points);

        //transform support points
        pcl::PointCloud<pcl::PointXYZ>::Ptr projected (new pcl::PointCloud<pcl::PointXYZ> ());
        projected->points.resize (support_points->points.size ());

        for (size_t k = 0; k < support_points->points.size (); k++)
        {
          projected->points[k].getVector4fMap () = transform * support_points->points[k].getVector4fMap ();
          projected->points[k].y = 0;
        }

        //check that centroid lays in the convex hull of the projection
        //convex hull projected points
        pcl::ConvexHull < pcl::PointXYZ > chull;
        chull.setInputCloud (projected);
        pcl::PointCloud < pcl::PointXYZ > hull;
        std::vector < pcl::Vertices > polygons;
        chull.reconstruct (hull, polygons);

        pcl::PointCloud < pcl::PointXYZ > polygon;
        polygon.points.resize (polygons[0].vertices.size ());
        for (size_t i = 0; i < polygons[0].vertices.size (); ++i)
        {
          polygon.points[i].x = hull.points[polygons[0].vertices[i]].x;
          polygon.points[i].z = hull.points[polygons[0].vertices[i]].z;
          polygon.points[i].y = 0;
        }

        Eigen::Vector4f CoM4f (CoM[0], CoM[1], CoM[2], 1);
        Eigen::Vector4f CoMTransformed = transform * CoM4f;
        pcl::PointXYZ PCLCoM;
        PCLCoM.x = CoMTransformed[0];
        PCLCoM.y = 0;
        PCLCoM.z = CoMTransformed[2];

        pcl::PointCloud<pcl::PointXYZ>::Ptr ComProjected (new pcl::PointCloud<pcl::PointXYZ> ());
        ComProjected->points.resize (1);
        ComProjected->points[0] = PCLCoM;

        if (pcl::isPointIn2DPolygon (PCLCoM, polygon))
        {
          //cloud_projected
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());
          cloud_projected->points.resize (transformed->points.size ());
          for (size_t k = 0; k < transformed->points.size (); k++)
          {
            cloud_projected->points[k].x = transformed->points[k].x;
            cloud_projected->points[k].z = transformed->points[k].z;
            cloud_projected->points[k].y = 0;
          }

          pcl::ConvexHull < pcl::PointXYZ > chull;
          chull.setInputCloud (cloud_projected);
          pcl::PointCloud < pcl::PointXYZ > hull;
          std::vector < pcl::Vertices > polygons_projected_cloud;
          chull.reconstruct (hull, polygons_projected_cloud);

          pcl::PointCloud < pcl::PointXYZ > polygon_projected_cloud;
          polygon_projected_cloud.points.resize (polygons_projected_cloud[0].vertices.size ());
          for (size_t k = 0; k < polygons_projected_cloud[0].vertices.size (); ++k)
          {
            polygon_projected_cloud.points[k].x = hull.points[polygons_projected_cloud[0].vertices[k]].x;
            polygon_projected_cloud.points[k].z = hull.points[polygons_projected_cloud[0].vertices[k]].z;
            polygon_projected_cloud.points[k].y = 0;
          }

          //create planes structure (center, normal, area, symmetry attributes)
          Plane p;

          for(size_t l=0; l < 3; l++)
        	  p.center[l] = centroid[l];

          for(size_t l=0; l < 3; l++)
        	  p.normal[l] = normal[l];

          p.area = polygonArea (polygon);
          p.areaRatio = p.area / polygonArea (polygon_projected_cloud);
          p.setSupportPoints (support_points);
          stable_planes.push_back (p);

        }
        else
        {
          continue;
        }
      }

      std::sort (stable_planes.begin (), stable_planes.end (), sortPlanesByAreaRatio);

      //TODO: filter planes where the object is the same! by doing point cloud intersection
      //we need to compute the rotation on the plane to align both

      pcl::visualization::PCLVisualizer vis2 ("On plane visualizer");
      vis2.setBackgroundColor (1, 1, 1);

      std::vector<Plane> stable_planes_filtered;
      for (size_t i = 0; i < stable_planes.size (); i++)
      {

        Eigen::Vector3f normal, centroid;
        normal = Eigen::Vector3f(stable_planes[i].normal[0],stable_planes[i].normal[1],stable_planes[i].normal[2]);
        centroid = Eigen::Vector3f(stable_planes[i].center[0],stable_planes[i].center[1],stable_planes[i].center[2]);

        //transform input cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZ> ());
        Eigen::Matrix4f transform;
        transformOnPlane (centroid, normal, transform);
        pcl::transformPointCloud (*cloud_out, *transformed, transform);

        Eigen::Vector4f center;
        pcl::compute3DCentroid(*transformed, center);
        center[1] = 0;
        pcl::demeanPointCloud(*transformed, center, *transformed);

        //project points on plane...
        pcl::PointCloud<pcl::PointXYZ>::Ptr projected_i (new pcl::PointCloud<pcl::PointXYZ> ());
        projected_i->points.resize (transformed->points.size ());

        for (size_t k = 0; k < projected_i->points.size (); k++)
        {
          projected_i->points[k].getVector4fMap () = transformed->points[k].getVector4fMap ();
          projected_i->points[k].y = 0;
        }

        EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
        Eigen::Vector4f xyz_centroid;
        compute3DCentroid (*projected_i, xyz_centroid);
        computeCovarianceMatrix (*projected_i, xyz_centroid, covariance_matrix);
        EIGEN_ALIGN16 Eigen::Vector3f eigen_values_i;
        EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors_i;
        pcl::eigen33 (covariance_matrix, eigen_vectors_i, eigen_values_i);

        bool found=false;
        for (size_t j = 0; j < stable_planes_filtered.size (); j++) {

          Eigen::Vector3f normal, centroid;
          normal = Eigen::Vector3f(stable_planes[j].normal[0],stable_planes[j].normal[1],stable_planes[j].normal[2]);
          centroid = Eigen::Vector3f(stable_planes[j].center[0],stable_planes[j].center[1],stable_planes[j].center[2]);

          //transform input cloud
          pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_j (new pcl::PointCloud<pcl::PointXYZ> ());
          Eigen::Matrix4f transform;
          transformOnPlane (centroid, normal, transform);
          pcl::transformPointCloud (*cloud_out, *transformed_j, transform);

          Eigen::Vector4f center;
          pcl::compute3DCentroid(*transformed_j, center);
          center[1] = 0;
          pcl::demeanPointCloud(*transformed_j, center, *transformed_j);

          //align on plane
          // * compute PCA of the projected points on the plane
          // * check eigenvalues
          // * if they match, align

          //project points on plane...
          pcl::PointCloud<pcl::PointXYZ>::Ptr projected_j (new pcl::PointCloud<pcl::PointXYZ> ());
          projected_j->points.resize (transformed_j->points.size ());

          for (size_t k = 0; k < projected_j->points.size (); k++)
          {
            projected_j->points[k].getVector4fMap () = transformed_j->points[k].getVector4fMap ();
            projected_j->points[k].y = 0;
          }

          EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
          Eigen::Vector4f xyz_centroid;
          compute3DCentroid (*projected_j, xyz_centroid);
          computeCovarianceMatrix (*projected_j, xyz_centroid, covariance_matrix);
          EIGEN_ALIGN16 Eigen::Vector3f eigen_values_j;
          EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors_j;
          pcl::eigen33 (covariance_matrix, eigen_vectors_j, eigen_values_j);

          //std::cout << "Eigen values i:" << eigen_values_i[0] << " " << eigen_values_i[1] << " " << eigen_values_i[2] << std::endl;
          //std::cout << "Eigen values j:" << eigen_values_j[0] << " " << eigen_values_j[1] << " " << eigen_values_j[2] << std::endl;

          double eigen_dist = sqrt((eigen_values_i[0] - eigen_values_j[0]) * (eigen_values_i[0] - eigen_values_j[0]) +
                              (eigen_values_i[1] - eigen_values_j[1]) * (eigen_values_i[1] - eigen_values_j[1]) +
                              (eigen_values_i[2] - eigen_values_j[2]) * (eigen_values_i[2] - eigen_values_j[2]));

          if (eigen_dist > 10) //the eigen values are different, alignment wont work..
            continue;

          //std::cout << eigen_vectors_j.col(2) << std::endl;
          //std::cout << eigen_vectors_i.col(2) << std::endl;

          //std::cout << acos(eigen_vectors_j.col(2).dot(eigen_vectors_i.col(2))) * 180 / 3.1416 << std::endl;

          std::vector<Eigen::Affine3f> combinations;
          Eigen::Affine3f m;
          m = Eigen::AngleAxisf(acos(eigen_vectors_j.col(2).dot(eigen_vectors_i.col(2))), Eigen::Vector3f::UnitY());
          combinations.push_back(m);
          m = Eigen::AngleAxisf(-acos(eigen_vectors_j.col(2).dot(eigen_vectors_i.col(2))), Eigen::Vector3f::UnitY());
          combinations.push_back(m);
          m = Eigen::AngleAxisf(acos(eigen_vectors_j.col(2).dot(eigen_vectors_i.col(2)*-1)), Eigen::Vector3f::UnitY());
          combinations.push_back(m);
          m = Eigen::AngleAxisf(-acos(eigen_vectors_j.col(2).dot(eigen_vectors_i.col(2)*-1)), Eigen::Vector3f::UnitY());
          combinations.push_back(m);

          eigen_vectors_j.col(2) *= -1;

          m = Eigen::AngleAxisf(acos(eigen_vectors_j.col(2).dot(eigen_vectors_i.col(2))), Eigen::Vector3f::UnitY());
          combinations.push_back(m);
          m = Eigen::AngleAxisf(-acos(eigen_vectors_j.col(2).dot(eigen_vectors_i.col(2))), Eigen::Vector3f::UnitY());
          combinations.push_back(m);
          m = Eigen::AngleAxisf(acos(eigen_vectors_j.col(2).dot(eigen_vectors_i.col(2)*-1)), Eigen::Vector3f::UnitY());
          combinations.push_back(m);
          m = Eigen::AngleAxisf(-acos(eigen_vectors_j.col(2).dot(eigen_vectors_i.col(2)*-1)), Eigen::Vector3f::UnitY());
          combinations.push_back(m);

          pcl::SegmentDifferences < pcl::PointXYZ > seg;
          typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;
          KdTreePtr normals_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > (true);
          pcl::PointCloud<pcl::PointXYZ>::Ptr difference (new pcl::PointCloud<pcl::PointXYZ> ());

          seg.setDistanceThreshold (threshold_non_inf_ * threshold_non_inf_);
          seg.setSearchMethod (normals_tree);
          seg.setInputCloud (transformed);

          for(size_t t=0; t < combinations.size() && !found; t++) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_j_minus_angle (new pcl::PointCloud<pcl::PointXYZ> ());
            pcl::transformPointCloud (*transformed_j, *transformed_j_minus_angle, combinations[t]);

            seg.setTargetCloud (transformed_j_minus_angle);
            seg.segment (*difference);
            double outliers_minus =  difference->points.size ();
            if (outliers_minus == 0) {
              found = true;
            }
          }

          if(found)
            break;

          /*pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_j_plus_angle (new pcl::PointCloud<pcl::PointXYZ> ());
          pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_j_minus_angle (new pcl::PointCloud<pcl::PointXYZ> ());

          pcl::transformPointCloud (*transformed_j, *transformed_j_plus_angle, m1);
          pcl::transformPointCloud (*transformed_j, *transformed_j_minus_angle, m2);

          //compute number of outliers after alignment...


          seg.setTargetCloud (transformed_j_plus_angle);

          pcl::PointCloud<pcl::PointXYZ>::Ptr difference (new pcl::PointCloud<pcl::PointXYZ> ());
          seg.segment (*difference);
          //std::cout << "Number of outliers (plus):" << difference->points.size () << std::endl;
          double outliers_plus =  difference->points.size ();

          seg.setInputCloud (transformed);
          seg.setTargetCloud (transformed_j_minus_angle);

          seg.segment (*difference);
          //std::cout << "Number of outliers (pminus):" << difference->points.size () << std::endl;

          double outliers_minus =  difference->points.size ();

          std::cout << outliers_minus << " " << outliers_plus << std::endl;
          if(outliers_minus < 0 || outliers_plus == 0) {
            found = true;
            break;
          }*/

          /*pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > handler_whole_pc (transformed, 0, 0, 125);
          vis2.addPointCloud (transformed,handler_whole_pc, "transformed");

          pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > handler_whole_pc_j (transformed_j_minus_angle, 125, 0, 0);
          vis2.addPointCloud (transformed_j_minus_angle, handler_whole_pc_j, "transformed_j_minus");

          pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > handler_whole_pc_j_2 (transformed_j_plus_angle, 0, 125, 0);
          vis2.addPointCloud (transformed_j_plus_angle, handler_whole_pc_j_2, "transformed_j_plus");

          vis2.addCoordinateSystem (0.1, 0);
          vis2.spin ();
          vis2.removePointCloud ("transformed_j_minus");
          vis2.removePointCloud ("transformed_j_plus");
          vis2.removePointCloud ("transformed");
          vis2.resetCamera ();*/
        }

        if(found) {
          //there is a stable in stable_planes_filtered on which the object is the same as on i,
          //just ignore this plane
        } else {
          std::cout << "Plane not found, adding to filtered" << std::endl;
          stable_planes_filtered.push_back(stable_planes[i]);
        }
      }

      stable_planes = stable_planes_filtered;
      std::cout << "Number of stable planes:" << stable_planes.size() << std::endl;
      //return final stable planes!

      boost::filesystem::path root_path = "./stable_planes";
      if (!boost::filesystem::exists (root_path))
      {
        boost::filesystem::create_directory (root_path);
      }

      std::stringstream path_to_file;
      path_to_file << "./stable_planes/" << filename_for_cache_ << ".txt";
      std::string file = path_to_file.str();

      std::cout << "Writting stable planes to:" << file.c_str() << std::endl;

      ofstream out (file.c_str ());
      if (!out)
      {
        cout << "Cannot open file.\n";
      } else {
        for (size_t i = 0; i < stable_planes.size (); i++)
        {
          //write stable planes to file!
          out << stable_planes[i].normal[0] << " " << stable_planes[i].normal[1] << " " << stable_planes[i].normal[2]
              << " " << stable_planes[i].center[0] << " " << stable_planes[i].center[1] << " " << stable_planes[i].center[2] << endl;
        }

        out.close ();

      }

      if (VISUALIZE_) {
        pcl::visualization::PCLVisualizer vis2 ("On plane visualizer");
        vis2.setBackgroundColor (1, 1, 1);

        for (size_t i = 0; i < stable_planes.size (); i++)
        {
          Eigen::Vector3f normal, centroid;
          normal = Eigen::Vector3f(stable_planes[i].normal[0],stable_planes[i].normal[1],stable_planes[i].normal[2]);
          centroid = Eigen::Vector3f(stable_planes[i].center[0],stable_planes[i].center[1],stable_planes[i].center[2]);

          std::cout << "Area:" << stable_planes[i].area << std::endl;
          std::cout << "Area_ratio:" << stable_planes[i].areaRatio << std::endl;

          //transform input cloud
          pcl::PointCloud<pcl::PointXYZ>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZ> ());
          Eigen::Matrix4f transform;
          transformOnPlane (centroid, normal, transform);
          pcl::transformPointCloud (*cloud_out, *transformed, transform);

          pcl::PointCloud < pcl::PointXYZ >::Ptr support_points = stable_planes[i].support_points_;
          pcl::PointCloud<pcl::PointXYZ>::Ptr projected (new pcl::PointCloud<pcl::PointXYZ> ());
          projected->points.resize (support_points->points.size ());

          for (size_t k = 0; k < support_points->points.size (); k++)
          {
            projected->points[k].getVector4fMap () = transform * support_points->points[k].getVector4fMap ();
            projected->points[k].y = 0;
          }

          //check that centroid lays in the convex hull of the projection
          //convex hull projected points
          pcl::ConvexHull < pcl::PointXYZ > chull;
          chull.setInputCloud (projected);
          pcl::PointCloud < pcl::PointXYZ > hull;
          std::vector < pcl::Vertices > polygons;
          chull.reconstruct (hull, polygons);

          pcl::PointCloud < pcl::PointXYZ > polygon;
          polygon.points.resize (polygons[0].vertices.size ());
          for (size_t i = 0; i < polygons[0].vertices.size (); ++i)
          {
            polygon.points[i].x = hull.points[polygons[0].vertices[i]].x;
            polygon.points[i].z = hull.points[polygons[0].vertices[i]].z;
            polygon.points[i].y = 0;
          }

          sensor_msgs::PointCloud2 msg;
          pcl::toROSMsg (hull, msg);
          pcl::PolygonMesh mesh;
          mesh.cloud = msg;
          mesh.polygons = polygons;

          pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > handler_whole_pc (transformed, 0, 0, 125);
          vis2.addPointCloud (transformed,handler_whole_pc, "transformed");
          vis2.addPointCloud (projected, "projected");
          vis2.addPolygonMesh (mesh, "mesh");
          vis2.addCoordinateSystem (0.1, 0);
          vis2.spin ();
          vis2.removePointCloud ("projected");
          vis2.removePointCloud ("transformed");
          vis2.removeShape ("mesh");
          vis2.resetCamera ();
        }
      }
    }
};

#endif
