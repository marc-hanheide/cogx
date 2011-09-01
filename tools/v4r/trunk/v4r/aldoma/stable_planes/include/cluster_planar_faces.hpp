/*
 * cluster_planar_faces.hpp
 *
 *  Created on: Apr 12, 2011
 *      Author: aa
 */

#include <string>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <map>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vtkTriangle.h>

inline std::pair<int, int>
makeOrderedPair (int p1, int p2)
{
  if (p1 < p2)
  {
    return std::make_pair (p1, p2);
  }
  else
  {
    return std::make_pair (p2, p1);
  }
}

class Cluster
{
private:
  int idx_;
  std::vector<Cluster *> children_;
  pcl::PointCloud<pcl::PointXYZ> points_;
  std::vector<int> edges_ids_;

public:

  std::vector<Cluster *> neighbors_;
  double error_;
  double area_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Cluster (int idx, std::vector<unsigned int> & vertices, pcl::PointCloud<pcl::PointXYZ> & hull)
  {
    idx_ = idx;
    points_.points.resize (3);
    points_.width = 3;
    points_.height = 1;

    points_.points[0].getVector4fMap () = hull.points[vertices[0]].getVector4fMap ();
    points_.points[1].getVector4fMap () = hull.points[vertices[1]].getVector4fMap ();
    points_.points[2].getVector4fMap () = hull.points[vertices[2]].getVector4fMap ();
    error_ = 0;
  }

  Cluster (Cluster * c1, Cluster * c2)
  {
    children_.push_back (c1);
    children_.push_back (c2);

    for (size_t i = 0; i < c1->neighbors_.size (); i++)
    {

      if (c1->neighbors_[i] != c2)
      {
        addNeighbor (c1->neighbors_[i]); //add neighbor to current cluster
        (c1->neighbors_[i])->addNeighbor (this); //add current cluster to neighbor id different than c2
      }
    }

    for (size_t i = 0; i < c2->neighbors_.size (); i++)
    {
      if (c2->neighbors_[i] != c1)
      {
        addNeighbor (c2->neighbors_[i]); //add neighbor to current cluster
        (c2->neighbors_[i])->addNeighbor (this); //add current cluster to neighbor
      }
    }

    //after updating neighbors, remove c1 and c2
    c1->removeSelfFromNeighbors ();
    c2->removeSelfFromNeighbors ();

    //set points to concatenation of both clusters
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_a (new pcl::PointCloud<pcl::PointXYZ> ()),
                                        points_b (new pcl::PointCloud<pcl::PointXYZ> ());
    c1->getPoints (points_a);
    c2->getPoints (points_b);

    points_ = *points_a;
    points_ += *points_b;

    //remove duplicate points
    /*pcl::VoxelGrid < pcl::PointXYZ > sor;
    sor.setInputCloud (points_.makeShared ());
    sor.setLeafSize (0.0001, 0.0001, 0.0001);
    sor.filter (points_);*/

    idx_ = -1;
  }

  ~Cluster ()
  {

  }
  ;

  void
  addNeighbor (Cluster * neighbor)
  {
    neighbors_.push_back (neighbor);
  }

  void
  addEdgeId (int id)
  {
    edges_ids_.push_back (id);
  }

  void
  getEdgesIds (std::vector<int> & ids)
  {
    ids.resize (edges_ids_.size ());
    for (size_t i = 0; i < edges_ids_.size (); i++)
    {
      ids[i] = edges_ids_[i];
    }

  }

  void
  removeClusterFromNeighbors (Cluster * to_remove)
  {
    std::vector<Cluster *>::iterator it;
    for (it = neighbors_.begin (); it != neighbors_.end (); it++)
    {
      if ((*it) == to_remove)
      {
        neighbors_.erase (it);
        break;
      }
    }
  }

  void
  removeSelfFromNeighbors ()
  {
    for (size_t i = 0; i < neighbors_.size (); i++)
    {
      neighbors_[i]->removeClusterFromNeighbors (this);
    }
  }

  int
  getId ()
  {
    return idx_;
  }

  void
  getPoints (pcl::PointCloud<pcl::PointXYZ>::Ptr ps)
  {
    *ps = points_;
  }

  void
  getCandidates (std::vector<Cluster *> & candidates, float threshold)
  {
    if (error_ < threshold)
    {
      candidates.push_back (this);
    }
    else
    {
      children_[0]->getCandidates (candidates, threshold);
      children_[1]->getCandidates (candidates, threshold);
    }
  }

  void
  getNormalAndCenter (Eigen::Vector3f & normal, Eigen::Vector3f & centroid)
  {
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    Eigen::Vector4f xyz_centroid;
    compute3DCentroid (points_, xyz_centroid);
    computeCovarianceMatrix (points_, xyz_centroid, covariance_matrix);
    EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
    EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
    pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

    normal = Eigen::Vector3f (eigen_vectors (0, 0), eigen_vectors (1, 0), eigen_vectors (2, 0));
    centroid = Eigen::Vector3f (xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
  }
};

class DualEdge
{
private:
  double contract_error_;
  int id_;

  double
  ComputeContractionError ()
  {
    //get points in a_ and b_ and fit a plane...
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_a (new pcl::PointCloud<pcl::PointXYZ> ()),
                                        points_b (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ> ());
    a_->getPoints (points_a);
    b_->getPoints (points_b);

    points->points.resize (points_a->points.size () + points_b->points.size ());
    int pidx = 0;
    for (size_t i = 0; i < points_a->points.size (); i++, pidx++)
      points->points[pidx].getVector4fMap () = points_a->points[i].getVector4fMap ();

    for (size_t i = 0; i < points_b->points.size (); i++, pidx++)
      points->points[pidx].getVector4fMap () = points_b->points[i].getVector4fMap ();

    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    Eigen::Vector4f xyz_centroid;
    compute3DCentroid (*points, xyz_centroid);
    computeCovarianceMatrix (*points, xyz_centroid, covariance_matrix);
    EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
    EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
    pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

    //use smallest eigen value
    return eigen_values (0);
  }

public:

  Cluster * a_;
  Cluster * b_;

  DualEdge (Cluster * a, Cluster * b, int id)
  {
    a_ = a;
    b_ = b;
    contract_error_ = ComputeContractionError ();
    id_ = id;
  }

  int
  getId ()
  {
    return id_;
  }

  double
  getContractError ()
  {
    return contract_error_;
  }
};

class MutablePriorityQueue
{
  //sorted vector with edges_id and priority queue
private:

  class compareEdges
  {
  public:
    compareEdges ()
    {
    }
    ;

    bool
    operator() (DualEdge * lhs, DualEdge * rhs) const
    {
      return (lhs->getContractError () > rhs->getContractError ());
    }
  };

  std::priority_queue<DualEdge *, std::vector<DualEdge *>, compareEdges> dual_edges_;
  std::map<int, bool> valid_ids_;

public:
  MutablePriorityQueue ()
  {

  }

  void
  push (DualEdge * edge)
  {
    dual_edges_.push (edge);
    valid_ids_[edge->getId ()] = true;
  }

  void
  updateId (int edgeId, bool val)
  {
    valid_ids_[edgeId] = val;
  }

  DualEdge *
  topAndPop ()
  {
    DualEdge * lowest_edge = NULL;
    do
    {
      lowest_edge = dual_edges_.top ();
      dual_edges_.pop ();
    } while (!dual_edges_.empty () && !valid_ids_[lowest_edge->getId ()]); //do until queue is empty or a valid id is found

    return lowest_edge;
  }

  bool
  empty ()
  {
    return dual_edges_.empty ();
  }

  int
  size ()
  {
    return dual_edges_.size ();
  }
};

inline void
computeCoM (vtkSmartPointer<vtkPolyData> polydata, double * CoM)
{
  vtkIdType npts = 0, *ptIds = NULL;

  vtkSmartPointer < vtkCellArray > cells = polydata->GetPolys ();

  double center[3], p1[3], p2[3], p3[3], area, totalArea = 0;
  double comx = 0, comy = 0, comz = 0;
  for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds);)
  {
    polydata->GetPoint (ptIds[0], p1);
    polydata->GetPoint (ptIds[1], p2);
    polydata->GetPoint (ptIds[2], p3);
    vtkTriangle::TriangleCenter (p1, p2, p3, center);
    area = vtkTriangle::TriangleArea (p1, p2, p3);
    comx += center[0] * area;
    comy += center[1] * area;
    comz += center[2] * area;
    totalArea += area;
  }

  CoM[0] = comx / totalArea;
  CoM[1] = comy / totalArea;
  CoM[2] = comz / totalArea;
}

class clusterPlanarFaces
{

private:
  int edgeId;
  vtkSmartPointer<vtkPolyData> polydata_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;
  Eigen::Vector3f CoM;

public:

  clusterPlanarFaces() {
    polydata_ = NULL;
  }

  void
  getCoMFromMesh (Eigen::Vector3f & CoMOut)
  {
    CoMOut[0] = CoM[0];
    CoMOut[1] = CoM[1];
    CoMOut[2] = CoM[2];
  }

  void
  setInputData (pcl::PointCloud<pcl::PointXYZ>::Ptr data)
  {
    cloud_out = data;
  }

  void
  setInputPolyData (vtkSmartPointer<vtkPolyData> polydata)
  {
    polydata_ = polydata;
  }

  void
  cluster (std::vector<Cluster *> & initial_clusters)
  {
    if (polydata_ != NULL) {
      double CoM_t[3];
      computeCoM (polydata_, CoM_t);
      CoM = Eigen::Vector3f (CoM_t[0], CoM_t[1], CoM_t[2]);
    } else {
      //use point cloud to compute CoM
      Eigen::Vector4f CoM4;
      pcl::compute3DCentroid(*cloud_out, CoM4);
      CoM = Eigen::Vector3f (CoM4[0], CoM4[1], CoM4[2]);
    }



    //compute convex hull and visualize
    pcl::PointCloud < pcl::PointXYZ > hull;
    std::vector < pcl::Vertices > polygons;
    pcl::ConvexHull < pcl::PointXYZ > chull;
    chull.setInputCloud (cloud_out);
    chull.reconstruct (hull, polygons);

    /*sensor_msgs::PointCloud2 msg_alpha;
    pcl::toROSMsg (hull, msg_alpha);

    pcl::PolygonMesh mesh;
    mesh.cloud = msg_alpha;
    mesh.polygons = polygons;

    pcl::visualization::PCLVisualizer vis ("Sampled visualizer");
    vis.addPointCloud (cloud_out);
    vis.addPolygonMesh (mesh);
    vis.spin ();*/

    //build edges from polygons
    std::map < std::pair<int, int>, std::vector<int> > edges;

    for (size_t i = 0; i < polygons.size (); i++)
    {
      edges[makeOrderedPair (polygons[i].vertices[0], polygons[i].vertices[1])].push_back (i);
      edges[makeOrderedPair (polygons[i].vertices[1], polygons[i].vertices[2])].push_back (i);
      edges[makeOrderedPair (polygons[i].vertices[2], polygons[i].vertices[0])].push_back (i);
      //initial_clusters[i] = new Cluster((int)i, polygons[i].vertices, hull);
      initial_clusters.push_back (new Cluster ((int)i, polygons[i].vertices, hull));
    }

    std::cout << "Number of triangles:" << polygons.size () << std::endl;
    std::cout << "Number of edges:" << edges.size () << " " << (polygons.size () / 2) * 3 << std::endl;

    //compute dual graph and initial clusters
    std::map<std::pair<int, int>, std::vector<int> >::iterator it;
    std::vector<DualEdge *> dual_edges;

    MutablePriorityQueue pq;

    //compute contraction costs for each edge and create priority queue
    for (it = edges.begin (); it != edges.end (); it++)
    {
      //cout << (*it).first.first << "," << (*it).first.second << " => " << (*it).second.size() << endl;
      initial_clusters[(*it).second[0]]->addNeighbor (initial_clusters[(*it).second[1]]);
      initial_clusters[(*it).second[1]]->addNeighbor (initial_clusters[(*it).second[0]]);
      DualEdge * de = new DualEdge (initial_clusters[(*it).second[0]], initial_clusters[(*it).second[1]], edgeId);
      dual_edges.push_back (de);

      //push edge to priority queue
      pq.push (de);
      //add edges id to clusters
      initial_clusters[(*it).second[0]]->addEdgeId (edgeId);
      initial_clusters[(*it).second[1]]->addEdgeId (edgeId);

      edgeId++;
    }

    //start contracting edges and building cluster hierarchy
    while (!pq.empty ())
    {
      DualEdge * contracted = pq.topAndPop ();
      if (contracted == NULL)
        break;

      //update priority queue by removing the edges from valid_ids_ between this two clusters and between the neighbors...
      //need access to the edges that need to be deleted
      std::vector<int> edgesA, edgesB;
      contracted->a_->getEdgesIds (edgesA);
      contracted->b_->getEdgesIds (edgesB);

      for (size_t i = 0; i < edgesA.size (); i++)
        pq.updateId (edgesA[i], false);

      for (size_t i = 0; i < edgesB.size (); i++)
        pq.updateId (edgesB[i], false);

      //create new cluster
      //the neighbors from c are the neighbors from a_ and b_
      Cluster * c = new Cluster (contracted->a_, contracted->b_);
      c->error_ = contracted->getContractError ();
      initial_clusters.push_back (c);

      //create new edges and add them to priority queue
      for (size_t i = 0; i < c->neighbors_.size (); i++)
      {
        DualEdge * de = new DualEdge (c, c->neighbors_[i], edgeId);
        pq.push (de);
        c->addEdgeId (edgeId);
        c->neighbors_[i]->addEdgeId (edgeId);
        edgeId++;
      }

    }
  }
};

