/*
 * cvfh_trainer.h
 *
 *  Created on: May 9, 2011
 *      Author: aldoma
 */

#ifndef TRAINER_H_
#define TRAINER_H_

#include <string>
#include <vector>
#include <pcl/common/common.h>
#include <cv.h>
#include "model.h"
#include "estimators/estimator.h"
#include "trainers/trainer.h"

namespace bf = boost::filesystem;

/* This class should take care of training the CVFH descriptor using
 * PLY files. Take care of generating views, compute descriptors, make data persistence
 * ,filtering duplicate views,etc.
 */

//template<typename PointInT, typename PointOutT, typename FeatureT, class EstimatorT>
template<typename PointInT, typename PointOutT, typename FeatureT>
class Trainer
{
private:

  typedef Estimator<PointInT,PointOutT,FeatureT> EstimatorT;
  //EstimatorT * estimator_;
  boost::shared_ptr<EstimatorT> estimator_;

  /** \brief Directory where the PLY models are found **/
  std::string MODEL_FILES_DIR_;

  /** \brief Directory where the data will be persistent **/
  std::string TRAINING_DIR_;

  /** \brief Sphere tesselation level. How many views per model are going to be generated. **/
  int tesselated_sphere_level_;

  /** \brief 1 if the PLY files are in meters, 0.001 for mm and so on. **/
  double model_scale_;

  /** \brief Resolution in pixels for the generated views, default=128 **/
  int resolution_;

  /** \brief Wether the input cloud needs to be voxel grided, default=true **/
  bool apply_voxel_grid_;

  /** \brief Wether if the input cloud needs to be filtered, default=true **/
  bool apply_radius_removal_;

  /** \brief Voxel grid size, default=5mm **/
  double leaf_size_;

  /** \brief Whether the views from the models should be noisified or not **/
  bool noisify_;

  /** \brief Amount of noise in z-direction, default=1mm **/
  double noise_;

  /** \brief Radius of the sphere where the cameras will be placed, default=1m **/
  float radius_virtual_camera_;

  /** \brief FOV of the virtual camera, default=45 deg **/
  float fov_virtual_camera_;

  /** \brief Wether to use vertices or faces on the tesselated sphere for virtual rendering.  default=true **/
  bool use_vertices_;

  /** \brief Generate the views for a given model.
   *  \param PLYModel - path to the file
   *  \param views - pointclouds with the partial views from different viewpoints. They are in "camera" coordinates.
   *  \param poses - pose from Object coordinates to Camera Coordinates.
   */

  void
  generate_views_for_model (std::string PLYModel, std::vector<pcl::PointCloud<PointInT>, Eigen::aligned_allocator<
      pcl::PointCloud<PointInT> > > & views, std::vector<Eigen::Matrix4f,
      Eigen::aligned_allocator<Eigen::Matrix4f> > & poses, std::vector<float> & enthropies);

protected:

  /** \brief Whether duplicate views should be filtered or not, default=true **/
  bool remove_duplicate_views_;

  /** \brief Name of the descriptor for naming the appropriate directory structure **/
  std::string D_NAME_;

  virtual void specificTraining(boost::shared_ptr<EstimatorT> & estimator, Model<View, PointOutT, FeatureT> & model, int pos) {
    //std::cout << "Nothing to be done for specific training" << std::endl;
  }
  virtual void specificPersistence(int i, std::string dir, boost::shared_ptr<View<PointOutT, FeatureT> > view) {
    //std::cout << "Nothing to be done for specific persistence" << std::endl;
  }

  virtual void removeDuplicateViews (boost::shared_ptr<EstimatorT> & estimator, Model<View, PointOutT, FeatureT> & model, Model<View, PointOutT, FeatureT> & filtered) {
    //std::cout << "Nothing to be done for removeDuplicateViews" << std::endl;
  }

public:
  Trainer (std::string files_dir, std::string train_dir);
  ~Trainer ();

  void incrementalTraining (std::vector<std::string> & files);

  void
  makePersistent (Model<View, PointOutT, FeatureT> & model);

  /*void
  setEstimator(EstimatorT * estimator) {
    estimator_ = estimator;
  }*/

  void
  setEstimator(boost::shared_ptr<EstimatorT> & estimator) {
    estimator_ = estimator;
  }

  static void
  getModelsInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths);


  //void
  //removeDuplicateViews (Model<View, PointOutT, FeatureT> & model, Model<View, PointOutT, FeatureT> & filtered);

  /** \brief Performs training for the models in MODEL_FILES_DIR_. Generate views,
   * compute CVFH, compute roll histograms, filters duplicate views and makes
   * the data persistent.
   */

  void
  train ();

  //Attribute setters
  void
  setTesselationLevel (int level)
  {
    tesselated_sphere_level_ = level;
  }

  void
  setResolution (int resolution)
  {
    resolution_ = resolution;
  }

  void
  setModelScale (double scale)
  {
    model_scale_ = scale;
  }

  void
  setDescriptorName (std::string dname)
  {
    D_NAME_ = dname;
  }

  void setRadiusVirtualcamera(float r) {
    radius_virtual_camera_ = r;
  }

  void setFOVVirtualCamera(float fov) {
      fov_virtual_camera_ = fov;
    }

  void
  setNosify(bool apply) {
    noisify_ = apply;
  }

  void
  setNoiseLevel(float noise) {
    noise_ = noise;
  }

  void
  setUsevertices(bool use) {
    use_vertices_ = use;
  }
};

#endif /* TRAINER_H_ */
