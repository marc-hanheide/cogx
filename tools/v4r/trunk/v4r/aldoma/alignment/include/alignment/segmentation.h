typedef pcl::PointXYZ Point;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

void
segmentation (const sensor_msgs::PointCloud2ConstPtr& cloud2_in, std::vector<pcl::PointCloud<Point>,Eigen::aligned_allocator<pcl::PointCloud<Point> > > & clusters, Eigen::Vector4f & table_coeffs)
{
  KdTreePtr normals_tree_, clusters_tree_;
  pcl::PassThrough<Point> pass_;
  pcl::VoxelGrid<Point> grid_, grid_objects_;
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  pcl::ProjectInliers<Point> proj_;
  pcl::ProjectInliers<Point> bb_cluster_proj_;
  pcl::ConvexHull<Point> hull_;
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::EuclideanClusterExtraction<Point> cluster_;

  double downsample_leaf_, downsample_leaf_objects_;
  int k_;
  double min_z_bounds_, max_z_bounds_;
  double sac_distance_threshold_;
  double normal_distance_weight_;

  // Min/Max height from the table plane object points will be considered from/to
  double object_min_height_, object_max_height_;

  // Object cluster tolerance and minimum cluster size
  double object_cluster_tolerance_, object_cluster_min_size_;

  // The raw, input point cloud data
  pcl::PointCloud<Point>::ConstPtr cloud_;
  // The filtered and downsampled point cloud data
  pcl::PointCloud<Point>::ConstPtr cloud_filtered_, cloud_downsampled_;
  // The resultant estimated point cloud normals for \a cloud_filtered_
  pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;
  // The vector of indices from cloud_filtered_ that represent the planar table component
  pcl::PointIndices::ConstPtr table_inliers_;
  // The model coefficients of the planar table component
  pcl::ModelCoefficients::ConstPtr table_coefficients_;
  // The set of point inliers projected on the planar table component from \a cloud_filtered_
  pcl::PointCloud<Point>::ConstPtr table_projected_;
  // The convex hull of \a table_projected_
  pcl::PointCloud<Point>::ConstPtr table_hull_;
  // The remaining of the \a cloud_filtered_ which lies inside the \a table_hull_ polygon
  pcl::PointCloud<Point>::ConstPtr cloud_objects_;
  // the single clusters
  pcl::PointCloud<Point>::ConstPtr cluster_object_;

  // Filtering parameters
  downsample_leaf_ = 0.01; // 1cm voxel size by default
  downsample_leaf_objects_ = 0.01; // 3mm voxel size by default
  grid_.setLeafSize (downsample_leaf_, downsample_leaf_, downsample_leaf_);
  grid_objects_.setLeafSize (downsample_leaf_objects_, downsample_leaf_objects_, downsample_leaf_objects_);
  grid_.setFilterFieldName ("z");
  pass_.setFilterFieldName ("z");

  min_z_bounds_ = 0.4; // restrict the Z dimension between 0.4m
  max_z_bounds_ = 1.25; // and 1.6m
  grid_.setFilterLimits (min_z_bounds_, max_z_bounds_);
  pass_.setFilterLimits (min_z_bounds_, max_z_bounds_);
  grid_.setDownsampleAllData (false);
  grid_objects_.setDownsampleAllData (false);

  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
  clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
  clusters_tree_->setEpsilon (1);
  //tree_.setSearchWindowAsK (10);
  //tree_.setMaxDistance (0.5);

  // Normal estimation parameters
  k_ = 10; // 50 k-neighbors by default
  n3d_.setKSearch (k_);
  //n3d_.setRadiusSearch (0.015);
  n3d_.setSearchMethod (normals_tree_);

  // Table model fitting parameters
  sac_distance_threshold_ = 0.01; // 5cm
  seg_.setDistanceThreshold (sac_distance_threshold_);
  seg_.setMaxIterations (2000);

  normal_distance_weight_ = 0.1;
  seg_.setNormalDistanceWeight (normal_distance_weight_);
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setProbability (0.99);

  proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  bb_cluster_proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);

  // Consider objects starting at 5mm from the table and ending at 0.xm
  object_min_height_ = 0.0075;
  object_max_height_ = 0.7;
  prism_.setHeightLimits (object_min_height_, object_max_height_);

  // Clustering parameters
  object_cluster_tolerance_ = 0.1; // 5cm between two objects
  object_cluster_min_size_ = 1000; // 100 points per object cluster
  cluster_.setClusterTolerance (object_cluster_tolerance_);
  cluster_.setMinClusterSize (object_cluster_min_size_);
  cluster_.setSearchMethod (clusters_tree_);

  pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point> ());
  pcl::fromROSMsg (*cloud2_in, *cloud);
  cloud_ = cloud;

  // ---[ PassThroughFilter       <-- cloud_     --> cloud_filtered_
  pcl::PointCloud<Point> cloud_filtered;
  pass_.setInputCloud (cloud_);
  pass_.filter (cloud_filtered);
  cloud_filtered_.reset (new pcl::PointCloud<Point> (cloud_filtered));
  ROS_INFO ("[TableObjectDetector::input_callback] Number of points left after filtering (%f -> %f): %d out of %d.",
            min_z_bounds_, max_z_bounds_, (int)cloud_filtered.points.size (), (int)cloud->points.size ());

  if ((int)cloud_filtered_->points.size () < k_)
  {
    ROS_WARN ("[TableObjectDetector::input_callback] Filtering returned %d points! Aborting.",
              (int)cloud_filtered_->points.size ());
    return;
  }

  // ---[ Create the voxel grid    <-- cloud_filtered_     --> cloud_downsampled_
  pcl::PointCloud<Point> cloud_downsampled;
  grid_.setInputCloud (cloud_filtered_);
  grid_.filter (cloud_downsampled);
  cloud_downsampled_.reset (new pcl::PointCloud<Point> (cloud_downsampled));

  // ---[ Estimate the point normals
  pcl::PointCloud < pcl::Normal > cloud_normals;
  n3d_.setInputCloud (cloud_downsampled_);
  n3d_.compute (cloud_normals);
  cloud_normals_.reset (new pcl::PointCloud<pcl::Normal> (cloud_normals));
  ROS_INFO ("[TableObjectDetector::input_callback] %d normals estimated.", (int)cloud_normals.points.size ());

  // ---[ Perform segmentation
  pcl::PointIndices table_inliers;
  pcl::ModelCoefficients table_coefficients;
  seg_.setInputCloud (cloud_downsampled_);
  seg_.setInputNormals (cloud_normals_);
  seg_.segment (table_inliers, table_coefficients);
  table_inliers_.reset (new pcl::PointIndices (table_inliers));
  table_coefficients_.reset (new pcl::ModelCoefficients (table_coefficients));
  if (table_coefficients.values.size () > 3)
    ROS_INFO ("[TableObjectDetector::input_callback] Model found with %d inliers: [%f %f %f %f].",
              (int)table_inliers.indices.size (), table_coefficients.values[0], table_coefficients.values[1],
              table_coefficients.values[2], table_coefficients.values[3]);
  if (table_inliers_->indices.size () == 0)
  {
    ROS_WARN ("[TableObjectDetector::input_callback] No Plane Inliers points! Aborting.");
    return;
  }

  // ---[ Extract the table
  pcl::PointCloud<Point> table_projected;
  //proj_.setInputCloud (cloud_filtered_);
  proj_.setInputCloud (cloud_downsampled_);
  proj_.setIndices (table_inliers_);
  proj_.setModelCoefficients (table_coefficients_);
  proj_.filter (table_projected);
  table_projected_.reset (new pcl::PointCloud<Point> (table_projected));
  ROS_INFO ("[TableObjectDetector::input_callback] Number of projected inliers: %d.",
            (int)table_projected.points.size ());

  // ---[ Estimate the convex hull
  std::vector<pcl::Vertices> polygons;
  pcl::PointCloud<Point>::Ptr table_hull(new pcl::PointCloud<Point>());
  hull_.setInputCloud (table_projected_);
  hull_.reconstruct (*table_hull, polygons);

  ROS_INFO ("[TableObjectDetector::input_callback] Number of points in hull: %d.",
              (int)table_hull->points.size());


  // Compute the plane coefficients
  Eigen::Vector4f model_coefficients;
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;

  model_coefficients[0] = table_coefficients.values[0];
  model_coefficients[1] = table_coefficients.values[1];
  model_coefficients[2] = table_coefficients.values[2];
  model_coefficients[3] = table_coefficients.values[3];

  // Need to flip the plane normal towards the viewpoint
  Eigen::Vector4f vp (0, 0, 0, 0);
  // See if we need to flip any plane normals
  vp -= table_hull->points[0].getVector4fMap ();
  vp[3] = 0;
  // Dot product between the (viewpoint - point) and the plane normal
  float cos_theta = vp.dot (model_coefficients);
  // Flip the plane normal
  if (cos_theta < 0)
  {
    model_coefficients *= -1;
    model_coefficients[3] = 0;
    // Hessian form (D = nc . p_plane (centroid here) + p)
    model_coefficients[3] = -1 * (model_coefficients.dot (table_hull->points[0].getVector4fMap ()));
  }

  //Set table_coeffs
  table_coeffs = model_coefficients;

  // ---[ Get the objects on top of the table
  pcl::PointIndices cloud_object_indices;
  prism_.setInputCloud (cloud_filtered_);
  prism_.setInputPlanarHull (table_hull);
  prism_.segment (cloud_object_indices);
  ROS_INFO ("[TableObjectDetector::input_callback] Number of object point indices: %d.",
            (int)cloud_object_indices.indices.size ());


  pcl::PointCloud<Point> cloud_objects;
  pcl::ExtractIndices<Point> extract_object_indices;
  extract_object_indices.setInputCloud (cloud_filtered_);
  extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
  extract_object_indices.filter (cloud_objects);
  cloud_objects_.reset (new pcl::PointCloud<Point> (cloud_objects));
  ROS_INFO ("[TableObjectDetector::input_callback] Number of object point candidates: %d.",
            (int)cloud_objects.points.size ());

  /*std::cout << "going to build message" << std::endl;
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*table_hull, msg);
  std::cout << "message built" << std::endl;

  pcl::PolygonMesh mesh;
  mesh.cloud = msg;
  mesh.polygons = polygons;

  std::cout << "going to visualize" << std::endl;
  pcl::visualization::PCLVisualizer vis3("Convex Hull PolygonMesh");
  vis3.addPolygonMesh(mesh);
  vis3.addPointCloud(cloud_filtered_);
  vis3.spin();*/

  if (cloud_objects.points.size () == 0)
    return;


  // ---[ Split the objects into Euclidean clusters
  std::vector < pcl::PointIndices > clusters2;
  //std::vector<pcl::PointIndices,Eigen::aligned_allocator<pcl::PointIndices> > clusters2;
  cluster_.setInputCloud (cloud_objects_);
  cluster_.extract (clusters2);
  ROS_INFO ("[TableObjectDetector::input_callback] Number of clusters found matching the given constraints: %d.",
            (int)clusters2.size ());

  clusters.resize (clusters2.size ());
  for (size_t i = 0; i < clusters2.size (); ++i)
  {
    clusters[i].points.resize (clusters2[i].indices.size ());
    for (size_t j = 0; j < clusters[i].points.size (); ++j)
    {
      clusters[i].points[j].x = cloud_objects.points[clusters2[i].indices[j]].x;
      clusters[i].points[j].y = cloud_objects.points[clusters2[i].indices[j]].y;
      clusters[i].points[j].z = cloud_objects.points[clusters2[i].indices[j]].z;
    }
  }

}
