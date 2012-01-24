//
// = FILENAME
//    HSSScan2DMatcher.cc
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "HSSScan2DMatcher.hh"
#include "HSSutils.hh"

namespace HSS {

bool 
Scan2DMatcher::scanMatch(const HSS::Scan2D &scanRef, 
                       const HSS::Scan2D &scanSens,
                       const Eigen::Vector3d &Tguess, 
                       Eigen::Vector3d &T, int &n,
                       std::vector< std::pair<int, int> > *corrs)
{
  struct sm_params params;
  setDefaultScanMatchParams(&params);
  params.min_reading = scanSens.min_range;
  params.max_reading = scanSens.max_range;
  params.sigma = scanSens.range_sigma;
  params.restart = 0;
  params.max_correspondence_dist = 2.0;
  params.max_iterations = 20;
  params.clustering_threshold = 0.05; // 0.05
  params.epsilon_xy = 1e-4;
  params.epsilon_theta = 1e-4;

  params.first_guess[0] = Tguess[0];
  params.first_guess[1] = Tguess[1];
  params.first_guess[2] = Tguess[2];

  // Setup the scans to use in the matching
  params.laser_ref  = new struct laser_data;
  params.laser_ref << scanRef;

  params.laser_sens = new struct laser_data;
  params.laser_sens << scanSens;

  // Print scanSens and scanRef on the log files scanRef.log and scanSens.logs
  FILE *scanRefLog; 
  scanRefLog = fopen("scanRef.log", "w");
  if (scanRefLog != NULL) {
    fprintf(scanRefLog, "%d\n", scanRef.valid.size());
    for (unsigned int i = 0; i < scanRef.valid.size(); i++) {
      if (scanRef.valid[i])
        fprintf(scanRefLog, "1 %f %f\n", scanRef.theta[i], scanRef.range[i]);
      else
        fprintf(scanRefLog, "0\n");
    }
    fclose(scanRefLog);
  } else {
    std::cerr << "Failed scanRef.log creation" << std::endl;
  }
  
  FILE *scanSensLog; 
  scanSensLog = fopen("scanSens.log", "w");
  if (scanSensLog != NULL) {
    fprintf(scanSensLog, "%d\n", scanSens.valid.size());
    for (unsigned int i = 0; i < scanSens.valid.size(); i++) {
      if (scanSens.valid[i])
        fprintf(scanSensLog, "1 %f %f\n", scanSens.theta[i], scanSens.range[i]);
      else
        fprintf(scanSensLog, "0\n");
    }
    fclose(scanSensLog);
  } else {
    std::cerr << "Failed scanSens.log creation" << std::endl;
  }

  struct sm_result result;
  sm_icp(&params, &result);

  // Print the correspondences on the log file corr.log
  FILE *corrLog;
  corrLog = fopen("corr.log", "w");
  if (corrLog != NULL) {
    struct correspondence* corr_sens;
    corr_sens = params.laser_sens->corr;
    for (int i = 0; i < params.laser_sens->nrays; i++) {
      if (corr_sens[i].valid == 1)
        fprintf(corrLog, "%d %d\n", i+1, corr_sens[i].j1+1);
          // IMPORTANT: we use +1 because this file is going to be parsed with
          // Matlab where the vectors are indexed starting with 1 instead of 0.
    }
    fclose(corrLog);
  } else {
    std::cerr << "Failed corr.log creation" << std::endl;
  }

  // Fill in the correspondences vector
  if ( corrs != NULL ) {
    struct correspondence* corr_sens;
    corr_sens = params.laser_sens->corr;
    for (int i = 0; i < params.laser_sens->nrays; i++) {
      if (corr_sens[i].valid == 1)
        corrs->push_back( std::pair<int, int>(corr_sens[i].j1, i) );
    }
  }

  ld_dealloc(params.laser_ref);
  delete params.laser_ref;
  ld_dealloc(params.laser_sens);
  delete params.laser_sens;

  if (result.valid && 
      result.iterations <= params.max_iterations &&
      result.nvalid > 0.3 * scanSens.range.size()) {

    T[0] = result.x[0];
    T[1] = result.x[1];
    T[2] = result.x[2];

    n = result.iterations;

    return true;
  } else {

    // std::cerr << "Scan matching failed\n";

    return false;
  }
}

void 
Scan2DMatcher::setDefaultScanMatchParams(struct sm_params *p)
{
  // Maximum angular displacement between scans [deg]
  p->max_angular_correction_deg = 90.0;

  // Maximum translation between scans [m]
  p->max_linear_correction = 2.0;

  // Number of iterations before giving up
  p->max_iterations = 1000;

  // A threshold for stopping [m]
  p->epsilon_xy = 1e-4;

  // A threshold for stopping [rad]
  p->epsilon_theta = 1e-4;

  // dubious parameter [m]
  p->max_correspondence_dist = 2.0;

  // Noise in the scan [m]
  p->sigma = 0.01;

  // Use smart tricks for finding correspondences (1 yes, 0 no)
  p->use_corr_tricks =  1;

  // Restart: Restart if error is over threshold
  p->restart = 0;

  // Restart: Threshold for restarting
  p->restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting [m]
  p->restart_dt = 0.01;

  // Restart: displacement for restarting [rad]
  p->restart_dtheta = HSS::deg2rad(1.5);

  // Max distance for staying in the same clustering
  p->clustering_threshold = 0.05;

  // Number of neighbour rays used to estimate the orientation.
  p->orientation_neighbourhood = 3;

  // If 0, it's vanilla ICP. Otherwise PLICP
  p->use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  p->do_alpha_test = 0;
  
  p->do_alpha_test_thresholdDeg = 20.0;

  p->outliers_maxPerc = 0.95;

  p->outliers_adaptive_order = 0.7;

  p->outliers_adaptive_mult = 2.0;

  p->do_visibility_test = 0;

  // No two points in laser_sens can have the same corr.
  p->outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  p->do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  p->debug_verify_tricks = 0;

  // GPM: Dimension of bins for finding first theta
  p->gpm_theta_bin_size_deg = 5.0;

  // GPM: Area around maximum
  p->gpm_extend_range_deg = 15.0;

  // Interval of points to consider (1: all points, 2: every other point, etc.)
  p->gpm_interval = 1;

  // Pose of the laser in the with respct to whatever coordinate system we work
  // laser.x [m]
  p->laser[0] = 0.0; 
  // laser.y [m]
  p->laser[1] = 0.0; 
  // laser.theta [rad]
  p->laser[2] = 0.0; 

  // Don't use readings less than min_reading [m]
  p->min_reading = 0.0;

  // Don't use readings longer than max_reading [m]
  p->max_reading = 1000.0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is
  // used to compute the incidence beta, and the factor
  // (1/cos^2(beta)) used to weight the correspondence.	
  p->use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to
  // weight the correspondence by 1/sigma^2
  p->use_sigma_weights = 0;
}
 
}; // namespace HSS

void operator<<(LDP sink, const HSS::Scan2D &src)
{
  ld_alloc(sink, src.range.size());
  sink->min_theta = src.min_theta;
  sink->max_theta = src.max_theta;
  
  for (int i = 0; i < sink->nrays; i++) {
    sink->theta[i] = src.theta[i];
    sink->valid[i] = src.valid[i];
    sink->readings[i] = src.range[i];
  }
  sink->tv = src.tv;
}

