/*
 * scale_distribution.h
 *
 *  Created on: Aug 9, 2011
 *      Author: aitor
 */

#ifndef SCALE_DISTRIBUTION_H_
#define SCALE_DISTRIBUTION_H_

float read_scale_from_file(std::string file) {

  std::cout << "read_scale_from_file:" << file << std::endl;

  //read scale and push it into the vector
  std::ifstream in;
  in.open (file.c_str (), std::ifstream::in);

  char linebuf[256];
  in.getline (linebuf, 256);
  std::string line (linebuf);
  std::vector<std::string> strs;
  boost::split (strs, line, boost::is_any_of (" "));

  return atof (strs[0].c_str ());
}

void
load_scale_samples (std::string directory, std::map<std::string, std::vector<float> > & category_scale_samples)
{
  bf::path dir = directory;
  bf::directory_iterator end_itr;
  for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    //check if its a directory, then get models in it
    if (bf::is_directory (*itr))
    {
      //this is a category...
      bf::path curr_path = itr->path ();
      std::vector<float> scales_for_category;

      bf::directory_iterator end_itr2;
      for (bf::directory_iterator itr (curr_path); itr != end_itr2; ++itr)
      {
        bf::path curr_path2 = itr->path ();
        std::string file = curr_path2.filename ();
        std::vector<std::string> strs;
        boost::split (strs, file, boost::is_any_of ("."));
        if (strs[strs.size () - 1] == "txt" && strs[strs.size () - 2] == "scale")
        {
          //std::cout << file << std::endl;
          std::stringstream path_to_scale_file;
          path_to_scale_file << directory << "/" << curr_path.filename () << "/" << file;
          float s = read_scale_from_file(path_to_scale_file.str());
          scales_for_category.push_back (s);
        }
      }

      category_scale_samples[curr_path.filename ()] = scales_for_category;
      //std::cout << "Number of scales read:" << scales_for_category.size() << std::endl;
    }
    else
    {
      //ignore...
    }
  }

}

void
meanShift (std::vector<float> & scale_samples, float bandwidth, float threshold,
           std::vector<std::vector<int> > & cluster_indices)
{

  std::vector<float> scale_samples_orig (scale_samples);

  for (size_t i = 0; i < scale_samples.size (); i++)
  {
    float diff_iterations = std::numeric_limits<float>::max ();

    int it = 0;

    while ((diff_iterations > threshold) && (it < 20))
    {
      float numerator = 0.0;
      float current_point = scale_samples[i];
      float denominator = 0;
      double kernelDist;

      for (size_t j = 0; j < scale_samples.size (); j++)
      {
        kernelDist = exp (-abs (current_point - scale_samples_orig[j]) / bandwidth);
        numerator += scale_samples_orig[j] * kernelDist;
        denominator += kernelDist;
      }

      numerator /= denominator;
      diff_iterations = abs (numerator - current_point);
      scale_samples[i] = numerator;
      it++;
    }

    /*if ((i % 100) == 0)
     std::cout << "Processed point " << i << std::endl;*/
  }

  //cluster scale_samples
  std::vector<float> cluster_centroids;
  std::vector<int> collapsed_in_cluster;

  bool COMPUTE_MODE_WEIGHT = true;
  //group cloud points
  for (size_t j = 0; j < scale_samples.size (); j++)
  {
    float current_point = scale_samples[j];
    int closestCentroid = -1;
    for (size_t i = 0; i < cluster_centroids.size (); i++)
    {
      double distance = abs (cluster_centroids[i] - current_point);

      if (distance < bandwidth)
      {
        closestCentroid = i;
        //one mode is about to be added, compute weight
        float weight_new_mode = 1.0 / (float)(collapsed_in_cluster[i] + 1);
        float weight_old_mode = 1.0 - weight_new_mode;

        cluster_centroids[i] = cluster_centroids[i] * weight_old_mode;
        current_point = current_point * weight_new_mode;
        cluster_centroids[i] += current_point;
      }
    }

    if (closestCentroid >= 0)
    {
      //points_to_cluster[j] = closestCentroid;
      cluster_indices[closestCentroid].push_back (j);
      collapsed_in_cluster[closestCentroid]++;
    }
    else
    {
      //new cluster found
      //points_to_cluster[j] = cluster_centroids.size ();
      cluster_centroids.push_back (current_point);
      collapsed_in_cluster.push_back (1); //cluster contains one mode so far

      std::vector<int> indices;
      indices.push_back (j);
      cluster_indices.push_back (indices);


    }

  }

  //copy samples back...
  scale_samples = scale_samples_orig;
}

float
compute_mean (std::vector<float> & samples, std::vector<int> & cluster_indices)
{
  float mean = 0;
  for (size_t i = 0; i < cluster_indices.size (); i++)
  {
    mean += samples[cluster_indices[i]];
  }

  mean /= cluster_indices.size ();
}

float
compute_variance (std::vector<float> & samples, std::vector<int> & cluster_indices, float mean)
{
  float variance = 0;
  for (size_t i = 0; i < cluster_indices.size (); i++)
  {
    variance += pow (mean - samples[cluster_indices[i]], 2);
  }

  variance /= cluster_indices.size ();
}

class NormalDistribution
{
public:
  float mean_;
  float sigma_;
};

void
createDistributions (std::map<std::string, std::vector<float> > & category_scale_samples,
                     std::map<std::string, std::vector<NormalDistribution> > & category_distributions)
{
  std::map<std::string, std::vector<float> >::iterator it;

  for (it = category_scale_samples.begin (); it != category_scale_samples.end (); it++)
  {
    cout << (*it).first << " => " << (*it).second.size () << endl;
    //cluster samples using mean shift

    std::vector<NormalDistribution> cat_distributions;

    std::vector<std::vector<int> > cluster_indices;
    meanShift ((*it).second, 0.01, 0.001, cluster_indices); //1cm bandwith, TODO: Play with this parameters once we have more data!!

    for (size_t i = 0; i < cluster_indices.size (); i++)
    {
      //compute normal distributions from the samples in a cluster
      float mean = compute_mean ((*it).second, cluster_indices[i]);
      float variance = compute_variance ((*it).second, cluster_indices[i], mean);

      std::cout << "mean:" << mean << " variance:" << variance << " sigma:" << sqrt (variance) << std::endl;

      NormalDistribution n;
      n.mean_ = mean;
      n.sigma_ = sqrt (variance);

      cat_distributions.push_back (n);
    }

    category_distributions[(*it).first] = cat_distributions;
  }

}

bool
checkScaleOnDistributions (std::string category, float scale,
                           std::map<std::string, std::vector<NormalDistribution> > & category_distributions)
{
  if (category_distributions.find(category) != category_distributions.end()) {
    //check if the scale is ok for the category distributions
    std::vector<bool> ok_for_distribution(category_distributions.size(),false);
    std::vector<NormalDistribution> * category_dist;
    std::map<std::string, std::vector<NormalDistribution> >::iterator it;
    it = category_distributions.find(category);
    category_dist = &it->second;

    std::cout << category_dist->size() << std::endl;
    for(size_t i = 0; i < category_dist->size(); i++) {
      //compute confidence interval
      float scale_min = category_dist->at(i).mean_ - category_dist->at(i).sigma_ * 0.98;
      float scale_max = category_dist->at(i).mean_ + category_dist->at(i).sigma_ * 0.98;

      if( (scale >= scale_min) && (scale <= scale_max)) {
        std::cout << " scale " << scale << " fits in distribution " << i << " for " << category <<  ", the limits are (" << scale_min << "," << scale_max << ")" << std::endl;
        ok_for_distribution[i] = true;
      } else {
        std::cout << " scale " << scale << " does not fit in distribution " << i << " for " << category <<  ", the limits are (" << scale_min << "," << scale_max << ")" << std::endl;
      }
    }

    if(ok_for_distribution.size() == 1) {
      return ok_for_distribution[0];
    } else {
      //decide to which of the distributions which are ok, the current scale fits
      std::cout << "We have multiple distributions for category: " << category << std::endl;
      return false;
    }
  } else {
    //if not found, we can always return true or let the user decide...
    return true;
  }
  return false;
}

#endif /* SCALE_DISTRIBUTION_H_ */
