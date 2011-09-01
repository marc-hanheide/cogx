/*
    Copyright (c) <year>, <copyright holder>
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY <copyright holder> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "average.h"

namespace V4R{

double mean_arithmetic(double *v, unsigned int n){
  double sum = 0;
  for(unsigned int  i = 0; i < n; i++) sum += v[i]; 
  return sum / (double) n;
}

float mean_arithmetic(float *v, unsigned int n){
  float sum = 0;
  for(unsigned int  i = 0; i < n; i++) sum += v[i]; 
  return sum / (float) n;
}
double mean_arithmetic(int *v, unsigned int n){
  int sum = 0;
  for(unsigned int  i = 0; i < n; i++) sum += v[i]; 
  return (double) sum / (double) n;
}

cv::Point3_<double> mean_arithmetic(const std::vector<cv::Point3_<double> > &_samples){
  double s = _samples.size();
  cv::Point3_<double> sum(0,0,0); 
  for(unsigned int i = 0; i < _samples.size(); i++){
    sum += _samples[i];
  }
  cv::Point3_<double> mean(sum.x / s, sum.y / s, sum.z / s); 
  return mean;
}
cv::Point3_<double> mean_arithmetic(const std::list<cv::Point3_<double> > &_samples){
  double s = _samples.size();
  cv::Point3_<double> sum(0,0,0); 
  std::list<cv::Point3_<double> >::const_iterator it;
  for ( it = _samples.begin() ; it != _samples.end(); it++ ) {
    sum += *it;
  }
  cv::Point3_<double> mean(sum.x / s, sum.y / s, sum.z / s); 
  return mean;
}
double varianz_arithmetic(const std::vector<cv::Point3_<double> > &_samples, const cv::Point3_<double> &_mean){
  double s = _samples.size();
  double sum2 = 0;
  double d, dx, dy, dz, var;
  for(unsigned int i = 0; i < _samples.size(); i++){
    const cv::Point3_<double> &sample = _samples[i];
    dx = _mean.x - sample.x, dy = _mean.y - sample.y, dz = _mean.z - sample.z;
    d = dx*dx + dy*dy + dz*dz;
    sum2 += d;
  }
  var = sqrt(sum2);
  return var;
}

double varianz_arithmetic(const std::list<cv::Point3_<double> > &_samples, const cv::Point3_<double> &_mean){
  double s = _samples.size();
  double sum2 = 0;
  double d, dx, dy, dz, var;
  std::list<cv::Point3_<double> >::const_iterator it;
  for ( it = _samples.begin() ; it != _samples.end(); it++ ) {
    const cv::Point3_<double> &sample = *it;
    dx = _mean.x - sample.x, dy = _mean.y - sample.y, dz = _mean.z - sample.z;
    d = dx*dx + dy*dy + dz*dz;
    sum2 += d;
  }
  var = sqrt(sum2);
  return var;
}
}