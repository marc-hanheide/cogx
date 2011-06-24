//
// = FILENAME
//    HSSLsqLineExtractor.hh
//
// = FUNCTION
//    Defines a class that can extract lines from
//    a sorted set of points. The points have to be sorted in angle,
//    that is if you describe the points as (r,a), a has to be increasing.
// 
//    This algorithm is based in part on the Master Thesis project
//    by Daniel Forsgren.
//    
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 1999 Patric Jensfelt
//                  2009 Patric Jensfelt
//                  2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef HSSLsqLineExtractor_hh
#define HSSLsqLineExtractor_hh

#include "HSSRedundantLine2DRep.hh"

#ifndef DEPEND
#include <list>
#endif // DEPEND

namespace HSS {

/**
 * Class used to extract lines using a least squares algorithm.
 *
 * @author Patric Jensfelt
 */
class LsqLineExtractor {
public:
  /*==================================================*/
  // = CONSTRUCTORS
  /*==================================================*/

  /**
   * Constructor with default parameter setting, defined in .cc file
   */
  LsqLineExtractor();

  /** 
   * Constructor where all parameters are specified 
   *
   * @param minNumOfPoints   min number points to be a line
   * @param minLength        min length to be a line
   * @param maxNBadOnesInRow max number of outliers in a row before 
   *                           breaking line
   * @param maxPt2LineDist   max dist from a point in the data set 
   *                           to the estimated line
   * @param maxPtDistRatio   max allowed ratio between consecutive point 
   *                           distances for points belonging to the line.
   * @param maxPtDist        max allowed separation between two consecutive
   *                           points belonging to the line 
   * @param useDistAndRatio  use both distance and ratio conditions
   *
   */
  LsqLineExtractor(int minNumOfPoints, double minLength, 
                   int maxNBadOnesInRow, double maxPt2LineDist, 
                   double maxPtDistRatio, double maxPtDist,
                   int useDistAndRatio = 0);
  
  /**
   * Extracts lines form a point data (x,y)
   *
   * The range array (r) gives the range readings and is only used to
   * determine if the points is an outlier or not. Do not pass this
   * one in if you do outlier rejection in some other way. If this is
   * data that comes from some source where the range readings are not
   * well defined just supply a 0 pointer.
   */
  int extract(double *x, double *y, int nData, double *r = 0);

  /** @return a reference to a list of extracted lines */
  std::list<HSS::RedundantLine2DRep>& lines();
  
  /** @return the number of extracted lines */
  int nLines();

  /** 
   * Set the flag that tells if we should extract uncertainty
   * information as well as line data 
   *
   * @param calcUnc 0 if you do not want to get uncertainty information
   * for the lines, else a positive number larger than 0
   */
  void setUncFlag(int calcUnc, double ptUnc);

  /**
   * Set the max valid range. Any data point further away than this
   * will be assumed to be invalid.
  */
  void setMaxRange(double r) { m_MaxRange = r; }  
  double getMaxRange() const { return m_MaxRange; }

protected:

private:
  /** list of extracted lines */
  std::list<HSS::RedundantLine2DRep> lines_;

  /** 0 if no uncertainty info should be extracted */
  int calcUnc_;
  
  /** The uncertainty in each data point, assuming diagonal Gaussian noise */
  double ptUnc_;

  /** Pointer to array of data point, x coord */
  double* x_;

  /** Pointer to array of data point, y coord */
  double* y_;

  /** Temp varibale used when calculating line params */
  double xSum_;

  /** Temp varibale used when calculating line params */
  double ySum_;

  /** Temp varibale used when calculating line params */
  double xySum_;

  /** Temp varibale used when calculating line params */
  double x2Sum_;

  /** Temp varibale used when calculating line params */
  double y2Sum_;

  /** Temp varibale used when calculating line params */
  double det_;

  /** Min numbe rof points to be a line */
  int minNumOfPoints_;

  /** Min length to be a line */
  double minLength_;

  /** Max number of consecutive outliers in the data not to break the
      line */
  int maxNBadOnesInRow_;

  /** Max allowed distance between a point and the estimated line not
      to consider the point to be an outlier */
  double maxPt2LineDist_;

  /** Max allowed ratio between the distances between two consecutive
      points */
  double maxPtDistRatio_;

  /** The inverse of maxPtDistRatio, used to avoid looking at both
      d1/d2 and d2/d1 so to speak */
  double minPtDistRatio_;

  /** Max allowed distance between two consecutive points not to break
      the line. Typically rather large as the points will be more
      spread out the further away we come. */
  double maxPtDist_;

  /** When 1, we use both distance and ratio conditions and it is not
      enough to fulfill one of them only */
  int useDistAndRatio_;

  double m_MaxRange;

  /** Clear the temporary variables xSu_, etc */
  void clearSums();

  /** Add new point to the temporari variabels used to calc line params
   *
   * @param x x coord of new point
   * @param y y coord of new point
   */
  void updateSums(double x, double y);

  /** 
   * Calculate the line params based on the sums, the result is
   * returned in the a, b and flipped varaible. The flipped variable
   * is used to denote the situations when the definition of x and y
   * have been flipped to give better numerical results on the line
   * params.
   *
   * @param a 
   * @param b
   * @param flipped true if x and y axis is flipped 
   * @param n number of data points used to form the sums 
   */
  void calcLineParams(double& a, double& b, bool& flipped, int n);

  /**
   * Does the final calculation of the line parameters based on the
   * points that was picked by the first stage of the algorithm.
   *
   * @param nUsed number of used points
   * @param used pointer to array of indices of used points
   * @param line resulting line
   */
  void fillInLine(int nUsed, int* used, HSS::RedundantLine2DRep& line);

}; // class LsqLineExtractor //

inline std::list<HSS::RedundantLine2DRep>&
LsqLineExtractor::lines()
{
  return lines_;
}

inline int
LsqLineExtractor::nLines()
{
  return int(lines_.size());
}

inline void
LsqLineExtractor::clearSums()
{
  xSum_ = ySum_ = xySum_ = x2Sum_ = y2Sum_ = 0;
}

inline void
LsqLineExtractor::updateSums(double x, double y)
{
  xSum_ += x;
  x2Sum_ += x * x;
  y2Sum_ += y * y;
  ySum_ += y;
  xySum_ += x * y;
}

inline void
LsqLineExtractor::calcLineParams(double& a, double& b, bool& flipped, int n)
{      
  if (n == 0) return;

  // calc line params
  double det = x2Sum_ * n - xSum_*xSum_;
  if((det < 1) || 
     (1 < fabs(b = ((-xSum_ * ySum_) + n * xySum_) / det))){
    flipped = true;
    det = y2Sum_ * n - ySum_*ySum_;
    b = ((-ySum_ * xSum_) + n * xySum_) / det;
    a = (y2Sum_ * xSum_ + (-ySum_ * xySum_)) / det;
  } else {
    flipped = false;
    a = (x2Sum_ * ySum_ + (-xSum_ * xySum_)) / det;
  }
}

inline void 
LsqLineExtractor::setUncFlag(int calcUnc, double ptUnc)
{
  calcUnc_ = calcUnc;
  ptUnc_ = ptUnc;
}

}; // namespace HSS

#endif // HSSLsqLineExtractor_hh

