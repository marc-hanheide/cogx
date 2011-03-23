//
// = filename 
//     HSSLsqLineExtractor.cc
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

#include "HSSLsqLineExtractor.hh"
#include "HSSutils.hh"

#ifndef DEPEND
#include <fstream>
#include <float.h>
#endif 

namespace HSS {

//#define PRINTLSQSTUFF 

#define LSQ_MIN_NPTS            4
#define LSQ_MAX_BADONESINROW    0
#define LSQ_MIN_LENGTH        500.0
#define LSQ_MAX_P2LDIST       100.0
#define LSQ_MAX_PDISTRATIO      3.0
#define LSQ_MAX_PTDIST        150.0 
#define LSQ_USE_DISTANDRATIO      0

#ifdef PRINTLSQSTUFF
std::fstream fs_allPts;
std::fstream fs_lsqPts;
std::fstream fs_lsqLines;
#endif

LsqLineExtractor::LsqLineExtractor()
  :minNumOfPoints_(LSQ_MIN_NPTS),
   minLength_(LSQ_MIN_LENGTH),
   maxNBadOnesInRow_(LSQ_MAX_BADONESINROW),
   maxPt2LineDist_(LSQ_MAX_P2LDIST),
   maxPtDistRatio_(LSQ_MAX_PDISTRATIO),
   maxPtDist_(LSQ_MAX_PTDIST),
   useDistAndRatio_(LSQ_USE_DISTANDRATIO),
   m_MaxRange(5.59)
{
  minPtDistRatio_ = 1.0 / maxPtDistRatio_;

  x_ = 0;
  y_ = 0;

  calcUnc_ = 0;
  ptUnc_ = 10000;  // Just to give it a well defined value

#ifdef PRINTLSQSTUFF
  fs_allPts.open("lsqAllPts.dat", std::ios::out);
  fs_lsqPts.open("lsqPts.dat", std::ios::out);
  fs_lsqLines.open("lsqLines.dat", std::ios::out);
#endif
}

LsqLineExtractor::LsqLineExtractor(int minNumOfPoints, double minLength, 
			   int maxNBadOnesInRow, double maxPt2LineDist, 
			   double maxPtDistRatio, double maxPtDist,
                           int useDistAndRatio)
  :minNumOfPoints_(minNumOfPoints),
   minLength_(minLength),
   maxNBadOnesInRow_(maxNBadOnesInRow),
   maxPt2LineDist_(maxPt2LineDist),
   maxPtDistRatio_(maxPtDistRatio),
   maxPtDist_(maxPtDist),
   useDistAndRatio_(useDistAndRatio),
   m_MaxRange(5.59)
{
  minPtDistRatio_ = 1.0 / maxPtDistRatio_;

  x_ = 0;
  y_ = 0;

  calcUnc_ = 0;
  ptUnc_ = 10000;  // Just to give it a well defined value

#ifdef PRINTLSQSTUFF
  fs_allPts.open("lsqAllPts.dat", std::ios::out);
  fs_lsqPts.open("lsqPts.dat", std::ios::out);
  fs_lsqLines.open("lsqLines.dat", std::ios::out);
#endif
}

int
LsqLineExtractor::extract(double *x, double *y, int nData, double *r)
{
#ifdef PRINTLSQSTUFF
  for (int i=0; i<nData; i++)
    fs_allPts << x[i] << " " << y[i] << " ";
  fs_allPts << std::endl;
#endif

  x_ = new double[nData];
  y_ = new double[nData];
  for (int i = 0; i < nData; i++) {
    x_[i] = x[i];
    y_[i] = y[i];
  }

  // Clear the old list of lines
  lines_.clear();

  // Variable declarations
  double oldD, newD, p2LDist = 0, distRatio;
  double prelA, prelB, a = 0, b = 0;
  HSS::RedundantLine2DRep line;
  bool prelFlipped, flipped=false;
  int index=0, startIndex;
  int used[nData];
  int nUsed = 0;
  int nBadOnesInRow = 0;

  startIndex = 0;
  do {
    clearSums();

    // Search for a set of data tha might be good for a line
    index = startIndex + 1;
    newD = oldD = hypot(x_[index - 1] - x_[index],     
                        y_[index - 1] - y_[index]);
    distRatio = 1;
    while ((index < (nData-1)) && 
	   (((newD < maxPtDist_) && (oldD < maxPtDist_)) ||
	    ((minPtDistRatio_  < distRatio) && 
	     (distRatio < maxPtDistRatio_))) &&
	   (((index - startIndex + 1) < minNumOfPoints_) || 
	    (hypot(x_[startIndex] - x_[index],      
                   y_[startIndex] - y_[index]) < minLength_))) {
      index++;
      oldD = newD;
      newD = hypot(x_[index - 1] - x_[index], y_[index-1] - y_[index]);
      distRatio = newD / oldD;
    }
    index--;

    // Check why we stopped

    // If we failed just start again with new point
    if ((useDistAndRatio_ && 
         (((newD < maxPtDist_)  && (oldD < maxPtDist_)) &&
          ((minPtDistRatio_  < distRatio) && 
           (distRatio < maxPtDistRatio_)))) ||
        (!useDistAndRatio_ && 
         (((newD < maxPtDist_)  && (oldD < maxPtDist_)) ||
          ((minPtDistRatio_  < distRatio) && 
           (distRatio < maxPtDistRatio_))))) {
      
      // We have a candidate
      // Start by calculating help sums
      for (int i = startIndex; i <= index; i++) {
	updateSums(x[i], y[i]);
      }

      calcLineParams(prelA, prelB, prelFlipped, (index - startIndex + 1));

      // Make sure that no point is too far away from the line
      nBadOnesInRow = 0;    
      nUsed = 0;
      clearSums();
      for(int i = startIndex; ((nBadOnesInRow <= maxNBadOnesInRow_) && 
			       (i <= index)); i++) {

        if (r && r[i] > m_MaxRange) {
          nBadOnesInRow++;
        } else {

          if(prelFlipped) {
            p2LDist = fabs(x[i] - (prelA + y[i] * prelB));
          } else {
            p2LDist = fabs(y[i] - (prelA + x[i] * prelB));
          }
          
          if (p2LDist > maxPt2LineDist_) {
            nBadOnesInRow++;
          } else {
            updateSums(x[i], y[i]);
            used[nUsed++] = i;
            nBadOnesInRow = 0;
          }
        }
      }
      
      // Check if there is any point in continuing,
      // that is that the line is long enough
      if (nBadOnesInRow <= maxNBadOnesInRow_) {
	
	// At this point we are sure that we at least have a line
	// from startIndex to index. Now we try to see how long we 
	// can make it
	
	a = prelA;
	b = prelB;
	flipped = prelFlipped;
	nBadOnesInRow = 0;
	index++;
	oldD = hypot(x_[index - 2] - x_[index - 1],
                     y_[index - 2] - y_[index - 1]);
	newD = hypot(x_[index - 1] - x_[index ],
                     y_[index - 1] - y_[index]);
	distRatio = newD / oldD;

	while((index < (nData-1)) && 
	      (((newD < maxPtDist_)  && (oldD < maxPtDist_)) ||
	       ((minPtDistRatio_  < distRatio) && 
		(distRatio < maxPtDistRatio_))) &&
	      (nBadOnesInRow <= maxNBadOnesInRow_)) {

	  updateSums(x[index], y[index]);
	  calcLineParams(prelA, prelB, prelFlipped, nUsed+1);

	  nUsed = 0;
	  nBadOnesInRow = 0;
	  clearSums();
	  for(int i = startIndex; ((nBadOnesInRow <= maxNBadOnesInRow_) && 
                                   (i <= index)); i++) {

            if (r && r[i] > m_MaxRange) {
              nBadOnesInRow++;
            } else {
              
              if(prelFlipped) {
                p2LDist = fabs(x[i] - (prelA + y[i] * prelB));
              } else {
                p2LDist = fabs(y[i] - (prelA + x[i] * prelB));
              }
              
              if (p2LDist > maxPt2LineDist_) {
                nBadOnesInRow++;
              } else {
                updateSums(x[i], y[i]);
                used[nUsed++] = i;
                nBadOnesInRow = 0;
              }
            }
	  }
	  
	  if (nBadOnesInRow <= maxNBadOnesInRow_) {
	    a = prelA;
	    b = prelB;
	    flipped = prelFlipped;
	    index++;
	    oldD = newD;

	    newD = hypot(x_[index - 1] - x_[index],     
                         y_[index - 1] - y_[index]);
	    distRatio = newD / oldD;
	  }	
	}
	index--;

	nUsed = 0;
	for(int i = startIndex; i <= index; i++) {

          if (r && r[i] > m_MaxRange) continue;

	  if(flipped) {
	    p2LDist = fabs(x[i] - (a + y[i] * b));
	  } else {
	    p2LDist = fabs(y[i] - (a + x[i] * b));
	  }
         
	  if (p2LDist < maxPt2LineDist_) {
	    used[nUsed++] = i;
	  }
	}

	if (nUsed >= 2) {
	  fillInLine(nUsed, used, line);
          if (line.h() >= 0.5 * minLength_) {
            startIndex = used[nUsed - 1];
          
            line.setWeight(nUsed);
            lines_.push_back(line);
#ifdef PRINTLSQSTUFF
            for (int i = 0; i < nUsed; i++)
              fs_lsqPts << x[used[i]] << " " << y[used[i]] << " ";
            fs_lsqPts << std::endl;
          
            fs_lsqLines << line.xS() << " " << line.yS() << " "
                        << line.xE() << " " << line.yE() << std::endl;
#endif
          } else {
            startIndex++;
          }
        } else {
          startIndex++;
        }

      } else {
	startIndex++;
      }
    } else {
      startIndex++;
    }

  } while(startIndex < (nData - minNumOfPoints_ - 1));

#ifdef PRINTLSQSTUFF
  // Add line to separate the data
  fs_lsqLines << "0 0 0 0" << std::endl;
#endif

  // Return the size of the line list
  delete [] x_;
  delete [] y_;
  return nLines();
}

void
LsqLineExtractor::fillInLine(int nUsed, int* used, 
                             RedundantLine2DRep& line)
{
  if (nUsed < 2) {
    std::cerr << "HSS::LsqLineExtractor::fillInLine(" << nUsed << ",...): Cannot create line with less than 2 points\n";
    return;
  }

  const double PiFourth = 0.25 * M_PI;
  const double PiHalf = 0.5 * M_PI;  

  int startIndex = used[0];
  int stopIndex = used[nUsed - 1];
  
  double xAvg = 0, yAvg = 0;
  for (int i = 0; i < nUsed; i++) {
    xAvg += x_[used[i]];
    yAvg += y_[used[i]];
  }
  
  xAvg = xAvg / nUsed;
  yAvg = yAvg / nUsed;
  
  double a=0, b=0, c=0;
  
  for (int i=0; i<nUsed; i++) {
    a += (x_[used[i]] - xAvg) * (x_[used[i]] - xAvg);
    b += 2*(x_[used[i]] - xAvg) * (y_[used[i]] - yAvg);
    c += (y_[used[i]] - yAvg) * (y_[used[i]] - yAvg);
  }

  double tmp = 0.5 * atan(b/(a-c));
  double alpha, theta;
  double approxTheta = atan2(y_[stopIndex] - y_[startIndex],
			     x_[stopIndex] - x_[startIndex]);
  
  while (fabs(HSS::pi_to_pi(tmp - approxTheta)) > PiFourth)
    tmp += PiHalf;
  theta = tmp;

  double approxAlpha = atan2(0.5 * (y_[startIndex] + y_[stopIndex]), 
			     0.5 * (x_[startIndex] + x_[stopIndex]));

  if (fabs(HSS::pi_to_pi((tmp - PiHalf) - approxAlpha)) > PiHalf)
    alpha = tmp - PiHalf + M_PI;
  else 
    alpha = tmp - PiHalf;

  double 
    rho = (yAvg*cos(alpha - PiHalf) - xAvg*sin(alpha - PiHalf));
  

  // The direction cosine of the line is given by
  double cosT = cos(theta);
  double sinT = sin(theta);

  // Find the first and last point along the line
  int firstIndex, lastIndex;
  double firstDotProd = DBL_MAX;
  double lastDotProd = -DBL_MAX;
  double dotProd;
  for (int i= 0; i < nUsed; i++) {
    dotProd = cosT * x_[used[i]] + sinT * y_[used[i]];
    if (dotProd < firstDotProd) {
      firstIndex = used[i];
      firstDotProd = dotProd;
    }
    if (dotProd > lastDotProd) {
      lastIndex = used[i];
      lastDotProd = dotProd;
    }
  }

  // Calculate the point where the normal to the line through the
  // sensor intersects the line
  double xI = rho * cos(alpha);
  double yI = rho * sin(alpha);

  line.setEndPts(xI + firstDotProd * cosT,
                 yI + firstDotProd * sinT,
                 xI + lastDotProd * cosT,
                 yI + lastDotProd * sinT);

  if (calcUnc_ != 0) {
    // Calculate parameter uncertainty based on the paper by Deriche et al

    double
      uncR = 0, uncA = 0, uncRA = 0, uncConst = 0,
      d = xAvg * cos(alpha) + yAvg * sin(alpha),
      stdData = ptUnc_,
      covXX = stdData * stdData,
      covYY = stdData * stdData,
      covXY = 0;
    
    uncConst = (a * covYY - b * covXY + c * covXX) / 
      ((a - c) * (a - c)+ b * b);
    
    uncA  = uncConst;
    uncRA = uncConst * (-d);
    uncR  = uncConst * d * d + (covYY * cos(alpha) * cos(alpha) + 
				covXX * sin(alpha) * sin(alpha) -
				covXY * cos(alpha) * sin(alpha) * 2.0) / nUsed;
    
    // Update line uncertainty. 
    line.R()(0,0) = uncR;       line.R()(0,1) = uncRA;
    line.R()(1,0) = uncRA;      line.R()(1,1) = uncA;
  }
}

}; // namespace HSS
