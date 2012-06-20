//
// = FILENAME
//    DoorExtractor.cc
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 1999 Patric Jensfelt
//                  2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/


#include "DoorExtractor.hh"
#include <Utils/HelpFunctions.hh>
#include <Utils/CureDebug.hh>

// Interval which is acceptable for a door
#define DOOREXTR_MIN_DOORWIDTH      0.6       // in m
#define DOOREXTR_MAX_DOORWIDTH      1.3       // in m 

// When checking if two line are parallel, see above, their
// c-parameters, that perp distance must not differ more than
#define DOOREXTR_MAX_CDIFF          0.1  // in m

// When checking if two lines are parallel, see above, their angles
// must not differ more than. Thsi angle difference is also used when
// checking to see if a line is parallel to the line between a jump
// point and the door leaf
#define DOOREXTR_ANGDIFF_THRPAR    (10.0*M_PI/180.0) // degs

// When looking for a supporting line for a door candidate that line
// must have an angle that must not differ from the door angle more
// than
#define DOOREXTR_ANGDIFF_THR       (25.0*M_PI/180.0)   // degs

// Maximum opening angle of the door to say that it is likely to be a
// door
#define DOOREXTR_MIN_OPANGLE          1.35   // rads

// For a line to be classified as a door leff it must not differ more
// than this much from the door width
#define DOOREXTR_DIFF_DOORLEAFLEN   0.3  // in m

// To be strong enough a single wall line must be at least this long
// to be able to support a door leaf
#define DOOREXTR_MIN_SUPPLEN       0.2  // in m

// The maximum distance between the hinge and the wall line 
#define DOOREXTR_MAX_HINGE2LINEDIST 0.1  // in m

// Threshold for defining a jump point, discontinuity must be more
// than this to be called a jump point
#define DOOREXTR_JUMP_THRES         0.5  // in m

// The wall have the following thichness
#define DOOREXTR_WALL_THICKNESS      0.5  // in m

// The tolerance when detecting the wall thickness is
#define DOOREXTR_WALLTHICKNESS_TOL   0.25  // in m

// The maximum allowed distance from the end point of a line to the
// jump point or a door leaf
#define DOOREXTR_MAXDIST_PT2PT   0.2  // in m

// The maximum allowed distance from the end point of a line to the
// jump point or a door leaf in findThickWallLeft3Lines and
// findThickWallRight3Lines
#define DOOREXTR_MAXDIST_WALL2JUMP  0.4  // in m

// The maximum allowed distance between lined defined by the wall line
// and a point that is supposed to be part of the wall on the other
// side of the door
#define DOOREXTR_PT2LINE_THR     0.1  // in m

// The lines representing the thichness of the walls should be close
// to parallel to be classified as such. The maximum deviation is
#define DOOREXTR_MAXANG_WALL         (30.0*M_PI/180.0) // 30 degs in rads

// When looking for a door based only parallel lines we demand that
// the point between the lines be behind at the door opening at least
// this much
#define DOOREXTR_MIN_BEHIND         0.5  // in m

// When looking for a door based only parallel lines we demand that
// the point between the lines be behind or only this much infront of
// the lines. Only a fraction of the poinst are allowed to be infront
// though
#define DOOREXTR_MAX_INFRONT        0.2  // in m 

// Margin from the side outside which we look for points that are not
// far enough behind the door candidate
#define DOOREXTR_SIDEDIST_MARGIN_OFFSET   0.05 // in m
#define DOOREXTR_SIDEDIST_MARGIN_SLOPE    0.17 // in tan(10deg)

// When looking for a door based only parallel lines we demand that
// points are behind the door. A fraction is allowed to be infront though
// to allow for outliers
#define DOOREXTR_MAX_INFRONT_FRACTION 0.2 // dim.less

// To be sure be more certain that we find a door we also demand that
// we find a point between the doors which are at least this much
// behind the doors
#define DOOREXTR_MUCH_BEHIND       1.0  // in m

// To avoid detecting window to often we require that the point in the
// door not be more than this much more away than the line bwteen the
// door posts. This means that we will not be able to detect a door
// going into a corridor that is very long though.
#define DOOREXTR_WINDOW_THRES     20.0  // m

// When merging two door their angles must not differ more than this.
#define DOOREXTR_MAX_MERGEANGDIFF     0.5236  // 30 degs in rads

// When merging two door their center points must not be further part than
#define DOOREXTR_MAX_MERGEDISTDIFF  1.0  // in m

#include <fstream>

using namespace Cure;
using namespace Cure::HelpFunctions;

typedef std::list<RedundantLine2DRep>::iterator LineIter;
typedef std::list<RedundantLine2DRep>::iterator DoorIter;

//#define PRINTDOORSTUFF

#ifdef PRINTDOORSTUFF
std::fstream fs_jumps;
std::fstream fs_rawDoors;
std::fstream fs_doorsFound;
#endif

DoorExtractor::DoorExtractor()
  :r_(0),
   a_(0),
   x_(0),
   y_(0)
{
  init();
}

void
DoorExtractor::init()
{
  m_Lsq = new Cure::LsqLineExtractor(6,     // min numb of points in a line 
                                     0.25,   // min length of a line
                                     0,     // n outliers in a row allowed
                                     0.075, // max pt to line dist
                                     3.0,   // max point dist ratio
                                     0.15); // max point dist

  setUsedAlgorithms(Cure::DoorExtractor::EXTR_2PAR |
                    Cure::DoorExtractor::EXTR_THICK2PAR |
                    Cure::DoorExtractor::EXTR_THICK3LIN);

#ifdef PRINTDOORSTUFF
  fs_jumps.open("jumpPts.dat", std::ios::out);
  fs_rawDoors.open("rawDoors.dat", std::ios::out);
  fs_doorsFound.open("doorsFound.dat", std::ios::out);
#endif

}

DoorExtractor::~DoorExtractor()
{
#ifdef PRINTDOORSTUFF
  fs_jumps.close();
  fs_rawDoors.close();
  fs_doorsFound.close();
#endif

  delete m_Lsq;
}
 
int 
DoorExtractor::extract(Cure::SICKScan &scan, int algs)
{
  m_Timestamp = scan.getTime();

  if (algs < 0) algs = m_Algorithms;

  // Clear the list of doors
  doors_.clear();

  if (scan.getNPts() <= 0) return nDoors();

  if (x_) delete [] x_;
  if (y_) delete [] y_;
  if (r_) delete [] r_;
  if (a_) delete [] a_;

  nData_ = scan.getNPts();
  x_ = new double[nData_];
  y_ = new double[nData_];
  r_ = new double[nData_];
  a_ = new double[nData_];

  // Find the distances and angles
  double a = scan.getStartAngle();
  double aStep = scan.getAngleStep();
  for (int i = 0; i < scan.getNPts(); i++) {
    a_[i] = a;
    r_[i] = scan.getRange(i);
    x_[i] = r_[i] * cos(a_[i]);
    y_[i] = r_[i] * sin(a_[i]);
    a += aStep;
  }

  // Extract lins to be used to verify doors 
  m_Lsq->extract(x_, y_, r_, nData_);
  CureCERR(60) << "Got " << lsqLines().size() << " lines.\n";

  if (m_Algorithms & (EXTR_THICK2PAR | EXTR_THICK3LIN)) {
    // Extract jump points in the data
    findJumps();
  }

  if (algs & EXTR_2PAR) {
    // Look for door based on only parallel lines and one point that is
    // much behind the lines, all other point between being behind or
    // very little infront
    findParLinesDoors();
  }

  if (algs & EXTR_THICK2PAR) {
    findThickWallDoor2ParLines();
  }

  if (algs & EXTR_THICK3LIN) {
    findThickWallRight3Lines();
    findThickWallLeft3Lines();
  }

  /*
  if (algs & EXTR_IN2LINES) {
    findInRightDoor2Lines();
    findInLeftDoor2Lines();
  }

  if (algs & EXTR_3LINES) {
    findDoor3Lines();
  }

  if (algs & EXTR_OUT2LINES) {
    findOutRightDoor2Lines();
    findOutLeftDoor2Lines();
  }
  */

  // Fix doors that were found by multiple methods
  cleanUpDoorList();

  return nDoors();
}

void
DoorExtractor::cleanUpDoorList()
{
  CureCERR(60) << "Fixing list of doors\n";
  
  CureDO(60) {
    CureCERR(0) << "Door list before:\n";
    for (DoorIter iter1 = doors_.begin(); iter1 != doors_.end(); iter1++) {
      iter1->print(std::cerr); 
    }
  }
  
  CureCERR(60) << "Fusing doubles\n";
  DoorIter iter2, tmpIter;
  for (DoorIter iter1 = doors_.begin(); iter1 != doors_.end(); iter1++) {
    //CureDO(60) { iter1->print(std::cerr); }
    iter2 = iter1;
    for (iter2++; iter2 != doors_.end(); ) {
      //CureDO(60) { iter2->print(std::cerr); }
      if ((fabs(angleDiffRad(iter1->theta(), iter2->theta())) <
	   DOOREXTR_MAX_MERGEANGDIFF) &&
	  (hypot(iter1->xC() - iter2->xC(), iter1->yC() - iter2->yC()) < 
           DOOREXTR_MAX_MERGEDISTDIFF)) {
	// These door are pretty close!! We will fuse them and place
	// the fused door into the first one (iter1) and then remove
	// the last one (iter2)
	
        CureDO(60) {
          CureCERR(0) << "Fusing these two doors:\n";
          iter1->print(std::cerr); 
          iter2->print(std::cerr);
        }
	
	// Fuse them
        double weightsum = iter1->getWeight() + iter2->getWeight();
	iter1->setEndPts((iter1->getWeight() * iter1->xS() + 
                          iter2->getWeight() * iter2->xS()) / weightsum,
                         (iter1->getWeight() * iter1->yS() + 
                          iter2->getWeight() * iter2->yS()) / weightsum,
                         (iter1->getWeight() * iter1->xE() + 
                          iter2->getWeight() * iter2->xE()) / weightsum,
                         (iter1->getWeight() * iter1->yE() + 
                          iter2->getWeight() * iter2->yE()) / weightsum);
	iter1->setWeight(weightsum);
	iter1->setKey(iter1->getKey() | iter2->getKey());
        
	// Remove the last one of them
	tmpIter = iter2;
	iter2++;
	CureDO(60) { 
          CureCERR(0) << "Removing "; tmpIter->print(std::cerr); 
        }
	doors_.erase(tmpIter);
      } else {
	iter2++;
      }
    }
  }

  CureDO(-60) {
    if (doors_.empty()) {
      CureDO(0) {
        CureCERR(0) << "No dooors found "; 
        fprintf(stderr, "t=%ld.%06ld\n", 
                m_Timestamp.Seconds, m_Timestamp.Microsec);
      }
    } else {
      CureDO(0) {
        CureCERR(0) << "List of doors ";
        fprintf(stderr, "(t=%ld.%06ld)\n", 
                m_Timestamp.Seconds, m_Timestamp.Microsec);
        for (DoorIter iter1 = doors_.begin(); iter1 != doors_.end(); iter1++) {
          std::cerr << "* "; iter1->print(std::cerr);
        }
      }
    }
  }
  
#ifdef PRINTDOORSTUFF
  for (DoorIter iter1 = doors_.begin(); iter1 != doors_.end(); iter1++)
    fs_doorsFound << iter1->getWeight() << " " 
	       << iter1->xS() << " " << iter1->yS() << " " 
	       << iter1->xE() << " " << iter1->yE() << std::endl;
  fs_doorsFound << "0 0 0 0 0" << std::endl;
  fs_rawDoors << "0 0 0 0 0" << std::endl;
#endif
}

void
DoorExtractor::findJumps()
{
  // Clear the jump edges lists
  shortLongJumps_.clear();
  longShortJumps_.clear();

  // Find the jumps
  for (int i = 0; i < nData_; i++) {

    // Add jump from short to long range reading
    if (r_[i+1] - r_[i] > DOOREXTR_JUMP_THRES)
      shortLongJumps_.push_back(i);

    // Add jump from long to short range reading
    if (r_[i] - r_[i+1] > DOOREXTR_JUMP_THRES)
      longShortJumps_.push_back(i+1);
  }
  
  // Since a measurement taken at the end of a structure is very
  // likely to be a phantom measurement we add the point before the
  // jump as well if it is not already in the list of jump points
  for (int i = 0; i < (nData_ - 2); i++) {

    // Add jump from short to long range reading
    if ((r_[i+2] - r_[i+1] > DOOREXTR_JUMP_THRES) &&
	(r_[i+1] - r_[i] > 0) &&  // a small jump to longer reading
	(r_[i+1] - r_[i] < DOOREXTR_JUMP_THRES))  // not in the list yet
      shortLongJumps_.push_back(i);

    // Add jump from long to short range reading
    if ((r_[i] - r_[i+1] > DOOREXTR_JUMP_THRES) && 
	(r_[i+1] - r_[i+2] > 0) &&   // a small jump to shorter reading
	(r_[i+1] - r_[i+2] < DOOREXTR_JUMP_THRES)) // not in the list yet
      longShortJumps_.push_back(i+2);
  }

  for (std::list<int>::iterator iter = shortLongJumps_.begin(); 
       iter != shortLongJumps_.end(); iter++) 
    CureCERR(60) << "sl " << x_[*iter] << " " << y_[*iter] << std::endl;

  for (std::list<int>::iterator iter = longShortJumps_.begin(); 
       iter != longShortJumps_.end(); iter++)
    CureCERR(60) << "ls " << x_[*iter] << " " << y_[*iter] << std::endl;


#ifdef PRINTDOORSTUFF
  for (std::list<int>::iterator iter = shortLongJumps_.begin(); 
       iter != shortLongJumps_.end(); iter++) 
    fs_jumps << "1 " << x_[*iter] << " " << y_[*iter] << std::endl;
  for (std::list<int>::iterator iter = longShortJumps_.begin(); 
       iter != longShortJumps_.end(); iter++)
    fs_jumps << "2 " << x_[*iter] << " " << y_[*iter] << std::endl;
  fs_jumps << "0 0 0" << std::endl;
#endif

  CureCERR(60) << "Got " << shortLongJumps_.size() << " short/long  and "
               << longShortJumps_.size() << " long/short jumps.\n";
}

void 
DoorExtractor::findParLinesDoors()
{
  CureCERR(60) << "findParLinesDoors\n";

  double width = 0;

  for (LineIter wallLineR = lsqLines().begin();
       wallLineR != lsqLines().end(); wallLineR++) {

    if (fabs(angleDiffRad(wallLineR->theta(), 1.3)) > 0.5) continue;

    for (LineIter wallLineL = lsqLines().begin();
	 wallLineL != lsqLines().end(); wallLineL++) {

      if (fabs(angleDiffRad(wallLineL->theta(), 1.3)) > 0.5) continue;

      if (// Not the same line
	  (wallLineR != wallLineL) && 

	  // Left wall to the left
	  (angleDiffRad(atan2(wallLineL->yS(), wallLineL->xS()),
			atan2(wallLineR->yE(), wallLineR->xE())) > 0) &&

	  // Right opening size
	  ((width = hypot(wallLineR->xE() - wallLineL->xS(),
                          wallLineR->yE() - wallLineL->yS())) <
	   DOOREXTR_MAX_DOORWIDTH) &&
	  (DOOREXTR_MIN_DOORWIDTH < width) &&

          linesParallel(wallLineR, wallLineL)) {
        
        CureDO(60) {
          CureCERR(0) << "Got a candidate with door width: " << width 
                      << std::endl;
          std::cerr << "R: "; wallLineR->print(std::cerr);
          std::cerr << "L: "; wallLineL->print(std::cerr);
        }

	// Now we check that all point that are between the lines are
	// behind or very little infront of the lines, plus that at
	// least one point is much behind.
	
	if (doorOpeningVerified(wallLineR->xE(), wallLineR->yE(),
				wallLineL->xS(), wallLineL->yS())) {
          CureDO(60) {
            CureCERR(0) << "Got a door:\n";
            std::cerr << "R: "; wallLineR->print(std::cerr);
            std::cerr << "L: "; wallLineL->print(std::cerr);
          }
	  // We have a door!
	  addNewDoor(EXTR_2PAR, 
                     wallLineR->xE(), wallLineR->yE(),
		     wallLineL->xS(), wallLineL->yS());
	}
      }
    }
  }
}

bool
DoorExtractor::linesParallel(LineIter &wallLineR, LineIter &wallLineL)
{
  // Walls have similar enough orientation
  double adiff = angleDiffRad(wallLineR->theta(), wallLineL->theta());
  if (fabs(adiff) > DOOREXTR_ANGDIFF_THRPAR) return false;    
  // If we draw a line between the first start point and the last end
  // point the points in between are close the lines that they form
  double distE = distPt2LinePts(wallLineR->xS(), wallLineR->yS(),
                                wallLineL->xE(), wallLineL->yE(),
                                wallLineR->xE(), wallLineR->yE());
  if (fabs(distE) > DOOREXTR_MAX_CDIFF) return false;
  double distS = distPt2LinePts(wallLineR->xS(), wallLineR->yS(),
                                wallLineL->xE(), wallLineL->yE(),
                                wallLineL->xS(), wallLineL->yS());
  if (fabs(distS) > DOOREXTR_MAX_CDIFF) return false;

  return true;
}

void 
DoorExtractor::findThickWallDoor2ParLines()
{
  CureCERR(60) << "findThickWallDoor2ParLines\n";
  
  // This function specializes in finding doors in buildings with very
  // thick walls like at the lab floor at CVAP.
  // Model:
  // * Two lines that are close to parallel and of approximetley the
  //   wall thickness long, typically 0.500 m. The right line's end point and
  //   the left ones start point is the door opening. The lines should be 
  //   close to parallel or slightly verged.
  // * To support this I require 2 jump point, one at each door post or
  //   a door leaf at one side and a jump point at the other

  double width = 0;
  double doorAng = 0;

  // Iterate over possible right wall lines
  for (LineIter wallLineR = lsqLines().begin();
       wallLineR != lsqLines().end(); wallLineR++) {

    bool restart = false;
	
    // Right wall is of the right length
    if (fabs(2.0 * wallLineR->h() - DOOREXTR_WALL_THICKNESS) <
	DOOREXTR_WALLTHICKNESS_TOL) {

      // Iterate over possible left wall lines
      for (LineIter wallLineL = lsqLines().begin();
	   !restart && wallLineL != lsqLines().end(); wallLineL++) {

        if (// Not the same lines
            (wallLineL != wallLineR) &&

	    // Left wall is of the right length
	    (fabs(2.0 * wallLineL->h() - DOOREXTR_WALL_THICKNESS) <
	     DOOREXTR_WALLTHICKNESS_TOL) &&
            
            // Door opening size is ok
            (DOOREXTR_MIN_DOORWIDTH < 
             (width = hypot(wallLineL->xE() - wallLineR->xS(),
                            wallLineL->yE() - wallLineR->yS()))) &&
            (width < DOOREXTR_MAX_DOORWIDTH) ) {

          // The angle of a line between the two door post candidates
          // have the following angle
          doorAng = atan2(wallLineL->yE() - wallLineR->yS(),
                          wallLineL->xE() - wallLineR->xS());
          
          // Check that they match the idea of two pieces of wall
          if (// Left wall to the left
              (angleDiffRad(atan2(wallLineL->yS(), wallLineL->xS()),
                            atan2(wallLineR->yE(), wallLineR->xE())) > 0) &&
              
              // The right wall thickness line is close to orthogonal to opening
              (fabs(angleDiffRad(doorAng - M_PI_2, wallLineR->theta())) < 
               DOOREXTR_MAXANG_WALL) &&
              
              // The right wall thickness line is close to ortogonal to opening
              (fabs(angleDiffRad(doorAng + M_PI_2, wallLineL->theta())) < 
               DOOREXTR_MAXANG_WALL) ) {

            CureDO(60) {
              CureCERR(0) << "Got a candidate with door width: " 
                          << width << "\n";
              std::cerr << "R: "; wallLineR->print(std::cerr);
              std::cerr << "L: "; wallLineL->print(std::cerr);
            }
            
            // Look for a jump point close to the end point of the right
            // wall piece
            for (std::list<int>::iterator jumpPtR = shortLongJumps_.begin(); 
                 !restart && jumpPtR != shortLongJumps_.end(); jumpPtR++) {

              if (hypot(x_[*jumpPtR] - wallLineR->xE(),
                        y_[*jumpPtR] - wallLineR->yE()) <
                  DOOREXTR_MAXDIST_PT2PT) {
                
                //Try finding a jump point on the other side
                for (std::list<int>::iterator jumpPtL = longShortJumps_.begin(); 
                     !restart && jumpPtL != longShortJumps_.end(); jumpPtL++) {

                  if (hypot(x_[*jumpPtL] - wallLineL->xS(),
                            y_[*jumpPtL] - wallLineL->yS()) <
                      DOOREXTR_MAXDIST_PT2PT) {

                    if (doorOpeningVerified(wallLineR->xE(), wallLineR->yE(),
                                            wallLineL->xS(), wallLineL->yS())) {
                      CureDO(60) {
                        CureCERR(0) << "Got a door 1: \n";
                        std::cerr << "R: "; wallLineR->print(std::cerr);
                        std::cerr << "L: "; wallLineL->print(std::cerr);
                      }
                      addNewDoor(EXTR_THICK2PAR,
                                 0.5 * (wallLineR->xS() + wallLineR->xE()),
                                 0.5 * (wallLineR->yS() + wallLineR->yE()),
                                 0.5 * (wallLineL->xS() + wallLineL->xE()),
                                 0.5 * (wallLineL->yS() + wallLineL->yE()));

                      restart = true;
                    }
                  }
                }

                // And we also try to match with a door leaf on that side
                for (LineIter doorLeaf = lsqLines().begin();
                     doorLeaf != lsqLines().end(); doorLeaf++) {
                  if ((fabs(2.0 * doorLeaf->h() - width) <
                       DOOREXTR_DIFF_DOORLEAFLEN) &&
                      (hypot(wallLineL->xS() - doorLeaf->xE(),
                             wallLineL->yS() - doorLeaf->yE()) <
                       DOOREXTR_MAXDIST_PT2PT)) {

                    CureDO(60) {
                      CureCERR(0) << "Got a door 2: \n";
                      std::cerr << "R: "; wallLineR->print(std::cerr);
                      std::cerr << "L: "; wallLineL->print(std::cerr);
                    }
                    addNewDoor(EXTR_THICK2PAR,
                               0.5 * (wallLineR->xS() + wallLineR->xE()),
                               0.5 * (wallLineR->yS() + wallLineR->yE()),
                               0.5 * (wallLineL->xS() + wallLineL->xE()),
                               0.5 * (wallLineL->yS() + wallLineL->yE()));
		}   
                }
              }
            }  // End looking for jump on right side
            
            // Look for a jump point close to the start point of the
            // left line. We only have to consider the case of a door
            // leaf on the other side as the two jump point case has
            // already been taken care off.
            for (std::list<int>::iterator jumpPtL = longShortJumps_.begin(); 
                 jumpPtL != longShortJumps_.end(); jumpPtL++) {
              if (hypot(x_[*jumpPtL] - wallLineL->xS(), 
                        y_[*jumpPtL] - wallLineL->yS()) <
                  DOOREXTR_MAXDIST_PT2PT) {
                
                // Now we try to find a door leaf on the right side
                for (LineIter doorLeaf = lsqLines().begin();
                     doorLeaf != lsqLines().end(); doorLeaf++) {
                  if ((fabs(2.0 * doorLeaf->h() - width) <
                       DOOREXTR_DIFF_DOORLEAFLEN) &&
                      (hypot(wallLineR->xE() - doorLeaf->xS(),
                             wallLineR->yE() - doorLeaf->yS()) <
                       DOOREXTR_MAXDIST_PT2PT)) {

                    CureDO(60) {
                      CureCERR(0) << "Got a door 3: \n";
                      std::cerr << "R: "; wallLineR->print(std::cerr);
                      std::cerr << "L: "; wallLineL->print(std::cerr);
                    }
                    addNewDoor(EXTR_THICK2PAR,
                               0.5 * (wallLineR->xS() + wallLineR->xE()),
                               0.5 * (wallLineR->yS() + wallLineR->yE()),
                               0.5 * (wallLineL->xS() + wallLineL->xE()),
                               0.5 * (wallLineL->yS() + wallLineL->yE()));
                  }
                }
              }
            } // End looking for jump on left side
            
          }
        }
      }
    }
  }
}

void
DoorExtractor::findThickWallRight3Lines()
{
  CureCERR(60) << "findThickWallRight3Lines\n";

  // This function specializes in finding doors in buildings with very
  // thich walls like at CVAP.
  // Model:
  // - 1 main wall line on the left
  // - 1 jump point at the start of the wall line
  // - 1 line from the wall thickness on the other side of the opening
  // - 1 supporting wall line end at the start of the wall thickness line

  double aDiff = 0;
  double doorAng = 0;
  double width = 0;
  
  // Iterate over possible left wall lines
  for (LineIter wallLineL = lsqLines().begin();
       wallLineL != lsqLines().end(); wallLineL++) {
    
    // Check that there is a jump point close to the end of it
    bool foundJumpPt = false;
    for (std::list<int>::iterator jumpPtL = longShortJumps_.begin(); 
	 !foundJumpPt && jumpPtL != longShortJumps_.end(); jumpPtL++) {

      if (hypot(x_[*jumpPtL] - wallLineL->xS(), 
                y_[*jumpPtL] - wallLineL->yS()) <
	  DOOREXTR_MAXDIST_WALL2JUMP) {

        // Since we do not use the jump point for anything except
        // verifying the wall line we should not look for a second
        // matching one but instead stop the loop after finding the
        // first one as the rest of the computations will be the same
        foundJumpPt = true;
        
	// Look for the wall thickness wall
	for (LineIter thickWall = lsqLines().begin();
	     thickWall != lsqLines().end(); thickWall++) {
	  if (// Not the same as the left wall
	      (thickWall != wallLineL) &&

	      // Thick wall to the right
	      (angleDiffRad(atan2(wallLineL->yS(), wallLineL->xS()),
                            atan2(thickWall->yE(), thickWall->xE())) > 0) &&

	      // Left wall parallel to line between start of wall
	      // thickness line and start of left line
	      (fabs(aDiff = angleDiffRad(wallLineL->theta(), 
					 (doorAng = atan2(wallLineL->yS() -
                                                          thickWall->yS(),
							  wallLineL->xS() -
                                                          thickWall->xS())))) <
	       DOOREXTR_ANGDIFF_THR) && 
	      
	      // Door opening size is ok
	      (DOOREXTR_MIN_DOORWIDTH < 
	       (width = hypot(thickWall->xS() - wallLineL->xS(),
                              thickWall->yS() - wallLineL->yS()))) &&
	      (width < DOOREXTR_MAX_DOORWIDTH) &&

	      // Thick wall is of the right length
	      (fabs(thickWall->h() - 0.5*DOOREXTR_WALL_THICKNESS) <
	       DOOREXTR_WALLTHICKNESS_TOL) &&
	      
	      // The wall thickness line is close to orthogonal to
	      // opening
	      (fabs(angleDiffRad(doorAng - M_PI_2, thickWall->theta())) < 
	       DOOREXTR_MAXANG_WALL)) {

            CureDO(60) {
              CureCERR(0) << "Found a candidate with width=" << width 
                          << " doorAng=" 
                          << Cure::HelpFunctions::rad2deg(doorAng)
                          << std::endl;
              std::cerr << "L: "; wallLineL->print(std::cerr);
              std::cerr << "T: "; thickWall->print(std::cerr);
            }
	    
	    // Iterate over possible support lines
	    for (LineIter wallLineR = lsqLines().begin();
		 wallLineR != lsqLines().end(); wallLineR++) {
	      if (// Not same as left or thick wall
                  (wallLineR != wallLineL) && (wallLineR != thickWall) && 

		  // At least some minimum length of support line
		  (wallLineR->h() > 0.5 * DOOREXTR_MIN_SUPPLEN) &&

		  // End of support line close enough to start of wall
		  // thickness line
                  (hypot(thickWall->xS() - wallLineR->xE(),
                         thickWall->yS() - wallLineR->yE()) <
                   DOOREXTR_MAXDIST_PT2PT)) {

                CureDO(60) {
                  CureCERR(0) << "Got a candidate with door witdh=" 
                              << width<<"\n";
                  std::cerr << "L: "; wallLineL->print(std::cerr);
                  std::cerr << "T: "; thickWall->print(std::cerr);
                  std::cerr << "R: "; wallLineR->print(std::cerr);
                }

                // Get a better estimate for the left door post by
                // looking at the intersection between the thick wall
                // and the left wall segment
                double t = 1;
                Cure::HelpFunctions::linesIntersect(wallLineR->xS(),
                                                    wallLineR->yS(),
                                                    wallLineR->xE(),
                                                    wallLineR->yE(),
                                                    thickWall->xS(),
                                                    thickWall->yS(),
                                                    thickWall->xE(),
                                                    thickWall->yE(),
                                                    &t);
                double xS, yS;
                if (fabs((t-1)*wallLineR->h()*2.0) > DOOREXTR_MAXDIST_PT2PT) {
                  // Intersection point is far away and probable
                  // caused by wallLineL and thickWall being close to
                  // parallel. Use the end of wallLineR instead.
                  xS = wallLineR->xE();
                  yS = wallLineR->yE();
                } else {

                  xS = wallLineR->xS() + t * (wallLineR->xE() -
                                              wallLineR->xS());
                  yS = wallLineR->yS() + t * (wallLineR->yE() -
                                              wallLineR->yS());
		}

		if (doorOpeningVerified(xS, yS, 
                                        wallLineL->xS(), wallLineL->yS())) {
		  
                  CureDO(60) {
                    CureCERR(0) << "Got a door: \n";
                    std::cerr << "R: "; wallLineR->print(std::cerr);
                    std::cerr << "T: "; thickWall->print(std::cerr);
                    std::cerr << "L: "; wallLineL->print(std::cerr);
                  }
		  addNewDoor(EXTR_THICK3LIN, 
			     xS, yS, wallLineL->xS(), wallLineL->yS());
		}
              }
	    }
          }
	}
      }
    }
  }
}

void
DoorExtractor::findThickWallLeft3Lines()
{
  CureCERR(60) << "findThickWallLeft3Lines\n";

  // This function specializes in finding doors in buildings with very
  // thich walls like at CVAP.
  // Model:
  // - 1 main wall line on the right
  // - 1 jump point at the end of the wall line
  // - 1 line from the wall thickness on the left side of the opening
  // - 1 supporting wall line starting at the end of the wall thickness line

  double aDiff = 0;
  double doorAng = 0;
  double width = 0;
  
  // Iterate over possible right wall lines
  for (LineIter wallLineR = lsqLines().begin();
       wallLineR != lsqLines().end(); wallLineR++) {
    
    // Check that there is a jump point close to the end of it
    bool foundJumpPt = false;
    for (std::list<int>::iterator jumpPtR = shortLongJumps_.begin(); 
	 !foundJumpPt && jumpPtR != shortLongJumps_.end(); jumpPtR++) {

      if (hypot(x_[*jumpPtR] - wallLineR->xE(), 
                y_[*jumpPtR] - wallLineR->yE()) <
	  DOOREXTR_MAXDIST_WALL2JUMP) {

        // Since we do not use the jump point for anything except
        // verifying the wall line we should not look for a second
        // matching one but instead stop the loop after finding the
        // first one as the rest of the computations will be the same
        foundJumpPt = true;
        
	// Look for the wall thickness wall
	for (LineIter thickWall = lsqLines().begin();
	     thickWall != lsqLines().end(); thickWall++) {
	  if (// Not the same as the right wall
	      (thickWall != wallLineR) &&

	      // Thick wall to the left
	      (angleDiffRad(atan2(thickWall->yS(), thickWall->xS()),
			    atan2(wallLineR->yE(), wallLineR->xE())) > 0) &&

	      // Right wall parallel to line between end of wall
	      // thickness line and end of right line
	      (fabs(aDiff = angleDiffRad(wallLineR->theta(), 
					 (doorAng = atan2(thickWall->yE() - 
							  wallLineR->yE(),
							  thickWall->xE() - 
							  wallLineR->xE())))) <
	       DOOREXTR_ANGDIFF_THR) && 
	      
	      // Door opening size is ok
	      (DOOREXTR_MIN_DOORWIDTH < 
	       (width = hypot(thickWall->xE() - wallLineR->xE(),
                              thickWall->yE() - wallLineR->yE()))) &&
	      (width < DOOREXTR_MAX_DOORWIDTH) &&

	      // Thick wall is of the right length
	      (fabs(thickWall->h() - 0.5*DOOREXTR_WALL_THICKNESS) <
	       DOOREXTR_WALLTHICKNESS_TOL) &&
	      
	      // The wall thickness line is close to orthogonal
	      // to opening
	      (fabs(angleDiffRad(doorAng + M_PI_2, thickWall->theta())) < 
	       DOOREXTR_MAXANG_WALL)) {

            CureDO(60) {
              CureCERR(0) << "Found a candidate with width=" << width 
                          << " doorAng=" 
                          << Cure::HelpFunctions::rad2deg(doorAng)
                          << std::endl;
              std::cerr << "R: "; wallLineR->print(std::cerr);
              std::cerr << "T: "; thickWall->print(std::cerr);
            }
	    
	    // Iterate over possible support lines
	    for (LineIter wallLineL = lsqLines().begin();
		 wallLineL != lsqLines().end(); wallLineL++) {
	      if (// Not same as right or thick wall
                  (wallLineL != wallLineR) && (wallLineL != thickWall) && 

		  // At least some minimum length of support line
		  (wallLineL->h() > 0.5 * DOOREXTR_MIN_SUPPLEN) &&

		  // Start of support line close enough to end of wall
		  // thickness line
                  (hypot(thickWall->xE() - wallLineL->xS(),
                         thickWall->yE() - wallLineL->yS()) <
                   DOOREXTR_MAXDIST_PT2PT)) {

                CureDO(60) {
                  CureCERR(0) << "Got a candidate with door witdh=" 
                              << width<<"\n";
                  std::cerr << "R: "; wallLineR->print(std::cerr);
                  std::cerr << "T: "; thickWall->print(std::cerr);
                  std::cerr << "L: "; wallLineL->print(std::cerr);
                }

                // Get a better estimate for the left door post by
                // looking at the intersection between the thick wall
                // and the left wall segment
                double t = 0;
                Cure::HelpFunctions::linesIntersect(wallLineL->xS(),
                                                    wallLineL->yS(),
                                                    wallLineL->xE(),
                                                    wallLineL->yE(),
                                                    thickWall->xS(),
                                                    thickWall->yS(),
                                                    thickWall->xE(),
                                                    thickWall->yE(),
                                                    &t);

                double xE, yE;
                if (fabs(t*wallLineL->h()*2.0) > DOOREXTR_MAXDIST_PT2PT) {
                  // Intersection point is far away and probable
                  // caused by wallLineL and thickWall being close to
                  // parallel. Use the start of wallLineL instead.
                  xE = wallLineL->xS();
                  yE = wallLineL->yS();
                } else {

                  xE = wallLineL->xS() + t * (wallLineL->xE() -
                                              wallLineL->xS());
                  yE = wallLineL->yS() + t * (wallLineL->yE() -
                                              wallLineL->yS());
		}

		if (doorOpeningVerified(wallLineR->xE(), wallLineR->yE(),
                                        xE, yE)) {
		  
                  CureDO(60) {
                    CureCERR(0) << "Got a door: \n";
                    std::cerr << "R: "; wallLineR->print(std::cerr);
                    std::cerr << "T: "; thickWall->print(std::cerr);
                    std::cerr << "L: "; wallLineL->print(std::cerr);
                  }
		  addNewDoor(EXTR_THICK3LIN, 
			     wallLineR->xE(), wallLineR->yE(), xE, yE);
		}
              }
	    }
	  }
	}
      }
    }
  }
}

bool
DoorExtractor::doorOpeningVerified(double xS, double yS, 
				   double xE, double yE,
                                   bool ignoreSidePoints)
{
  // We use the sine-theorem to calculate the expected distance
  // at a certain angle given that we use linear interpolation
  // of the distance between the two line end points.
  
  // First we find the index of the first point after the end of the
  // right line.
  double startAng = atan2(yS, xS);
  int startIndex = -1;
  for (int ii = 0; ((startIndex == -1) && ii < nData_); ii++) {
    double da = angleDiffRad(a_[ii], startAng);
    // Check if the angle is larger than but not too large meaning
    // that it is positive but close to PI
    if (da > 0 && fabs(da) < 0.1) {
      startIndex = ii;
    }
  }
  CureCERR(60) << "StartAng = " << startAng * 180.0 / M_PI
               << " startIndex = " << startIndex
               << " a_[startindex] = " << a_[startIndex] * 180.0 / M_PI
               << std::endl;
  
  // Then we look for the index of the point just before the start
  // point of the left line
  double stopAng = atan2(yE, xE);
  int stopIndex = -1;
  for (int ii = nData_ - 1; ((stopIndex == -1) && ii >= 0); ii--) {
    double da = angleDiffRad(a_[ii], stopAng);
    if (da < 0 && fabs(da) < 0.1) {
      stopIndex = ii;
    }
  }
  if (stopIndex == -1) stopIndex = nData_ - 1;
  
  CureCERR(60) << "StopAng = " << stopAng * 180.0 / M_PI 
               << " stopIndex = " << stopIndex
               << " a_[stopindex] = " << a_[stopIndex] * 180.0 / M_PI
               << std::endl;

  // Now we loop over the the points between the line ends and
  // check that they are behind, we also keep our eyes open for
  // point that are far behind to indicate that it actually is a
  // door.
  bool foundFarBehindPoint = false;
  bool doorStillOK = true;
  int infrontCntr = 0;
  double width = hypot(yE - yS, xE - xS);
  double doorAng = atan2(yE - yS, xE - xS);
  double cosL = cos(doorAng);
  double sinL = sin(doorAng);
  double distBehind = 0, sideDist = 0;
  for (int i = startIndex; (doorStillOK && (i< stopIndex)); i++) {

    distBehind = distPt2Line(x_[i], y_[i], xS, yS, cosL, sinL);

    if (distBehind > DOOREXTR_MUCH_BEHIND) {
      foundFarBehindPoint = true;
    }

    if (ignoreSidePoints) {
      // First we check if the point is not too close to the "sides" of
      // the door which will typically be explained by points that come
      // from the thickness of the wall, an open door left etc
      // cos(a + pi/2) = -sin(a)
      // sin(a + pi/2) = cos(a)
      sideDist = distPt2Line(x_[i], y_[i], xS, yS, -sinL, cosL);
      if (sideDist > width/2) sideDist = width - sideDist;
      double d = (distBehind<0?0:distBehind);
      if (sideDist < (DOOREXTR_SIDEDIST_MARGIN_OFFSET + 
                      d * DOOREXTR_SIDEDIST_MARGIN_SLOPE)) {
        /*
        std::cerr << i << "; " << sideDist << "; "
                  << (DOOREXTR_SIDEDIST_MARGIN_OFFSET + 
                      d * DOOREXTR_SIDEDIST_MARGIN_SLOPE) << "; "
                  << distBehind << "; hold on; plot(" << x_[i] << ", " 
                  << y_[i] << ",\'kx\'); hold off\n";
        */
        continue;
      } else {
        /*
        std::cerr << i << "; " << sideDist << "; "
                  << (DOOREXTR_SIDEDIST_MARGIN_OFFSET + 
                      d * DOOREXTR_SIDEDIST_MARGIN_SLOPE) << "; "
                  << distBehind << "; hold on; plot(" << x_[i] << ", " 
                  << y_[i] << ",\'ko\'); hold off\n";
        */
      }
    }

    if (distBehind < DOOREXTR_MIN_BEHIND &&
        distBehind > -DOOREXTR_MAX_INFRONT) {

      // This point is not enough infront of the door and not not far
      // enough behind it. This means we discard the door
      doorStillOK = false;	    

      CureCERR(60) << "Not a door! A point in the door opening\n";

    } else if (distBehind < -DOOREXTR_MAX_INFRONT) {

      infrontCntr++;

    }
  }

  if (foundFarBehindPoint && doorStillOK) {

    if (1.0 * infrontCntr / (stopIndex - startIndex) > 
	DOOREXTR_MAX_INFRONT_FRACTION) {
      CureCERR(60) << "Not a door! Too many points infront of it "
		<< infrontCntr << " out of " << stopIndex - startIndex 
		<< ")\n";
      return false;
    } else {
      return true;
    }
  } else {
    return false;
  }
}

int 
DoorExtractor::addNewDoor(int method,
			  double xR, double yR, double xL, double yL)
{
  RedundantLine2DRep newDoor;
  newDoor.setEndPts(xR, yR, xL, yL);

  // We give all algorithms the same weight
  newDoor.setWeight(1);  

  // Mark the algorithm used for this
  newDoor.setKey(method);

  doors_.push_back(newDoor);

  CureDO(60) {
    CureCERR(0) << "Added new door to the list\n";
    std::cerr <<"new:"; newDoor.print(std::cerr); 
  }

#ifdef PRINTDOORSTUFF
  fs_rawDoors << method << " " 
	   << xR << " " << yR << " " << xL << " " << yL << std::endl;
#endif

  return int(doors_.size());
}

/*

void 
DoorExtractor::findInRightDoor2Lines()
{
  CureCERR(60) << "findInRightDoor2Lines\n";

  // This function will look for a right hand side door leaf going into the
  // room where the scan was taken. 
  // Model: 
  // * 1 door leaf where the end point is were the hinge is, length correct.
  // * 1 supporting line which angle is close to the angle of the line 
  //     between the door leaf hinge and the start of the line. 
  // * All points between door hing and start point of line lie behind 
  //     the line between the two door posts.

  double width = 0;
  double aDiff = 0;

  // Iterate over possible door leafs
  for (LineIter doorLeaf = lsqLines().begin();
       doorLeaf != lsqLines().end(); doorLeaf++) {
    
    // Iterate over possible wall lines
    for (LineIter wallLine = lsqLines().begin();
	 wallLine != lsqLines().end(); wallLine++) {
      
      // Check that the angle, length and other conditions are met
      if (// Door leaf not the supporting wall
	  (doorLeaf != wallLine) &&

	  // Supportive wall is long enough
	  (2.0 * wallLine->h() > DOOREXTR_MIN_SUPPLEN) &&

	  // Door leaf on the right side
	  (angleDiffRad(atan2(wallLine->yS(), wallLine->xS()),
			atan2(doorLeaf->yE(), doorLeaf->xE())) > 0) &&

	  // Supportive line is parallel to door opening
	  (fabs(distPt2Line(doorLeaf->xE(), doorLeaf->yE(),
			    wallLine->xS(), wallLine->yS(), 
			    wallLine->theta())) < DOOREXTR_PT2LINE_THR) &&

	  // Door opens out
	  (angleDiffRad(wallLine->theta(), doorLeaf->theta()) > 
	   DOOREXTR_ANGDIFF_THRPAR) &&

	  // Is more than half open, otherwise we demand to see other wall
	  (angleDiffRad(wallLine->theta(), doorLeaf->theta()) < 0.5 * M_PI) &&
	  
	  // Door opening of right size
	  (DOOREXTR_MIN_DOORWIDTH < 
	   (width = hypot(wallLine->yS() - doorLeaf->yE(),
                          wallLine->xS() - doorLeaf->xE()))) &&
	  (width < DOOREXTR_MAX_DOORWIDTH) &&
	  
	  // Door leaf of right size to fit opening
	  (fabs(2.0 * doorLeaf->h() - width) < DOOREXTR_DIFF_DOORLEAFLEN)) {
		
        
        CureDO(60) {
          CureCERR(0) << "Got a candidate with door width: " << width 
                      << std::endl;
          std::cerr << "W: "; wallLine->print(std::cerr);
          std::cerr << "D: "; doorLeaf->print(std::cerr);
        }
	
	// Now we check that all point that are between the lines are
	// behind or very little infront of the lines, plus that at
	// least one point is much behind.
	
	if (doorOpeningVerified(doorLeaf->xE(), doorLeaf->yE(),
				wallLine->xS(), wallLine->yS(), true)) {

          CureDO(60) {
            CureCERR(0) << "Got a door:\n";
            std::cerr << "D: "; doorLeaf->print(std::cerr);
            std::cerr << "W: "; wallLine->print(std::cerr);
          }
	  addNewDoor(EXTR_IN2LINES,
		     doorLeaf->xE(),
		     doorLeaf->yE(),
		     doorLeaf->xE() + 
		     (width * cos(aDiff) * cos(wallLine->theta())), 
		     doorLeaf->yE() + 
		     (width * cos(aDiff) * sin(wallLine->theta())));
	}
      }
    }
  }
}

void  
DoorExtractor::findInLeftDoor2Lines()
{
  CureCERR(60) << "findInLeftDoor2Lines\n";

  // This function will look for a left hand side door leaf going into the
  // room where the scan was taken. 
  // Model: 
  // * 1 door leaf where the start point is were the hinge is, length correct.
  // * 1 supporting line which angle is close to the angle of the line 
  //     between the start of the line and the door leaf hinge. 
  // * All points between start point of line and door hinge lie behind 
  //     the line between the two door posts.

  double width = 0;
  double aDiff = 0;

  // Iterate over possible door leafs
  for (LineIter doorLeaf = lsqLines().begin();
       doorLeaf != lsqLines().end(); doorLeaf++) {
    
    // Iterate over possible wall lines
    for (LineIter wallLine = lsqLines().begin();
	 wallLine != lsqLines().end(); wallLine++) {
      
      // Check that the angle, length and other conditions are met
      if (// Door leaf is no the same line as the supportive
	  (doorLeaf != wallLine) &&
	  
	  // Suportive line is long enough
	  (2.0 * wallLine->h() > DOOREXTR_MIN_SUPPLEN) &&
	  
	  // Door leaf to the left
	  (angleDiffRad(atan2(doorLeaf->yS(), doorLeaf->xS()),
			atan2(wallLine->yE(), wallLine->xE())) > 0) &&
	  
	  // Supportive line is parallel to door opening
	  (fabs(distPt2Line(doorLeaf->xS(), doorLeaf->yS(),
			    wallLine->xE(), wallLine->yE(), 
			    wallLine->theta())) < DOOREXTR_PT2LINE_THR) &&

	  // Door leaf opens in
	  (angleDiffRad(doorLeaf->theta(), wallLine->theta()) > 
	   DOOREXTR_ANGDIFF_THRPAR) &&
	  
	  // Is more than half open, otherwise we demand to see other wall
	  (angleDiffRad(doorLeaf->theta(), wallLine->theta()) < 0.5 * M_PI) &&
	  
	  // Right size of door opening
	  (DOOREXTR_MIN_DOORWIDTH < 
	   (width = hypot(doorLeaf->yS() - wallLine->yE(),
                          doorLeaf->xS() - wallLine->xE()))) &&
	  (width < DOOREXTR_MAX_DOORWIDTH) &&
	  
	  // Door leaf and opening same size
	  (fabs(2.0 * doorLeaf->h() - width) < DOOREXTR_DIFF_DOORLEAFLEN)) {
	
        CureDO(60) {
          CureCERR(0) << "Got a candidate with door width: " << width << "\n";
          std::cerr << "W: "; wallLine->print(std::cerr);
          std::cerr << "D: "; doorLeaf->print(std::cerr);
        }
	
	// Now we check that all point that are between the lines are
	// behind or very little infront of the lines, plus that at
	// least one point is much behind.
	if (doorOpeningVerified(wallLine->xE(), wallLine->yE(),
				doorLeaf->xS(), doorLeaf->yS(), true)) {

          CureDO(60) {
            CureCERR(0) << "Got a door: \n";
            std::cerr << "D: "; doorLeaf->print(std::cerr);
            std::cerr << "W: "; wallLine->print(std::cerr);
          }
	  addNewDoor(EXTR_IN2LINES,
		     doorLeaf->xS() -
		     (width * cos(aDiff) * cos(wallLine->theta())),
		     doorLeaf->yS() - 
		     (width * cos(aDiff) * sin(wallLine->theta())),
		     doorLeaf->xS(),
		     doorLeaf->yS());
	}
      }
    }
  }
}

void 
DoorExtractor::findOutRightDoor2Lines()
{
  CureCERR(60) << "findOutRightDoor2Lines\n";

  // This function will look for a right hand side door leaf going into the
  // other room, away from the scanner.
  // Model: 
  // * 1 door leaf where the start point is were the hinge is, length correct.
  // * 1 supporting line which angle is close to the angle of the line 
  //     between the door leaf hinge and the start of the line. 
  // * All points between tip of the leaf and start point of line lie behind 
  //     the line between theses two points.

  double width = 0;
  double aDiff = 0;

  // Iterate over possible door leafs
  for (LineIter doorLeaf = lsqLines().begin();
       doorLeaf != lsqLines().end(); doorLeaf++) {
    
    // Iterate over possible wall lines
    for (LineIter wallLine = lsqLines().begin();
	 wallLine != lsqLines().end(); wallLine++) {
      
      // Check that the angle, length and other conditions are met
      if (// Suportive line long enough
	  (2.0 * wallLine->h() > DOOREXTR_MIN_SUPPLEN) &&
	  
	  // Door leaf to the right
	  (angleDiffRad(atan2(wallLine->yS(), wallLine->xS()),
			atan2(doorLeaf->yE(), doorLeaf->xE())) > 0) &&
	  
	  // Supportive line is parallel to door opening
	  (fabs(distPt2Line(doorLeaf->xS(), doorLeaf->yS(),
			    wallLine->xS(), wallLine->yS(), 
			    wallLine->theta())) < DOOREXTR_PT2LINE_THR) &&
	  
	  // Door goes out
	  (angleDiffRad(wallLine->theta(), doorLeaf->theta()) > 
	   DOOREXTR_ANGDIFF_THRPAR) &&

	  // Door is not too open
	  (angleDiffRad(wallLine->theta(), doorLeaf->theta()) < 
	   DOOREXTR_MIN_OPANGLE) &&

	  // Door width is ok
	  (DOOREXTR_MIN_DOORWIDTH < 
	   (width = hypot(wallLine->yS() - doorLeaf->yS(),
                          wallLine->xS() - doorLeaf->xS()))) &&
	  (width < DOOREXTR_MAX_DOORWIDTH) &&

	  // Door leaf has right length
	  (fabs(2.0 * doorLeaf->h() - width) < DOOREXTR_DIFF_DOORLEAFLEN)) {

        CureDO(60) {
          CureCERR(0) << "Got a candidate with door witdh: " << width << "\n";
          std::cerr << "W: "; wallLine->print(std::cerr);
          std::cerr << "D: "; doorLeaf->print(std::cerr);
        }
	
	// Now we check that all point that are between the lines are
	// behind or very little infront of the lines, plus that at
	// least one point is much behind.
	
	if (doorOpeningVerified(doorLeaf->xE(), doorLeaf->yE(),
				wallLine->xS(), wallLine->yS(), true)) {

          CureDO(60) {
            CureCERR(0) << "Got a door: \n";
            std::cerr << "D: "; doorLeaf->print(std::cerr);
            std::cerr << "W: "; wallLine->print(std::cerr);
          }
	  addNewDoor(EXTR_OUT2LINES,
		     doorLeaf->xS(),
		     doorLeaf->yS(),
		     doorLeaf->xS() + 
		     (width * cos(aDiff) * cos(wallLine->theta())), 
		     doorLeaf->yS() + 
		     (width * cos(aDiff) * sin(wallLine->theta())));
	}
      }
    }
  }
}

void 
DoorExtractor::findOutLeftDoor2Lines()
{
  CureCERR(60) << "findOutLeftDoor2Lines\n";

  // This function will look for a left hand side door leaf going into the
  // other room, way from the scanner.
  // Model: 
  // * 1 door leaf where the end point is were the hinge is, length correct.
  // * 1 supporting line which angle is close to the angle of the line 
  //     between the start of the line and the door leaf hinge. 
  // * All points between start point of line and leaf tip lie behind 
  //     the line between these two points.

  double width = 0;
  double aDiff = 0;

  // Iterate over possible door leafs
  for (LineIter doorLeaf = lsqLines().begin();
       doorLeaf != lsqLines().end(); doorLeaf++) {
    
    // Iterate over possible wall lines
    for (LineIter wallLine = lsqLines().begin();
	 wallLine != lsqLines().end(); wallLine++) {
      
      // Check that the angle, length and other conditions are met
      if (// Supportive line long enough
	  (2.0 * wallLine->h() > DOOREXTR_MIN_SUPPLEN) &&
          
	  // Door leaf to the left
	  (angleDiffRad(atan2(doorLeaf->yS(), doorLeaf->xS()),
			atan2(wallLine->yE(), wallLine->xE())) > 0) &&
          
	  // Supportive line is parallel to door opening
	  (fabs(distPt2Line(doorLeaf->xE(), doorLeaf->yE(),
			    wallLine->xE(), wallLine->yE(), 
			    wallLine->theta())) < DOOREXTR_PT2LINE_THR) &&
          
	  // Points out
	  (angleDiffRad(doorLeaf->theta(), wallLine->theta()) > 
	   DOOREXTR_ANGDIFF_THRPAR) &&
          
	  // Door is not too open
	  (angleDiffRad(doorLeaf->theta(), wallLine->theta()) <
	   DOOREXTR_MIN_OPANGLE) &&
          
	  // Door opening right size
	  (DOOREXTR_MIN_DOORWIDTH < 
	   (width = hypot(doorLeaf->yE() - wallLine->yE(),
                          doorLeaf->xE() - wallLine->xE()))) &&
	  (width < DOOREXTR_MAX_DOORWIDTH) &&
          
	  // Door leaf right length
	  (fabs(2.0 * doorLeaf->h() - width) < DOOREXTR_DIFF_DOORLEAFLEN)) {
        
        CureDO(60) {
          CureCERR(0) << "Got a candidate with door witdh: " << width << "\n";
          std::cerr << "W: "; wallLine->print(std::cerr);
          std::cerr << "D: "; doorLeaf->print(std::cerr);
        }
	
        // Now we check that all point that are between the lines are
        // behind or very little infront of the lines, plus that at
        // least one point is much behind.
	
	
        if (doorOpeningVerified(wallLine->xE(), wallLine->yE(),
                                doorLeaf->xE(), doorLeaf->yE(), true)) {

          CureDO(60) {
            CureCERR(0) << "Got a door:";
            std::cerr << "D: "; doorLeaf->print(std::cerr);
            std::cerr << "W: "; wallLine->print(std::cerr);
          }
          addNewDoor(EXTR_OUT2LINES,
                     doorLeaf->xE() -
                     (width * cos(aDiff) * cos(wallLine->theta())),
                     doorLeaf->yE() - 
                     (width * cos(aDiff) * sin(wallLine->theta())),
                     doorLeaf->xE(),
                     doorLeaf->yE());
        }
      }
    }
  }
}

void  
DoorExtractor::findDoor3Lines()
{
  CureCERR(60) << "findDoor3Lines\n";

  // This function will look for a right hand side door leaf going
  // into the room from where the scan was taken.
  // Model:
  // * 2 line representing the wall, separated with about the distance of 
  //     a normal door.
  // * 1 door leaf with its start point close to the end point of the right 
  //     side wall line, length correct.

  double width = 0;

  // Iterate over possible right wall lines
  for (LineIter wallLineR = lsqLines().begin();
       wallLineR != lsqLines().end(); wallLineR++) {

    // Iterate over possible right wall lines
    for (LineIter wallLineL = lsqLines().begin();
	 wallLineL != lsqLines().end(); wallLineL++) {

      // Check angle and door width stuff
      if (// Left and right wall not the same
	  (wallLineR != wallLineL) &&
          
	  // Left line really left
	  (angleDiffRad(atan2(wallLineL->yS(), wallLineL->xS()),
			atan2(wallLineR->yE(), wallLineR->xE())) > 0) &&
          
	  // Door opening is ok in size
	  (DOOREXTR_MIN_DOORWIDTH < 
	   (width = hypot(wallLineR->xE() - wallLineL->xS(),
                          wallLineR->yE() - wallLineL->yS()))) &&
	  (width < DOOREXTR_MAX_DOORWIDTH) &&
          
          linesParallel(wallLineR, wallLineL)) {
        
        // First look for a door leaf on the right side
	// Iterate over possible door leafs
	for (LineIter doorLeaf = lsqLines().begin();
	     doorLeaf != lsqLines().end(); doorLeaf++) {
	  if (// Door leaf is not left or right wall
	      (doorLeaf != wallLineL) && (doorLeaf != wallLineR) &&
	      
	      // Door leaf start where right wall ends
	      (hypot(doorLeaf->xS() - wallLineR->xE(),
                     doorLeaf->yS() - wallLineR->yE()) <
	       DOOREXTR_MAX_HINGE2LINEDIST) &&

	      // Door leaf has right length
	      (fabs(2.0 * doorLeaf->h() - width) <
	       DOOREXTR_DIFF_DOORLEAFLEN)) {

            CureDO(60) {
              CureCERR(0) << "Got a candidate with door witdh: " << width 
                          << "\n";
              std::cerr << "R: "; wallLineR->print(std::cerr);
              std::cerr << "L: "; wallLineL->print(std::cerr);
            }
	    
	    // Now we check that all point that are between the lines are
	    // behind or very little infront of the lines, plus that at
	    // least one point is much behind.
	    
	    if (doorOpeningVerified(doorLeaf->xE(), doorLeaf->yE(),
				    wallLineL->xS(), wallLineL->yS())) {

              CureDO(60) {
                CureCERR(0) << "Got a door: \n";
                std::cerr << "R: "; wallLineR->print(std::cerr);
                std::cerr << "L: "; wallLineL->print(std::cerr);
              }
	      addNewDoor(EXTR_3LINES, 
			 wallLineR->xE(),
			 wallLineR->yE(),
			 wallLineR->xE() + width * cos(wallLineR->theta()),
			 wallLineR->yE() + width * sin(wallLineR->theta()));
	    }
	  }
	}	
	
	// Then we look for a door leaf on the left side
	// Iterate over possible door leafs
	for (LineIter doorLeaf = lsqLines().begin();
	     doorLeaf != lsqLines().end(); doorLeaf++) {
	  if (// Door leaf is not left or right wall
	      (doorLeaf != wallLineR) && (doorLeaf != wallLineL) &&
	      
	      // Door leaf ends close enough to the left wall starts
	      (hypot(doorLeaf->xE() - wallLineL->xS(),
                     doorLeaf->yE() - wallLineL->yS()) <
	       DOOREXTR_MAX_HINGE2LINEDIST) &&
	      
	      // Door leaf is of the right length
	      (fabs(2.0 * doorLeaf->h() - width) <
	       DOOREXTR_DIFF_DOORLEAFLEN)) {
	    
            CureDO(60) {
              CureCERR(0) << "Got a candidate with door witdh: " << width 
                          << "\n";
              std::cerr << "R: "; wallLineR->print(std::cerr);
              std::cerr << "L: "; wallLineL->print(std::cerr);
            }
	    
	    // Now we check that all point that are between the lines are
	    // behind or very little infront of the lines, plus that at
	    // least one point is much behind.
	    if (doorOpeningVerified(wallLineR->xE(), wallLineR->yE(),
				    doorLeaf->xS(), doorLeaf->yS())) {
              CureDO(60) {
                CureCERR(0) << "Got a door: \n";
                std::cerr << "R: "; wallLineR->print(std::cerr);
                std::cerr << "L: "; wallLineL->print(std::cerr);
              }
	      addNewDoor(EXTR_3LINES, 
			 wallLineL->xS() - width * cos(wallLineL->theta()),
			 wallLineL->yS() - width * sin(wallLineL->theta()),
			 wallLineL->xS(),
			 wallLineL->yS());
	    }
	  }
	}
      }
    }
  }
}

*/
