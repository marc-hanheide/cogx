#ifndef _RECOGNIZER_H
#define _RECOGNIZER_H

#include <vector>
#include <string>
#include <map>

using namespace std;

// ~~~ CONSTANTS ~~~
static const int SIFT_LENGTH=128;
static const float DEF_MAX_F2S_RATIO = 0.89; // 0.6...
static const int BIN_SIZE_X = 10;
static const int BIN_SIZE_Y = 10;

static const int DEF_MIN_NUM_MATCHES = 3;
static const double DEF_MAX_OBJ_MATCH_RATIO = 0.91;

// ~~~ TYPEDEFS ~~~

// (column, row) location of a keypoint 
struct Location {
  int  x;
  int  y;
};

typedef unsigned char SiftDescriptor[SIFT_LENGTH];

// DOG point information + SIFT descriptor 
struct Feature {
  Location        loc;
  float           fScale;
  float           fOrientation;
  SiftDescriptor  descriptor;
  
  // 		dxCenter = loc.x - objCenter.x
  // 		dyCenter = loc.y - objCenter.y
  //
  // and d(loc, objCenter) = sqrt(dxCenter^2 + dyCenter^2).
    
  int 		dxCenter;
  int		dyCenter;
};

typedef vector<Feature>  FeatureVector;

typedef vector< float >  FloatSequence;
  
struct StringIntPair { //  map<string, int>
  string str;
  int n;
};

struct MatchInfo {
  /** sequence of < object, n > pairs, where n denotes 
   *  the number of SIFTs matching the object */
  vector<StringIntPair> mapMatchCount;
  
  string strMostProbableObject;
  float matchProbability;
  float objectConfidence;
  
  /** .[f] = 1  <==> f-th SIFT point is a true match (matches the most probable object)
   *  .[f] = -1 <==> f-th SIFT point is a false match
   *  .[f] = 0  <==> f-th SIFT point is unmatched */
  vector< int >  vSiftClassification;
        
  Location objCenter;
};


// static Location topLeft;
// static Location bottomRight;


#endif

