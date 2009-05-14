#ifndef _MATCHER_H_
#define _MATCHER_H_


#include <vcl_algorithm.h>
#include <vcl_map.h>
#include <vcl_set.h>
#include <vcl_utility.h>
#include <vcl_vector.h>

#include <vnl/vnl_matrix.h>
#include "Recognizer.h"
#include "DatabaseEntry.h"

// Thresholds on probability of matches to decide on the 'empty' and
// 'unknown' class probabilities...
#define EMPTY_PROBABILITY_THRESH 0.30 // Should be = 1/numObjects...
#define UNKNOWN_PROBABILITY_THRESH (EMPTY_PROBABILITY_THRESH + 0.05)

// At least N% of all detected features should match stored feature
// vectors from the correct object class...
#define OBJECT_CONFIDENCE_THRESH 0.20 // 0.20


class Matcher {
public:
  Matcher(int imgMaxWidth=640, int imgMaxHeight=480);
  ~Matcher();

  void addObject(const string strObject, 
		 FeatureVector *pfeatures);

  MatchInfo* match(FeatureVector *pvFeatures);
 
  /**
   * Function sets the center of the detected object based on the
   * voting of the object feature locations -- assumes that the object
   * has been found and that the features in the region have already
   * voted into the voting space...
   */
  void setObjectCenter( MatchInfo *matchInfo );

  /**
   * The closest object match is accepted iff the second closest match
   * (if it exists) is not a very close contender for the top spot --
   * assumes that the matching across all valid object classes has
   * already been accomplished...
   */
  void filterClosestMatch( MatchInfo *matchInfo, int nMostProbableObjectMatches );

  /** Returns the name of the object whose representation contains 
   *  a feature being most similar to a given feature, provided 
   *  the closest-to-next distance ratio falls below \c #
   *  m_fMaxF2SRatio. If the latter condition fails, 
   *  the function returns a null string. */
  string findMatchToFeature(const Feature& feature);

  /** Returns the number of objects stored in the database. */
  int getObjectCount() {return m_database.size();};
  /** Returns string associated with object at 'index' in the database. */
  string getObjectString(int index);
    
  /** Saves the database into a file. 
   *  Returns \c true if the operation succeeds and false otherwise. */
  bool save(const string strFile);
    
  /** Loads the database from a file. 
   *  Returns \c true if the operation succeeds and false otherwise. */
  bool load(const string strFile);

  /** Clears the whole database. */
  void clearAll();

  /** Clears all representations pertaining to a given object. */
  void clearObject(const string strObject);

  /** Returns \c true if the object specified by its label has its record in the database. */
  bool isObjectStored(const char* strObject);

protected:

  vector<DatabaseEntry *>  m_database;

  // max ratio of within-class distance vs out-of-class distance
  float m_fMaxF2SRatio;
  int m_minNumValidMatches;
  double m_maxS2FObjMatchRatio;

  // Thresholds on match probability to decide on class names...
  double m_emptyProbThresh, m_unknownProbThresh, m_objectConfThresh;

  vnl_matrix<unsigned int> votingSpace;

  int findObjectInDatabase(const char* strObject);
  
  /**
   * Function is supposed to provide the closest and second closest
   * neighbor to the input feature vector among the stored fecture
   * vectors... BUGGY IF THE FIRST STORED VECTOR CORRESPONDS TO THE
   * NEAREST NEIGHBOR OF THE TEST VECTOR...
   */
  double findNNandNN2( const Feature&  feature,
		       const FeatureVector& vFeatures, 
		       double& nn2dist );
  /**
   * Returns the distance between \a feature and the nearest neighbor
   * in \a vFeatures.
   */
  double findNN( const Feature& feature, const FeatureVector& vFeatures );

  double getSquaredDist( const Feature& feat1, const Feature& feat2 );
    
  /**
   * Function that computes an entropy-based distance measure
   * (Jensen-Shannon) instead of the 'weak' Euclidean distance
   * measure, between the two input feature vectors...
   */
  double getJSDist( const Feature& feat1, const Feature& feat2 );

  void rparseDelim(const string strObject, const string sDelim,
		   string &sObjectname,string &sViewname);
};

#endif

