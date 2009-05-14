#include <iostream>
#include <fstream>
#include "matcher.h"
#include "siftGlobal.h"

Matcher::Matcher(int imgMaxWidth, int imgMaxHeight) {
  m_fMaxF2SRatio = DEF_MAX_F2S_RATIO;
  m_minNumValidMatches = DEF_MIN_NUM_MATCHES;
  m_maxS2FObjMatchRatio = DEF_MAX_OBJ_MATCH_RATIO;

  m_emptyProbThresh = EMPTY_PROBABILITY_THRESH;
  m_unknownProbThresh = UNKNOWN_PROBABILITY_THRESH;
  m_objectConfThresh = OBJECT_CONFIDENCE_THRESH;

  unsigned nBinsX = 
    int(ceil((double) imgMaxWidth/(double)BIN_SIZE_X));
  unsigned nBinsY = 
    int(ceil((double) imgMaxHeight/(double)BIN_SIZE_Y));

  votingSpace.set_size(nBinsY, nBinsX);
}


Matcher::~Matcher() {

  clearAll();

}


//--------------------------------
void Matcher::clearAll() {
  for( size_t i = 0; i < m_database.size(); ++i ) {
    delete(m_database[i]);
  }

  m_database.clear();
}

void Matcher::clearObject(const string strObject) {

  int i;
  int n = (int)m_database.size();
  vector<DatabaseEntry *>::iterator iter;


  iter=m_database.begin(); 
  for( i = 0; i < n; i++, iter++ ) {
    if (strObject == m_database[i]->strObject)
      break;
  }
  if (i==n) 
    cerr << strObject.c_str() << "cannot be found in the database" << endl;
  else {

    cerr << "Deleting " << m_database[i]->strObject.c_str(); 

    delete(m_database[i]);
    m_database.erase(iter);
    
    cerr << ", database size is " << m_database.size() << endl;
  }

}

//------------------------------------
bool Matcher::isObjectStored(const char* strObject) {
  return (findObjectInDatabase(strObject) >= 0);
}


//-------------------------------------
string Matcher::getObjectString(int index) {
  if( index<-1 || index >= (int)m_database.size() ) 
    return string("");
  else 
    return m_database[index]->strObject;
}


//-----------------------------
void Matcher::rparseDelim(const string strObject, const string sDelim,
			  string &sObjectname,string &sViewname) {

  string::size_type delimIdx;
  string::size_type lastIdx=strObject.size();
  string::size_type firstIdx=0;

  delimIdx = strObject.rfind(sDelim);
  sObjectname = strObject.substr(firstIdx,delimIdx);
  sViewname = strObject.substr(delimIdx+1,lastIdx);

  cerr << "obj: " << sObjectname.c_str() << ", view: " << sViewname.c_str() << endl;

}

//-----------------------------------
void Matcher::addObject(const string strObject, 
			FeatureVector *pfeatures) {
  
  const string sDot(".");
  string sObjectname, sViewname;  
  rparseDelim(strObject, sDot, sObjectname, sViewname);

  cout << "modifying representation for " << sObjectname.c_str() << endl;

  int index = findObjectInDatabase(sObjectname.c_str());

  if (index < 0) { // object not yet in the database;
    DatabaseEntry *pentry = new DatabaseEntry(sObjectname);
    pentry->add_features(sViewname, pfeatures);
    m_database.push_back(pentry);
  }
  else { // object found in the database; add feature vector to its representation
    m_database[index]->add_features(sViewname,pfeatures);
  }
}




/**
 * Function takes as input the features extracted from an input image
 * (typically the feature vectors from a ROI), and matches them with
 * the database feature vectors. Returns a struct with information on
 * the closest match of input vectors with database vectors...
 */
MatchInfo* Matcher::match( FeatureVector *pvFeatures ) {
  // .[object] = indexes of matched SIFTs
  vcl_map< string, vcl_set<int> > mapMatchedSifts;  

  votingSpace.fill(0);  
  
  // Initialize the struct that is eventually going to be returned...
  MatchInfo* matchInfo = new MatchInfo();
  matchInfo->objCenter.x = -1;
  matchInfo->objCenter.y = -1;
  matchInfo->strMostProbableObject = "";

  // Number of feature vectors in the current test set, for instance
  // the SIFT features extracted from a ROI in the image...
  unsigned int nFeatures = pvFeatures->size();
  cout << "SIFT: match: num features in input ROI " << nFeatures << endl;

  // Vector of <int> (of the same size of the test set) to store
  // whether the particular feature vector from the input set was
  // matched or unmatched with database entries...
  matchInfo->vSiftClassification.resize( nFeatures );  // .[i] = 1, 0, or -1
  
  // If the database of known features is empty, there is nothing to
  // do here...
  if( m_database.size() == 0 ) {
    matchInfo->vSiftClassification.assign( nFeatures, 0 );
    return matchInfo;
  }
  else {
    matchInfo->vSiftClassification.assign( nFeatures, -1 );
  }
  
  // Vector of <int> to denote the indices of the features in the
  // input set that were not matched with any samples in the trained
  // database...
  vcl_set< int > setRemainingSiftIndexes;
  for( size_t f = 0;  f < nFeatures;  ++f ) {
    setRemainingSiftIndexes.insert(f);
  }

  // For each feature in the test set...
  for( size_t f = 0;  f < nFeatures;  ++f ) {
    // Find the object 'name' (a string) corresponding to the closest
    // match in the database...
    string strMatch = findMatchToFeature( (*pvFeatures)[f] );
    // Store a multimap of string names and feature indices...
    mapMatchedSifts[strMatch].insert(f);
  }

  // Initialize variables, to be used later...
  int nMostProbableObjectMatches = 0;
  string strMostProbableObject = ""; // m_database[0]->strObject;
  vcl_map< string, FeatureVector >::const_iterator itd, itd_e;
  
  // Number of entries in the database -- NOT a count of the number of
  // vectors, but a count of the no.of independent object classes
  // being considered...
  size_t nObjects = m_database.size();
  cout << "SIFT: match: num objects in DB " << nObjects << endl;

  int totalMatches = 0;

  // Ah well, some more variable initialization...
  map<string, Representation *>::iterator iter; 
  string strName;

  // For each object in the database (say cuboid, mug, jar etc)...
  for( size_t iObj = 0;  iObj < nObjects;  ++iObj ) {
    string strObj = m_database[iObj]->strObject;
    
    // Iterate over the different views of the object -- each view is
    // considered to be an independent entity (may not be the right
    // thing to do!!)...
    for( iter =  m_database[iObj]->map_viewreps.begin();
	 iter !=  m_database[iObj]->map_viewreps.end(); ++iter ) {
      // Get the string name for the object, taking into account the
      // fact that there might be multiple views trained for the same
      // object...
      strName = strObj + "." + (*iter).first;
	
      // Get the number of the matches in the input feature set
      // corresponding to a particular object (for a particular
      // view)...
      int nMatches =(int)( mapMatchedSifts[strName].size() );

      StringIntPair sipair;
      sipair.str = strName;
      sipair.n = nMatches;
    
      // Store a map of the particular (object, number of matches)
      // pair, which takes into account the entire input test set...
      matchInfo->mapMatchCount.push_back(sipair);
      
      // Also store a running count of the total number of matches
      // over the entire database, corresponding to the input test
      // set...
      totalMatches += nMatches;
      
      cout << "SIFT: match: Valid match = (" << strName
	     << ", " << nMatches << ")\n";
      
      // Store a running estimate of the most probable object match to
      // the input feature set (the database object type that matches
      // the largest number of input vectors)...
      if( nMatches > nMostProbableObjectMatches ) {
	nMostProbableObjectMatches = nMatches;
	strMostProbableObject = strName;
	cout << "SIFT: match: Current most probable match...\n";
      }
      
      // Next update setRemainingSiftIndexes -- the vector of <int>
      // storing indices of the features in the input set that were
      // not matched with any samples in the trained database...
      vcl_vector< int > temp(setRemainingSiftIndexes.size());
      vcl_vector< int >::iterator ite =
	vcl_set_difference(setRemainingSiftIndexes.begin(), setRemainingSiftIndexes.end(),
			   mapMatchedSifts[strName].begin(), mapMatchedSifts[strName].end(), 
			   temp.begin());
      temp.erase(ite, temp.end());
      setRemainingSiftIndexes = vcl_set<int>(temp.begin(), temp.end());
    } // for (iView)
  }  // for (iObj)
  
  // If the number of matches (between test set and stored database)
  // corresponding to the most likely object type is less than a
  // threshold (for now 3-5), the match is not to be accepted...
  if( nMostProbableObjectMatches < m_minNumValidMatches ) {
    cout << "SIFT: match: Not enough matches in the test set... " 
	 << nMostProbableObjectMatches << "\n";
    return matchInfo;
  }

  // Set the most probably object type...
  matchInfo->strMostProbableObject = strMostProbableObject;
  // Set the probablity of this most likely match...
  matchInfo->matchProbability = (float)nMostProbableObjectMatches/(float)totalMatches;

  // Set values (1, 0, -1) to determine which of the test features
  // were true matches, unmatched etc...
  int count_ones=0;
  int count_sifts= (matchInfo->vSiftClassification).size();

  vcl_set<int>::iterator itm, itm_e;
  // True matches...
  for( itm = mapMatchedSifts[strMostProbableObject].begin(), 
	 itm_e = mapMatchedSifts[strMostProbableObject].end();  itm != itm_e;  ++itm ) {
    matchInfo->vSiftClassification[*itm] = 1;
    count_ones++;
  }
  
  // Unmatched features...
  for( itm = setRemainingSiftIndexes.begin(), itm_e = setRemainingSiftIndexes.end();
       itm != itm_e;  ++itm ) {
    matchInfo->vSiftClassification[*itm] = 0;
  }
  // The rest are all false matches...

  // Finally, set the confidence on the object found...
  matchInfo->objectConfidence = (float)count_ones/(float)count_sifts;
  
  // Now that it is confirmed that a valid match exists, need to
  // determine if the test data closely matches multiple classes...
  filterClosestMatch( matchInfo, nMostProbableObjectMatches );

  cout << "SIFT: match: probability (match, object) = (" << matchInfo->matchProbability 
       << ", " << matchInfo->objectConfidence << ")\n";
//   cout << "SIFT:match: special case prob computation (" << nMostProbableObjectMatches << ", " 
//        << numValidTotalMatches << ", " << nMostProbableObjectMatches/(double)numValidTotalMatches
//        << ")\n";
  
  // Set the center of the object (in image coordinates)...
  setObjectCenter( matchInfo );
  
  return matchInfo;
}



/**
 * The closest object match is accepted iff the second closest match
 * (if it exists) is not a very close contender for the top spot --
 * assumes that the matching across all valid object classes has
 * already been accomplished...
 */
void Matcher::filterClosestMatch( MatchInfo *matchInfo, int nMostProbableObjectMatches ) {
  // First perform checks based  on match (and object) probability...
  if( matchInfo->matchProbability <= m_emptyProbThresh ) {
    cout << "SIFT: match: label empty: " << matchInfo->strMostProbableObject.c_str() 
	 << ", probs: (" << matchInfo->matchProbability << ", "
	 << matchInfo->objectConfidence << ")\n";
    matchInfo->strMostProbableObject = "";
    return;
  }
  else if( matchInfo->matchProbability <= m_unknownProbThresh || 
	   matchInfo->objectConfidence <= m_objectConfThresh ) {
    cout << "SIFT: match: label unknown: " << matchInfo->strMostProbableObject.c_str() 
	 << ", probs: (" << matchInfo->matchProbability << ", "
	 << matchInfo->objectConfidence << ")\n";
    matchInfo->strMostProbableObject = string("unknown");
    return;
  }
  
  // If multiple valid matches exist, also perform a check on possible
  // multiple matches...
  if( matchInfo->mapMatchCount.size() > 1 ) {
    // Need to determine the second-best match (if one exists)...
    int matchCount2 = 0;
    for( size_t m = 0; m < matchInfo->mapMatchCount.size(); ++m ) {
      // Do not consider already found match and/or the match with
      // less than required number of matches with database...
      StringIntPair si = matchInfo->mapMatchCount.at(m);
      if( si.str == matchInfo->strMostProbableObject ||
	  si.n < m_minNumValidMatches ) {
	continue;
      }
      if( si.n > matchCount2 ) {
	matchCount2 = si.n;
      }
    }
    // If the second-best match is very close to the best match,
    // change assigned label...
    if( matchCount2/(double)nMostProbableObjectMatches > m_maxS2FObjMatchRatio ) {
      matchInfo->strMostProbableObject = string("unknown");
      cout << "SIFT:match: Test set closely matches more than one object... ("
	   << matchCount2 << "," << nMostProbableObjectMatches << ")\n";
    }
  }
}




/**
 * Function sets the center of the detected object based on the voting
 * of the object feature locations -- assumes that the object has been
 * found and that the features in the region have already voted into
 * the voting space...
 */
void Matcher::setObjectCenter( MatchInfo *matchInfo ) {
  // Use a simple voting scheme to determine the center of the object
  // in the image plane...
  int bestBinC = -1;
  int bestBinR = -1;
  int bestCount = 0;
  for( int r = 0; r < (int)votingSpace.rows(); ++r ) {
    for( int c = 0; c < (int)votingSpace.cols(); ++c ) {
      if( (int)votingSpace(r, c) > bestCount ) {
	bestCount = (int)votingSpace(r, c);
	bestBinR = r;
	bestBinC = c;
      } // if
    } // for
  } // for
  
  // Set the object centers based on the voting scheme mentioned
  // above...
  matchInfo->objCenter.x = BIN_SIZE_X * bestBinC + BIN_SIZE_X / 2;
  matchInfo->objCenter.y = BIN_SIZE_Y * bestBinR + BIN_SIZE_Y / 2;
}



//---------------------------------
//file.write((char *)&nVectors, sizeof(nVectors)); 
bool Matcher::save(const string strFile) {
  ofstream file(strFile.c_str(), ios::out);
  
  int nObjects = m_database.size();

  //file << nObjects << " ";
  file.write((char*)&nObjects,sizeof(nObjects));

  // for each object
  for (int iObj = 0;  iObj < nObjects;  ++iObj) 
    m_database[iObj]->save(file);

  file.close();
  return true;
}

//-----------------------------------
bool Matcher::load(const string strFile) {
  
  ifstream file(strFile.c_str());

  int nObjects;
  // file >> nObjects;  
  file.read((char*)&nObjects, sizeof(nObjects));
  
  cout << "totalObjs " << nObjects << ", ";
  
  clearAll(); // clear m_database
  for (int obj = 0;  obj < nObjects;  obj++) {


    DatabaseEntry *pentry = new DatabaseEntry();
    pentry->load(file);

    m_database.push_back(pentry);
    
    //cout << "Done loading obj " << obj << endl;

  }  // for (obj)
  
  //cout << "closing file" << endl;

  file.close();
  return true;
}


//////////////////////////////////
int Matcher::findObjectInDatabase(const char* strObject) {
  int i;
  int n = m_database.size();

  // compare the given object string to all stored objects
  for (i = 0;  
       i < n && strcmp(m_database[i]->strObject.c_str(), strObject) != 0;  ++i);
  return (i == n) ? (-1) : (i);
}



/**
 * Function takes as input a single feature vector, corresponding to a
 * single feature vector in the test image ROI, passed in by the
 * 'match' function above, and produces a string corresponding to the
 * object (+view if appropriate!) label of the feature vector in the
 * stored database that best matches the input vector -- returns empty
 * ("") string is no such match is possible...
 */
string Matcher::findMatchToFeature( const Feature& feature ) {
  // Number of independent objects (mug, jar, picture etc) in the
  // database...
  unsigned int nObjects = m_database.size();
  // vcl_multimap< int, string > mapNNs;
  vcl_multimap< double, string > mapNNs;

  map<string, Representation *>::iterator iter; 
  FeatureVector *pFeatureVector=NULL;
  Representation *prep = NULL;

  // Initialize some variables for later use...
  double nn2dist, nndist, nndistPerView, nndistPerObject;
  int nFeatureVectors;

  int iVec;
  double dst;     // int dst;

  string strName;
  string strObject;
  
  int res;

  // If only ONE OBJECT CLASS has been trained for...
  if( nObjects == 1 ) {
    strObject = (m_database[0]->strObject);
    
    // If only one view of the single object has been trained for...
    if( m_database[0]->has_singleView(prep) ) {
      iter =  m_database[0]->map_viewreps.begin();
      strName = strObject + "." + (*iter).first;

      // If only one feature vector exists
      if (prep->has_singleFeatureVector(pFeatureVector)) { 
	// 1 View, 1 featureVec	
	// (ie. the database is based upon a single vector);
	// both NN and NN' are searched for in this feature vector
	nn2dist = INT_MAX;
	nndist = findNNandNN2(feature, (*pFeatureVector), nn2dist);
      
	mapNNs.insert(vcl_make_pair(nndist, strName));
	mapNNs.insert(vcl_make_pair(nn2dist, strName));
      }
      // Else is 1Object, 1View but multiple vectors in database...
      else {
	nFeatureVectors = prep->get_numFeatureVectors();
	for (iVec = 0;  iVec < nFeatureVectors;  ++iVec) {
	  
	  res = prep->get_pfeatureVector(iVec, pFeatureVector);
	  printError(res);

	  dst = findNN(feature, *pFeatureVector);

	  // Insert in the multimap the distance to each stored
	  // (previously learned) vector to the object class under
	  // consideration...
	  mapNNs.insert(vcl_make_pair(dst, strName));
	}	
      }
    }  // has_singleView
    
    // Multiple views of the same OBJECT...
    else { // > 1 views
      for (iter =  m_database[0]->map_viewreps.begin();
	   iter !=  m_database[0]->map_viewreps.end(); iter++) {
	strName = strObject + "." + (*iter).first;
	prep = (*iter).second;

	// > 1 feature vectors (single object, vectors {I_1, ..., I_f});
	// NN = global nearest neighbor;
	// NN' = nearest neighbor within the feature vectors {I_1, ..., I_k-1, I_k+1, ..., I_f}
	// if I_k contains the global NN

	nndistPerView = INT_MAX;
	nFeatureVectors = prep->get_numFeatureVectors();
	for (iVec = 0;  iVec < nFeatureVectors;  ++iVec) {

	  res = prep->get_pfeatureVector(iVec, pFeatureVector);
	  printError(res);

	  dst = findNN(feature, *pFeatureVector);
	  if (dst < nndistPerView)
	    nndistPerView = dst;
	}
	// Insert into the multimap the smallest distance per view,
	// along with the corresponding view name (picture.0, mug.1
	// etc)...
	mapNNs.insert(vcl_make_pair(nndistPerView, strName));
      }
    }
  }
  // Finally, the typical case, where there are multiple features for
  // each object class (mug, jar, picture etc), and possibly multiple
  // views for each object class as well...
  else {
    // For each object class...
    for( size_t iObj = 0;  iObj < nObjects; ++iObj ) {
      strObject = (m_database[iObj]->strObject);
      nndistPerObject = INT_MAX;

      // Consider each view of this object (if one exists in the
      // database) as a separate 'object' instance, i.e. obj1.0 and
      // obj1.1 are different sets...
      for( iter = m_database[iObj]->map_viewreps.begin();
	   iter != m_database[iObj]->map_viewreps.end(); ++iter ) {
	nndistPerView = INT_MAX;
	prep = (*iter).second;
	nFeatureVectors = prep->get_numFeatureVectors();

	// Compare against all feature vectors in the stored/trained
	// database that correspond to the particular object class
	// (and view) that is currently under consideration...
	for( iVec = 0; iVec < nFeatureVectors; ++iVec ) {
	  res = prep->get_pfeatureVector(iVec, pFeatureVector);
	  printError(res);
	  
	  // A simple NNr operation...
	  dst = findNN(feature, *pFeatureVector);
	  // Keep a running estimate of the smallest sample distance
	  // (hence best match distance) (for a particular view)...
	  if( dst < nndistPerView ) {
	    nndistPerView = dst;
	  }
	}  // for (iVec)

	// As soon as one particular view has been evaluated, update
	// running estimate of the closest training sample (to input
	// sample) of the particular object class (view labels are
	// considered separately)...
	if (nndistPerView < nndistPerObject) {
	  strName = strObject + "." + (*iter).first;
	  nndistPerObject = nndistPerView;
	}
      }
      // Store in a map the smallest distance for each object class,
      // to be used for a comparison later on (see below)...
      mapNNs.insert(vcl_make_pair(nndistPerObject, strName));
    } // for (iObj)
  }  // if
  

  // vcl_multimap< int, string>::const_iterator it = mapNNs.begin();
  vcl_multimap< double, string>::const_iterator it = mapNNs.begin();
  
  // Aha, remember to find the smallest and second smallest values in
  // the multimap before trying to compare them -- SURPRISINGLY THIS
  // WAS NOT DONE UNTIL NOVEMBER 2007!!
  nndist = it->first;
  string nearestCategory = it->second;
  nn2dist = (++it)->first;
  
  // Accept a particular match IFF the closest training sample is
  // significantly closer than the second closest training sample --
  // another experimentally determined threshold test designed by
  // David Lowe... :)
  // if( (double)nndist <= m_fMaxF2SRatio * m_fMaxF2SRatio * (double)nn2dist ) {
  if( nndist <= m_fMaxF2SRatio * nn2dist ) {
    // Note that the square of the threshold value is used because we
    // compute squared Euclidean distances...
//     cout << "SIFT:match: match ratio within limits " << nndist
// 	 << ", " << nn2dist << endl;
    // Match found -- celebrate and hope for the best... ;)
    return nearestCategory;
  }
  else {
//     cout << "SIFT:match: match ratio NOT within limits " << nndist
// 	 << ", " << nn2dist << endl;
  }

  // No match -- return 'empty' handed...
  return string("");
}




//-----------------------------------------------------
double Matcher::findNN( const Feature& feature,
			const FeatureVector& vFeatures ) {

  double nearest = INT_MAX;
  unsigned nFeatures = vFeatures.size();
  int nearestIx;

  for (unsigned f = 0;  f < nFeatures;  ++f) {
    // double dsq = getSquaredDist(feature, vFeatures[f]);
    double dsq = getJSDist(feature, vFeatures[f]);
    if (dsq < nearest) {
      nearest = dsq;
      
      // NEW
      // Store the index of the current nearest neighbor.
      nearestIx = f;
    }
  }
  

  // ARGGH! BUGGY -- SHOULD *ONLY* USE THOSE FEATURE VECTORS TO
  // COMPUTE THE CENTROID, WHICH HAVE BEEN DETERMINED TO CORRESPOND TO
  // THE FINAL OBJECT CLASS!! JUST BECAUSE A FEATURE VECTOR HAS A
  // NEAREST NEIGHBOR IN THE STORED SET DOES NOT MEAN IT GETS TO VOTE
  // AUTOMATICALLY!!

  //----------------------------------------------------
  // Update the voting space.
  //----------------------------------------------------  
  Feature nearestFeature = vFeatures[nearestIx];  
  double ratio = feature.fScale / nearestFeature.fScale;
  Location objCenter;
  objCenter.x = int(feature.loc.x - ratio * nearestFeature.dxCenter);
  objCenter.y = int(feature.loc.y - ratio * nearestFeature.dyCenter);
  
  int r = objCenter.y / BIN_SIZE_Y;
  int c = objCenter.x / BIN_SIZE_X;
  if ((r >= 0) && (r < (int)votingSpace.rows()) && (c >= 0) && (c < (int)votingSpace.cols())) {
    votingSpace(r, c)++;
  } // if
  

  // Return the distance to closest neighbor... :)
  return nearest;
}



double Matcher::findNNandNN2(const Feature& feature,
			  const FeatureVector& vFeatures, 
			  double& nn2dist) {
  double nndist = INT_MAX;
  nn2dist = INT_MAX;
  unsigned nFeatures = vFeatures.size();
  
  for (unsigned f = 0;  f < nFeatures;  ++f) {
    // double dist = getSquaredDist(feature, vFeatures[f]);
    double dist = getJSDist(feature, vFeatures[f]);
    if (dist < nndist) {
      nn2dist = nndist;
      nndist = dist;
    }
  }
  return nndist;
}



double Matcher::getSquaredDist( const Feature& feat1, 
				const Feature& feat2 ) {
    const unsigned char * buf1 = feat1.descriptor;
    const unsigned char * buf2 = feat2.descriptor;

    int distsq = 0;
    for (int i = 0;  i < SIFT_LENGTH;  ++i) {
        int diff = (int) *buf1++ - (int) *buf2++;
        distsq += diff * diff;
    }
    return (double)distsq;
}



/**
 * Function that computes an entropy-based distance measure
 * (Jensen-Shannon) instead of the 'weak' Euclidean distance measure,
 * between the two input feature vectors...
 */
double Matcher::getJSDist( const Feature& feat1, const Feature& feat2 ) {
  const unsigned char * buff1 = feat1.descriptor;
  const unsigned char * buff2 = feat2.descriptor;

  double jsVal = 0.0;
  double klAM = 0.0;
  double klBM = 0.0;

  // A really small number added to all quantities to eliminate
  // occurrences of divide-by-zero errors...
  double minAdd = 0.0000001;

  double m[128]; // Well, actually it is SIFT_LENGTH, but will do for
		 // now...

  for( int i = 0; i < SIFT_LENGTH; ++i ) {
    double a = (double)( *buff1++ ) + minAdd;
    double b = (double)( *buff2++ ) + minAdd;
    
    // The intermediate value to be used in the JS measure...
    m[i] = ( ( a + b ) / 2.0 );

    klAM += a * std::log( a / m[i] );
    klBM += b * std::log( b / m[i] );
    
    jsVal += ( klAM + klBM )/2.0;
  }

  return( jsVal );
}



