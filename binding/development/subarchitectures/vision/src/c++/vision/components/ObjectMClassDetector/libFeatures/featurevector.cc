/*********************************************************************/
/*                                                                   */
/* FILE         featurevector.cc                                     */
/* AUTHORS      Bastian Leibe                                        */
/* EMAIL        leibe@inf.ethz.ch                                    */
/*                                                                   */
/* CONTENT      Define a general feature vector class derived from a */
/*              vector<float> and provide some basic functions.      */
/*                                                                   */
/* BEGIN        Wed Jul 24 2002                                      */
/* LAST CHANGE  Wed Jul 24 2002                                      */
/*                                                                   */
/*********************************************************************/

#ifdef _USE_PERSONAL_NAMESPACES
//namespace Leibe {
#endif

/****************/
/*   Includes   */
/****************/
#include <iostream>
#include <fstream>
#include <iomanip>

#include <math.h>
#include <stdlib.h>   // for system()
#include <sys/timeb.h>  

#include <libChiSquare/chisquare.hh>
#include <libFeatures/featurevector.hh>

#ifdef __APPLE__
#include <limits.h>   // for FLT_MAX
#include <float.h>
#endif

#ifdef __linux__
#include <values.h>
#endif

/*******************/
/*   Definitions   */
/*******************/
const float MIN_STDDEV   = 0.00001;
const float MIN_VARIANCE = 0.00001;

/*===================================================================*/
/*                         Class FeatureVector                       */
/*===================================================================*/

/***********************************************************/
/*                      Constructors                       */
/***********************************************************/

FeatureVector::FeatureVector()
/* standard constructor */
{
  clear();
  m_nDims = 0;
  m_bSizeDefined = false;
  m_targetClass = -1;
}


FeatureVector::FeatureVector( int nDims )
/* alternate constructor with given number of dimensions */
{
  clear();
  m_nDims = nDims;
  m_bSizeDefined = true;
  m_targetClass = -1;
  initBins();
}


FeatureVector::FeatureVector( const FeatureVector &other )
    : m_vBins(other.m_vBins)
    /* copy constructor */
{
  copyFromOther( other );
}


/*FeatureVector::FeatureVector( vector<float> data, int targetClass )
    : m_vBins(data)
    //copy constructor
{
  m_nDims = data.size();
  m_bSizeDefined = true;
  //m_vBins = data;
  m_nTotalNumBins = data.size();
  m_targetClass = targetClass;
}*/


FeatureVector& FeatureVector::operator=( FeatureVector other )
/* assignment operator */
{
  //cout << "    FeatureVector::operator=() called." << endl;
  copyFromOther( other );
  return *this;
}


FeatureVector& FeatureVector::operator=( vector<float> data)
/* assignment operator */
{
  //cout << "    FeatureVector::operator=() called." << endl;
  m_nDims = data.size();
  m_bSizeDefined = true;
  m_vBins = data;
  m_nTotalNumBins = data.size();
  m_targetClass = -1;
  return *this;
}


FeatureVector& FeatureVector::operator=( vector<double> data)
/* assignment operator */
{
  //cout << "    FeatureVector::operator=() called." << endl;
  m_nDims = data.size();
  m_bSizeDefined = true;
  m_vBins.clear();
  for( int i=0; i<(int)data.size(); i++ )
    m_vBins.push_back( (float) data[i] );
  m_nTotalNumBins = data.size();
  m_targetClass = -1;
  return *this;
}


FeatureVector::~FeatureVector()
/* standard destructor */
{
  clear();
}


void FeatureVector::copyFromOther( const FeatureVector &other )
/*******************************************************************/
/* Auxiliary function to copy from another feature vector.         */
/*******************************************************************/
{
  m_nDims        = other.m_nDims;
  m_bSizeDefined = other.m_bSizeDefined;
  m_vBins        = other.m_vBins;
  m_nTotalNumBins= other.m_nTotalNumBins;
  m_targetClass  = other.m_targetClass;
}


void FeatureVector::clear()
/*******************************************************************/
/* Completely clear this feature vector. All size information will */
/* be reset to 0, and all bin data will be lost.                   */
/*******************************************************************/
{
  m_nDims = 0;
  m_bSizeDefined = false;

  m_nTotalNumBins = -1;
  m_targetClass = -1;
  m_vBins.clear();
}


void FeatureVector::initBins()
/*******************************************************************/
/* Initialize all bins to 0. All size parameters must be defined   */
/* before this function can be called.                             */
/*******************************************************************/
{
  assert( m_bSizeDefined );

  /* calculate the total number of bins in the vector */
  int nTotalBins = calcTotalNumBins();

  m_vBins.clear();
  m_vBins.resize( nTotalBins );
  for( int i=0; i<nTotalBins; i++ )
    m_vBins[i] = 0.0;
}


int FeatureVector::calcTotalNumBins() const
/*******************************************************************/
/* Return the total number of bins in this vector. Since this      */
/* number is used quite often, it is calculated once and will be   */
/* saved as part of the feature vector's internal information.     */
/*******************************************************************/
{
  assert( isValid() );

  //m_nTotalNumBins = m_nDims;

  //return m_nTotalNumBins;
  return m_nDims;
}


/***********************************************************/
/*                    Access Functions                     */
/***********************************************************/

bool FeatureVector::isValid() const
/*******************************************************************/
/* Check if the feature vector definition is valid, that is if all */
/* size parameters have been specified, and all dimensions have    */
/* been defined. This function is mostly used in assert() state-   */
/* ments, but it can also be called from external programs.        */
/*******************************************************************/
{
  return m_bSizeDefined;
}


void  FeatureVector::setNumDims  ( int nDims )
/*******************************************************************/
/* Set or change the number of dimensions in the vector. This      */
/* can be useful, for example, if the vector has been instan-      */
/* tiated with the standard constructor and the size has not yet   */
/* been specified.                                                 */
/*******************************************************************/
{
  clear();
  m_nDims = nDims;
  m_bSizeDefined = true;

  initBins();
}


void  FeatureVector::setData( vector<float> data, int targetClass )
/* set the feature vector data to the given float values */
{
  assert( isValid() );

  /* check if the feature vector size matches the data */
  int nTotalBins = calcTotalNumBins();
  assert( nTotalBins == (int)data.size() );

  /* copy the data values */
  for( int i=0; i<nTotalBins; i++ )
    m_vBins[i] = data[i];
  
  m_targetClass = targetClass;
}


void  FeatureVector::setData( vector<double> data, int targetClass )
/* set the feature vector data to the given double values */
{
  assert( isValid() );

  /* check if the feature vector size matches the data */
  int nTotalBins = calcTotalNumBins();
  assert( nTotalBins == (int)data.size() );

  /* copy the data values */
  for( int i=0; i<nTotalBins; i++ )
    m_vBins[i] = (float) data[i];
    
  m_targetClass = targetClass;
}


void  FeatureVector::print()
/*******************************************************************/
/* Print a text description of this feature vector (useful for de- */
/* bugging). The vector content is not printed.                    */
/*******************************************************************/
{
  cout << "  FeatureVector description:" << endl;
  if ( !m_bSizeDefined )
    cout << "    #Dimensions: not defined!" << endl;

  else
    cout << "    #Dimensions: " << m_nDims << endl;
    
  cout << "Target class: " << m_targetClass << endl;

  cout << "  -----------------------------------" << endl;
}


void  FeatureVector::printContent()
/*******************************************************************/
/* Print the content of this feature vector (useful for debugging) */
/*******************************************************************/
{
  cout << "\t";
  int nTotalBins = calcTotalNumBins();
  for( int i=0; i<nTotalBins; i++ )
    cout << m_vBins[i] << "\t";
  if (m_targetClass > -1) cout << m_targetClass; 
  cout << endl;
}


/***********************************************************/
/*                  FeatureVector File I/O                 */
/***********************************************************/

bool  FeatureVector::save( string filename )
/*******************************************************************/
/* Save this feature vector to disk in text file format. The vec-  */
/* tor will be saved under the given file name. If a file with     */
/* this name already exists, it will be overwritten.               */
/*******************************************************************/
{
  ofstream ofile;

  /* try to open the file */
  ofile.open( filename.c_str() );
  if ( !ofile )
  {
    cerr << "Error in FeatureVector::save(): Couldn't open file '" << filename
    << "'!" << endl;
    return false;
  }

  /* write the key word 'FeatureVector' */
  ofile << "FeatureVector" << endl;

  /* write the header information */
  writeHeader( ofile );

  /* write the data key word */
  ofile << "Data:" << endl;

  /* write the data */
  writeData( ofile );

  /* close the file */
  ofile.close();

  return true;
}


void  FeatureVector::writeHeader( ofstream &ofile )
{
  /* write the number of dimensions */
  ofile << "Dimensions:" << endl;
  ofile << m_nDims << endl;

  /* write the data file format */
  ofile << "Format:" << endl;
  ofile << "Ascii" << endl;
  
  ofile << "TargetClass:" << endl;
  ofile << m_targetClass << endl;
}


void  FeatureVector::writeData( ofstream &ofile ) const
{
  /* write the data key word */
  //ofile << "Data:" << endl;

  /* write the data itself in Ascii format */
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    ofile << m_vBins[i] << " ";
  ofile << endl;
}


bool  FeatureVector::load( string filename, bool verbose )
/*******************************************************************/
/* Load the feature vector from the given file. The file must      */
/* exist and must be in text format. The function will return      */
/* 'true' in case of success, otherwise 'false'.                   */
/*******************************************************************/
{
  ifstream ifile;
  bool     success = false;
  bool     isAscii = true;

  /* clear this feature vector */
  clear();

  /* try to open the file */
  ifile.open( filename.c_str() );
  if ( !ifile )
  {
    cerr << "Error in FeatureVector::load(): Couldn't open file '" << filename
    << "'!" << endl;
    return false;
  }

  /* read the first line => determines the type of the file, must be */
  /* 'FeatureVector'                                                 */
  if ( !readKeyWord( ifile, "FeatureVector featurevector FEATUREVECTOR" ) )
  {
    cerr << "Error in FeatureVector::load(): Not a FeatureVector file!"
    << endl;
    return false;
  }
  if ( verbose )
    cout << "  Loading feature vector..." << endl;

  /* read the header information */
  success = readHeader( ifile, isAscii, verbose );

  if ( success )
    /* read the data */
    success = readData( ifile, isAscii );

  /* close the file */
  ifile.close();

  return success;
}


bool  FeatureVector::readHeader( ifstream &ifile, bool &isAscii,
                                 bool verbose )
{
  /* read the next line => must be key word 'Dimensions:' */
  if ( !readKeyWord( ifile, "Dimensions: dimensions:" ) )
    return false;

  /* read the number of dimensions */
  int nDims;
  ifile >> nDims;
  if ( verbose )
    cout << "    Dimensions: " << nDims << endl;
  setNumDims( nDims );

  /* read the next line => must be key word 'Format:' */
  if ( !readKeyWord( ifile, "Format: format:" ) )
    return false;

  /* read the next line => must be key word 'ASCII' or 'BIN' */
  string format;
  isAscii = false;
  ifile >> format;
  if ( (format == "ascii") || (format == "Ascii") || (format == "ASCII") )
  {
    if ( verbose )
      cout << "    Reading data in ASCII format..." << endl;
    isAscii = true;

  }
  else if ( (format == "binary") || (format == "Binary") ||
            (format == "BINARY") )
  {
    if ( verbose )
      cout << "    Reading data in binary format..." << endl;
    isAscii = false;

  }
  else
  {
    cerr << "Error in FeatureVector::readHeader(): "
    << "Expected key word 'Ascii' or 'Binary'." << endl;
    return false;
  }
  
  //Can skip TargetClass to be compatible with old file format
  if ( !readKeyWord( ifile, "TargetClass: targetclass:" ) ) {
    m_targetClass = -1;
    return true;
  }
  int tc;
  ifile >> tc;
  setTargetClass(tc); 

  return true;
}


bool  FeatureVector::readData( gzFile& file, bool isAscii )
{
  /* read the next line => must be key word 'Data:' */
  /* read key word 'Data:' */
  
  char buff[30];
  gzgets(file,buff,30);
  
  if ( !readKeyWord( buff, "Data: data:" ) )
    return false;

  /* read the data */
  if ( isAscii )
  {
    int nTotalBins = calcTotalNumBins();
    int i=0,buflength, i1=0;
    char buf[513];
    char* pos;
    char* next;
    bool loadnew, addwhitespace;
    /* read the data as ASCII values word by word and translates */
    
  
    
    gzgets(file,buf,512);
    strcat(buf,"e");
    i1++;
    loadnew=strchr(buf,'\n')==NULL;

    pos=strtok(buf," ");
    while (pos!=NULL) {
       next=strtok(NULL," \n");
       if (((next==NULL) || (next=="e")) && (loadnew)) {
//	  if (pos[buflength+1]=='\0') {
	  if (next=="e") {
	     strcat(pos," ");
	     buflength++;
	  } else if (pos[strlen(pos)-1]=='e'){
	     pos[strlen(pos)-1]='\0';
          }
          buflength=strlen(pos);
          strcpy(buf,pos); 
          gzgets(file,buf+buflength,512-buflength);
	  strcat(buf,"e");
	  i1++;
	  loadnew=strchr(buf,'\n')==NULL;
	  pos=strtok(buf," ");	  
          next=strtok(NULL," \n");
       }
       //if (i>=nTotalBins) break;
       sscanf(pos,"%f",&m_vBins[i]);
       i++;
       pos=next;
    }
 //    cout<< i << " "<< i1 << "  " <<nTotalBins<<endl;
     
  
 /* 
  char c;
  string word="";
      while (((c=gzgetc(file))!=-1) && (c!='\n')) {
       if ((c==' ') || (c=='\t')) {
       	if (!word.empty()) {
 	  sscanf(word.c_str(),"%f",&m_vBins[i]);
 	  word="";
 	  if ((i++)>=nTotalBins) break;
 	}
       } else {
         word+=c;
       }
     } 
*/
  }
  else
  {
    /* read the data in binary format */
    cerr << "    Sorry! Not implemented yet!" << endl;
    return false;
  }

  return true;
}


bool  FeatureVector::readData( ifstream &ifile, bool isAscii )
{
  /* read the next line => must be key word 'Data:' */
  string line;

  /* read key word 'Data:' */
  if ( !readKeyWord( ifile, "Data: data:" ) )
    return false;

  /* read the data */
  if ( isAscii )
  {
    /* read the data as ASCII values */
    int nTotalBins = calcTotalNumBins();
    for( int i=0; i<nTotalBins; i++ )
    {
      float value;
      ifile >> value;
      m_vBins[i] = value;
    }

  }
  else
  {
    /* read the data in binary format */
    cerr << "    Sorry! Not implemented yet!" << endl;
    return false;
  }

  return true;
}


/***********************************************************/
/*         FeatureVector Combination/Manipulation          */
/***********************************************************/

void  FeatureVector::addVector( const FeatureVector &other )
/*******************************************************************/
/* Add the content of another feature vector to this one. The      */
/* other vector must have the same dimension.                      */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( isValid() && other.isValid() );
  assert( m_nDims == other.m_nDims );

  /* for all cells of the new histogram */
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    /* add their content to the respective histogram cell */
    /* in this histogram.                                 */
    m_vBins[i] += other.m_vBins[i];
}


void  FeatureVector::subVector( const FeatureVector &other )
/*******************************************************************/
/* Subtract the content of another feature vector to this one. The */
/* other vector must have the same dimensions and number of bins.  */
/*******************************************************************/
{
  /* check if the two vectors are compatible */
  assert( isValid() && other.isValid() );
  assert( m_nDims == other.m_nDims );

  /* for all cells of the new histogram */
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    /* subtract their content from the respective histogram cell */
    /* in this histogram.                                        */
    m_vBins[i] -= other.m_vBins[i];
}


void  FeatureVector::multFactor( float factor )
/*******************************************************************/
/* Multiply the vector by a given scalar factor.                   */
/*******************************************************************/
{
  /* for all cells of the new vector */
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    /* multiply the factor */
    m_vBins[i] *= factor;
}


FeatureVector&  FeatureVector::operator+=( const FeatureVector &other )
/*******************************************************************/
/* Add the content of another feature vector to this one. The      */
/* other vector must have the same dimension.                      */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( isValid() && other.isValid() );
  assert( m_nDims == other.m_nDims );

  /* for all cells of the new vector */
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    /* add their content to the respective histogram cell */
    /* in this histogram.                                 */
    m_vBins[i] += other.m_vBins[i];
  return *this;
}


FeatureVector&  FeatureVector::operator-=( const FeatureVector &other )
/*******************************************************************/
/* Subtract the content of another feature vector from this one.   */
/* The other vector must have the same dimension.                  */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( isValid() && other.isValid() );
  assert( m_nDims == other.m_nDims );

  /* for all cells of the new vector */
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    /* add their content to the respective histogram cell */
    /* in this histogram.                                 */
    m_vBins[i] -= other.m_vBins[i];
  return *this;
}


FeatureVector operator+( const FeatureVector &a, const FeatureVector &b )
/*******************************************************************/
/* Add the content of two feature vectors. Both vectors must have  */
/* the same dimension.                                             */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( a.isValid() && b.isValid() );
  assert( a.m_nDims == b.m_nDims );

  /* for all cells of the new vector */
  FeatureVector result( a );
  int nTotalBins = a.calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    /* add their content to the respective dimension */
    result.m_vBins[i] += b.m_vBins[i];
  return result;
}


FeatureVector operator-( const FeatureVector &a, const FeatureVector &b )
/*******************************************************************/
/* Add the content of two feature vectors. Both vectors must have  */
/* the same dimension.                                             */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( a.isValid() && b.isValid() );
  assert( a.m_nDims == b.m_nDims );

  /* for all cells of the new vector */
  FeatureVector result( a );
  int nTotalBins = a.calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    /* add their content to the respective dimension */
    result.m_vBins[i] -= b.m_vBins[i];
  return result;
}


float  FeatureVector::dot( const FeatureVector &other )
/*******************************************************************/
/* Build the dot product of both feature vectors. The other vector */
/* must have the same dimension.                                   */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( isValid() && other.isValid() );
  assert( m_nDims == other.m_nDims );

  /* for all cells of the new vector */
  float result = 0.0;
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    result += m_vBins[i] * other.m_vBins[i];
  return result;
}


FeatureVector  FeatureVector::cross( const FeatureVector &other )
/*******************************************************************/
/* Build the cross product of both feature vectors. The other      */
/* vector must have the same dimension.                            */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( isValid() && other.isValid() );
  assert( m_nDims == other.m_nDims );

  int nTotalBins = calcTotalNumBins();
  FeatureVector result( nTotalBins );
  if (nTotalBins != 3)
  {
    cerr << "Error in FeatureVector::cross(): "
    << "Operation only defined for 3 dimensions!" << endl;
    return result;
  }

  result.at(0) = m_vBins[1]*other.m_vBins[2] - m_vBins[2]*other.m_vBins[1];
  result.at(1) = m_vBins[2]*other.m_vBins[0] - m_vBins[0]*other.m_vBins[2];
  result.at(2) = m_vBins[0]*other.m_vBins[1] - m_vBins[1]*other.m_vBins[0];

  return result;
}


FeatureVector&  FeatureVector::operator+=( float x )
/*******************************************************************/
/* Add a scalar to the content of this feature vector.             */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( isValid() );

  /* for all cells of the new vector */
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    m_vBins[i] += x;
  return *this;
}


FeatureVector&  FeatureVector::operator-=( float x )
/*******************************************************************/
/* Subtract a scalar from the content of this feature vector.      */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( isValid() );

  /* for all cells of the new vector */
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    m_vBins[i] -= x;
  return *this;
}


FeatureVector&  FeatureVector::operator*=( float x )
/*******************************************************************/
/* Multiply the content of this feature vector with a scalar.      */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( isValid() );

  /* for all cells of the new vector */
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    m_vBins[i] *= x;
  return *this;
}


FeatureVector&  FeatureVector::operator/=( float x )
/*******************************************************************/
/* Divide the content of this feature vector by a scalar.          */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( isValid() );

  /* for all cells of the new vector */
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    /* add their content to the respective histogram cell */
    /* in this histogram.                                 */
    m_vBins[i] /= x;
  return *this;
}


FeatureVector operator+( const FeatureVector& a, float x )
/*******************************************************************/
/* Add a scalar to the content of this feature vector.             */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( a.isValid() );

  /* for all cells of the new vector */
  FeatureVector result( a );
  int nTotalBins = a.calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    /* add their content to the respective histogram cell */
    /* in this histogram.                                 */
    result.m_vBins[i] += x;
  return result;
}


FeatureVector operator-( const FeatureVector& a, float x )
/*******************************************************************/
/* Subtract a scalar from the content of this feature vector.      */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( a.isValid() );

  /* for all cells of the new vector */
  FeatureVector result( a );
  int nTotalBins = a.calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    /* add their content to the respective histogram cell */
    /* in this histogram.                                 */
    result.m_vBins[i] -= x;
  return result;
}


FeatureVector operator*( const FeatureVector& a, float x )
/*******************************************************************/
/* Multiply a scalar to the content of this feature vector.        */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( a.isValid() );

  /* for all cells of the new vector */
  FeatureVector result( a );
  int nTotalBins = a.calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    /* add their content to the respective histogram cell */
    /* in this histogram.                                 */
    result.m_vBins[i] *= x;
  return result;
}


FeatureVector operator/( const FeatureVector& a, float x )
/*******************************************************************/
/* Divide the content of this feature vector by a scalar.          */
/*******************************************************************/
{
  /* check if the two feature vectors are compatible */
  assert( a.isValid() );

  /* for all cells of the new vector */
  FeatureVector result( a );
  int nTotalBins = a.calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    /* add their content to the respective histogram cell */
    /* in this histogram.                                 */
    result.m_vBins[i] /= x;
  return result;
}


void FeatureVector::normalizeVector( float newSum )
/*******************************************************************/
/* Normalize this feature vector to a fixed sum.                   */
/*******************************************************************/
{
  assert( isValid() );

  /* calculate the current sum */
  float oldSum = getSum();
  int nTotalBins = calcTotalNumBins();

  /* and divide each vector element by this value */
  float factor = newSum / oldSum;
  for ( int i=0; i<nTotalBins; i++ )
    m_vBins[i] *= factor;
}


void  FeatureVector::normalizeEntries( vector<float> vBinFactors )
/*******************************************************************/
/* Normalize every vector bin with a (potentially) different       */
/* factor. This may be useful, for example, to normalize by bin    */
/* variances (in order to get Mahalanobis distances).              */
/*******************************************************************/
{
  assert( isValid() );

  int nTotalBins = calcTotalNumBins();
  assert( (int)vBinFactors.size() == nTotalBins );

  /* divide each histogram cell by the respective factor */
  for ( int i=0; i<nTotalBins; i++ )
    m_vBins[i] /= vBinFactors[i];
}


void  FeatureVector::subtractMean()
/*******************************************************************/
/* Subtract the mean of the feature vector.                        */
/*******************************************************************/
{
  assert( isValid() );

  int nTotalBins = calcTotalNumBins();

  /* calculate the mean */
  double dMean = 0.0;
  for ( int i=0; i<nTotalBins; i++ )
    dMean += m_vBins[i];
  dMean /= (double) nTotalBins;

  /* subtract the mean */
  for ( int i=0; i<nTotalBins; i++ )
    m_vBins[i] -= dMean;
}


void  FeatureVector::normalizeEnergy()
/*******************************************************************/
/* Normalize the feature vector by its energy.                     */
/*******************************************************************/
{
  assert( isValid() );

  int nTotalBins = calcTotalNumBins();

  /* calculate the energy */
  double dEnergy = 0.0;
  for ( int i=0; i<nTotalBins; i++ )
    dEnergy += m_vBins[i]*m_vBins[i];
  dEnergy = ( dEnergy / (double) nTotalBins );

  /* divide each histogram cell by the energy */
  for ( int i=0; i<nTotalBins; i++ )
    m_vBins[i] /= dEnergy;
}


void  FeatureVector::normalizeEnergy2()
/*******************************************************************/
/* Normalize the feature vector by its energy.                     */
/*******************************************************************/
{
  assert( isValid() );

  int nTotalBins = calcTotalNumBins();

  /* calculate the energy */
  double dEnergy = 0.0;
  for ( int i=0; i<nTotalBins; i++ )
    dEnergy += m_vBins[i]*m_vBins[i];
  dEnergy = sqrt( dEnergy );

  /* divide each histogram cell by the energy */
  for ( int i=0; i<nTotalBins; i++ )
    m_vBins[i] /= dEnergy;
}


void  FeatureVector::normalizeZeroMeanUnitVar()
/*******************************************************************/
/* Normalize the feature vector to zero mean, unit variance.       */
/*******************************************************************/
{
  assert( isValid() );

  int nTotalBins = calcTotalNumBins();

  /* calculate the mean */
  double dMean = 0.0;
  for ( int i=0; i<nTotalBins; i++ )
    dMean += m_vBins[i];
  dMean /= (double) nTotalBins;

  /* subtract the mean */
  for ( int i=0; i<nTotalBins; i++ )
    m_vBins[i] -= dMean;

  /* calculate the variance */
  double dVar = 0.0;
  for ( int i=0; i<nTotalBins; i++ )
    dVar += m_vBins[i]*m_vBins[i];
  dVar = ( dVar / (double) (nTotalBins-1) );

  /* divide each histogram cell by the variance */
  for ( int i=0; i<nTotalBins; i++ )
    if( dVar >= MIN_VARIANCE )
      m_vBins[i] /= dVar;
}


void  FeatureVector::normalizeZeroMeanUnitStdDev()
/*******************************************************************/
/* Normalize the feature vector to zero mean, unit std. dev.       */
/*******************************************************************/
{
  assert( isValid() );

  int nTotalBins = calcTotalNumBins();

  /* calculate the mean */
  double dMean = 0.0;
  for ( int i=0; i<nTotalBins; i++ )
    dMean += m_vBins[i];
  dMean /= (double) nTotalBins;

  /* subtract the mean */
  for ( int i=0; i<nTotalBins; i++ )
    m_vBins[i] -= dMean;

  /* calculate the variance */
  double dVar = 0.0;
  for ( int i=0; i<nTotalBins; i++ )
    dVar += m_vBins[i]*m_vBins[i];
  dVar = sqrt( dVar / (double) (nTotalBins-1) );

  /* divide each histogram cell by the variance */
  for ( int i=0; i<nTotalBins; i++ )
    if( dVar >= MIN_STDDEV )
      m_vBins[i] /= dVar;
}


void  FeatureVector::normalizeZeroMeanUnitStdDev2()
/*******************************************************************/
/* Normalize the feature vector to zero mean, unit std. dev.       */
/*******************************************************************/
{
  assert( isValid() );

  int nTotalBins = calcTotalNumBins();

  /* calculate the mean */
  double dMean = 0.0;
  for ( int i=0; i<nTotalBins; i++ )
    dMean += m_vBins[i];
  dMean /= (double) nTotalBins;

  /* subtract the mean */
  for ( int i=0; i<nTotalBins; i++ )
    m_vBins[i] -= dMean;

  /* calculate the variance */
  double dVar = 0.0;
  for ( int i=0; i<nTotalBins; i++ )
    dVar += m_vBins[i]*m_vBins[i];
  dVar = sqrt( dVar );

  /* divide each histogram cell by the variance */
  for ( int i=0; i<nTotalBins; i++ )
    if( dVar >= MIN_STDDEV )
      m_vBins[i] /= dVar;
}


/***********************************************************/
/*                  FeatureVector Statistics                   */
/***********************************************************/

float FeatureVector::getSum() const
/*******************************************************************/
/* Compute the sum over all vector entries.                        */
/*******************************************************************/
{
  assert( isValid() );

  float sum = 0.0;
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
    sum += m_vBins[i];
  return sum;
}


void  FeatureVector::getMinMax( float &min, float &max ) const
/*******************************************************************/
/* Get the minimum and maximum entries in the feature vector.      */
/*******************************************************************/
{
  assert( isValid() );

  min =  99999999999.9;
  max = -99999999999.9;
  int nTotalBins = calcTotalNumBins();
  for ( int i=0; i<nTotalBins; i++ )
  {
    if ( m_vBins[i] < min )
      min = m_vBins[i];
    if ( m_vBins[i] > max )
      max = m_vBins[i];
  }
}


/***********************************************************/
/*                  FeatureVector Comparison                   */
/***********************************************************/

float FeatureVector::compSSD( const FeatureVector &other ) const
/*******************************************************************/
/* Calculate the sum-square distance between the entries of this   */
/* feature vector and another one.                                 */
/*******************************************************************/
{
  /* check if the two vectors have the same dimensions */
  assert( isValid() && other.isValid() );
  assert( m_nDims == other.m_nDims );

  int nTotalBins = calcTotalNumBins();
  assert( other.calcTotalNumBins() == nTotalBins );

  /* calculate the minimum and the sums */
  float ssd = 0.0;
  for( int i=0; i<nTotalBins; i++ )
  {
    float diff = m_vBins[i] - other.m_vBins[i];
    ssd       += diff*diff;
  }

  return ssd;
}


float FeatureVector::compCorrelation( const FeatureVector &other ) const
/*******************************************************************/
/* Calculate the correlation between the entries of this           */
/* feature vector and another one.                                 */
/*******************************************************************/
{
  /* check if the two vectors have the same dimensions */
  assert( isValid() && other.isValid() );
  assert( m_nDims == other.m_nDims );

  int nTotalBins = calcTotalNumBins();
  assert( other.calcTotalNumBins() == nTotalBins );

  /* calculate the minimum and the sums */
  float corr = 0.0;
  for( int i=0; i<nTotalBins; i++ )
  {
    corr += m_vBins[i] * other.m_vBins[i];
  }

  return corr;
}


float FeatureVector::compIntersection( const FeatureVector &other,
                                       bool bNormalizeResult ) const
/*******************************************************************/
/* Calculate the intersection between the two feature vectors.     */
/* The intersection int(h1, h2) is defined as                      */
/*   int'(h1,h2) = sum( min(p1 from h1, p2 from h2) )              */
/*   int(h1,h2)  = ( int'(h1,h2)/sum(h1) + int'(h1,h2)/sum(h2) )/2 */
/* Unless otherwise specified, the normalized value will be        */
/* returned.                                                       */
/*******************************************************************/
{
  /* check if the two histograms have the same dimensions */
  assert( isValid() && other.isValid() );
  assert( m_nDims == other.m_nDims );

  int nTotalBins = calcTotalNumBins();
  assert( other.calcTotalNumBins() == nTotalBins );

  /* calculate the minimum and the sums */
  float sum1 = 0.0;
  float sum2 = 0.0;
  float summin = 0.0;
  for ( int i=0; i<nTotalBins; i++ )
  {
    sum1 += m_vBins[i];
    sum2 += other.m_vBins[i];
    if ( m_vBins[i] < other.m_vBins[i] )
      summin += m_vBins[i];
    else
      summin += other.m_vBins[i];
  }

  if ( bNormalizeResult )
    return 0.5*(summin/sum1 + summin/sum2);
  else
    return summin;
}


float FeatureVector::compChi2Measure( const FeatureVector &other,
                                      bool bNormalize ) const
/*******************************************************************/
/* Calculate the chi-square statistic to compare two feature vec-  */
/* tors. The chi-square measure X(h1, h2) is defined as            */
/*   X(h1, h2) = sum( (h1[i]-h2[i])^2/(h1[i]+h2[i]) )              */
/*******************************************************************/
{
  /* check if the two histograms have the same dimensions */
  assert( isValid() && other.isValid() );
  assert( m_nDims == other.m_nDims );

  int nTotalBins = calcTotalNumBins();
  assert( other.calcTotalNumBins() == nTotalBins );

  /* calculate the minimum and the sums */
  double df, chisq;
  if ( bNormalize )
    chstwo_measure( m_vBins, other.m_vBins, 0, getSum(), other.getSum(),
                    &df, &chisq );
  else
    chstwo_measure( m_vBins, other.m_vBins, 0, 1.0, 1.0, &df, &chisq );

  return chisq;
}


float FeatureVector::compChi2Significance( const FeatureVector &other,
    bool bNormalize ) const
/*******************************************************************/
/* Calculate the chi-square statistic to compare two feature vec-  */
/* tors. The chi-square measure X(h1, h2) is defined as            */
/*   X(h1, h2) = sum( (h1[i]-h2[i])^2/(h1[i]+h2[i]) )              */
/* From this measure, the function calculates a significance esti- */
/* mate in order to decide whether the measure gives sufficient    */
/* proof to reject the null hypothesis that both vectors are       */
/* drawn from the same distribution. A return value close to zero  */
/* indicates a significant difference in the vector distributions. */
/*******************************************************************/
{
  /* check if the two vectors have the same dimensions */
  assert( isValid() && other.isValid() );
  assert( m_nDims == other.m_nDims );

  int nTotalBins = calcTotalNumBins();
  assert( other.calcTotalNumBins() == nTotalBins );

  /* calculate the minimum and the sums */
  double df, chisq, prob;
  if ( bNormalize )
    chstwo( m_vBins, other.m_vBins, 0, getSum(), other.getSum(),
            &df, &chisq, &prob );
  else
    chstwo( m_vBins, other.m_vBins, 0, 1.0, 1.0, &df, &chisq, &prob );

  return prob;
}


float FeatureVector::compBhattacharyya( const FeatureVector &other,
                                        bool bNormalizeInputs ) const
/*******************************************************************/
/* Calculate the Bhattacharyya distance between the two vectors.   */
/* The distance B(h1, h2) is defined as                            */
/*   B(h1,h2) = - ln( sum( sqrt(h1(i)*h2(i)) ) )                   */
/* The Bhattacharyya distance is only defined for normalized his-  */
/* tograms. If they are not normalized yet, the last parameter     */
/* should be set to 'true'. The routine will then compensate.      */
/*******************************************************************/
{
  /* check if the two vectors have the same dimensions */
  assert( isValid() && other.isValid() );
  assert( m_nDims == other.m_nDims );

  int nTotalBins = calcTotalNumBins();
  assert( other.calcTotalNumBins() == nTotalBins );

  /* compensate for non-normalized inputs */
  float factor = 1.0;
  if( bNormalizeInputs )
  {
    float sum1 = getSum();
    float sum2 = other.getSum();
    factor = 1.0/(sum1*sum2);
  }

  /* calculate the minimum and the Bhattacharyya distance */
  float sum = 0.0;
  for ( int i=0; i<nTotalBins; i++ )
  {
    float fact = sqrt(m_vBins[i]*other.m_vBins[i]*factor);
    sum += fact;
  }

  if ( sum > 0.0 )
    return -log(sum);
  else
    return FLT_MAX; //numeric_limits<float>::max();
}


/***********************************************************/
/*                    Service Functions                    */
/***********************************************************/

int FeatureVector::idx( int x ) const
/* calculate the cell index (only useful in derived classes). */
{
  assert( isValid() );
  assert( x >= 0 );
  assert( x < m_nDims );

  return x;
}


/***********************************************************/
/*                  Associated Functions                   */
/***********************************************************/

bool saveFeatureVectorList( string filename,
                            const vector<FeatureVector> &vFeatureVectors,
                            bool gzip, bool verbose )
/*******************************************************************/
/* Save the given vector of feature vectors to disk in text format.*/
/* The vectors will be saved under the given file name. If a file  */
/* with this name already exists, it will be overwritten.          */
/* The parameter gzip specifies if the file shall be gzipped       */
/* after saving.                                                   */
/*******************************************************************/
{
  assert( vFeatureVectors.size() > 0 );
  for( int i=0; i<(int)vFeatureVectors.size(); i++ )
  {
    assert( vFeatureVectors[i].isValid() );
  }

  ofstream ofile;

  /* check for the .flz extension and change it into .fls */
  int pos;
  if ( gzip )
    if ( (pos=filename.rfind( ".flz" )) != (int)string::npos )
    {
      filename = filename.substr(0, pos) + ".fls";
    }

  /* check for the .gz file extension and remove it if it's there */
  if ( gzip )
    if ( (pos=filename.rfind( ".gz" )) != (int)string::npos )
      filename = filename.substr(0, pos);

  /* try to open the file */
  ofile.open( filename.c_str() );
  if ( !ofile )
  {
    /* try to compensate for broken network connections */
    cerr << "Couldn't open output file, trying again... ";
    ofile.open( filename.c_str() );
    if ( ofile )
      cerr << "SUCCESS!" << endl;
    else
    {
      cerr << "FAILED!" << endl;
      cerr << "Error in saveFeatureVectorList(): Couldn't open file '" << filename
      << "'!" << endl;
      return false;
    }
  }

  /* write the key word 'FeatureVectorList' */
  ofile << "FeatureVectorList" << endl;

  /* write the number of feature vectors */
  ofile << "FeatureVectors:" << endl;
  ofile << vFeatureVectors.size() << endl;

  /* write the number of dimensions */
  ofile << "Dimensions:" << endl;
  ofile << vFeatureVectors[0].numDims() << endl;

  /* write the data file format */
  ofile << "Format:" << endl;
  ofile << "Ascii" << endl;

  /* write the data for all vectors */
  for( int i=0; i<(int)vFeatureVectors.size(); i++ )
  {
    /* write the data key word */
    ofile << "Data:" << endl;

    vFeatureVectors[i].writeData( ofile );
  }

  /* close the file */
  ofile.close();

  if ( gzip )
  {
    /* gzip the file */
    if ( verbose )
      cout << "  gzipping..." << endl;
    system( string("gzip -f " + filename).c_str() );
    filename = filename + ".gz";
  }

  /* check for the .fls.gz file extension and change it into .flz */
  if ( (pos=filename.rfind( ".fls.gz" )) != (int)string::npos )
  {
    if ( verbose )
      cout << "  shortening extension .fls.gz into .flz..." << endl;
    filename = filename.substr(0, pos);
    system( string("mv " + filename+".fls.gz " + filename+".flz").c_str() );
  }

  return true;
}


bool loadFeatureVectorList( string filename,
                            vector<FeatureVector> &vFeatureVectors,
                            bool verbose )
/*******************************************************************/
/* Load the feature vectors from the given file. The file must ex- */
/* ist and must contain a vector list in text format. The function */
/* will return a vector of feature vectors and the function value  */
/* 'true' in case of success, otherwise 'false'.                   */
/*******************************************************************/
{
  cout << "loadFeatureVectorList() begin\n";
  verbose = true;
    
  /* clear this vector */
  vFeatureVectors.clear();

  /* try to open the file */
  gzFile file = gzopen(filename.c_str(), "rb");
  if ( !file )
  {
      cerr << "Error in loadFeatureVectorList(): Couldn't open file '"
      << filename
      << "'!" << endl;
      return false;
  }
  
  char line[30];
  /* read the first line => determines the type of the file, */
  /* must be 'FeatureVectorList' */
  gzgets(file,line,30);
  if ( !readKeyWord( line, "FeatureVectorList featurevectorlist FEATUREVECTORLIST" ) )
  {
    cerr << "Error in loadFeatureVectorList: Not a FeatureVector list file!" << endl;
    return false;
  }
  
  if ( verbose )
    cout << "  Loading header..." << endl;

  /* read the next line => must be key word 'FeatureVectors:' */
  gzgets(file,line,30);
  if ( !readKeyWord( line, "FeatureVectors: featurevectors:" ) )
    return false;

  
  /* read the number of vectors */
  gzgets(file,line,30);
  int nVectors;
  sscanf (line,"%d",&nVectors);
  if ( verbose )
    cout << "    FeatureVectors: " << nVectors << endl;
  vFeatureVectors.reserve( nVectors );

  /* read the next line => must be key word 'Dimensions:' */
  gzgets(file,line,30);
  if ( !readKeyWord( line, "Dimensions: dimensions:" ) )
    return false;

  /* read the number of dimensions */
  gzgets(file,line,30);
  int nDims;
  sscanf (line,"%d",&nDims);
  if ( verbose )
    cout << "    Dimensions: " << nDims << endl;
  for( int j=0; j<nVectors; j++ )
  {
    FeatureVector hist;
    hist.setNumDims( nDims );
    //vFeatureVectors[j] = hist;
    vFeatureVectors.push_back(hist);
    //vFeatureVectors[j].setNumDims( nDims );
  }

  /* read the next line => must be key word 'Format:' */
  gzgets(file,line,30);
  if ( !readKeyWord( line, "Format: format:" ) )
    return false;

  /* read the next line => must be key word 'ASCII' or 'BIN' */
  char buff[30];
  bool isAscii = false;
  gzgets(file,line,30);
  sscanf (line,"%s",buff);
  string format(buff);
  
  cout << "Format2: " << format;
  if ( (format == "ascii") || (format == "Ascii") || (format == "ASCII") )
  {
    if ( verbose )
      cout << "    Reading data in ASCII format..." << endl;
    isAscii = true;

  }
  else if ( (format == "binary") || (format == "Binary") ||
            (format == "BINARY") )
  {
    if ( verbose )
      cout << "    Reading data in binary format..." << endl;
    isAscii = false;

  }
  else
  {
    cerr << "Error in FeatureVector::load(): "
    << "Expected key word 'Ascii' or 'Binary'."
    << endl;
    return false;
  }

struct timeb startTime;
struct timeb endTime;

ftime(&startTime);
  
  /* read the data */
  if ( verbose )
    cout << "  Loading feature vectors..." << endl;
  bool success = true;
  for( int j=0; j<nVectors; j++ )
    if( success )
      success = vFeatureVectors[j].readData( file, isAscii );
  if ( verbose )
    cout << "    found " << vFeatureVectors.size() << " feature vectors."
    << endl;

 ftime(&endTime);

printf("Computation time (in seconds): %d\n\n",
int(endTime.time-startTime.time));   
    
  gzclose(file);
  return success;
}



vector<string> extractWords( string WordList )
/* extract the words (separated by spaces) from a string.          */
{
  vector<string> items;
  int pos_old = 0;
  int pos_new = 0;

  while ( pos_old != (int)string::npos )
  {
    /* search for the next space character */
    if ( (pos_new=WordList.find( " ", pos_old )) == (int)string::npos )
      break;
    else
    {
      /* found a space character => extract the list item */
      items.push_back( WordList.substr( pos_old, pos_new - pos_old ) );

      /* skip all following spaces */
      pos_old = pos_new;
      while( (pos_old<(int)WordList.length()) && (WordList[pos_old] == ' ') )
        pos_old++;
    }
  }

  /* extract the last list item */
  items.push_back( WordList.substr(pos_old) );

  return items;
}

bool  readKeyWord( char* ifile, string KeyWords )
{
  /* extract the key words to look for */
  vector<string> vWords = extractWords( KeyWords );
  char buff[30];
  sscanf(ifile,"%s",buff);
  string line(buff);
  
  assert( vWords.size() > 0 );

  bool success = false;
  for ( int i=0; i<(int)vWords.size(); i++ )
    if ( !success )
      if ( line == vWords[i] )
        success = true;

  if ( !success )
    cerr << "Error loading file: Expected key word '" << vWords[0]
    << "', found '" << line << "' instead!" << endl;

  return success;
}

bool  readKeyWord( ifstream &ifile, string KeyWords )
{
  /* extract the key words to look for */
  vector<string> vWords = extractWords( KeyWords );
  string line;

  assert( vWords.size() > 0 );

  ifile >> line;
  bool success = false;
  for ( int i=0; i<(int)vWords.size(); i++ )
    if ( !success )
      if ( line == vWords[i] )
        success = true;

  if ( !success )
    cerr << "Error loading file: Expected key word '" << vWords[0]
    << "', found '" << line << "' instead!" << endl;

  return success;
}


void computeFeatureStatistics( vector<FeatureVector> vFeatureVectors,
                               vector<float> &vMeans,
                               vector<float> &vVariances )
/*******************************************************************/
/* Compute the per-bin means and variances from a collection of    */
/* feature vectors.                                                */
/*******************************************************************/
{
  /* check if the vector collection is valid */
  assert( vFeatureVectors.size() >= 1 );
  assert( vFeatureVectors[0].isValid() );

  int nVectors   = vFeatureVectors.size();
  int nDims      = vFeatureVectors[0].numDims();
  int nTotalBins = vFeatureVectors[0].calcTotalNumBins();
  for ( int i=1; i<nVectors; i++ )
  {
    assert( vFeatureVectors[i].isValid() );
    assert( vFeatureVectors[i].numDims() == nDims );
    assert( vFeatureVectors[i].calcTotalNumBins() == nTotalBins );
  }

  /* initialize the means and variances */
  vMeans.clear();
  vVariances.clear();
  vMeans.resize( nTotalBins );
  vVariances.resize( nTotalBins );

  /* calculate the means */
  for ( int i=0; i<nTotalBins; i++ )
  {
    vMeans[i] = 0.0;

    for ( int hist=0; hist < nVectors; hist++ )
      vMeans[i] += vFeatureVectors[hist].m_vBins[i];
    vMeans[i] /= nVectors;
  }

  /* calculate the variances */
  for ( int i=0; i<nTotalBins; i++ )
  {
    vVariances[i] = 0.0;

    for ( int hist=0; hist < nVectors; hist++ )
    {
      float diff = (vFeatureVectors[hist].m_vBins[i] - vMeans[i]);
      vVariances[i] += diff*diff;
    }
    vVariances[i] /= nVectors;
  }
}

/*QString readGZLine(gzFile& file)
{
  char c;
  QString line;
  while ((c=gzgetc(file))!=-1) {
    if (c=='\n') break;
    line += c;
  }
  
  return line;
}
*/
string readGZLine(gzFile& file)
{
  char c[30];
  //QString line;
  string line;
  if (gzgets(file,c,25)!=Z_NULL)
  {
    line += c;
  }
    
  //return line.stripWhiteSpace();
  return line;
}

ostream& operator<<(ostream& output, FeatureVector& fv) {
	assert(fv.m_bSizeDefined);
	output << std::setprecision(15) << fv.numDims() << "\t";
	for(int i = 0; i < fv.numDims(); i++)
		output << fv.at(i) << "\t";
	output << fv.getTargetClass() << endl;
	return output;
}

istream& operator>>(istream& input, FeatureVector& fv) {
	int dim;
	input >> dim;	
	fv.setNumDims(dim);	
	fv.m_bSizeDefined = true;
	fv.m_nTotalNumBins = dim;
	
	for(int i = 0; i < dim; i++) {
		float value;
		input >> value;
		fv.at(i) = value;
	}

	int target;
	input >> target;
	fv.setTargetClass(target);

	return input;
}

#ifdef _USE_PERSONAL_NAMESPACES
}
#endif

