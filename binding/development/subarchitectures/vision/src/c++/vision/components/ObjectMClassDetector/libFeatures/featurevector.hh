/*********************************************************************/
/*                                                                   */
/* FILE         featurevector.hh                                     */
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

#ifndef LEIBE_FEATUREVECTOR_HH
#define LEIBE_FEATUREVECTOR_HH

#ifdef _USE_PERSONAL_NAMESPACES
//namespace Leibe {
#endif

//#define NDEBUG

/****************/
/*   Includes   */
/****************/
#include <vector>
#include <string>
#include <cassert>
#include <fstream>
#include <zlib.h>

//#include <qtextstream.h>

//#include <libGrayImage/grayimage.hh>

using namespace std;

/*************************/
/*   Class Definitions   */
/*************************/

/*===================================================================*/
/*                         Class FeatureVector                       */
/*===================================================================*/
/* Define a general feature vector class */
class FeatureVector
{
    friend ostream& operator<<(ostream& output, FeatureVector& fv);
    friend istream& operator>>(istream& input, FeatureVector& fv);
public:
    FeatureVector();
    FeatureVector( int nDims );
    template<typename T>FeatureVector( vector<T> data, int targetClass = -1);
    //FeatureVector( vector<double> data, int targetClass = -1);
    template<typename T> FeatureVector( T* data, int dataLen, int targetClass = -1 );
    FeatureVector( const FeatureVector &other );
    virtual ~FeatureVector();

    FeatureVector& operator=( FeatureVector other );
    FeatureVector& operator=( vector<float> other);
    FeatureVector& operator=( vector<double> other);

protected:
    virtual void  copyFromOther( const FeatureVector &other );

    virtual void  initBins();

    virtual int   calcTotalNumBins() const;

public:
  /*******************************/
  /*   Content Access Functions  */
  /*******************************/
  virtual bool  isValid() const;

  int   numDims() const     { assert( m_bSizeDefined ); return m_nDims; }

  virtual void  setNumDims  ( int nDims );

  inline float  at ( int x ) const          { return m_vBins[idx(x)]; }
  virtual float& at ( int x )                { return m_vBins[idx(x)]; }

  inline void  setValue( int x, float val )  { m_vBins[idx(x)] = val; }
  inline void  setTargetClass (int targetClass) {m_targetClass = targetClass;}
  
  virtual void  setData( vector<float> data, int targetClass = -1 );
  virtual void  setData( vector<double> data, int targetClass = -1);
  virtual vector<float> getData() const { return m_vBins; }   
  virtual float getData(int index) const {return m_vBins[idx(index)];}
  virtual int getTargetClass() const 	{return m_targetClass;}
  
  /*Low level access*/
  virtual vector<float>& getData() {return m_vBins; }
  //Call corretDim if you directly manipulated the vector
  inline void correctDim() {m_nDims = m_vBins.size(); m_nTotalNumBins = m_nDims; m_bSizeDefined = true;}
  
  virtual void  clear();

  virtual void  print();
  virtual void  printContent();

public:
  /******************************/
  /*   FeatureVector File I/O   */
  /******************************/
  virtual bool  save( string filename );
  virtual bool  load( string filename, bool verbose=false );

  virtual void  writeHeader( ofstream &ofile );
  virtual void  writeData( ofstream &ofile ) const;
  virtual bool  readHeader( ifstream &ifile, bool &isAscii, 
														bool verbose=false );
  virtual bool  readData( ifstream &ifile, bool isAscii=true );
  virtual bool  readData( gzFile& file, bool isAscii );

public:
  /**********************/
  /*   Vector Algebra   */
  /**********************/
  /*--------------------------*/
  /* Vector-Vector operations */
  /*--------------------------*/
  virtual void  addVector   ( const FeatureVector &other );
  virtual void  subVector   ( const FeatureVector &other );

  FeatureVector& operator+=( const FeatureVector &other );
  FeatureVector& operator-=( const FeatureVector &other );

  friend FeatureVector operator+( const FeatureVector &a, 
                                  const FeatureVector &b );
  friend FeatureVector operator-( const FeatureVector &a, 
                                  const FeatureVector &b );

  float         dot  ( const FeatureVector &other );
  FeatureVector cross( const FeatureVector& other );

  /*--------------------------*/
  /* Vector-Scalar operations */
  /*--------------------------*/
  virtual void  multFactor  ( float factor );
  FeatureVector& operator+=( float x );
  FeatureVector& operator-=( float x );
  FeatureVector& operator*=( float x );
  FeatureVector& operator/=( float x );

  friend FeatureVector operator+( const FeatureVector& a, float x );
  friend FeatureVector operator-( const FeatureVector& a, float x );
  friend FeatureVector operator*( const FeatureVector& a, float x );
  friend FeatureVector operator/( const FeatureVector& a, float x );

public:
  /**********************************/
  /*   FeatureVector Manipulation   */
  /**********************************/
  virtual void  normalizeVector ( float newSum );
  virtual void  normalizeEntries( vector<float> vBinFactors );
  void subtractMean();
  void normalizeEnergy();
  void normalizeEnergy2();
  void normalizeZeroMeanUnitVar();
  void normalizeZeroMeanUnitStdDev();
  void normalizeZeroMeanUnitStdDev2();

public:
  /********************************/
  /*   FeatureVector Statistics   */
  /********************************/
  float getSum() const;
  void  getMinMax( float &min, float &max ) const;
  
  friend void computeFeatureStatistics( vector<FeatureVector> vFeatureVectors, 
																				vector<float> &vMeans, 
																				vector<float> &vVariances );

public:
  /*************************************/
  /*   Histogram Comparison Measures   */
  /*************************************/
  float compSSD             ( const FeatureVector &other ) const;
  float compCorrelation     ( const FeatureVector &other ) const;
  float compIntersection    ( const FeatureVector &other, 
                              bool bNormalizeResult=true ) const;
  float compChi2Measure     ( const FeatureVector &other, 
                              bool bNormalize=true ) const;
  float compChi2Significance( const FeatureVector &other, 
                              bool bNormalize=true ) const;
  float compBhattacharyya   ( const FeatureVector &other, 
                              bool bNormalizeInputs=true ) const;

protected:
    virtual int   idx( int x ) const;

    int            m_nDims;
    bool           m_bSizeDefined;

    int            m_nTotalNumBins;
    vector<float>  m_vBins;     	// contains all bins, access via idx function
    int			   m_targetClass;	// target class for multi-class problems
};


/****************************/
/*   Associated Functions   */
/****************************/
bool saveFeatureVectorList( string filename,
                            const vector<FeatureVector> &vFeatureVectors,
                            bool gzip=true, bool verbose=false );
bool loadFeatureVectorList( string filename,
                            vector<FeatureVector> &vFeatureVectors,
                            bool verbose=false );
//bool loadFeatureVectorList( string filename,
//                            vector<FeatureVector> &vFeatureVectors,
//                            int w, int h );


string readGZLine(gzFile& file);
//QTextStream* readGZLine(gzFile& file);


bool  readKeyWord( char* ifile, string KeyWords );
bool readKeyWord( ifstream &ifile, string KeyWords );
vector<string> extractWords( string WordList );

void computeFeatureStatistics( vector<FeatureVector> vFeatureVectors,
                               vector<float> &vMeans,
                               vector<float> &vVariances );

ostream& operator<<(ostream& output, FeatureVector& fv);
istream& operator>>(istream& input, FeatureVector& fv);


/****************************/
/*      Inline Methods      */
/****************************/

template<typename T>FeatureVector::FeatureVector( vector<T> data, int targetClass )
{
  m_nDims = data.size();
  m_bSizeDefined = true;

  m_vBins.clear();
  m_vBins.reserve(data.size());
  for( int i=0; i<(int)data.size(); i++ )
    m_vBins.push_back( static_cast<float>(data[i]) );
  m_nTotalNumBins = data.size();
  m_targetClass = targetClass;
}

template<typename T>FeatureVector::FeatureVector( T* data, int dataLen, int targetClass )
{
  m_nDims = dataLen;
  m_bSizeDefined = true;

  m_vBins.clear();
  m_vBins.reserve(dataLen);
  for( int i=0; i < dataLen; i++ )
    m_vBins.push_back( static_cast<float>(data[i]) );
  m_nTotalNumBins = dataLen;
  m_targetClass = targetClass;
}

#ifdef _USE_PERSONAL_NAMESPACES
//}
#endif

#endif // LEIBE_FEATUREVECTOR_HH
