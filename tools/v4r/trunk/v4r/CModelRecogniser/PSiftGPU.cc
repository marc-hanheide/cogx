/**
 * $Id$
 * Johann Prankl, 2011-04-13 
 * prankl@acin.tuwien.ac.at
 */


#include "PSiftGPU.hh"

namespace P 
{

PSiftGPU::PSiftGPU(Parameter p, cv::Ptr<SiftGPU> _sift, cv::Ptr<SiftMatchGPU> _matcher, int memSize)
{
  gpuMemSize = memSize;
  param=p;

  if (_sift.empty())
  {
    //init sift
    char * argv[] = {"-m", "-fo","-1", "-s", "-v", "1", "-pack"};

    int argc = sizeof(argv)/sizeof(char*);
    sift = new SiftGPU();
    sift->ParseParam(argc, argv);

    //create an OpenGL context for computation
    if(sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
      throw runtime_error ("PSiftGPU::PSiftGPU: No GL support!");
  }
  else
  {
    sift = _sift;
  }
  if (_matcher.empty())
  {
    gpuMemSize = 4096;
    matcher = new SiftMatchGPU(4096);
    matcher->VerifyContextGL();
  }
  else
  {
    matcher = _matcher;
  }
}

PSiftGPU::~PSiftGPU()
{
}




/************************************** PRIVATE ************************************/


/**
 * compute descriptors for given keypoints
 */
void PSiftGPU::computeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, 
                 cv::Mat& descriptors) const
{
  if (keypoints.size()==0)
    return;

  cv::Mat grayImage;
  if( image.type() != CV_8U ) cv::cvtColor( image, grayImage, CV_BGR2GRAY );
  else grayImage = image;

  vector<SiftGPU::SiftKeypoint> ks(keypoints.size());
  for (unsigned i=0; i<ks.size(); i++)
  {
    ks[i].x = keypoints[i].pt.x;
    ks[i].y = keypoints[i].pt.y;
    ks[i].s = keypoints[i].size / 3.;
    ks[i].o = -keypoints[i].angle;
  }

  SiftGPU *pSift = (SiftGPU*)&(*sift);
  pSift->SetKeypointList(ks.size(), &ks[0], 0);

  if(pSift->RunSIFT(grayImage.cols,grayImage.rows,grayImage.ptr<uchar>(0),GL_LUMINANCE,GL_UNSIGNED_BYTE))
  {
    int num = pSift->GetFeatureNum();
    if (num==(int)keypoints.size())
    {
      descriptors = cv::Mat(num,128,CV_32F);
      pSift->GetFeatureVector(NULL, descriptors.ptr<float>(0));
    }
    else cout<<"No SIFT found"<<endl;
  }
  else throw runtime_error ("PSiftGPU::computeImpl: SiftGPU Error!");
}

/**
 * knnMatchImpl
 * mask and compactResult are not supported
 */
void PSiftGPU::knnMatchImpl( const cv::Mat& queryDescriptors, vector<vector<cv::DMatch> >& matches,
                 int k, const vector<cv::Mat>& masks, bool compactResult) 
{
  matches.clear();
  if (trainDescCollection.size()==0 || queryDescriptors.rows == 0)
    return;

  cv::Mat tmpTrainDesc, tmpQueryDesc;
  int num, numTrain=0;
  int (*match_buf)[2] = new int[(int)queryDescriptors.rows][2];

  for (unsigned i=0; i<trainDescCollection.size(); i++)
    numTrain += trainDescCollection[i].rows;

  vector<unsigned> ltView, ltKey;

  if (gpuMemSize < (int)queryDescriptors.rows)
  { 
    gpuMemSize=(int)queryDescriptors.rows;
    matcher->SetMaxSift((int)queryDescriptors.rows);
  }
  if (gpuMemSize < (int)numTrain)
  {
    gpuMemSize=numTrain;
    matcher->SetMaxSift(numTrain);
  }

  // prepare data structure
  if (queryDescriptors.isContinuous()) tmpQueryDesc = queryDescriptors;
  else queryDescriptors.copyTo(tmpQueryDesc);

  if (trainDescCollection.size()==1)
  {
    if (trainDescCollection[0].isContinuous()) tmpTrainDesc = trainDescCollection[0];
    else trainDescCollection[0].copyTo(tmpTrainDesc);
  }
  else
  {
    unsigned z=0;
    tmpTrainDesc = cv::Mat(numTrain,128,CV_32F);
    ltView.resize(numTrain), ltKey.resize(numTrain);

    for (unsigned i=0; i<trainDescCollection.size(); i++)
    {
      for (unsigned j=0; j<trainDescCollection[i].rows; j++, z++)
      {
        tmpTrainDesc.row(z) = trainDescCollection[i].row(j);
        ltView[z] = i;
        ltKey[z] = j;
      }
    }
  }
  
  matcher->SetDescriptors(0, tmpQueryDesc.rows, tmpQueryDesc.ptr<float>(0));  // image keypoints
  matcher->SetDescriptors(1, tmpTrainDesc.rows, tmpTrainDesc.ptr<float>(0));  // codebook

  num = matcher->GetSiftMatch(tmpQueryDesc.rows, match_buf, param.distmax,
                              param.ratiomax, param.mutual_best_match);
    
  matches.resize(num,vector<cv::DMatch>(1));
  if (ltView.size()>0)
  {
    for (unsigned i=0; i<(unsigned)num; i++)
    {
      matches[i][0] = cv::DMatch( match_buf[i][0], ltKey[match_buf[i][1]],ltView[match_buf[i][1]], 
                                  Distance128(tmpQueryDesc.ptr<float>(match_buf[i][0]),
                                              tmpTrainDesc.ptr<float>(match_buf[i][1])) );
    }
  }
  else
  {
    for (unsigned i=0; i<(unsigned)num; i++)
    {
      matches[i][0] = cv::DMatch( match_buf[i][0], match_buf[i][1], 
                                  Distance128(tmpQueryDesc.ptr<float>(match_buf[i][0]),
                                              tmpTrainDesc.ptr<float>(match_buf[i][1])) );
    }
  }

  delete[] match_buf;
}

/**
 * radiusMatchImpl
 */
void PSiftGPU::radiusMatchImpl(const cv::Mat& queryDescriptors,vector<vector<cv::DMatch> >& matches,
             float maxDistance, const vector<cv::Mat>& masks, bool compactResult )
{
  matches.clear();
  if (trainDescCollection.size()==0 || queryDescriptors.rows == 0)
    return;

  cv::Mat tmpTrainDesc, tmpQueryDesc;
  int num, numTrain=0;
  int (*match_buf)[2] = new int[(int)queryDescriptors.rows][2];

  for (unsigned i=0; i<trainDescCollection.size(); i++)
    numTrain += trainDescCollection[i].rows;

  vector<unsigned> ltView, ltKey;

  if (gpuMemSize < (int)queryDescriptors.rows)
  { 
    gpuMemSize=(int)queryDescriptors.rows;
    matcher->SetMaxSift((int)queryDescriptors.rows);
  }
  if (gpuMemSize < (int)numTrain)
  {
    gpuMemSize=numTrain;
    matcher->SetMaxSift(numTrain);
  }

  // prepare data structure
  if (queryDescriptors.isContinuous()) tmpQueryDesc = queryDescriptors;
  else queryDescriptors.copyTo(tmpQueryDesc);

  if (trainDescCollection.size()==1)
  {
    if (trainDescCollection[0].isContinuous()) tmpTrainDesc = trainDescCollection[0];
    else trainDescCollection[0].copyTo(tmpTrainDesc);
  }
  else
  {
    unsigned z=0;
    tmpTrainDesc = cv::Mat(numTrain,128,CV_32F);
    ltView.resize(numTrain), ltKey.resize(numTrain);

    for (unsigned i=0; i<trainDescCollection.size(); i++)
    {
      for (unsigned j=0; j<trainDescCollection[i].rows; j++, z++)
      {
        tmpTrainDesc.row(z) = trainDescCollection[i].row(j);
        ltView[z] = i;
        ltKey[z] = j;
      }
    }
  }
  
  matcher->SetDescriptors(0, tmpQueryDesc.rows, tmpQueryDesc.ptr<float>(0));  // image keypoints
  matcher->SetDescriptors(1, tmpTrainDesc.rows, tmpTrainDesc.ptr<float>(0));  // codebook

  num = matcher->GetSiftMatch(tmpQueryDesc.rows, match_buf, maxDistance,
                            param.ratiomax, param.mutual_best_match);
    
  matches.resize(num,vector<cv::DMatch>(1));
  if (ltView.size()>0)
  {
    for (unsigned i=0; i<(unsigned)num; i++)
    {
      matches[i][0] = cv::DMatch( match_buf[i][0], ltKey[match_buf[i][1]],ltView[match_buf[i][1]], 
                                  Distance128(tmpQueryDesc.ptr<float>(match_buf[i][0]),
                                              tmpTrainDesc.ptr<float>(match_buf[i][1])) );
    }
  }
  else
  {
    for (unsigned i=0; i<(unsigned)num; i++)
    {
      matches[i][0] = cv::DMatch( match_buf[i][0], match_buf[i][1], 
                                  Distance128(tmpQueryDesc.ptr<float>(match_buf[i][0]),
                                              tmpTrainDesc.ptr<float>(match_buf[i][1])) );
    }
  }

  delete[] match_buf;

}



/************************************** PUBLIC ************************************/
/**
 * compute dog and sift descriptor
 */
void PSiftGPU::DetectDescriptor(const cv::Mat &img, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat &descriptors, cv::Mat mask)
{
  keys.clear();
  cv::Mat grayImage;
  if( img.type() != CV_8U ) cv::cvtColor( img, grayImage, CV_BGR2GRAY );
  else grayImage = img;

  if(sift->RunSIFT(grayImage.cols,grayImage.rows,grayImage.ptr<uchar>(0),GL_LUMINANCE,GL_UNSIGNED_BYTE))
  {
    int num = sift->GetFeatureNum();
    if (num>0)
    {
      vector<SiftGPU::SiftKeypoint> ks(num);
      descriptors = cv::Mat(num, 128, CV_32F);

      sift->GetFeatureVector(&ks[0], descriptors.ptr<float>(0));

      //copy sift
      if (mask.size() != img.size() || mask.type()!=CV_8U)
      {
        keys.resize(num);
        for (int i=0; i<num; i++)
        {
          keys[i] = new PKeypoint(cv::Point2d(ks[i].x,ks[i].y), ks[i].s*3, -ks[i].o, 1);
          keys[i]->id = i;
        }
      }
      else
      {
        for (int i=0; i<num; i++)
          if (mask.at<uchar>((int)(ks[i].y+.5),(int)(ks[i].x+.5)) > 0)
          {
            keys.push_back(new PKeypoint(cv::Point2d(ks[i].x,ks[i].y), ks[i].s*3, -ks[i].o, 1));
            keys.back()->id = i;
          }
      }
    }else cout<<"No SIFT found"<<endl;
  }else throw runtime_error ("PSiftGPU::DetectDescriptor: SiftGPU Error!");
}

/**
 * compute dog
 */
void PSiftGPU::Detect(const cv::Mat &img, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask)
{
  keys.clear();
  cv::Mat grayImage;
  if( img.type() != CV_8U ) cv::cvtColor( img, grayImage, CV_BGR2GRAY );
  else grayImage = img;

  if(sift->RunSIFT(grayImage.cols,grayImage.rows,grayImage.ptr<uchar>(0),GL_LUMINANCE,GL_UNSIGNED_BYTE))
  {
    int num = sift->GetFeatureNum();
    if (num>0)
    {
      vector<SiftGPU::SiftKeypoint> ks(num);

      sift->GetFeatureVector(&ks[0], NULL);

      //copy sift
      if (mask.size() != img.size() || mask.type()!=CV_8U)
      {
        keys.resize(num);
        for (int i=0; i<num; i++)
        {
          keys[i] = new PKeypoint(cv::Point2d(ks[i].x,ks[i].y), ks[i].s*3, -ks[i].o, 1);
          keys[i]->id = i;
        }
      }
      else
      {
        for (int i=0; i<num; i++)
          if (mask.at<uchar>((int)(ks[i].y+.5),(int)(ks[i].x+.5)) > 0)
          {
            keys.push_back(new PKeypoint(cv::Point2d(ks[i].x,ks[i].y), ks[i].s*3, -ks[i].o, 1));
            keys.back()->id = i;
          }
      }
    }else cout<<"No SIFT found"<<endl;
  }else throw runtime_error ("PSiftGPU::Detect: SiftGPU Error!");
}

/**
 * compute descriptors for given keypoints
 */
void PSiftGPU::compute(const cv::Mat& image, vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors )
{
  computeImpl(image, keypoints, descriptors);
}

/**
 * match descriptors on gpu
 * no mask used
 */
/*void PSiftGPU::match( const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors, vector<cv::DMatch>& matches, const cv::Mat& mask)
{
  add( vector<cv::Mat>(1, trainDescriptors) );
  vector<vector<cv::DMatch> > knnMatches;
  knnMatchImpl(queryDescriptors, knnMatches, 1, vector<cv::Mat>(1,mask), true);

  // copy matches
  matches.clear();
  for (unsigned i=0; i<knnMatches.size(); i++)
    for (unsigned j=0; j<knnMatches[i].size(); j++)
      matches.push_back(knnMatches[i][j]);
}*/

/**
 * add train descriptors for matching
 */
/*void PSiftGPU::add( const vector<cv::Mat>& descriptors )
{
cout<<"add"<<endl;
  for (unsigned i=0; i<descriptors.size(); i++)
    trainDescriptors.push_back(descriptors[i]);
}*/

/**
 * clear train descriptors of matching storage
 */
/*void PSiftGPU::clear()
{
  trainDescriptors.clear();
}*/

/**
 * clone
 */
cv::Ptr<cv::DescriptorMatcher> PSiftGPU::clone( bool emptyTrainData ) const 
{
  PSiftGPU* tmpMatcher = new PSiftGPU(param, sift,matcher, gpuMemSize);

  return tmpMatcher;
}

}

