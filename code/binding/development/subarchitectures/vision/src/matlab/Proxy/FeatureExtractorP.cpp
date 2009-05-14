#include "FeatureExtractorP.h"
#include "MatlabHelper.h"
#include "libFeatureLearningCtf.h"

void FE_extract_features(Vision::ROI &Roi)
{
   // Add the ROI to Matlab engine
   Matlab::Matrix roiMx;
   CMatlabHelper::sequence2matrix(&(Roi.m_region.m_image[0]), roiMx,
         Roi.m_region.m_width, Roi.m_region.m_height,
         Roi.m_region.m_nChannels);
   mwArray x = CMatlabHelper::idl2array(roiMx);

   // Add the segmentation mask to the Matlab engine.
   Matlab::Matrix maskMx;
   CMatlabHelper::sequence2matrix(&(Roi.m_mask.m_image[0]), maskMx,
         Roi.m_mask.m_width, Roi.m_mask.m_height,
         Roi.m_mask.m_nChannels);
   mwArray b0 = CMatlabHelper::idl2array(maskMx);

   // Segment the Roi.
   mwArray b;
   cosyFeatureExtractor_limitvalue(1, b, b0, mwArray(1.0));

   // Extract features.
   mwArray f;
   extAPfeatures(1, f, x, b);

   // Add extracted features to the Roi.
   CMatlabHelper::array2idl(f, Roi.m_features);
}
