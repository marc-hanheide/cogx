#include "RecogniserP.h"
#include "MatlabHelper.h"
#include "libFeatureLearningCtf.h"

void R_LoadAvModels(const char* filename)
{
      mwArray fname (filename);
      LRloadAVmodels(fname);
}

void R_RunComponent(int fl)
{
//   mwArray flag(1, 1, mxSINGLE_CLASS);
   mwArray flag(fl);
   //flag = CMatlabHelper::idl2array(par);
   CLFstart(flag); // 0 - don't need the return value

/*   
   mwArray vmHS;
   VMstart(0, vmHS); // 0 - don't need the return value
   mwArray lrHs;
   LRstart(0, lrHs); // 0 - don't need the return value
*/
}

Vision::RecognisedAttributes* R_Recognise(const Vision::ROI &Roi)
{
   mwArray ansYes, ansPy, f;
   f = CMatlabHelper::idl2array(Roi.m_features);
   cosyRecogniser_recognise(2, ansYes, ansPy, f);

   Vision::RecognisedAttributes* attr = new Vision::RecognisedAttributes();
   CMatlabHelper::array2idl(ansYes, attr->m_attributes);
   CMatlabHelper::array2idl(ansPy, attr->m_probableAttributes);
   
   return attr;
}

void R_Update(const Vision::LearnInstruction &learnInstruction, const Vision::ROI &Roi)
{
   // Put features of the ROI to the matlab engine.
   mwArray features = CMatlabHelper::idl2array(Roi.m_features);

   long cntPlus = 0, cntMinus = 0;
   for (unsigned i = 0; i < learnInstruction.m_features.length(); i++) {
      if (learnInstruction.m_features[i].m_confidence > 0) cntPlus++;
      if (learnInstruction.m_features[i].m_confidence < 0) cntMinus++;
   }

   if (cntPlus > 0) {
      mwArray avw(cntPlus, 2, mxDOUBLE_CLASS, mxREAL);
      int row = 1;
      for (unsigned i = 0; i < learnInstruction.m_features.length(); i++) {
         if (learnInstruction.m_features[i].m_confidence > 0) {
            avw(row, 1) =  (double) learnInstruction.m_features[i].m_int;
            avw(row, 2) =  (double) learnInstruction.m_features[i].m_confidence;
            row++;
         }
      }
      cosyRecogniser_update(features, avw);
   }

   if (cntMinus > 0) {
      mwArray avw(cntMinus, 2, mxDOUBLE_CLASS, mxREAL);
      int row = 1;
      for (unsigned i = 0; i < learnInstruction.m_features.length(); i++) {
         if (learnInstruction.m_features[i].m_confidence < 0) {
            avw(row, 1) =  (double) learnInstruction.m_features[i].m_int;
            avw(row, 2) =  (double) - learnInstruction.m_features[i].m_confidence;
            row++;
         }
      }
      cosyRecogniser_unlearn(features, avw);
   }
}
