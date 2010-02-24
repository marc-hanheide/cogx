#ifndef OBJECT_RECOGNIZER_BACKEND_ICE
#define OBJECT_RECOGNIZER_BACKEND_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>
#include <Video.ice>

module ObjectRecognizerIce {
   sequence<double> DoubleSeq;
   sequence<float> FloatSeq;

   // simple object pose
   struct ObjectPose {
      double omega;
      double phi;
      double rotation;
   };
   sequence<ObjectPose> ObjectPoseSeq;

   struct RecognitionResult {
      string label;
      double probability;
      ObjectPoseSeq poses;
      DoubleSeq posePd;
   };
   sequence<RecognitionResult> RecognitionResultSeq;

   interface ObjectRecognizerInterface {
      // Finds SIFT features in the image using (Py)SiftGPU.
      // Returns the number of features found. The features are stored in:
      //    out features: array with 4*n elements (order: X, Y, Scale, Rotation)
      //    out descriptors: array with 128*n elements (values in range: 0-0.5)
      long GetSifts(Video::Image image,
            out FloatSeq features, out FloatSeq descriptors);

      void FindMatchingObjects(
            Video::Image image, cogx::Math::Rect2 region,
            out RecognitionResultSeq recognized);
   };
};

#endif

