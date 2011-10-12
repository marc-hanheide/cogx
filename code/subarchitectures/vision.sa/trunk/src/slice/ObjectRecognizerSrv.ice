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
      double phi;
      double lambda;
      double theta;
   };
   sequence<ObjectPose> ObjectPoseSeq;

   struct RecognitionResult {
      string label;
      double probability;
      ObjectPoseSeq poses;
      DoubleSeq posePd;
   };
   sequence<RecognitionResult> RecognitionResultSeq;

   class ObjectRecognitionTask {
      // REQUEST:
      // (review2010) TODO: remove protoObjectAddr, use visualObjectAddr.protoObjectID instead
      cast::cdl::WorkingMemoryAddress protoObjectAddr;
      cast::cdl::WorkingMemoryAddress visualObjectAddr;

      // (review2011) true if the recognizer should write the result into the visualObject
      bool overwriteVisualObject;

      // RESPONSE
      RecognitionResultSeq matches;
   };

   interface ObjectRecognizerInterface {
      long LoadObjectModel(string modelPath);

      // Finds SIFT features in the image using SiftGPU.
      // Returns the number of features found. The features are stored in:
      //    out features: array with 4*n elements (order: X, Y, Scale, Rotation)
      //    out descriptors: array with 128*n elements (values in range: 0-0.5)
      long GetSifts(Video::Image image,
            int left, int top, int width, int height,
            out FloatSeq features, out FloatSeq descriptors);

      void FindMatchingObjects(Video::Image image,
            int left, int top, int width, int height,
            out RecognitionResultSeq recognized);

      // update the model modelName with features extracted from the image
      // TODO: add image area to process (box)
      // TODO: add object pose relative to camera position/orientation
      void UpdateModel(string modelName, Video::Image image);
   };
};

#endif

