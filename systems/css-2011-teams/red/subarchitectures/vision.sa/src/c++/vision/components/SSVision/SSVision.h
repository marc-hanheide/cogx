/**
 * @author Team RED
 * @date 2011
 * @brief Vision system for the spring school system
 */

#ifndef SS_VISION_H
#define SS_VISION_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include "Math.hpp"


namespace cast
{

class SSVision : public ManagedComponent
{
private:
  bool stopRecognizeObjects;
  bool stopDetectingTables;
  cdl::WorkingMemoryAddress addr;     ///< Address of the incomming working memory command
  
  bool objectFound;                   ///< set to true if new object was recognized
  std::string objectFoundID;               ///< ID of the visual object in working memory
  std::vector<std::string> labels;    ///< labels of the objects (same than in Recognizer3D cast-file) 

  void receivedVisionCommand(const cdl::WorkingMemoryChange & _wmc);
  void newVisualObject(const cdl::WorkingMemoryChange & _wmc);
  void RecognizeObject();
  void DetectTable();
  void WriteCommandToWM(cast::cdl::WorkingMemoryAddress addr, int cmd, bool success, std::string id);


protected:

  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  SSVision();
  virtual ~SSVision() {}
};

}

#endif



