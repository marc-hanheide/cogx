/**
 * @author Kai Zhou	
 * @date April 2009
 *
 * Just receives images and displays them.
 * A dummy component showing how to get images.
 */

#ifndef RECONSTRCUTOR_H
#define RECONSTRCUTOR_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>
#include <TooN/numerics.h>

namespace cast
{

class Reconstructor : public VideoClient,
                    public ManagedComponent
{
private:
  /**
   * Which camera to get images from
   */
  int camId;
  std::vector<std::string> model_labels;
  void postObjectToWM(const std::vector<std::string> & labels, const Video::Image &image, Vector<3> center, Vector<3> size, double radius);
  void postObjectToWM_Internal(unsigned int i, const Video::Image &image, Vector<3> center, Vector<3> size, double radius);
  /**
   * WM Id of our tracked objects
   */
  std::vector<std::string> objWMIds;

  VisionData::SOIPtr createObj(unsigned int i, const Video::Image &image, Vector<3> center, Vector<3> size, double radius);

protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();

public:
  Reconstructor() : camId(0) {}
  virtual ~Reconstructor() {}
};

}

#endif



