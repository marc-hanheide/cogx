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

typedef struct ObjP
{
	Vector<3> c;
	Vector<3> s;
	double r;
	std::string id;
}ObjPara;

private:
  /**
   * Which camera to get images from
   */
  int camId;

  VisionData::SOIPtr createObj(Vector<3> center, Vector<3> size, double radius);

  std::vector<ObjPara> PreviousObjList;
  std::vector<ObjPara> CurrentObjList;
  bool Compare2SOI(ObjPara obj1, ObjPara obj2);
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



