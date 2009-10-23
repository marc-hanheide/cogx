#ifndef DisplayConvexHullPB_hpp
#define DisplayConvexHullPB_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <string>

#include <peekabot.hh>
#include <peekabot/Types.hh>
#include <VisionData.hpp>

class DisplayConvexHullPB : public cast::ManagedComponent 
{
public:
  DisplayConvexHullPB();
  virtual ~DisplayConvexHullPB();
  
  virtual void runComponent();
  virtual void start();
  
protected:

  virtual void configure(const std::map<std::string, std::string>& _config);

private:
  std::vector<double> previouscenter;
  void newConvexHull(const cast::cdl::WorkingMemoryChange &objID);
  void connectPeekabot();
  int m_RetryDelay; // Seconds to retry if cannot connect. -1 means dont retry

  double m_FovH; // horisontal fov in degs
  double m_FovV; // vertical fov in degs

  IceUtil::Mutex m_Mutex;

  std::string m_PbHost;
  int m_PbPort;

  peekabot::PeekabotClient m_PeekabotClient;
  peekabot::ObjectProxy m_ProxyRoot;
};

#endif // DisplayConvexHullPB_hpp
