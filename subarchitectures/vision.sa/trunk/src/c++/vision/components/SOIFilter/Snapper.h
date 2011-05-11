#ifndef SNAPPER_IL0I9H0X
#define SNAPPER_IL0I9H0X

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <Video.hpp>

#include <map>
#include <string>

namespace cast
{

class Snapper
{
  // snapshot support: save video images when -v flag active
  Video::Image m_LeftImage;
  Video::Image m_RightImage;
  int m_idLeftImage;
  int m_idRightImage;

  Video::Image m_ImageLeft;
  Video::Image m_ImageRight;
  Video::Image m_ImageRectLeft;
  std::string m_snapshotFiles;
  std::string m_snapshotFlags; // A:ll, p:oints, l:eft, r:ight, s:segmented, m:mask, L:eftRect, R:ightRect
  
public:
  bool m_bAutoSnapshot;
  /*
   * WARNING!!! These variables depend on the mercy of the owner.
   * If not set correctly, they will crash the system.
  */
  Video::VideoInterfacePrx videoServer;
  CASTComponent* logger;
  VisionData::ProtoObjectPtr m_LastProtoObject;

private:
  bool hasSnapFlag(char ch);
  void saveImage(const std::string& name, const std::string& path, const Video::Image& image);
  void saveMask(const std::string& name, const std::string& path, const VisionData::SegmentMask& image);
public:
  Snapper();
  void configure(const std::map<std::string,std::string> & _config);
  void snapVideo();
  void saveSnapshot();
};

} // namespace

#endif /* end of include guard: SNAPPER_IL0I9H0X */
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */
