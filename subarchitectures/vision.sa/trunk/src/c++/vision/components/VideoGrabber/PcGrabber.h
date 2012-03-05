/**
 * @author Marko Mahnič
 * @created March 2011
 *
 */

#ifndef _VIDEOGRABBER_PCGRABBER_H_4F547C7F_
#define _VIDEOGRABBER_PCGRABBER_H_4F547C7F_

#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
#include "CoreStructs.h"

#include <PointCloudClient.h>
#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include <map>
#include <string>
#include <vector>

namespace cogxgrabber
{

class CPcGrabClient: public cast::PointCloudClient, public CDataSource
{
  CGrabbedItemPtr mLastPoints;
  CGrabbedItemPtr mLastDepth;
  CGrabbedItemPtr mLastRectImage;
public:
  bool mbGrabPoints;
  bool mbGrabDepth;
  bool mbGrabRectImage;
  CPcGrabClient();
  void configurePcComm(const std::map<std::string,std::string> & _config)
  {
    configureServerCommunication(_config);
  }
  void startPcComm(cast::CASTComponent &owner)
  {
    startPCCServerCommunication(owner);
  };
  virtual void grab(std::vector<CGrabbedItemPtr>& items) /*override*/;
  virtual void getPreviews(std::vector<CPreview>& previews,
      int width, int height, bool isGrabbing) /*override*/;
#ifdef FEAT_VISUALIZATION
  virtual void configExtraV11n(cogx::display::CDisplayClient& display) /*override*/;
  virtual void displayExtra(cogx::display::CDisplayClient& display) /*override*/;
#endif
};

class CGrabbedPcPoints: public CGrabbedItem
{
private:
  friend class CPcGrabClient;
  std::vector<PointCloud::SurfacePoint> mPoints;
public:
  CGrabbedPcPoints()
  {
    mName = "Point Cloud Points";
  }
  virtual CExtraSaverPtr save(const CRecordingInfo& recinfo, int deviceId) /*override*/;
};
#endif

} // namespace
#endif /* _VIDEOGRABBER_PCGRABBER_H_4F547C7F_ */
/* vim: set sw=2 ts=8 et: */
