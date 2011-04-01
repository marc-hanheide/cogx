#include "PlayerVideoServer.h"

namespace cast
{

void PlayerVideoServer::init(const vector<int> &dev_nums)
  throw(runtime_error)
{
  if (dev_nums.size() < 1)
    throw runtime_error(exceptionMessage(__HERE__,
          "must specify at least one camera"));

  if(dev_nums.size() != camIds.size())
    throw runtime_error(exceptionMessage(__HERE__,
          "number of devices %d does not match number of camera IDs %d",
          (int)dev_nums.size(), (int)camIds.size()));

  m_pPlayer = new PlayerCc::PlayerClient(m_playerHost, m_playerPort);
  if (! m_pPlayer) {
    throw runtime_error(exceptionMessage(__HERE__,
          "failed to connect to Player '%s:%d'"
          m_playerHost.c_str(), m_playerPort));
  }

  // We will pull the data so we can control the framerate. Should we use PUSH instead?
  m_pPlayer->SetDataMode(PLAYER_DATAMODE_PULL);
  m_pPlayer->SetReplaceRule(true, PLAYER_MSGTYPE_DATA, -1, -1);


  // Initialize camera proxies
  for (int i = 0; i < dev_nums.size(); i++) {
    PlayerCc::CameraProxy* pcam = new PlayerCc::CameraProxy(m_pPlayer, dev_nums[i]);
    if (! pcam) {
      throw runtime_error(exceptionMessage(__HERE__,
            "failed to create a camera proxy for camera %d"
            dev_nums[i]));
    }
    m_cameras.push_back(pcam);
  }

  // Wait for cameras
  for (int i = 0; i < m_cameras.size(); i++) {
    PlayerCc::CameraProxy* pcam = m_cameras[i];
    for (int t = 0; t < 100; t++)
    {
      // TODO: may need to call Read once and then check all the cameras
      m_pPlayer->Read();
      if (pcam->GetWidth() > 0) break;
    }
  }

  // TODO: get the sizes of all the cameras
  camera->Decompress();
}

}; // namespace
/* vim:set sw=2 sts=4 ts=8 et:vim */
