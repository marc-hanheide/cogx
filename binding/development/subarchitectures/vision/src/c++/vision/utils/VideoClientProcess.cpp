/**
 * Video client process.
 * A process which can receive images from a video server.
 *
 * @author Michael Zillich
 * @date October 2006
 */

#include "vision/utils/VisionUtils.h"
#include "VideoClientProcess.h"

using namespace Vision;

VideoClientProcess::VideoClientProcess(const string &_id)
  :  WorkingMemoryAttachedComponent(_id), ManagedProcess(_id)
{
  connHub = new ConnHub();
  video = 0;
}

VideoClientProcess::~VideoClientProcess()
{
  delete connHub;
}

void VideoClientProcess::configure(map<string,string> & _config)
{
  string videoHost;
  int videoPort;

  // first let the base class configure itself
  ManagedProcess::configure(_config);

  videoHost = _config["--videohost"];
  videoPort = strtol(_config["--videoport"].c_str(), NULL, 10);
  if(!videoHost.empty() && videoPort != 0)
    openVideoConnection(videoHost, videoPort);
}

void VideoClientProcess::openVideoConnection(const string &host, int port)
{
  video = new VideoClt();
  connHub->openConnection(video, host, port);
}


void VideoClientProcess::addChangeFilter(const cdl::WorkingMemoryChangeFilter & _filter, WorkingMemoryChangeReceiver * _pReceiver) 
{
    ManagedProcess::addChangeFilter(_filter, _pReceiver);
}

/*
void VideoClientProcess::addChangeFilter(const string &_type, 
				    const cdl::WorkingMemoryOperation & _op, 
				    const bool & _local,
				    WorkingMemoryChangeReceiver * _pReceiver)
{
  ManagedProcess::addChangeFilter(_type, _op, _local, _pReceiver);
}
*/

void VideoClientProcess::setPullConnector(const string &_connectionID,
					  PullConnectorOut<ImageFrame> *pc)
{
  img_pull = pc;
}

ImageFrame* VideoClientProcess::GetImage(int camNum)
{
  if(img_pull)
  {
    char reference[100];

    snprintf(reference, 100, "%d", camNum);
    FrameworkQuery query(getProcessIdentifier(), "");
    query.setQuery(string(reference));

    FrameworkLocalData<ImageFrame> *img_local_data = 0;
    img_pull->pull(query, img_local_data);
    if(img_local_data)
    {
      ImageFrame *img = img_local_data->data();
      // protect our data
      img_local_data->data() = NULL;
      // clean up rest
      delete img_local_data;
      return img;
    }
    else {
      throw BALTException(__HERE__, "image pull failed");
    }
  }
  else {
    throw BALTException(__HERE__, "pull connector for image not set");
  }
}

void VideoClientProcess::getImage(int camNum, ImageFrame &img)
{
  if(video != 0)
    video->getImage(camNum, img);
  else
    throw BALTException(__HERE__, "video connection is not configured, use "
        "options --videoHost and --videoPort");
}

