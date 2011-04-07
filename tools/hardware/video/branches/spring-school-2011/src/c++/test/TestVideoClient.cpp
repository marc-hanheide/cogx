/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <cstdio>
#include "TestVideoClient.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::TestVideoClient();
  }
}

namespace cast
{

using namespace std;

void TestVideoClient::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camids")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }
}

void TestVideoClient::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

  // start receiving images pushed by the video server
  videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);
}

void TestVideoClient::receiveImages(const std::vector<Video::Image>& images)
{
  for(size_t i = 0; i < images.size(); i++)
  {
    char filename[100];
    FILE *ppm;
    snprintf(filename, 100, "testvideoclient-%d.ppm", (int)i);
    ppm = fopen(filename, "w");
    fprintf(ppm,"P6\n%d %d\n255\n",images[i].width,images[i].height);
    fwrite(&images[i].data[0], 1, 3*images[i].width*images[i].height, ppm);
    fclose(ppm);
    log("image %dx%d from cam %d at time %d.%06d", 
        images[i].width, images[i].height, images[i].camId,
        images[i].time.s, images[i].time.us);
  }
}

}

