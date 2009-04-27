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

void TestVideoClient::start()
{
  startVideoCommunication(*this);
}

void TestVideoClient::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  configureVideoCommunication(_config);
}

void TestVideoClient::runComponent()
{
  while(isRunning())
  {
    log("TestVideoClient - getting and storing image");
    vector<Video::Image> images;
    getImages(images);
    for(size_t i = 0; i < images.size(); i++)
    {
      FILE *ppm;
      ppm = fopen("testvideoclient.ppm","w");
      fprintf(ppm,"P6\n%d %d\n255\n",images[i].width,images[i].height);
      fwrite(&images[i].data[0], 1, 3*images[i].width*images[i].height, ppm);
      fclose(ppm);
      log("image from cam %d at time %d.%06d", images[i].camId,
          images[i].time.s, images[i].time.us);
    }
    sleepComponent(1000);
  }
}

}

