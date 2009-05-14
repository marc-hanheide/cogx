/**
 * @author Somboon Hongeng, Michael Zillich
 * @date October 2006
 */

#ifndef V4LDEVICE_H_
#define V4LDEVICE_H_

#include <vector>
#include <string>
#include <asm/types.h>
#include <linux/videodev2.h>
#include "VideoDevice.h"

struct s_buffer
{
  void *start;
  size_t length;
};

typedef enum
{
  IO_METHOD_READ,
  IO_METHOD_MMAP,
  IO_METHOD_USERPTR,
} io_method;

/**
 */
class V4LDevice : public VideoDevice
{
private:
  std::vector<int> cam_ids;
  std::vector<std::string> dev_names;
  std::vector<int> fds;
  std::vector<struct s_buffer *> buffers;
  std::vector<int> n_buffers;
  io_method io;
  int width, height;
  __u32 pref_formats[5];
  __u32 pixel_format;

  void open_devices();
  int open_device(const std::string &dev_name);
  void close_device(int i);
  void close_devices();
  void init_device(int i);
  void init_devices();
  void init_read(int i, unsigned int buffer_size);
  void init_mmap(int i);
  void init_userp(int i, unsigned int buffer_size);
  void uninit_device(int i);
  void uninit_devices();
  void start_capturing(int i);
  void start_capturing();
  void stop_capturing(int i);
  void stop_capturing();
  int read_frame(int i, std::vector<Vision::ImageFrame*> &frames);
  void convert_frame(const char *buf, Vision::ImageFrame *frame);

public:
  V4LDevice(const std::vector<int> &ids,
    const std::vector<std::string> &devs,
    int req_width, int req_height);
  virtual ~V4LDevice();
  virtual void GrabFrames();
  virtual void RetrieveFrames(std::vector<Vision::ImageFrame*> &frames);
  virtual void RetrieveFrame(int camNum, Vision::ImageFrame *frame);
  virtual int NumCameras();
  virtual void GetCameraIds(std::vector<int> &ids);
  virtual void GetImageSize(int &width, int &height);
  virtual int GetFramerate();
};

#endif

