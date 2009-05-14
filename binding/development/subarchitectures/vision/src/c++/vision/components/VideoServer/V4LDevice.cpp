/**
 * Video4Linux2 device.
 * Derived from V4L2 video capture example capture.c
 * (see http://www.linuxtv.org/downloads/video4linux/API/V4L2_API)
 */


//#define B21_SERVER

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>		/* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>		/* for videodev2.h */
#include <algorithm>

#include <balt/interface/BALTTimer.hpp>
#include <vision/utils/VisionUtils.h>
#include "V4LDevice.h"

using namespace Vision;

#define CLEAR(x) memset (&(x), 0, sizeof (x))

static int xioctl(int fd, int request, void *arg)
{
  int r;

  do
    r = ioctl(fd, request, arg);
  while (-1 == r && EINTR == errno);

  return r;
}

static const char* pix_fmt_str(__u32 fmt)
{
  static char str[5] = "";
  str[0] = (fmt >> 0) & 0xff;
  str[1] = (fmt >> 8) & 0xff;
  str[2] = (fmt >> 16) & 0xff;
  str[3] = (fmt >> 24) & 0xff;
  str[4] = '\0';
  return str;
}

#define PUSH_RGB24	1
#define PUSH_BGR24	2
#define PUSH_RGB32	3
#define PUSH_BGR32	4

/* This is a simplistic approach. */
static void ccvt_420p(int width, int height, const unsigned char *src,
    ImageFrame *dst, int push)
{
	int line, col, linewidth;
	int y, u, v, yy, vr, ug, vg, ub;
	int r, g, b;
	const unsigned char *py, *pu, *pv;
  int dst_cnt = 0;

	linewidth = width >> 1;
	py = src;
	pu = py + (width * height);
	pv = pu + (width * height) / 4;

	y = *py++;
	yy = y << 8;
	u = *pu - 128;
	ug =   88 * u;
	ub =  454 * u;
	v = *pv - 128;
	vg =  183 * v;
	vr =  359 * v;

	for (line = 0; line < height; line++) {
		for (col = 0; col < width; col++) {
			r = (yy +      vr) >> 8;
			g = (yy - ug - vg) >> 8;
			b = (yy + ub     ) >> 8;

			if (r < 0)   r = 0;
			if (r > 255) r = 255;
			if (g < 0)   g = 0;
			if (g > 255) g = 255;
			if (b < 0)   b = 0;
			if (b > 255) b = 255;

			switch(push) {
			case PUSH_RGB24:
				dst->m_image[dst_cnt++] = r;
				dst->m_image[dst_cnt++] = g;
				dst->m_image[dst_cnt++] = b;
				break;

			case PUSH_BGR24:
				dst->m_image[dst_cnt++] = b;
				dst->m_image[dst_cnt++] = g;
				dst->m_image[dst_cnt++] = r;
				break;
			
			case PUSH_RGB32:
				dst->m_image[dst_cnt++] = r;
				dst->m_image[dst_cnt++] = g;
				dst->m_image[dst_cnt++] = b;
				dst->m_image[dst_cnt++] = 0;
				break;

			case PUSH_BGR32:
				dst->m_image[dst_cnt++] = b;
				dst->m_image[dst_cnt++] = g;
				dst->m_image[dst_cnt++] = r;
				dst->m_image[dst_cnt++] = 0;
				break;
			}
			
			y = *py++;
			yy = y << 8;
			if (col & 1) {
				pu++;
				pv++;

				u = *pu - 128;
				ug =   88 * u;
				ub =  454 * u;
				v = *pv - 128;
				vg =  183 * v;
				vr =  359 * v;
			}
		} /* ..for col */
		if ((line & 1) == 0) { // even line: rewind
			pu -= linewidth;
			pv -= linewidth;
		}
	} /* ..for line */
}

static void ccvt_420p_bgr24(int width, int height, const char *src,
   ImageFrame *dst)
{
  ccvt_420p(width, height, (const unsigned char *)src, dst, PUSH_BGR24);
}

static void convert_bgr24_bgr24(int width, int height, const char *src,
  ImageFrame *dst)
{
  for(unsigned i = 0; i < width*height*3; i++)
    dst->m_image[i] = src[i];
}

static void convert_rgb24_bgr24(int width, int height, const char *src,
  ImageFrame *dst)
{
  for(unsigned i = 0; i < width*height; i += 3)
  {
    dst->m_image[i] = src[i+2];
    dst->m_image[i+1] = src[i+1];
    dst->m_image[i+2] = src[i];
  }
}

static void convert_bgr32_bgr24(int width, int height, const char *src,
  ImageFrame *dst)
{
  for(unsigned i = 0, j = 0; i < width*height; i += 4, j += 3)
  {
    dst->m_image[j] = src[i];
    dst->m_image[j+1] = src[i+1];
    dst->m_image[j+2] = src[i+2];
  }
}

static void convert_rgb32_bgr24(int width, int height, const char *src,
  ImageFrame *dst)
{
  for(unsigned i = 0, j = 0; i < width*height; i += 4, j += 3)
  {
    dst->m_image[j] = src[i+2];
    dst->m_image[j+1] = src[i+1];
    dst->m_image[j+2] = src[i];
  }
}

int V4LDevice::read_frame(int i, vector<ImageFrame*> &frames)
{
  struct v4l2_buffer buf;
  unsigned int j;
  ssize_t read_bytes;
  unsigned int total_read_bytes;

  switch (io)
  {
    case IO_METHOD_READ:
      total_read_bytes = 0;
      do
      {
        read_bytes = read(fds[i], buffers[i][0].start, buffers[i][0].length);
        if (read_bytes < 0)
        {
          switch (errno)
          {
            case EIO:
            case EAGAIN:
              continue;
            default:
            	throw BALTException(__HERE__, "read");
          }
        }
        total_read_bytes += read_bytes;

      } while (total_read_bytes < buffers[i][0].length);
      convert_frame((char*)buffers[i][0].start, frames[i]);

      break;

    case IO_METHOD_MMAP:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == xioctl(fds[i], VIDIOC_DQBUF, &buf))
      {
        switch (errno)
        {
          case EAGAIN:
            return 0;

          case EIO:
          /* Could ignore EIO, see spec. */

          /* fall through */

          default:
            throw BALTException(__HERE__, "VIDIOC_DQBUF");
        }
      }

      assert(buf.index < n_buffers[i]);

      convert_frame((char*)buffers[i][buf.index].start, frames[i]);

      if (-1 == xioctl(fds[i], VIDIOC_QBUF, &buf))
        throw BALTException(__HERE__, "VIDIOC_QBUF");

      break;

    case IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl(fds[i], VIDIOC_DQBUF, &buf))
      {
        switch (errno)
        {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            throw BALTException(__HERE__, "VIDIOC_DQBUF");
        }
      }

      for (j = 0; j < n_buffers[i]; ++j)
        if (buf.m.userptr == (unsigned long) buffers[i][j].start
          && buf.length == buffers[i][j].length)
          break;

      assert(j < n_buffers[i]);
      convert_frame((char*)buf.m.userptr, frames[i]);

      if (-1 == xioctl(fds[i], VIDIOC_QBUF, &buf))
        throw BALTException(__HERE__, "VIDIOC_QBUF");

      break;
  }

  return 1;
}

/**
 * Convert whatever we get from the device to our framework format BGR24.
 */
void V4LDevice::convert_frame(const char *buf, ImageFrame *frame)
{
  frame->m_width = width;
  frame->m_height = height;
  frame->m_image.length(width*height*3);
  switch(pixel_format)
  {
    case V4L2_PIX_FMT_BGR24:
      convert_bgr24_bgr24(width, height, buf, frame);
      break;
    case V4L2_PIX_FMT_RGB24:
      convert_rgb24_bgr24(width, height, buf, frame);
      break;
    case V4L2_PIX_FMT_BGR32:
      convert_bgr32_bgr24(width, height, buf, frame);
      break;
    case V4L2_PIX_FMT_RGB32:
      convert_rgb32_bgr24(width, height, buf, frame);
      break;
    case V4L2_PIX_FMT_YUV420:
      ccvt_420p_bgr24(width, height, buf, frame);
      break;
  }
}

/**
 * TODO: for now everything happens in RetrieveFrames.
 */
void V4LDevice::GrabFrames()
{
}

/**
 * TODO: for now everything happens in RetrieveFrames.
 */
void V4LDevice::RetrieveFrames(vector<ImageFrame*> &frames)
{
  fd_set fdset;
  struct timeval tv;
  unsigned int i;
  int maxfd = -1, r = 0, sum_captured = 0;
  vector<int> have_captured(fds.size());

  if(frames.size() != NumCameras())
    throw BALTException(__HERE__, "requested %d images from %d cameras",
        frames.size(), NumCameras());

  for(i = 0; i < fds.size(); i++)
    have_captured[i] = 0;
  while(sum_captured < fds.size())
  {
    maxfd = -1;
    FD_ZERO(&fdset);
    for(i = 0; i < fds.size(); i++)
    {
      if(!have_captured[i])
      {
        FD_SET(fds[i], &fdset);
        maxfd = max(maxfd, fds[i]);
      }
    }
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    r = select(maxfd+1, &fdset, NULL, NULL, &tv);
    if (-1 == r)
    {
      if (EINTR != errno)
        throw BALTException(__HERE__, "select");
    }
    else if (0 == r)
      throw BALTException(__HERE__, "select timeout");
    else
      sum_captured += r;
    for(i = 0; i < fds.size(); i++)
    {
      if(FD_ISSET(fds[i], &fdset))
      {
        read_frame(i, frames);
        have_captured[i] = 1;
      }
    }
  }
  BALTTime time = BALTTimer::getBALTTime();
  for(i = 0; i < fds.size(); i++)
  {
    frames[i]->m_time = time;
    frames[i]->m_camNum = cam_ids[i];
  }
}

void V4LDevice::RetrieveFrame(int camNum, Vision::ImageFrame *frame)
{
  printf("v4l not implemented, use OpenCV video device instead (option -o 0)\n");
}

void V4LDevice::stop_capturing(int i)
{
  enum v4l2_buf_type type;

  switch (io)
  {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == xioctl(fds[i], VIDIOC_STREAMOFF, &type))
        throw BALTException(__HERE__, "VIDIOC_STREAMOFF");
      break;
  }
}

void V4LDevice::stop_capturing()
{
  for(unsigned i = 0; i < fds.size(); i++)
    stop_capturing(i);
}

void V4LDevice::start_capturing(int i)
{
  unsigned int j;
  enum v4l2_buf_type type;

  switch (io)
  {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
      for (j = 0; j < n_buffers[i]; ++j)
      {
        struct v4l2_buffer buf;

    	  CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = j;

        if (-1 == xioctl(fds[i], VIDIOC_QBUF, &buf))
          throw BALTException(__HERE__, "VIDIOC_QBUF buffer %d", j);
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fds[i], VIDIOC_STREAMON, &type))
        throw BALTException(__HERE__, "VIDIOC_STREAMON");

      break;

    case IO_METHOD_USERPTR:
      for (j = 0; j < n_buffers[i]; ++j)
      {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.m.userptr = (unsigned long) buffers[i][j].start;
        buf.length = buffers[i][j].length;

        if (-1 == xioctl(fds[i], VIDIOC_QBUF, &buf))
          throw BALTException(__HERE__, "VIDIOC_QBUF");
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fds[i], VIDIOC_STREAMON, &type))
        throw BALTException(__HERE__, "VIDIOC_STREAMON");

      break;
  }
}

void V4LDevice::start_capturing()
{
  for(unsigned i = 0; i < fds.size(); i++)
    start_capturing(i);
}

void V4LDevice::uninit_device(int i)
{
  unsigned int j;

  switch (io)
  {
    case IO_METHOD_READ:
      free(buffers[i][0].start);
      break;

    case IO_METHOD_MMAP:
      for (j = 0; j < n_buffers[i]; ++j)
        if (-1 == munmap(buffers[i][j].start, buffers[i][j].length))
          throw BALTException(__HERE__, "munmap failed");
      break;

    case IO_METHOD_USERPTR:
      for (j = 0; j < n_buffers[i]; ++j)
        free(buffers[i][j].start);
      break;
  }

  free(buffers[i]);
}

void V4LDevice::uninit_devices()
{
  for(unsigned i = 0; i < fds.size(); i++)
    uninit_device(i);
}

void V4LDevice::init_read(int i, unsigned int buffer_size)
{
  buffers[i] = (struct s_buffer*)calloc(1, sizeof(struct s_buffer));

  if (!buffers[i])
    throw BALTException(__HERE__, "Out of memory");

  buffers[i][0].length = buffer_size;
  buffers[i][0].start = malloc(buffer_size);

  if (!buffers[i][0].start)
    throw BALTException(__HERE__, "Out of memory");
}

void V4LDevice::init_mmap(int i)
{
  struct v4l2_requestbuffers req;

  CLEAR(req);
  req.count = 4;  // request 4 buffers
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fds[i], VIDIOC_REQBUFS, &req))
  {
    if (EINVAL == errno)
      throw BALTException(__HERE__, "%s does not support memory mapping",
          dev_names[i].c_str());
    else
      throw BALTException(__HERE__, "VIDIOC_REQBUFS");
  }

  if (req.count < 2)
    throw BALTException(__HERE__, "Insufficient buffer memory on %s",
        dev_names[i].c_str());

  buffers[i] = (struct s_buffer*)calloc(req.count, sizeof(struct s_buffer));
  if (!buffers[i])
    throw BALTException(__HERE__, "Out of memory");
  n_buffers[i] = req.count;

  for (int j = 0; j < n_buffers[i]; ++j)
  {
    struct v4l2_buffer buf;

    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = j;

    if (-1 == xioctl(fds[i], VIDIOC_QUERYBUF, &buf))
      throw BALTException(__HERE__, "VIDIOC_QUERYBUF");

    buffers[i][j].length = buf.length;
    buffers[i][j].start = mmap(NULL /* start anywhere */ ,
            buf.length,
            PROT_READ | PROT_WRITE /* required */ ,
            MAP_SHARED /* recommended */ ,
            fds[i], buf.m.offset);

    if (MAP_FAILED == buffers[i][j].start)
      throw BALTException(__HERE__, "mmap failed");
  }
}

void V4LDevice::init_userp(int i, unsigned int buffer_size)
{
  struct v4l2_requestbuffers req;

  CLEAR(req);
  req.count = 4;  // request 4 buffers
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl(fds[i], VIDIOC_REQBUFS, &req))
  {
    if (EINVAL == errno)
      throw BALTException(__HERE__, "%s does not support user pointer i/o",
          dev_names[i].c_str());
    else
      throw BALTException(__HERE__, "VIDIOC_REQBUFS");
  }

  buffers[i] = (struct s_buffer*)calloc(4, sizeof(struct s_buffer));
  if (!buffers[i])
    throw BALTException(__HERE__, "Out of memory");
  n_buffers[i] = req.count;

  for (int j = 0; j < n_buffers[i]; ++j)
  {
    buffers[i][j].length = buffer_size;
    buffers[i][j].start = malloc(buffer_size);
    if (!buffers[i][j].start)
      throw BALTException(__HERE__, "Out of memory");
  }
}

void V4LDevice::init_device(int i)
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  struct v4l2_streamparm streamparm;

  if(-1 == xioctl(fds[i], VIDIOC_QUERYCAP, &cap))
  {
    if(EINVAL == errno)
      throw BALTException(__HERE__, "%s is no V4L2 device", dev_names[i].c_str());
    else
      throw BALTException(__HERE__, "%s: VIDIOC_QUERYCAP: %s",
         dev_names[i].c_str(), strerror(errno));
  }

  if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    throw BALTException(__HERE__, "%s is no video capture device",
        dev_names[i].c_str());

  switch (io)
  {
    case IO_METHOD_READ:
      if(!(cap.capabilities & V4L2_CAP_READWRITE))
        throw BALTException(__HERE__, "%s does not support read i/o",
            dev_names[i].c_str());
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if(!(cap.capabilities & V4L2_CAP_STREAMING))
        throw BALTException(__HERE__, "%s does not support streaming i/o",
            dev_names[i].c_str());
      break;
  }

#ifdef B21_SERVER
  // Select video input, video standard and tune here.
  int index;
  index = 1;

  if (-1 == ioctl (fds[i], VIDIOC_S_INPUT, &index)) {
    perror ("VIDIOC_S_INPUT");
    exit (EXIT_FAILURE);
  }

  struct v4l2_input input;
  v4l2_std_id std_id = V4L2_STD_NTSC_M;

  memset (&input, 0, sizeof (input));

  if (-1 == ioctl (fds[i], VIDIOC_G_INPUT, &input.index)) {
    perror ("VIDIOC_G_INPUT");
    exit (EXIT_FAILURE);
  }

  if (-1 == ioctl (fds[i], VIDIOC_ENUMINPUT, &input)) {
    perror ("VIDIOC_ENUM_INPUT");
    exit (EXIT_FAILURE);
  }

  if (0 == (input.std & std_id)) {
    fprintf (stderr, "Oops. standard is not supported.\n");
    exit (EXIT_FAILURE);
  }

  // Note this is also supposed to work when only B
  // or G/PAL is supported.

  if (-1 == ioctl (fds[i], VIDIOC_S_STD, &std_id)) {
    perror ("VIDIOC_S_STD");
    exit (EXIT_FAILURE);
  }
#endif

  /*cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (-1 == xioctl(fds[i], VIDIOC_CROPCAP, &cropcap))
    throw BALTException(__HERE__, "failed VIDIOC_CROPCAP");

  crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  crop.c = cropcap.defrect;	// reset to default

  if (-1 == xioctl(fds[i], VIDIOC_S_CROP, &crop))
  {
    switch (errno)
    {
      case EINVAL:
        throw BALTException(__HERE__, "Cropping not supported");
        break;
      default:
        throw BALTException(__HERE__, "some error");
        break;
    }
  }*/

  /*CLEAR(streamparm);
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(fds[i], VIDIOC_G_PARM, &streamparm))
    throw BALTException(__HERE__, "VIDIOC_G_PARM");
  printf("rate: %d/%d\n", streamparm.parm.output.timeperframe.numerator,
      streamparm.parm.output.timeperframe.denominator);*/

  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(fds[i], VIDIOC_G_FMT, &fmt))
    throw BALTException(__HERE__, "VIDIOC_G_FMT");
  printf("video: format '%s'  width %u height %u\n",
      pix_fmt_str(fmt.fmt.pix.pixelformat),
      fmt.fmt.pix.width, fmt.fmt.pix.height);

  for(int j = 0; j < 5; j++)
  {
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pref_formats[j];
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    printf("trying pixel format %dx%d '%s' ...\n",
        fmt.fmt.pix.width, fmt.fmt.pix.height,
        pix_fmt_str(fmt.fmt.pix.pixelformat));
    if (-1 == xioctl(fds[i], VIDIOC_S_FMT, &fmt))
      printf("nope\n");
      //throw BALTException(__HERE__, "VIDIOC_S_FMT");
    else if(fmt.fmt.pix.pixelformat == pref_formats[j])
    {
      printf("available - chosing\n");
      break;
    }
  }

  if (-1 == xioctl(fds[i], VIDIOC_S_FMT, &fmt))
    throw BALTException(__HERE__, "VIDIOC_S_FMT");

  /* Note VIDIOC_S_FMT may change width and height. */
  width = fmt.fmt.pix.width;
  height = fmt.fmt.pix.height;
  pixel_format = fmt.fmt.pix.pixelformat;

  switch (io)
  {
    case IO_METHOD_READ:
      init_read(i, fmt.fmt.pix.sizeimage);
      break;
    case IO_METHOD_MMAP:
      init_mmap(i);
      break;
    case IO_METHOD_USERPTR:
      init_userp(i, fmt.fmt.pix.sizeimage);
      break;
  }
}

void V4LDevice::init_devices()
{
  buffers.resize(fds.size());
  n_buffers.resize(fds.size());
  for(unsigned i = 0; i < fds.size(); i++)
  {
    buffers[i] = 0;
    n_buffers[i] = 0;
    init_device(i);
  }
}

void V4LDevice::close_device(int i)
{
  if(close(fds[i]) == -1)
    throw BALTException(__HERE__, "failed closing device %s", dev_names[i].c_str());
}

void V4LDevice::close_devices()
{
  for(unsigned i = 0; i < fds.size(); i++)
    close_device(fds[i]);
}

int V4LDevice::open_device(const string &dev_name)
{
  struct stat st;

  if(stat(dev_name.c_str(), &st) == -1)
    throw BALTException(__HERE__, "Cannot identify '%s': %d, %s",
      dev_name.c_str(), errno, strerror(errno));
  if(!S_ISCHR(st.st_mode))
    throw BALTException(__HERE__, "%s is no device", dev_name.c_str());
  int fd = open(dev_name.c_str(), O_RDWR /* required */  | O_NONBLOCK, 0);
  if(fd == -1)
    throw BALTException(__HERE__, "Cannot open '%s': %d, %s",
      dev_name.c_str(), errno, strerror(errno));
  return fd;
}

void V4LDevice::open_devices()
{
  fds.resize(dev_names.size());
  for(unsigned i = 0; i < dev_names.size(); i++)
    fds[i] = open_device(dev_names[i]);
}

V4LDevice::V4LDevice(const vector<int> &ids, const vector<string> &devs,
    int req_width, int req_height)
{
  if(ids.size() == 0)
    throw BALTException(__HERE__, "must specify at least one camera");
  if(ids.size() != devs.size())
    throw BALTException(__HERE__, "number of cameras and devices does not match");

  // preferred pixel formats
	pref_formats[0] = V4L2_PIX_FMT_BGR24;
  pref_formats[1] = V4L2_PIX_FMT_RGB24;
  pref_formats[2] = V4L2_PIX_FMT_BGR32;
  pref_formats[3] = V4L2_PIX_FMT_RGB32;
  pref_formats[4] = V4L2_PIX_FMT_YUV420;

  // preferred I/O method
  io = IO_METHOD_MMAP;
  // io = IO_METHOD_READ;
  // io = IO_METHOD_USERPTR;

  // note that these values might be changed depending on actual device
  // capabilities
  width = req_width;
  height = req_height;

  cam_ids = ids;
  dev_names = devs;
  open_devices();
  init_devices();
  start_capturing();
}

V4LDevice::~V4LDevice()
{
  stop_capturing();
  uninit_devices();
  close_devices();
}

int V4LDevice::NumCameras()
{
  return (int)cam_ids.size();
}

void V4LDevice::GetCameraIds(std::vector<int> &ids)
{
  ids = cam_ids;
}

void V4LDevice::GetImageSize(int &w, int &h)
{
  w = width;
  h = height;
}

int V4LDevice::GetFramerate()
{
  return 40;  // HACK: assume 40 ms = 25 Hz
}

