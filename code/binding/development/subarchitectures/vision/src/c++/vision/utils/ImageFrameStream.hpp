#ifndef IMAGE_FRAME_STREAM_H_
#define IMAGE_FRAME_STREAM_H_

#include "vision/utils/XDRStream.hpp"
#include "vision/idl/Vision.hh"

XDROutputStream& operator<<(XDROutputStream &stream,
    const Vision::ImageFrame &img);

XDRInputStream& operator>>(XDRInputStream &stream, Vision::ImageFrame &img);

#endif

