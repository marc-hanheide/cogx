/*
 * specific_point_types.h
 *
 *  Created on: Aug 28, 2011
 *      Author: aitor
 */

#ifndef SPECIFIC_POINT_TYPES_H_
#define SPECIFIC_POINT_TYPES_H_

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<640>,
    (float[640], histogram, histogram640)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<480>,
    (float[480], histogram, histogram480)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<416>,
    (float[416], histogram, histogram416)
)

inline int
getHistogramLength (pcl::FPFHSignature33 & hist)
{
  return 33;
}

inline int
getHistogramLength (pcl::VFHSignature308 & hist)
{
  return 308;
}

inline int
getHistogramLength (pcl::Histogram<640> & hist)
{
  return 640;
}

inline int
getHistogramLength (pcl::Histogram<480> & hist)
{
  return 480;
}

inline int
getHistogramLength (pcl::Histogram<416> & hist)
{
  return 416;
}

inline int
getHistogramLength (pcl::Histogram<153> & hist)
{
  return 153;
}

#endif /* SPECIFIC_POINT_TYPES_H_ */
