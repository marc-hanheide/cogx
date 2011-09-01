/**
 * $Id$
 */


#ifndef P_UV_HISTOGRAM_HH
#define P_UV_HISTOGRAM_HH

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "v4r/CEdge/PNamespace.hh"
#include "v4r/PMath/PMath.hh"

namespace P 
{
 
/**
 * A three channel color histogram.
 */
class UVHistogram
{
public:
  unsigned short bins[64][64];
  unsigned sum;

  UVHistogram();
  ~UVHistogram(){};
  void Normalise(unsigned v);

  inline void Add(unsigned char u, unsigned char v);
  inline unsigned Num();
  inline void Clear();
};

/****************************** INLINE METHODES ********************************/
/**
 * @brief Add a value consisting of 3 channels to the histogram.
 * @brief We only use the UV channels of an YUV image
 * @param c1 r channel
 * @param c2 g channel
 * @param c3 b channel
 */
inline void UVHistogram::Add(unsigned char u, unsigned char v)
{
  //we only use the highest 4 bits, we have 16 bins
  u >>= 2;
  v >>= 2;
  bins[u][v]++;
  sum++;
}

inline unsigned UVHistogram::Num()
{
  return sum;
}

inline void UVHistogram::Clear()
{
  sum=0;

  for(unsigned i = 0; i < 64; i++)
    for(unsigned j = 0; j < 64; j++)
      bins[i][j] = 0;
}



}

#endif

