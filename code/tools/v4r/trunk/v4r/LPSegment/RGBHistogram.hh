/**
 * $Id$
 */


#ifndef P_RGB_HISTOGRAM_HH
#define P_RGB_HISTOGRAM_HH

#include <opencv2/core/core.hpp>
#include "v4r/CEdge/PNamespace.hh"
#include "v4r/PMath/PMath.hh"

namespace P 
{
 
/**
 * A three channel color histogram.
 */
class RGBHistogram
{
public:
  unsigned short bins[16][16][16];
  unsigned sum;

  RGBHistogram();
  ~RGBHistogram(){};
  void Normalise(unsigned v);

  inline void Add(unsigned char c1, unsigned char c2, unsigned char c3);
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
inline void RGBHistogram::Add(unsigned char c1, unsigned char c2, unsigned char c3)
{
  //we only use the highest 4 bits, we have 16 bins
  c1 >>= 4;
  c2 >>= 4;
  c3 >>= 4;
  bins[c1][c2][c3]++;
  sum++;
}

inline unsigned RGBHistogram::Num()
{
  return sum;
}

inline void RGBHistogram::Clear()
{
  sum=0;

  for(unsigned i = 0; i < 16; i++)
    for(unsigned j = 0; j < 16; j++)
      for(unsigned k = 0; k < 16; k++)
        bins[i][j][k] = 0;
}



}

#endif

