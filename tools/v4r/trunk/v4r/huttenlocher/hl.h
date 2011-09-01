/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/
/**
 * @file hl.h
 * @author Markus Bader
 * @brief  Class based Felzenszwalb and Huttenlocher segmentation algorithm. \n
 * Based on the orignal code of Pedro Felzenszwalb
 */
#ifndef HUTTENLOCHER_H
#define HUTTENLOCHER_H
#include <opencv/cv.h>
#include <vector>


#ifndef IMAGE_REGION
#define IMAGE_REGION
namespace V4R {
/**
 * @class Image Region
 * @author Markus Bader
 */
struct ImgRegion {
    int id;                 /// id of the region
    double area;            /// occupied area in pixels
    double x1,y1,x2,y2;     /// bounding box (x1,y1) - (x2,y2)
    double cen_x,cen_y;     /// centroid
    double color[4];        /// color
    /**
    * Draws the image region Creates a Camera instance if none exists
    * @param img 
    */
    void draw(IplImage *img){
      CvScalar *pColor = (CvScalar *) color;
      cvLine ( img, cvPoint(x1,y1), cvPoint(x1,y2), *pColor);
      cvLine ( img, cvPoint(x1,y2), cvPoint(x2,y2), *pColor);
      cvLine ( img, cvPoint(x2,y2), cvPoint(x2,y1), *pColor);
      cvLine ( img, cvPoint(x2,y1), cvPoint(x1,y1), *pColor);
      cvLine ( img, cvPoint(cen_x+1,cen_y+1), cvPoint(cen_x-1,cen_y-1), *pColor);
      cvLine ( img, cvPoint(cen_x-1,cen_y+1), cvPoint(cen_x+1,cen_y-1), *pColor);
    }
};

#endif //IMAGE_REGION


#ifndef COLOR_REGION
#define COLOR_REGION
/**
 * @class Color Region
 * @author Markus Bader
 */
struct ColorRegion : public ImgRegion {
    double average[4];       /// average color
    double histogram[16];    /// color histogram
    template <typename T>
    /**
    * sets the color
    * @param v0 
    * @param v1 
    * @param v2 
    * @param v3 
    */
    void setColor(const T &v0 = 0, const T &v1 = 0, const T &v2 = 0, const T &v3 = 0){
      color[0] = v0, color[1] = v1, color[2] = v2, color[3] = v3;
    }
};
#endif //COLOR_REGION

#ifndef COLOR_SEGMENT
#define COLOR_SEGMENT
/**
 * @class Color Segment
 * @author Markus Bader
 */
class ColorSegment : public ColorRegion {
public:
    /**
    * constructor
    */
    ColorSegment() {
        clear();
    }
    /**
    * constructor
    * @param x 
    * @param y 
    */
    ColorSegment(int x, int y) {
        clear();
        x1 = x, x2 = x, y1 = y, y2 = y;
    }
    /**
    * clears all entries to zero
    */
    void clear() {
        memset(this, 0, sizeof(ColorSegment));
    }
    /**
    * adds a pixel
    * @param x 
    * @param y 
    */
    void addPix(int x, int y){
      sumX+=x, sumY+= y, pixCount++;
      if(x < x1) x1 = x;
      if(x > x2) x2 = x;
      if(y < y1) y1 = y;
      if(y > y2) y2 = y;
    }
    /**
    * computes the center and area
    */
    void compute_region(){
      cen_x = sumX / pixCount;
      cen_y = sumY / pixCount;
      area = pixCount;
    }
private:
    long sumX; 
    long sumY;
    long pixCount;
};
#endif //COLOR_SEGMENT

/**
 * @class Segmentation\n
 * Based on the code of 2006 Pedro Felzenszwalb
 * @author Markus Bader
 */
class Huttenlocher {
public:
    /**
    * @struct Segment data\n
    * Based on the code of 2006 Pedro Felzenszwalb
    * @author Markus Bader
    */
    typedef struct {
        int rank;
        int p;
        int size;
    } uni_elt;

    /**
    * @class Class to manage the graph\n
    * Based on the code of 2006 Pedro Felzenszwalb
    * @author Markus Bader
    */
    class Universe {
    public:
        Universe(int elements);
        ~Universe();
        int find(int x);
        void join(int x, int y);
        int size(int x) const;
        int num_sets() const;
    private:
        uni_elt *elts;
        int num;
    };


    /**
    * @struct Edge between segments\n
    * Based on the code of 2006 Pedro Felzenszwalb
    * @author Markus Bader
    */
    typedef struct {
        float w;  ///weight
        int a, b; ///leafs
    } Edge;

    /**
    * Construtor
    */
    Huttenlocher();
    /**
    * Destrutor
    */
    ~Huttenlocher();
    /**
    * Init
    * @param sigma
    * @param threshold
    * @param min_size
    * @param rand_color on true ist uses random colors for the destiantion image
    * @post Huttenlocher::segment_image
    */
    void init(float sigma, float threshold, int min_size, bool rand_color = true);
    /**
    * starts the segmenation and initializes the data structurs on the first run \n
    * -> secound run will be a little bit faster
    * @param pImgSrc source image it can be a 0-4 channel IPL_DEPTH_8U image
    * @param pImgDes destiantion image it can be a 0-4 channel IPL_DEPTH_8U image or NULL
    * @pre Huttenlocher::init
    * @post Huttenlocher::getSegments
    */
    void segment_image( IplImage *pImgSrc, IplImage *pImgDes = NULL);
    /**
    * Returns the smoothed image
    * @param pImgDes same format as the pImgSrc on Huttenlocher::segment_image
    * @post Huttenlocher::segment_image
    */
    void getSmooth(IplImage *pImgDes);
    /**
    * Returns the detected segmetns
    * @return color regions
    */
    std::vector<ColorRegion> getSegments();

private:
    int mNrOfSegments;
    float mSigma;
    float mThreshold;
    int mMinSize;
    bool mRandColor;
    int mWidth;
    int mHeight;
    int mChannels;
    IplImage *mpSmooth;
    CvMat *mpGaussKernel;
    Universe *mpUniverse;
    Edge *mpEdges;
    std::map<int, ColorSegment> mSegments;
    CvMat *compute_kernel(float sigma);
    Universe *segment_graph ( int num_vertices, int num_edges, Edge *edges, float c );
    void compute_segments( IplImage *pImgDes);
    float diffPix(IplImage *pImg, int x1, int y1, int x2, int y2);
};

inline bool operator < ( const Huttenlocher::Edge &a, const Huttenlocher::Edge &b ) {
    return a.w < b.w;
}
}
#endif
// kate: indent-mode cstyle; space-indent on; indent-width 0; 
