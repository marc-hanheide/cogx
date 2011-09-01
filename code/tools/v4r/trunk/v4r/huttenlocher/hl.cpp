
#include <stdio.h>
#include <time.h>
#include <cstdlib>
#include "hl.h"

namespace V4R {
#define THRESHOLD(size, c) (c/size)

// dissimilarity measure between pixels
float Huttenlocher::diffPix(IplImage *pImg, int x1, int y1, int x2, int y2) {
    float *p = (float*) pImg->imageData;
    float *a = p + ((y1 * pImg->width + x1) * pImg->nChannels);
    float *b = p + ((y2 * pImg->width + x2) * pImg->nChannels);
    float sum = 0;
    for (int i = 0; i < pImg->nChannels; i++) {

        sum += (a[i]-b[i]) * (a[i]-b[i]);
    }
    return sqrt(sum);
}


Huttenlocher::~Huttenlocher() {
    if (mpUniverse != NULL) delete mpUniverse;
    if (mpSmooth != NULL) cvReleaseImage(&mpSmooth);
    if (mpEdges != NULL) delete [] mpEdges;
    if (mpGaussKernel != NULL) cvReleaseMat(&mpGaussKernel);;
}
Huttenlocher::Huttenlocher()
        : mNrOfSegments(0)
        , mSigma(0)
        , mThreshold(0)
        , mMinSize(0)
        , mRandColor(true)
        , mWidth(0)
        , mHeight(0)
        , mChannels(0)
        , mpSmooth(NULL)
        , mpGaussKernel(NULL)
        , mpUniverse(NULL)
        , mpEdges(NULL) {
}

void Huttenlocher::init(float sigma, float threshold, int min_size, bool rand_color) {
    mSigma = sigma;
    mThreshold = threshold;
    mMinSize = min_size;
    mRandColor = rand_color;
    if (mpGaussKernel != NULL) cvReleaseMat(&mpGaussKernel);
    mpGaussKernel = compute_kernel(mSigma);
}

CvMat *Huttenlocher::compute_kernel(float sigma) {
    int radii = (int) ceil (sigma * 2.0);
    int len = 2 * radii + 1;
    CvMat *pKernel = cvCreateMat(len, len, CV_32F);
    float *pData = pKernel->data.fl;
    float sum = 0;
    for (int x = -radii; x <= radii; x++) {
        for (int y = -radii; y <= radii; y++) {
            float distance = sqrt( x*x+y*y );
            float pdf = exp(-0.5 * ( distance / sigma ) * ( distance / sigma ));
            sum += pdf;
            *pData++ = pdf;
        }
    }
    for (int i = 0; i < len*len; i++) {
        pKernel->data.fl[i] = pKernel->data.fl[i] / sum;
    }
    return pKernel;
}

void Huttenlocher::getSmooth(IplImage *pImgDes) {
    char *pDes = pImgDes->imageData;
    float *pSrc = (float *) mpSmooth->imageData;
    for ( int y = 0; y < mHeight; y++ ) {
        for ( int x = 0; x < mWidth; x++ ) {
            for ( int c = 0; c < mpSmooth->nChannels; c++) {
                *pDes++ = (char) *pSrc++;
            }
        }

    }
}

void Huttenlocher::segment_image( IplImage *pImgSrc, IplImage *pImgDes) {
    bool bInit = false;
    if ((pImgSrc->width != mWidth) || (pImgSrc->height != mHeight) ||  (pImgSrc->nChannels != mChannels)) {
        bInit = true;
    }

    if (bInit) {
        mWidth = pImgSrc->width;
        mHeight = pImgSrc->height;
        mChannels = pImgSrc->nChannels;
        mpSmooth = cvCreateImage(cvSize(mWidth,mHeight), IPL_DEPTH_32F, pImgSrc->nChannels);
        mpEdges = new Huttenlocher::Edge[mWidth*mHeight*4];
    }

    cvFilter2D(pImgSrc, mpSmooth, mpGaussKernel);

    // build graph
    clock_t t0 = clock();
    int num = 0;
    for ( int y = 0; y < mHeight; y++ ) {
        for ( int x = 0; x < mWidth; x++ ) {
            if ( x < mWidth-1 ) {
                mpEdges[num].a = y * mWidth + x;
                mpEdges[num].b = y * mWidth + ( x+1 );
                mpEdges[num].w = diffPix ( mpSmooth, x, y, x+1, y );
                num++;
            }

            if ( y < mHeight-1 ) {
                mpEdges[num].a = y * mWidth + x;
                mpEdges[num].b = ( y+1 ) * mWidth + x;
                mpEdges[num].w = diffPix ( mpSmooth, x, y, x, y+1 );
                num++;
            }

            if ( ( x < mWidth-1 ) && ( y < mHeight-1 ) ) {
                mpEdges[num].a = y * mWidth + x;
                mpEdges[num].b = ( y+1 ) * mWidth + ( x+1 );
                mpEdges[num].w = diffPix ( mpSmooth, x, y, x+1, y+1 );
                num++;
            }

            if ( ( x < mWidth-1 ) && ( y > 0 ) ) {
                mpEdges[num].a = y * mWidth + x;
                mpEdges[num].b = ( y-1 ) * mWidth + ( x+1 );
                mpEdges[num].w = diffPix ( mpSmooth, x, y, x+1, y-1 );
                num++;
            }
        }
    }
    clock_t dt0 = clock() - t0;
    printf ( "%5i cycles or %5i ms for build graph\n", ( int ) dt0, ( int ) ( ( dt0 * 1000 ) / CLOCKS_PER_SEC ) );

    // segment
    clock_t t1 = clock();
    mpUniverse = segment_graph ( mWidth*mHeight, num, mpEdges, mThreshold );
    clock_t dt1 = clock() - t1;
    printf ( "%5i cycles or %5i ms for segment_graph\n", ( int ) dt1, ( int ) ( ( dt1 * 1000 ) / CLOCKS_PER_SEC ) );

    // post process small components
    clock_t t2 = clock();
    for ( int i = 0; i < num; i++ ) {
        int a = mpUniverse->find ( mpEdges[i].a );
        int b = mpUniverse->find ( mpEdges[i].b );
        if ( ( a != b ) && ( ( mpUniverse->size ( a ) < mMinSize ) || ( mpUniverse->size ( b ) < mMinSize ) ) )
            mpUniverse->join ( a, b );
    }
    mNrOfSegments = mpUniverse->num_sets();
    if ( pImgDes == NULL ) {
        return;
    }

    clock_t dt2 = clock() - t2;
    printf ( "%5i cycles or %5i ms for post process small components\n", ( int ) dt2, ( int ) ( ( dt2 * 1000 ) / CLOCKS_PER_SEC ) );

    // pick random colors for each component
    clock_t t3 = clock();
    compute_segments(pImgDes);
    clock_t dt3 = clock() - t3;
    printf ( "%5i cycles or %5i ms for pick random colors for each component\n", ( int ) dt3, ( int ) ( ( dt3 * 1000 ) / CLOCKS_PER_SEC ) );
}


Huttenlocher::Universe *Huttenlocher::segment_graph ( int num_vertices, int num_edges, Huttenlocher::Edge *edges, float c ) {
    // sort edges by weight
    std::sort ( edges, edges + num_edges );

    // make a disjoint-set forest
    Universe *u = new Universe ( num_vertices );

    // init thresholds
    float *threshold = new float[num_vertices];
    for ( int i = 0; i < num_vertices; i++ ) {
        threshold[i] = THRESHOLD ( 1,c );
    }

    // for each edge, in non-decreasing weight order...
    for ( int i = 0; i < num_edges; i++ ) {
        Edge *pedge = &edges[i];

        // components conected by this edge
        int a = u->find ( pedge->a );
        int b = u->find ( pedge->b );
        if ( a != b ) {
            if ( ( pedge->w <= threshold[a] ) && ( pedge->w <= threshold[b] ) ) {
                u->join ( a, b );
                a = u->find ( a );
                threshold[a] = pedge->w + THRESHOLD ( u->size ( a ), c );
            }
        }
    }

    // free up
    delete threshold;
    return u;
}


void Huttenlocher::compute_segments(IplImage *pImgDes) {
    mSegments.clear();
    std::map<int,ColorSegment>::iterator iter;
    char *pDes = pImgDes->imageData;
    int id = 0;
    for (int y = 0; y < mHeight; y++) {
        for (int x = 0; x < mWidth; x++) {
            int idx = y * mWidth + x;
            int comp = mpUniverse->find(idx);
            iter = mSegments.find(comp);
            if (iter == mSegments.end()) {
                mSegments[comp] = ColorSegment(x,y);
                iter = mSegments.find(comp);
                iter->second.id = id++;
                if(mRandColor){
                  iter->second.setColor(rand()%255, rand()%255, rand()%255);
                } else {
                  iter->second.setColor(iter->second.id, iter->second.id, iter->second.id);
                }
            }
            iter->second.addPix(x,y);
            if ( pImgDes != NULL) {
                for (int c = 0; c < pImgDes->nChannels; c++) {
                    *pDes++ = (char) iter->second.color[c];
                }
            }
        }
    }
    for ( iter=mSegments.begin() ; iter != mSegments.end(); iter++ ) {
        iter->second.compute_region();
    }
}
std::vector<ColorRegion> Huttenlocher::getSegments() {
    std::vector<ColorRegion> segments(mSegments.size());
    std::map<int,ColorSegment>::iterator iter;
    for ( iter=mSegments.begin() ; iter != mSegments.end(); iter++ ) {
        segments[iter->second.id] = iter->second;
    }
    return segments;
}

Huttenlocher::Universe::Universe(int elements) {
    elts = new uni_elt[elements];
    num = elements;
    for (int i = 0; i < elements; i++) {
        elts[i].rank = 0;
        elts[i].size = 1;
        elts[i].p = i;
    }
}

Huttenlocher::Universe::~Universe() {
    delete [] elts;
}

int Huttenlocher::Universe::find(int x) {
    int y = x;
    while (y != elts[y].p) {
        y = elts[y].p;
    }
    elts[x].p = y;
    return y;
}
void Huttenlocher::Universe::join(int x, int y) {
    if (elts[x].rank > elts[y].rank) {
        elts[y].p = x;
        elts[x].size += elts[y].size;
    } else {
        elts[x].p = y;
        elts[y].size += elts[x].size;
        if (elts[x].rank == elts[y].rank)
            elts[y].rank++;
    }
    num--;
}
int Huttenlocher::Universe::size(int x) const {
    return elts[x].size;
}
int Huttenlocher::Universe::num_sets() const {
    return num;
}
} // namespace AK
// kate: indent-mode cstyle; space-indent on; indent-width 0; 
