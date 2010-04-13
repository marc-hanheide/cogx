#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <cuda.h>
#include <cutil.h>

#include "align.h"

#include "census.h"



unsigned int g_Width, g_Height;
unsigned int g_disp_min, g_disp_max, g_disp_step;
unsigned int g_blockSize;
bool g_sparse;
int g_streamNr;

// CUDA arrays
cudaArray *cuImageLeft, *cuImageRight;

// CUDA streams
cudaStream_t streamImageLeft[2], streamImageRight[2], streamDM[2];

// Textures
texture<unsigned char, 2, cudaReadModeElementType> texImageLeft;
texture<unsigned char, 2, cudaReadModeElementType> texImageRight;
texture<unsigned int, 1, cudaReadModeElementType> texDSI1d;
texture<unsigned int, 1, cudaReadModeElementType> texDSITemp1d;

// Host-side page-locked memory
unsigned char *h_left[2], *h_right[2], *h_dm[2];

// device data pointer
unsigned int *d_DSI;
unsigned int *d_DSI_Temp;
unsigned int *d_censusLeft, *d_censusRight;
int *d_Costs, *d_Costs_LR, *d_Costs_RL;
float *d_DMI, *d_DMI_LR, *d_DMI_RL;
unsigned int *d_Confidence, *d_Confidence_LR, *d_Confidence_RL;




float g_CensusTiming[6];


#define PRINT_BANDWIDTH 0
#define USE_FAST_AGGREGATION 1

// 24 bit multiplication takes only 4 clock cycles compared to 16 clock
// cycles for the normal multiplication
#define USE_24BIT_MULTIPLICATION 1

#if USE_24BIT_MULTIPLICATION
	#define IMUL(a, b) __mul24(a, b)
#else
	#define IMUL(a, b) (a * b)
#endif

#define DATA_SIZE (g_Width*g_Height*sizeof(unsigned char))

#define CENSUS_RADIUS 8
#define CENSUS_TILE_W 64
#define CENSUS_TILE_H 64
#define CENSUS_SMEM_W (2 * CENSUS_RADIUS + CENSUS_TILE_W)
#define CENSUS_SMEM_H (2 * CENSUS_RADIUS + CENSUS_TILE_H)

#define CENSUS_SPARSE_RADIUS 8
#define CENSUS_SPARSE_TILE_W 64
#define CENSUS_SPARSE_TILE_H 32
#define CENSUS_SPARSE_SMEM_W (2 * CENSUS_SPARSE_RADIUS + CENSUS_SPARSE_TILE_W)
#define CENSUS_SPARSE_SMEM_H (2 * CENSUS_SPARSE_RADIUS + CENSUS_SPARSE_TILE_H)
#define CENSUS_SPARSE_THREAD_X (CENSUS_SPARSE_TILE_W / 16)
#define CENSUS_SPARSE_THREAD_Y (CENSUS_SPARSE_TILE_H / 16)

#define START_TIMER \
		unsigned int hTimer; \
		CUT_SAFE_CALL(cutCreateTimer(&hTimer)); \
		CUDA_SAFE_CALL( cudaThreadSynchronize() ); \
		CUT_SAFE_CALL( cutResetTimer(hTimer) ); \
		CUT_SAFE_CALL( cutStartTimer(hTimer) );

#define STOP_TIMER(gpuTime) \
		CUDA_SAFE_CALL( cudaThreadSynchronize() ); \
		CUT_SAFE_CALL(cutStopTimer(hTimer)); \
		gpuTime = cutGetTimerValue(hTimer);

#define DISPARITY_SCALING 4


void setCensusTiming(CensusStep s, float timeMs) {
	g_CensusTiming[s] = timeMs;
}

extern "C" float getCensusTiming(CensusStep s) {
	return g_CensusTiming[s];
}

extern "C" unsigned int getCensusFLOP(CensusStep s) {
	int disparities = (g_disp_max + 1 - g_disp_min) / g_disp_step;

	switch (s) {
		case eAggregateCosts:
			return disparities*60*16*16*iDivUp(g_Width, 16)*iDivUp(g_Height, 16);

		case eCalcDSI:
			return (5*(128+disparities)*2*g_Height*iDivUp(g_Width, 128)+(17+43*disparities)*g_Width*g_Height);

		case eCensusTransform:
			return 2*3000*16*16*iDivUp(g_Width, 64)*iDivUp(g_Height, 64);

		case eCompareDisps:
			return 16*16*16*iDivUp(g_Width, 16)*iDivUp(g_Height, 16);

		case eRefineSubPixel:
			return ((disparities*7+35)*g_Width*g_Height) + ((disparities*7+43)*g_Width*g_Height);

		case eRoundAndScaleDisparities:
			return 12*16*16*iDivUp(g_Width, 16)*iDivUp(g_Height, 16);

		default:
			return 0;
	}
}

extern "C" unsigned int getCensusMemory(CensusStep s) {
	int disparities = (g_disp_max + 1 - g_disp_min) / g_disp_step;

	switch (s) {
		case eAggregateCosts:
			return disparities*(g_Width*g_Height*sizeof(int) + iDivUp(g_Width, 16)*iDivUp(g_Height, 16)*21*21*sizeof(int));

		case eCalcDSI:
			return (iDivUp(g_Width, 128)*g_Height*2*(128 + disparities)*2*sizeof(unsigned int)+2*(g_Width-g_disp_max)*g_Height*sizeof(unsigned int));

		case eCensusTransform:
			return 2*(iDivUp(g_Width, CENSUS_SPARSE_TILE_W)*iDivUp(g_Height, CENSUS_SPARSE_TILE_H)*CENSUS_SPARSE_SMEM_W*CENSUS_SMEM_H*sizeof(unsigned int)+g_Width*g_Height*8);

		case eCompareDisps:
			return 2*g_Width*g_Height*sizeof(float);

		case eRefineSubPixel:
			return 2*(disparities+2)*g_Width*g_Height*sizeof(int);

		case eRoundAndScaleDisparities:
			return 2*g_Width*g_Height*sizeof(float);

		default:
			return 0;
	}
}


inline
__device__ int f2i(float f) {

	return f < 0 ? f - .5 : f + .5;

}

template <class T>
inline
__device__ T getArrayValue(const T *arr, int x, int y, int width) {

	return arr[IMUL(y, width) + x];

}

template <class T>
inline
__device__ T getArrayValue(const T *arr, int x, int y, int z, int width, int height) {

	return arr[z * IMUL(width, height) + IMUL(y, width) + x];

}

template <class T>
inline
__device__ void setArrayValue(T *arr, T value, int x, int y, int width) {

	arr[IMUL(y, width) + x] = value;

}

template <class T>
inline
__device__ void setArrayValue(T *arr, T value, int x, int y, int z, int width, int height) {

	arr[z * IMUL(width, height) + IMUL(y, width) + x] = value;

}

template <bool sparse>
inline
__device__ unsigned char censusRow(unsigned char ref, unsigned char *row) {
	unsigned char c = 0;

	if (sparse) {
		if (ref > row[ 0]) c |= (1 << 7);
		if (ref > row[ 2]) c |= (1 << 6);
		if (ref > row[ 4]) c |= (1 << 5);
		if (ref > row[ 6]) c |= (1 << 4);
		if (ref > row[ 8]) c |= (1 << 3);
		if (ref > row[10]) c |= (1 << 2);
		if (ref > row[12]) c |= (1 << 1);
		if (ref > row[14]) c |= (1 << 0);
	}

	return c;
}
inline
__device__ unsigned char censusRowSparse(unsigned char ref, unsigned char *row) {
	unsigned char c = 0;

	if (ref > row[ 0]) c |= (1 << 7);
	if (ref > row[ 2]) c |= (1 << 6);
	if (ref > row[ 4]) c |= (1 << 5);
	if (ref > row[ 6]) c |= (1 << 4);
	if (ref > row[ 8]) c |= (1 << 3);
	if (ref > row[10]) c |= (1 << 2);
	if (ref > row[12]) c |= (1 << 1);
	if (ref > row[14]) c |= (1 << 0);

	return c;
}
template <bool left>
__global__ void censusTransformSparse(unsigned int *d_census, int iWidth, int iHeight) {
	const int blockX = IMUL(blockIdx.x, CENSUS_SPARSE_TILE_W);
	const int blockY = IMUL(blockIdx.y, CENSUS_SPARSE_TILE_H);
	int x, y;

	__shared__ unsigned char smTileData[CENSUS_SPARSE_SMEM_H][CENSUS_SPARSE_SMEM_W];

	for (y = threadIdx.y; y < CENSUS_SPARSE_SMEM_H; y+=blockDim.y) { // 130 FLOP
		for (x = threadIdx.x; x < CENSUS_SPARSE_SMEM_W; x+=blockDim.x) {
			if (left)
				smTileData[y][x] = tex2D(texImageLeft, blockX+x-CENSUS_SPARSE_RADIUS, blockY+y-CENSUS_SPARSE_RADIUS);
			else
				smTileData[y][x] = tex2D(texImageRight, blockX+x-CENSUS_SPARSE_RADIUS, blockY+y-CENSUS_SPARSE_RADIUS);
		}
	}


	// assure that shared memory is completely loaded
	__syncthreads();


	// start calculation
	int2 c;
	unsigned char p1;
	int xref, yref;
	int idx = 0;

	//for (yref = threadIdx.y * 4; yref < threadIdx.y * 4 + 4 && yref < CENSUS_SPARSE_TILE_H && blockY+yref < iHeight; yref++) { // 2868 FLOP
	for (yref = threadIdx.y * CENSUS_SPARSE_THREAD_Y; yref < threadIdx.y * CENSUS_SPARSE_THREAD_Y + CENSUS_SPARSE_THREAD_Y && yref < CENSUS_SPARSE_TILE_H && blockY+yref < iHeight; yref++) { // 2868 FLOP
		for (xref = threadIdx.x * CENSUS_SPARSE_THREAD_X; xref < threadIdx.x * CENSUS_SPARSE_THREAD_X + CENSUS_SPARSE_THREAD_X && xref < CENSUS_SPARSE_TILE_W && blockX+xref < iWidth; xref++) {
			p1 = smTileData[yref+CENSUS_SPARSE_RADIUS][xref+CENSUS_SPARSE_RADIUS];
			idx = (IMUL((blockY+yref), iWidth)+(blockX+xref));

			c.x = (censusRowSparse(p1, &smTileData[yref-7+CENSUS_SPARSE_RADIUS][xref-7+CENSUS_SPARSE_RADIUS]) << 24) | \
				  (censusRowSparse(p1, &smTileData[yref-5+CENSUS_SPARSE_RADIUS][xref-7+CENSUS_SPARSE_RADIUS]) << 16) | \
				  (censusRowSparse(p1, &smTileData[yref-3+CENSUS_SPARSE_RADIUS][xref-7+CENSUS_SPARSE_RADIUS]) <<  8) | \
				  (censusRowSparse(p1, &smTileData[yref-1+CENSUS_SPARSE_RADIUS][xref-7+CENSUS_SPARSE_RADIUS]) <<  0);

			c.y = (censusRowSparse(p1, &smTileData[yref+1+CENSUS_SPARSE_RADIUS][xref-7+CENSUS_SPARSE_RADIUS]) << 24) | \
				  (censusRowSparse(p1, &smTileData[yref+3+CENSUS_SPARSE_RADIUS][xref-7+CENSUS_SPARSE_RADIUS]) << 16) | \
				  (censusRowSparse(p1, &smTileData[yref+5+CENSUS_SPARSE_RADIUS][xref-7+CENSUS_SPARSE_RADIUS]) <<  8) | \
				  (censusRowSparse(p1, &smTileData[yref+7+CENSUS_SPARSE_RADIUS][xref-7+CENSUS_SPARSE_RADIUS]) <<  0);

			((int2*)d_census)[idx] = c;
		}
	}
}
//
//template <bool left>
//__global__ void censusTransform(unsigned int *d_census, int iWidth, int iHeight) {
//
//	const int blockX = IMUL(blockIdx.x, CENSUS_TILE_W);
//	const int blockY = IMUL(blockIdx.y, CENSUS_TILE_H);
//	int x, y;
//
//	__shared__ unsigned char smTileData[CENSUS_SMEM_H][CENSUS_SMEM_W];
//
//	for (y = threadIdx.y; y < CENSUS_SMEM_H; y+=blockDim.y) { // 130 FLOP
//		for (x = threadIdx.x; x < CENSUS_SMEM_W; x+=blockDim.x) {
//			if (left)
//				smTileData[y][x] = tex2D(texImageLeft, blockX+x-CENSUS_RADIUS, blockY+y-CENSUS_RADIUS);
//			else
//				smTileData[y][x] = tex2D(texImageRight, blockX+x-CENSUS_RADIUS, blockY+y-CENSUS_RADIUS);
//		}
//	}
//
//
//	// assure that shared memory is completely loaded
//	__syncthreads();
//
//
//	// start calculation
//	unsigned int c;
//	char p1;
//	char p2;
//	int xref, yref;
//	int idx = 0;
//	int i;
//
//	for (yref = threadIdx.y * 4; yref < threadIdx.y * 4 + 4 && yref < CENSUS_SPARSE_TILE_H && blockY+yref < iHeight; yref++) { // 2868 FLOP
//		for (xref = threadIdx.x * 4; xref < threadIdx.x * 4 + 4 && xref < CENSUS_SPARSE_TILE_W && blockX+xref < iWidth; xref++) {
//			p1 = smTileData[yref+CENSUS_RADIUS][xref+CENSUS_RADIUS];
//			idx = (IMUL((blockY+yref), iWidth)+(blockX+xref))*8;
//
//			for (i = 0; i < 8; i++) {
//				c = 0;
//
//				for (y = -7+i*2; y <= -6+i*2; y++) {
//					for (x = -7; x <= 8; x++) {
//						p2 = smTileData[yref+y+CENSUS_RADIUS][xref+x+CENSUS_RADIUS];
//
//						c <<= 1;
//
//						if (p1 > p2)
//							c |= 1;
//					}
//				}
//
//				d_census[idx+i] = c;
//			}
//		}
//	}
//}

template <int nrOfInt>
inline
__device__ int calcHamming(unsigned int *a, unsigned int *b) {
	unsigned int exor;
	int result = 0, j;

	for (j = 0; j < nrOfInt; j++)
	{
		exor = a[j] ^ b[j];

		// counts the set bits
		exor = exor - ((exor >> 1) & 0x55555555);
		exor = (exor & 0x33333333) + ((exor >> 2) & 0x33333333);
		exor = ((exor + (exor >> 4) & 0xF0F0F0F) * 0x1010101) >> 24;

		result += exor;
	}

	return result;
}

template <int nrOfInt>
inline
__device__ int calcHamming2(unsigned int *a, unsigned int *b) {
	unsigned int exor;
	int result = 0, j, i;

	for (j = 0; j < nrOfInt; j++)
	{
		exor = a[j] ^ b[j];

		for (i = 0; i < sizeof(unsigned int); i++) {
			result += exor & 0x1u;
			exor >>= 1;
		}
	}

	return result;
}

template <int blockSize, int nrOfInt>
__global__ void calcDSI(unsigned int *DSI, unsigned int *census_L, unsigned int *census_R, int disp_start, int disp_stop, int disp_step, int iWidth, int iHeight) {
	const int disparities = (disp_stop - disp_start + 1) / disp_step;
	//const int smWidth = (blockDim.x + disparities) * nrOfInt;
	const int smWidth = blockDim.x * nrOfInt;
	const int x = IMUL(blockIdx.x, blockDim.x) + threadIdx.x;
	const int y = IMUL(blockIdx.y, blockDim.y) + threadIdx.y;
	const int blockX = IMUL(blockIdx.x, blockDim.x) * nrOfInt;
	int offset;
	int index;
	int i;

	// ((2 * (#threads + maxdisparity)) * nrOfInt) int values must be allocated
	extern __shared__ unsigned int smCensusData[];

	unsigned int *smCensusL = smCensusData;
	unsigned int *smCensusR = &smCensusData[smWidth+disp_stop*nrOfInt];

	offset = IMUL(y, iWidth) * nrOfInt + blockX;
	//for (i = threadIdx.x; i < smWidth && i + blockX < iWidth * nrOfInt; i += blockDim.x) { //5*(blockDim.x + disparities) * nrOfInt FLOPS
	//	smCensusL[i] = census_L[offset+i];
	//	smCensusR[i] = census_R[offset+i];
	//}

	for (i = threadIdx.x; i < smWidth && i + blockX < iWidth * nrOfInt; i += blockDim.x) { //5*(blockDim.x + disparities) * nrOfInt FLOPS
		smCensusL[i] = census_L[offset+i];
		smCensusR[i] = census_R[offset+i];
	}
	for (i = threadIdx.x + smWidth; i < smWidth + disp_stop * nrOfInt && i + blockX < iWidth * nrOfInt; i += blockDim.x) {
		smCensusL[i] = census_L[offset+i];
	}



	unsigned int cost;
	int offset2;

	__syncthreads();

	//Calculation
	if (x < iWidth && y < iHeight) {
		//offset  = IMUL(iWidth+blockSize, iHeight+blockSize); // 1 DSI page
		offset  = IMUL(iWidth, iHeight); // 1 DSI page
		//offset2 = IMUL(iWidth+blockSize, y+blockSize/2) + x+blockSize/2;   // index within DSI page
		offset2 = IMUL(iWidth, y) + x;   // index within DSI page

		for (i = disp_start; i < disparities + disp_start && x < iWidth-(disp_start+disp_step*i); i++) { //43 FLOP per cycle
			index = (i-disp_start) * offset + offset2;

			cost = calcHamming<nrOfInt>(&smCensusR[threadIdx.x * nrOfInt], &smCensusL[(threadIdx.x + i*disp_step + disp_start) * nrOfInt]);
			DSI[index] = cost;

			// synchronize to avoid serialization
			//__syncthreads();
		}
	}
}


template <int c>
__device__ unsigned int sumValues(unsigned int* arr) {
	return sumValues<c-1>(arr) + arr[c-1];
}

template <>
__device__ unsigned int sumValues<0>(unsigned int* arr) {
	return 0;
}

template <bool computeCapability_1_2, int radius>
__global__ void aggregateFilter(unsigned int *d_imageOut, unsigned int *d_imageIn, int iWidth, int iHeight) {
	const int blockX = IMUL(blockDim.x, blockIdx.x);
	const int blockY = IMUL(blockDim.y, blockIdx.y);
	//const int radius = blockSize/2;
	int x = blockX + threadIdx.x;
	int y = blockY + threadIdx.y;
	int i, j;
	unsigned int data;

	__shared__ unsigned int smImageBlock[16+radius*2][16+radius*2];

		if (computeCapability_1_2) {
			/* // ~40 ms
			for (y = threadIdx.y - radius; y < 16 + radius; y+=blockDim.y) {
				for (x = threadIdx.x - radius; x < 16 + radius; x+=blockDim.x) {
					if (x < 0 || y < 0 || x + blockX >= iWidth || y + blockY >= iHeight)
						data = 0;
					else
						data = getArrayValue(d_imageIn, blockX+x, blockY+y, iWidth);

					smImageBlock[y+radius][x+radius] = data;
				}
			}*/

			// Bad performance due to uncoalesced load, because -radius leads to
			// misalignment on devices with Compute Capability < 1.2

			//~24 ms
			#pragma unroll
			for (j = 0; threadIdx.y + j < 16 + 2 * radius; j += 16) {
				y = blockY + threadIdx.y + j - radius;

				#pragma unroll
				for (i = 0; threadIdx.x + i < 16 + 2 * radius; i += 16) {
					x = blockX + threadIdx.x + i - radius;

					if (x < 0 || y < 0 || x >= iWidth || y >= iHeight)
						data = 0;
					else
						data = getArrayValue(d_imageIn, x, y, iWidth);

					smImageBlock[threadIdx.y + j][threadIdx.x + i] = data;
				}
			}

			x = blockX + threadIdx.x;
			y = blockY + threadIdx.y;
		}


	if (x < iWidth && y < iHeight) {
		if (!computeCapability_1_2) {
			//~18 ms (but with many uncoalesced reads)
			smImageBlock[threadIdx.y+radius][threadIdx.x+radius] = getArrayValue(d_imageIn, x, y, iWidth);

			if (threadIdx.x < radius) {
				if (x-radius >= 0)
					smImageBlock[threadIdx.y+radius][threadIdx.x] = getArrayValue(d_imageIn, x-radius, y, iWidth);
				else
					smImageBlock[threadIdx.y+radius][threadIdx.x] = 0;

				if (threadIdx.y < radius) {
					if (y-radius >= 0 && x-radius >= 0)
						smImageBlock[threadIdx.y][threadIdx.x] = getArrayValue(d_imageIn, x-radius, y-radius, iWidth);
					else
						smImageBlock[threadIdx.y][threadIdx.x] = 0;
				}
				else if (threadIdx.y + radius >= 16) {
					if (y+radius < iHeight && x-radius >= 0)
						smImageBlock[threadIdx.y+radius*2][threadIdx.x] = getArrayValue(d_imageIn, x-radius, y+radius, iWidth);
					else
						smImageBlock[threadIdx.y+radius*2][threadIdx.x] = 0;
				}
			}
			else if (threadIdx.x + radius >= 16) {
				if (x+radius < iWidth)
					smImageBlock[threadIdx.y+radius][threadIdx.x+radius*2] = getArrayValue(d_imageIn, x+radius, y, iWidth);
				else
					smImageBlock[threadIdx.y+radius][threadIdx.x+radius*2] = 0;

				if (threadIdx.y < radius) {
					if (y-radius >= 0 && x+radius < iWidth)
						smImageBlock[threadIdx.y][threadIdx.x+radius*2] = getArrayValue(d_imageIn, x+radius, y-radius, iWidth);
					else
						smImageBlock[threadIdx.y][threadIdx.x+radius*2] = 0;
				}
				else if (threadIdx.y + radius >= 16) {
					if (y+radius < iHeight && x+radius < iWidth)
						smImageBlock[threadIdx.y+radius*2][threadIdx.x+radius*2] = getArrayValue(d_imageIn, x+radius, y+radius, iWidth);
					else
						smImageBlock[threadIdx.y+radius*2][threadIdx.x+radius*2] = 0;
				}
			}

			if (threadIdx.y < radius) {
				if (y-radius >= 0)
					smImageBlock[threadIdx.y][threadIdx.x+radius] = getArrayValue(d_imageIn, x, y-radius, iWidth);
				else
					smImageBlock[threadIdx.y][threadIdx.x+radius] = 0;
			}
			else if (threadIdx.y + radius >= 16) {
				if (y+radius < iHeight)
					smImageBlock[threadIdx.y+radius*2][threadIdx.x+radius] = getArrayValue(d_imageIn, x, y+radius, iWidth);
				else
					smImageBlock[threadIdx.y+radius*2][threadIdx.x+radius] = 0;
			}
		}


		data = 0;

		__syncthreads();


		//#pragma unroll
		//for(j=0; j < radius*2+1; j++) {
		//	#pragma unroll
		//	for (i=0; i < radius*2+1; i++) {
		//		data += smImageBlock[j+threadIdx.y][i+threadIdx.x];
		//	}
		//}

		for (j = threadIdx.y; j < threadIdx.y+radius*2+1; j++) { // 36 FLOP
			//for (x = threadIdx.x; x < threadIdx.x+blockSize; x++) {
			//	data += smImageBlock[y][x];
			//}
			data += sumValues<radius*2+1>(&smImageBlock[j][threadIdx.x]); // about 3ms faster than a loop
		}

		setArrayValue(d_imageOut, data, x, y, iWidth);
	}
}
//
//template <bool computeCapability_1_2, int blockSize>
//__global__ void aggregateFilter(unsigned int *d_imageOut, unsigned int *d_imageIn, int iWidth, int iHeight) {
//	const int blockX = IMUL(blockDim.x, blockIdx.x);
//	const int blockY = IMUL(blockDim.y, blockIdx.y);
//	const int radius = blockSize/2;
//	int x = blockX + threadIdx.x;
//	int y = blockY + threadIdx.y;
//	int i, j;
//	unsigned int data;
//
//	__shared__ unsigned int smImageBlock[16+blockSize][16+blockSize+1];
//
//
//	if (x < iWidth && y < iHeight) {
//
//		if (computeCapability_1_2) {
//			/* // ~40 ms
//			for (y = threadIdx.y - radius; y < 16 + radius; y+=blockDim.y) {
//				for (x = threadIdx.x - radius; x < 16 + radius; x+=blockDim.x) {
//					if (x < 0 || y < 0 || x + blockX >= iWidth || y + blockY >= iHeight)
//						data = 0;
//					else
//						data = getArrayValue(d_imageIn, blockX+x, blockY+y, iWidth);
//
//					smImageBlock[y+radius][x+radius] = data;
//				}
//			}*/
//
//			// Bad performance due to uncoalesced load, because -radius leads to
//			// misalignment on devices with Compute Capability < 1.2
//
//			//~24 ms
//			for (j = 0; threadIdx.y + j < 16 + blockSize; j+=16) {
//				for (i = 0; threadIdx.x + i < 16 + blockSize; i+=16) {
//					x = blockX + i + threadIdx.x - radius;
//					y = blockY + j + threadIdx.y - radius;
//
//					if (x < 0 || y < 0 || x >= iWidth || y >= iHeight)
//						data = 0;
//					else
//						data = getArrayValue(d_imageIn, x, y, iWidth);
//						//data = tex1Dfetch(texInt, y*iWidth+x);
//
//					smImageBlock[j+threadIdx.y][i+threadIdx.x] = data;
//
//
//					//smImageBlock[j+threadIdx.y][i+threadIdx.x] = tex2D(texDSI, x, y);
//				}
//			}
//
//			x = blockX + threadIdx.x;
//			y = blockY + threadIdx.y;
//		}
//		else {
//			//~18 ms (but with many uncoalesced reads)
//			smImageBlock[threadIdx.y+radius][threadIdx.x+radius] = getArrayValue(d_imageIn, x, y, iWidth);
//
//			if (threadIdx.x < radius) {
//				if (x-radius >= 0)
//					smImageBlock[threadIdx.y+radius][threadIdx.x] = getArrayValue(d_imageIn, x-radius, y, iWidth);
//				else
//					smImageBlock[threadIdx.y+radius][threadIdx.x] = 0;
//
//				if (threadIdx.y < radius) {
//					if (y-radius >= 0 && x-radius >= 0)
//						smImageBlock[threadIdx.y][threadIdx.x] = getArrayValue(d_imageIn, x-radius, y-radius, iWidth);
//					else
//						smImageBlock[threadIdx.y][threadIdx.x] = 0;
//				}
//				else if (threadIdx.y + radius >= 16) {
//					if (y+radius < iHeight && x-radius >= 0)
//						smImageBlock[threadIdx.y+radius*2][threadIdx.x] = getArrayValue(d_imageIn, x-radius, y+radius, iWidth);
//					else
//						smImageBlock[threadIdx.y+radius*2][threadIdx.x] = 0;
//				}
//			}
//			else if (threadIdx.x + radius >= 16) {
//				if (x+radius < iWidth)
//					smImageBlock[threadIdx.y+radius][threadIdx.x+radius*2] = getArrayValue(d_imageIn, x+radius, y, iWidth);
//				else
//					smImageBlock[threadIdx.y+radius][threadIdx.x+radius*2] = 0;
//
//				if (threadIdx.y < radius) {
//					if (y-radius >= 0 && x+radius < iWidth)
//						smImageBlock[threadIdx.y][threadIdx.x+radius*2] = getArrayValue(d_imageIn, x+radius, y-radius, iWidth);
//					else
//						smImageBlock[threadIdx.y][threadIdx.x+radius*2] = 0;
//				}
//				else if (threadIdx.y + radius >= 16) {
//					if (y+radius < iHeight && x+radius < iWidth)
//						smImageBlock[threadIdx.y+radius*2][threadIdx.x+radius*2] = getArrayValue(d_imageIn, x+radius, y+radius, iWidth);
//					else
//						smImageBlock[threadIdx.y+radius*2][threadIdx.x+radius*2] = 0;
//				}
//			}
//
//			if (threadIdx.y < radius) {
//				if (y-radius >= 0)
//					smImageBlock[threadIdx.y][threadIdx.x+radius] = getArrayValue(d_imageIn, x, y-radius, iWidth);
//				else
//					smImageBlock[threadIdx.y][threadIdx.x+radius] = 0;
//			}
//			else if (threadIdx.y + radius >= 16) {
//				if (y+radius < iHeight)
//					smImageBlock[threadIdx.y+radius*2][threadIdx.x+radius] = getArrayValue(d_imageIn, x, y+radius, iWidth);
//				else
//					smImageBlock[threadIdx.y+radius*2][threadIdx.x+radius] = 0;
//			}
//		}
//
//
//		data = 0;
//
//		__syncthreads();
//
//		for (j = threadIdx.y; j < threadIdx.y+blockSize; j++) { // 36 FLOP
//			/*
//			for (x = threadIdx.x; x < threadIdx.x+blockSize; x++) {
//				data += smImageBlock[y][x];
//			}
//			*/
//			data += sumValues<blockSize>(&smImageBlock[j][threadIdx.x]); // about 3ms faster than a loop
//		}
//
//		setArrayValue(d_imageOut, data, x, y, iWidth);
//	}
//}

template <class T>
__global__ void setArray(T* arr, T value, int width, int height) {
	int x = IMUL(blockDim.x, blockIdx.x) + threadIdx.x;
	int y = IMUL(blockDim.y, blockIdx.y) + threadIdx.y;

	if (x < width && y < height) {
		arr[y*width+x] = value;
	}
}


template <bool RL, int blockSize>
__global__ void refineSubPixel(float *d_DMI, int *d_Cost, unsigned int *d_Confidence, unsigned int *d_DSI,
							   int disp_start, int disp_stop, int disp_step, int maxCosts,
							   int iWidth, int iHeight) {
	const int disparities = (disp_stop - disp_start + 1) / disp_step;
	int x = IMUL(blockDim.x, blockIdx.x) + threadIdx.x;
	int y = IMUL(blockDim.y, blockIdx.y) + threadIdx.y;
	int minDisp, minCost, curCost;
	int predecessor, successor;
	int d;
	int offset1, offset2;
	float subPixelDisp;
	int disp_cut;

	if (x < iWidth && y < iHeight) {
		minDisp = 0;
		minCost = maxCosts;

		offset1 = IMUL(iWidth, iHeight);
		offset2 = IMUL(y, iWidth);

		//if (x == 400 && y == 49)
		//	x+=0;

		if (RL) {
			for (d = 0, disp_cut = disp_start+disp_step*d; d < disparities && x < iWidth-disp_cut; d++, disp_cut+=disp_step) {
			//for (d = 0; d < disparities; d++) {
				//disp_cut = disp_start+disp_step*d;

				//if (x < iWidth-disp_cut) {
					// load w/o texture, because loading coalesces
					curCost = d_DSI[d*offset1 + offset2 + x];
					//curCost = tex1Dfetch(texDSI1d, d*offset1 + offset2 + x);

					if (curCost <= minCost) {
						minDisp = d;
						minCost = curCost;
					}
				//}
			}
		}
		else {
			//for (d = 0, disp_cut = disp_start+disp_step*d; d < disparities && x >= disp_cut; d++, disp_cut+=disp_step) {
			for (d = 0, disp_cut = disp_start+disp_step*d; d < disparities && x >= disp_cut; d++, disp_cut+=disp_step) {
				//disp_cut = disp_start+disp_step*d;

				//if (x >= disp_cut) {
					//curCost = d_DSI[d*offset1 + offset2 + x-disp_cut];
					curCost = tex1Dfetch(texDSI1d, d*offset1 + offset2 + x-disp_cut);

					if (curCost <= minCost) {
						minDisp = d;
						minCost = curCost;
					}
				//}
			}
		}



		if (minDisp > 0 && minDisp < disp_stop && (x != disp_cut-1)) {
			if (RL) {
				//predecessor = getArrayValue(d_DSI, x, y, minDisp-1, iWidth, iHeight);
				//successor = getArrayValue(d_DSI, x, y, minDisp+1, iWidth, iHeight);
				predecessor = tex1Dfetch(texDSI1d, (minDisp-1)*offset1 + offset2 + x);
				successor = tex1Dfetch(texDSI1d, (minDisp+1)*offset1 + offset2 + x);
			}
			else {
				//predecessor = getArrayValue(d_DSI, x-disp_start-(minDisp-1)*disp_step, y, minDisp-1, iWidth, iHeight);
				//successor = getArrayValue(d_DSI, x-disp_start-(minDisp+1)*disp_step, y, minDisp+1, iWidth, iHeight);
				predecessor = tex1Dfetch(texDSI1d, (minDisp-1)*offset1 + offset2 + x-disp_start-(minDisp-1)*disp_step);
				successor = tex1Dfetch(texDSI1d, (minDisp+1)*offset1 + offset2 + x-disp_start-(minDisp+1)*disp_step);
			}

			if (2*minCost-predecessor-successor == 0)
				subPixelDisp = (minDisp * disp_step + disp_start);
			else {
				subPixelDisp = (minDisp * disp_step + disp_start) + ((successor-predecessor) / (float)(2*(2*minCost-predecessor-successor)));

				if (subPixelDisp > disp_stop)
					subPixelDisp = disp_stop;
				else if (subPixelDisp < disp_start)
					subPixelDisp = disp_start;
			}
		}
		else {
			subPixelDisp = (minDisp * disp_step + disp_start);
		}

/*
		// Confidence
		int minDispConf = 0;
		int minCostConf = maxCosts;
		if (RL) {
			for (d = 0, disp_cut = disp_start+disp_step*d; d < disparities && x < iWidth-disp_cut; d++, disp_cut+=disp_step) {
				curCost = d_DSI[d*offset1 + offset2 + x];

				if (curCost <= minCostConf && abs(minDisp - d) > 2) {
					minCostConf = curCost;
				}
			}
		}
		else {
			for (d = 0, disp_cut = disp_start+disp_step*d; d < disparities && x >= disp_cut; d++, disp_cut+=disp_step) {
				curCost = tex1Dfetch(texDSI1d, d*offset1 + offset2 + x-disp_cut);

				if (curCost <= minCostConf && abs(minDisp - d) > 2) {
					minCostConf = curCost;
				}
			}
		}
		int conf = 1024*(abs(minCostConf-minCost)/(float)maxCosts);
		if (conf > 255)
			conf = 255;*/


		if (!RL) {
			int minDispConf = 0;
			int minCostConf = maxCosts;
				for (d = 0, disp_cut = disp_start+disp_step*d; d < disparities && x >= disp_cut; d++, disp_cut+=disp_step) {
					curCost = tex1Dfetch(texDSI1d, d*offset1 + offset2 + x-disp_cut);

					if (curCost <= minCostConf && abs(minDisp - d) > 2) {
						minCostConf = curCost;
					}
				}
			unsigned int conf = 1024*(abs(minCostConf-minCost)/(float)maxCosts);
			if (conf > 255)
				conf = 255;

			setArrayValue(d_Confidence, conf, x, y, iWidth);

		}

		setArrayValue(d_DMI, subPixelDisp, x, y, iWidth);
		setArrayValue(d_Cost, minCost, x, y, iWidth);
		//setArrayValue(d_Confidence, conf, x, y, iWidth);
	}
}
//template <bool RL>
//__global__ void refineSubPixel(float *d_DMI, int *d_Cost, unsigned int *d_DSI, int disp_start, int disp_stop, int disp_step, int iWidth, int iHeight) {
//	const int disparities = (disp_stop - disp_start + 1) / disp_step;
//	int x = IMUL(blockDim.x, blockIdx.x) + threadIdx.x;
//	int y = IMUL(blockDim.y, blockIdx.y) + threadIdx.y;
//	int minDisp, minCost, curCost;
//	int predecessor, successor;
//	int d;
//	int offset1, offset2;
//	float subPixelDisp;
//	int disp_cut;
//
//	if (x < iWidth && y < iHeight) {
//		minDisp = -1;
//		minCost = ~(1<<31);
//
//		offset1 = IMUL(iWidth, iHeight);
//		offset2 = IMUL(y, iWidth);
//
//		if (RL) {
//			for (d = 0; d < disparities; d++) {
//				disp_cut = disp_start+disp_step*d;
//
//				if (x < iWidth-disp_cut) {
//					// load w/o texture, because loading coalesces
//					curCost = d_DSI[d*offset1 + offset2 + x];
//					//curCost = tex1Dfetch(texDSI1d, d*offset1 + offset2 + x);
//
//					if (curCost <= minCost) {
//						minDisp = d;
//						minCost = curCost;
//					}
//				}
//			}
//		}
//		else {
//			for (d = 0; d < disparities; d++) {
//				disp_cut = disp_start+disp_step*d;
//
//				if (x >= disp_cut) {
//					//curCost = d_DSI[d*offset1 + offset2 + x-disp_cut];
//					curCost = tex1Dfetch(texDSI1d, d*offset1 + offset2 + x-disp_cut);
//
//					if (curCost <= minCost) {
//						minDisp = d;
//						minCost = curCost;
//					}
//				}
//			}
//		}
//
//		if (minDisp > 0 && minDisp < disp_stop) {
//			if (RL) {
//				//predecessor = getArrayValue(d_DSI, x, y, minDisp-1, iWidth, iHeight);
//				//successor = getArrayValue(d_DSI, x, y, minDisp+1, iWidth, iHeight);
//				predecessor = tex1Dfetch(texDSI1d, (minDisp-1)*offset1 + offset2 + x);
//				successor = tex1Dfetch(texDSI1d, (minDisp+1)*offset1 + offset2 + x);
//			}
//			else {
//				//predecessor = getArrayValue(d_DSI, x-disp_start-(minDisp-1)*disp_step, y, minDisp-1, iWidth, iHeight);
//				//successor = getArrayValue(d_DSI, x-disp_start-(minDisp+1)*disp_step, y, minDisp+1, iWidth, iHeight);
//				predecessor = tex1Dfetch(texDSI1d, (minDisp-1)*offset1 + offset2 + x-disp_start-(minDisp-1)*disp_step);
//				successor = tex1Dfetch(texDSI1d, (minDisp+1)*offset1 + offset2 + x-disp_start-(minDisp+1)*disp_step);
//			}
//
//			if (2*minCost-predecessor-successor == 0)
//				subPixelDisp = (minDisp * disp_step + disp_start);
//			else {
//				subPixelDisp = (minDisp * disp_step + disp_start) + ((successor-predecessor) / (float)(2*(2*minCost-predecessor-successor)));
//
//				if (subPixelDisp > disp_stop)
//					subPixelDisp = disp_stop;
//				else if (subPixelDisp < disp_start)
//					subPixelDisp = disp_start;
//			}
//		}
//		else {
//			subPixelDisp = (minDisp * disp_step + disp_start);
//		}
//
//		setArrayValue(d_DMI, subPixelDisp, x, y, iWidth);
//		setArrayValue(d_Cost, minCost, x, y, iWidth);
//	}
//}
__global__ void compareDispsLeft(float *d_DMI, int *d_Costs, unsigned int *d_Confidence,
								 float *d_DMI_LR, float *d_DMI_RL,
								 int *d_CostsLR, int *d_CostsRL,
								 unsigned int *d_ConfidenceLR, unsigned int *d_ConfidenceRL,
								 float max_disp_diff, int maxCosts, int iWidth, int iHeight) {
	const int x = IMUL(blockDim.x, blockIdx.x) + threadIdx.x;
	const int y = IMUL(blockDim.y, blockIdx.y) + threadIdx.y;

	float a, b, diff;

	//if (x == 400 && y == 49)
	//	a = 5;
	if (x < iWidth && y < iHeight) {
		a = getArrayValue(d_DMI_LR, x, y, iWidth);
		if (a > x || x-a >= iWidth) b = a; // for debuging only
		else b = getArrayValue(d_DMI_RL, f2i(x-a), y, iWidth);
		diff = abs(a - b);

		/*
		if (diff == a) {
			setArrayValue(d_DMI, a, x, y, iWidth);
			setArrayValue(d_Costs, getArrayValue(d_CostsLR, x, y, iWidth), x, y, iWidth);
		}
		else if (diff == b) {
			if (x+b < iWidth) {
				setArrayValue(d_DMI, b, f2i(x+b), y, iWidth);
				setArrayValue(d_Costs, getArrayValue(d_CostsRL, f2i(x+b), y, iWidth), f2i(x+b), y, iWidth);
			}
		}
		else */if (diff <= max_disp_diff) {
			setArrayValue(d_DMI, (a+b)/2, x, y, iWidth);
			setArrayValue(d_Costs, getArrayValue(d_CostsLR, x, y, iWidth), x, y, iWidth);
			setArrayValue(d_Confidence, getArrayValue(d_ConfidenceLR, x, y, iWidth), x, y, iWidth);
		}
		else {
			setArrayValue(d_DMI, 0.0f, x, y, iWidth);
			setArrayValue(d_Costs, maxCosts, x, y, iWidth);
			setArrayValue(d_Confidence, 0u, x, y, iWidth);
		}
	}
}
//
//__global__ void compareDispsLeft(float *d_DMI, int *d_Costs, float *d_DMI_LR, float *d_DMI_RL, int *d_CostsLR, int *d_CostsRL, float max_disp_diff, int iWidth, int iHeight) {
//	const int x = IMUL(blockDim.x, blockIdx.x) + threadIdx.x;
//	const int y = IMUL(blockDim.y, blockIdx.y) + threadIdx.y;
//
//	float a, b, diff;
//
//
//	if (x < iWidth && y < iHeight) {
//		a = getArrayValue(d_DMI_LR, x, y, iWidth);
//		if (a > x || x-a >= iWidth) b = 0; // for debuging only
//		else b = getArrayValue(d_DMI_RL, f2i(x-a), y, iWidth);
//		diff = abs(a - b);
//
//		if (diff == a) {
//			setArrayValue(d_DMI, a, x, y, iWidth);
//			setArrayValue(d_Costs, getArrayValue(d_CostsLR, x, y, iWidth), x, y, iWidth);
//		}
//		else if (diff == b) {
//			if (x+b < iWidth) {
//				setArrayValue(d_DMI, b, f2i(x+b), y, iWidth);
//				setArrayValue(d_Costs, getArrayValue(d_CostsRL, f2i(x+b), y, iWidth), f2i(x+b), y, iWidth);
//			}
//		}
//		else if (diff <= max_disp_diff) {
//			setArrayValue(d_DMI, (a+b)/2, x, y, iWidth);
//			setArrayValue(d_Costs, getArrayValue(d_CostsLR, x, y, iWidth), x, y, iWidth);
//		}
//		else {
//			setArrayValue(d_DMI, 0.0f, x, y, iWidth);
//			setArrayValue(d_Costs, 0, x, y, iWidth);
//		}
//	}
//}

__global__ void roundDisparities(float *d_DMI, int resolution, int iWidth, int iHeight) {
	const int x = IMUL(blockDim.x, blockIdx.x) + threadIdx.x;
	const int y = IMUL(blockDim.y, blockIdx.y) + threadIdx.y;
	float value;

	if (x < iWidth && y < iHeight) {
		value = getArrayValue(d_DMI, x, y, iWidth);

		value *= resolution;
		value += 0.5f;
		value = (float)((int)value) / resolution;

		setArrayValue(d_DMI, value, x, y, iWidth);
	}
}

__global__ void scaleDisparities(float *d_DMI, int scale, int iWidth, int iHeight) {
	const int x = IMUL(blockDim.x, blockIdx.x) + threadIdx.x;
	const int y = IMUL(blockDim.y, blockIdx.y) + threadIdx.y;
	float value;

	if (x < iWidth && y < iHeight) {
		value = getArrayValue(d_DMI, x, y, iWidth);

		value *= scale;

		setArrayValue(d_DMI, value, x, y, iWidth);
	}
}

__global__ void roundAndScaleDisparities(float *d_DMI, int resolution, int scale, int iWidth, int iHeight) {
	const int x = IMUL(blockDim.x, blockIdx.x) + threadIdx.x;
	const int y = IMUL(blockDim.y, blockIdx.y) + threadIdx.y;
	float value;

	if (x < iWidth && y < iHeight) {
		value = getArrayValue(d_DMI, x, y, iWidth);

		// round
		value *= resolution;
		value += 0.5f;
		value = (float)((int)value) / resolution;

		// scale
		value *= scale;

		setArrayValue(d_DMI, value, x, y, iWidth);
	}
}


// takes about 1.84 ms for a 16x16 census
extern "C" void gpuCensusTransform() {

	cudaStreamSynchronize( streamImageLeft[g_streamNr] ) ;
	cudaStreamSynchronize( streamImageRight[g_streamNr] ) ;

	START_TIMER;

	if (g_sparse) {
		dim3 grid(iDivUp(g_Width, CENSUS_SPARSE_TILE_W), iDivUp(g_Height, CENSUS_SPARSE_TILE_H));
		dim3 block(16, 16);

		// left
		censusTransformSparse<true> <<<grid, block>>>(d_censusLeft, g_Width, g_Height);

		// right
		censusTransformSparse<false> <<<grid, block>>>(d_censusRight, g_Width, g_Height);

//		printf("  Bandwidth: %.1f GB/s\n", 2*(iDivUp(g_Width, CENSUS_SPARSE_TILE_W)*iDivUp(g_Height, CENSUS_SPARSE_TILE_H)*CENSUS_SPARSE_SMEM_W*CENSUS_SMEM_H*sizeof(unsigned int)+g_Width*g_Height*8)/(gpuTime*1000000));
//		printf("  GFLOPS:  : %.1f\n", 2*3000*16*16*iDivUp(g_Width, 64)*iDivUp(g_Height, 64)/(gpuTime*1000000));
	}
	//else {
	//	dim3 grid(iDivUp(g_Width, CENSUS_TILE_W), iDivUp(g_Height, CENSUS_TILE_H));
	//	dim3 block(8, 24);

	//	// left
	//	censusTransform<true> <<<grid, block>>>(d_censusLeft, g_Width, g_Height);

	//	// right
	//	censusTransform<false> <<<grid, block>>>(d_censusRight, g_Width, g_Height);
	//}

	STOP_TIMER(g_CensusTiming[eCensusTransform]);
}

extern "C" void gpuCalcDSI() {
	unsigned int disparities = (g_disp_max + 1 - g_disp_min) / g_disp_step;
	int sharedMem;

	START_TIMER;

	// ((2 * (#threads + maxdisparity)) * nrOfInt) int values must be allocated
	if (g_sparse) {
		dim3 grid(iDivUp(g_Width, 128), g_Height);
		dim3 block(128, 1);

		//sharedMem = 2 * (128 + disparities) * 2 * sizeof(unsigned int);
		sharedMem = (2 * 128 + disparities) * 2 * sizeof(unsigned int);

#if USE_FAST_AGGREGATION
		calcDSI<5, 2> <<<grid, block, sharedMem>>>(d_DSI_Temp, d_censusLeft, d_censusRight, g_disp_min, g_disp_max, g_disp_step, g_Width, g_Height);
#else
		calcDSI<2> <<<grid, block, sharedMem>>>(d_DSI, d_censusLeft, d_censusRight, g_disp_min, g_disp_max, g_disp_step, g_Width, g_Height);
#endif
	}
	else {
//		dim3 grid(iDivUp(g_Width, 128), g_Height);
//		dim3 block(128, 1);
//
//		sharedMem = 2 * (128 + disparities) * 8 * sizeof(unsigned int);
//
//#if USE_FAST_AGGREGATION
//		calcDSI<8> <<<grid, block, sharedMem>>>(d_DSI_Temp, d_censusLeft, d_censusRight, g_disp_min, g_disp_max, g_disp_step, g_Width, g_Height);
//#else
//		calcDSI<8> <<<grid, block, sharedMem>>>(d_DSI, d_censusLeft, d_censusRight, g_disp_min, g_disp_max, g_disp_step, g_Width, g_Height);
//#endif
	}

	STOP_TIMER(g_CensusTiming[eCalcDSI]);

//	printf("  Bandwidth: %.1f GB/s\n", (iDivUp(g_Width, 128)*g_Height*2*(128 + disparities)*2*sizeof(unsigned int)+2*(g_Width-g_disp_max)*g_Height*sizeof(unsigned int))/(gpuTime*1000000));
//	printf("  GFLOPS:  : %.1f\n", (5*(128+disparities)*2*g_Height*iDivUp(g_Width, 128)+(17+43*disparities)*g_Width*g_Height)/(gpuTime*1000000));
}

extern "C" void gpuAggregateCosts() {
	int disparities = (g_disp_max + 1 - g_disp_min) / g_disp_step;
	int d;

	dim3 grid(iDivUp(g_Width, 16), iDivUp(g_Height, 16));
	dim3 block(16, 16);

	START_TIMER;

//#if USE_FAST_AGGREGATION
	for (d = 0; d < disparities; d++) {
		aggregateFilter<false, 2> <<<grid, block>>> (&d_DSI[g_Width*g_Height*d], &d_DSI_Temp[g_Width*g_Height*d], g_Width, g_Height);
	}
//#else
//	unsigned int *d_Temp;
//	CUDA_SAFE_CALL(cudaMalloc((void**)&d_Temp, g_Width * g_Height * sizeof(unsigned int)));
//
//	for (d = 0; d < disparities; d++) {
//		CUDA_SAFE_CALL( cudaMemcpy(d_Temp, &d_DSI[g_Width*g_Height*d], g_Width*g_Height*sizeof(unsigned int), cudaMemcpyDeviceToDevice) );
//		aggregateFilter<false, 5> <<<grid, block>>> (&d_DSI[g_Width*g_Height*d], d_Temp, g_Width, g_Height);
//	}
//
//	CUDA_SAFE_CALL( cudaFree(d_Temp) );
//#endif

	STOP_TIMER(g_CensusTiming[eAggregateCosts]);

//	printf("  Bandwidth: %.1f GB/s\n", disparities*(g_Width*g_Height*sizeof(int) + iDivUp(g_Width, 16)*iDivUp(g_Height, 16)*21*21*sizeof(int))/(gpuTime*1000000));
//	printf("  GFLOPS:  : %.1f\n", disparities*60*16*16*iDivUp(g_Width, 16)*iDivUp(g_Height, 16) /(gpuTime*1000000) );
}
extern "C" void gpuRefineSubPixel() {
	/*
	dim3 grid(iDivUp(g_Width, 16), iDivUp(g_Height, 8));
	dim3 block(16, 8);
	*/
	dim3 grid(iDivUp(g_Width, 128), iDivUp(g_Height, 1));
	dim3 block(128, 1);

	START_TIMER;

	refineSubPixel<true, 5> <<<grid, block>>>(d_DMI_RL, d_Costs_RL, d_Confidence_RL, d_DSI, g_disp_min, g_disp_max, g_disp_step, 1600, g_Width, g_Height);

	//printf("  Bandwidth: %.1f GB/s\n", ((disparities+2)*g_Width*g_Height*sizeof(int))/(gpuTime*1000000));
	//printf("  GFLOPS:  : %.1f\n", ((disparities*7+35)*g_Width*g_Height) /(gpuTime*1000000) );

	refineSubPixel<false, 5> <<<grid, block>>>(d_DMI_LR, d_Costs_LR, d_Confidence_LR, d_DSI, g_disp_min, g_disp_max, g_disp_step, 1600, g_Width, g_Height);
	STOP_TIMER(g_CensusTiming[eRefineSubPixel]);

	//printf("  Bandwidth: %.1f GB/s\n", ((disparities+2)*g_Width*g_Height*sizeof(int))/(gpuTime*1000000));
	//printf("  GFLOPS:  : %.1f\n", ((disparities*7+43)*g_Width*g_Height) /(gpuTime*1000000) );
}
//extern "C" void gpuRefineSubPixel() {
//	/*
//	dim3 grid(iDivUp(g_Width, 16), iDivUp(g_Height, 8));
//	dim3 block(16, 8);
//	*/
//	dim3 grid(iDivUp(g_Width, 128), iDivUp(g_Height, 1));
//	dim3 block(128, 1);
//
//	START_TIMER;
//
//	refineSubPixel<true> <<<grid, block>>>(d_DMI_RL, d_Costs_RL, d_DSI, g_disp_min, g_disp_max, g_disp_step, g_Width, g_Height);
//
//	//printf("  Bandwidth: %.1f GB/s\n", ((disparities+2)*g_Width*g_Height*sizeof(int))/(gpuTime*1000000));
//	//printf("  GFLOPS:  : %.1f\n", ((disparities*7+35)*g_Width*g_Height) /(gpuTime*1000000) );
//
//	refineSubPixel<false> <<<grid, block>>>(d_DMI_LR, d_Costs_LR, d_DSI, g_disp_min, g_disp_max, g_disp_step, g_Width, g_Height);
//
//	STOP_TIMER(g_CensusTiming[eRefineSubPixel]);
//
//	//printf("  Bandwidth: %.1f GB/s\n", ((disparities+2)*g_Width*g_Height*sizeof(int))/(gpuTime*1000000));
//	//printf("  GFLOPS:  : %.1f\n", ((disparities*7+43)*g_Width*g_Height) /(gpuTime*1000000) );
//}

extern "C" void gpuCompareDisps() {
	dim3 grid(iDivUp(g_Width, 16), iDivUp(g_Height, 16));
	dim3 block(16, 16);

	START_TIMER;

	compareDispsLeft<<<grid, block>>>(d_DMI, d_Costs, d_Confidence, d_DMI_LR, d_DMI_RL, d_Costs_LR, d_Costs_RL, d_Confidence_LR, d_Confidence_RL, 1.0, 1600, g_Width, g_Height);

	STOP_TIMER(g_CensusTiming[eCompareDisps]);

//	printf("  Bandwidth: %.1f GB/s\n", (2*g_Width*g_Height*sizeof(float))/(gpuTime*1000000));
//	printf("  GFLOPS:  : %.1f\n", 16*16*16*iDivUp(g_Width, 16)*iDivUp(g_Height, 16) /(gpuTime*1000000) );
}

//extern "C" void gpuCompareDisps() {
//	dim3 grid(iDivUp(g_Width, 16), iDivUp(g_Height, 16));
//	dim3 block(16, 16);
//
//	START_TIMER;
//
//	compareDispsLeft<<<grid, block>>>(d_DMI, d_Costs, d_DMI_LR, d_DMI_RL, d_Costs_LR, d_Costs_RL, 1.0, g_Width, g_Height);
//
//	STOP_TIMER(g_CensusTiming[eCompareDisps]);
//
////	printf("  Bandwidth: %.1f GB/s\n", (2*g_Width*g_Height*sizeof(float))/(gpuTime*1000000));
////	printf("  GFLOPS:  : %.1f\n", 16*16*16*iDivUp(g_Width, 16)*iDivUp(g_Height, 16) /(gpuTime*1000000) );
//}

extern "C" void gpuRoundAndScaleDisparities() {
	dim3 grid(iDivUp(g_Width, 16), iDivUp(g_Height, 16));
	dim3 block(16, 16);

	START_TIMER;

	roundAndScaleDisparities<<<grid, block>>>(d_DMI, 10, DISPARITY_SCALING, g_Width, g_Height);

	STOP_TIMER(g_CensusTiming[eRoundAndScaleDisparities]);

//	printf("  Bandwidth: %.1f GB/s\n", (2*g_Width*g_Height*sizeof(float))/(gpuTime*1000000));
//	printf("  GFLOPS:  : %.1f\n", 12*16*16*iDivUp(g_Width, 16)*iDivUp(g_Height, 16) /(gpuTime*1000000) );
}

extern "C" void gpuCensusImageSetup(unsigned int w, unsigned int h, unsigned int disp_min, unsigned int disp_max, unsigned int disp_step, bool sparse, unsigned int blockSize) {
	unsigned int disparities = (disp_max + 1 - disp_min) / disp_step;

	g_Width = w;
	g_Height = h;

	g_disp_min = disp_min;
	g_disp_max = disp_max;
	g_disp_step = disp_step;
	g_sparse = sparse;
	g_blockSize = blockSize;

	for (int i=0; i<2; i++) {
		CUDA_SAFE_CALL( cudaStreamCreate(&streamImageLeft[i]) );
		CUDA_SAFE_CALL( cudaStreamCreate(&streamImageRight[i]) );
		CUDA_SAFE_CALL( cudaStreamCreate(&streamDM[i]) );

		CUDA_SAFE_CALL( cudaMallocHost((void**)&h_left[i],  g_Width*g_Height*sizeof(unsigned char)) );
		CUDA_SAFE_CALL( cudaMallocHost((void**)&h_right[i], g_Width*g_Height*sizeof(unsigned char)) );
		CUDA_SAFE_CALL( cudaMallocHost((void**)&h_dm[i],    g_Width*g_Height*sizeof(unsigned char)) );
	}

	cudaChannelFormatDesc channelDescChar = cudaCreateChannelDesc<unsigned char>();
	cudaChannelFormatDesc channelDescInt = cudaCreateChannelDesc<unsigned int>();

	CUDA_SAFE_CALL(cudaMallocArray(&cuImageLeft, &channelDescChar, g_Width, g_Height));
	CUDA_SAFE_CALL(cudaMallocArray(&cuImageRight, &channelDescChar, g_Width, g_Height));
	CUDA_SAFE_CALL(cudaBindTextureToArray(texImageLeft,  cuImageLeft, channelDescChar));
	CUDA_SAFE_CALL(cudaBindTextureToArray(texImageRight, cuImageRight, channelDescChar));

	if (sparse) {
		CUDA_SAFE_CALL(cudaMalloc((void**)&d_censusLeft, g_Width*g_Height*8));
		CUDA_SAFE_CALL(cudaMalloc((void**)&d_censusRight, g_Width*g_Height*8));
	}
	else {
		CUDA_SAFE_CALL(cudaMalloc((void**)&d_censusLeft, g_Width*g_Height*256/8));
		CUDA_SAFE_CALL(cudaMalloc((void**)&d_censusRight, g_Width*g_Height*256/8));
	}

	CUDA_SAFE_CALL(cudaMalloc((void**)&d_DSI, disparities * g_Width * g_Height * sizeof(unsigned int)));
	CUDA_SAFE_CALL(cudaBindTexture(0, texDSI1d, d_DSI, disparities * g_Width * g_Height * sizeof(unsigned int)));

#if USE_FAST_AGGREGATION
	CUDA_SAFE_CALL(cudaMalloc((void**)&d_DSI_Temp, disparities * g_Width * g_Height * sizeof(unsigned int)));
	CUDA_SAFE_CALL(cudaBindTexture(0, texDSITemp1d, d_DSI_Temp, disparities * g_Width * g_Height * sizeof(unsigned int)));
#endif

	//CUDA_SAFE_CALL(cudaMalloc((void**)&d_integralImage, ((g_Width-disp_max) + 2 * g_blockSize) * (g_Height + 2 * g_blockSize) * sizeof(unsigned int)));

	CUDA_SAFE_CALL(cudaMalloc((void**)&d_DMI, g_Width * g_Height * sizeof(float)));
//	CUDA_SAFE_CALL(cudaMalloc((void**)&d_DMI2, g_Width * g_Height * sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&d_DMI_RL, g_Width * g_Height * sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&d_DMI_LR, g_Width * g_Height * sizeof(float)));

	CUDA_SAFE_CALL(cudaMalloc((void**)&d_Costs, g_Width * g_Height * sizeof(int)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&d_Costs_RL, g_Width * g_Height * sizeof(int)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&d_Costs_LR, g_Width * g_Height * sizeof(int)));

	CUDA_SAFE_CALL(cudaMalloc((void**)&d_Confidence, g_Width * g_Height * sizeof(int)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&d_Confidence_LR, g_Width * g_Height * sizeof(int)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&d_Confidence_RL, g_Width * g_Height * sizeof(int)));


	//CUDA_SAFE_CALL(cudaMemset(d_integralImage, 0, ((g_Width-disp_max) + 2 * g_blockSize) * (g_Height + 2 * g_blockSize) * sizeof(unsigned int)));
	CUDA_SAFE_CALL(cudaMemset(d_DSI, 0, disparities * g_Width * g_Height * sizeof(unsigned int)));
	CUDA_SAFE_CALL(cudaMemset(d_Costs, 0, g_Width * g_Height * sizeof(int)));
	CUDA_SAFE_CALL(cudaMemset(d_Costs_RL, 0, g_Width * g_Height * sizeof(int)));
	CUDA_SAFE_CALL(cudaMemset(d_Costs_LR, 0, g_Width * g_Height * sizeof(int)));

	dim3 grid(iDivUp(g_Width, 16), iDivUp(g_Height, 16));
	dim3 block(16, 16);

	setArray<<<grid, block>>>(d_DMI, 0.f, g_Width, g_Height);
	setArray<<<grid, block>>>(d_DMI_RL, -1.f, g_Width, g_Height);
	setArray<<<grid, block>>>(d_DMI_LR, -1.f, g_Width, g_Height);
}

extern "C" void gpuCensusImageCleanup() {
	CUDA_SAFE_CALL( cudaUnbindTexture(texImageLeft) );
	CUDA_SAFE_CALL( cudaUnbindTexture(texImageRight) );

	for (int i=0; i<2; i++) {
		CUDA_SAFE_CALL( cudaStreamDestroy(streamImageLeft[i]) );
		CUDA_SAFE_CALL( cudaStreamDestroy(streamImageRight[i]) );
		CUDA_SAFE_CALL( cudaStreamDestroy(streamDM[i]) );

		CUDA_SAFE_CALL( cudaFreeHost(h_left[i]) );
		CUDA_SAFE_CALL( cudaFreeHost(h_right[i]) );
		CUDA_SAFE_CALL( cudaFreeHost(h_dm[i]) );
	}

	CUDA_SAFE_CALL( cudaFreeArray(cuImageLeft) );
	CUDA_SAFE_CALL( cudaFreeArray(cuImageRight) );

    CUDA_SAFE_CALL( cudaFree(d_censusLeft) );
    CUDA_SAFE_CALL( cudaFree(d_censusRight) );
    CUDA_SAFE_CALL( cudaFree(d_DSI) );
#if USE_FAST_AGGREGATION
    CUDA_SAFE_CALL( cudaFree(d_DSI_Temp) );
#endif
    CUDA_SAFE_CALL( cudaFree(d_DMI) );
//    CUDA_SAFE_CALL( cudaFree(d_DMI2) );
    CUDA_SAFE_CALL( cudaFree(d_DMI_LR) );
    CUDA_SAFE_CALL( cudaFree(d_DMI_RL) );
    CUDA_SAFE_CALL( cudaFree(d_Costs) );
    CUDA_SAFE_CALL( cudaFree(d_Costs_LR) );
    CUDA_SAFE_CALL( cudaFree(d_Costs_RL) );
}

extern "C" unsigned char* gpuGetLeftImageBuffer(int nr) {
	if (nr == 0 || nr == 1)
		return h_left[nr];
	else
		return NULL;
}

extern "C" unsigned char* gpuGetRightImageBuffer(int nr) {
	if (nr == 0 || nr == 1)
		return h_right[nr];
	else
		return NULL;
}

extern "C" void gpuCensusSetAsyncImageNr(int nr) {
	g_streamNr = nr;
}

extern "C" void gpuCensusLoadImages(int nr) {
	CUDA_SAFE_CALL( cudaMemcpyToArray(cuImageLeft, 0, 0, h_left[nr], DATA_SIZE, cudaMemcpyHostToDevice) );
	CUDA_SAFE_CALL( cudaMemcpyToArray(cuImageRight, 0, 0, h_right[nr], DATA_SIZE, cudaMemcpyHostToDevice) );
}

extern "C" void gpuCensusSetImages(unsigned char *left, unsigned char *right) {
	CUDA_SAFE_CALL( cudaMemcpyToArray(cuImageLeft, 0, 0, left, DATA_SIZE, cudaMemcpyHostToDevice) );
	CUDA_SAFE_CALL( cudaMemcpyToArray(cuImageRight, 0, 0, right, DATA_SIZE, cudaMemcpyHostToDevice) );
}

extern "C" void gpuGetDisparityMap(float *h_data) {
	CUDA_SAFE_CALL( cudaMemcpy(h_data, d_DMI, g_Width*g_Height*sizeof(float), cudaMemcpyDeviceToHost) );
}

extern "C" void debugGetCensusLeft(int *h_data) {
	CUDA_SAFE_CALL( cudaMemcpy(h_data, d_censusLeft, g_Width*g_Height*sizeof(int)*2, cudaMemcpyDeviceToHost) );
}
extern "C" void debugGetCensusRight(int *h_data) {
	CUDA_SAFE_CALL( cudaMemcpy(h_data, d_censusRight, g_Width*g_Height*sizeof(int)*8, cudaMemcpyDeviceToHost) );
}
extern "C" void debugGetDSI(int *h_data, int d) {
	CUDA_SAFE_CALL( cudaMemcpy(h_data, &d_DSI[g_Width*g_Height*d], g_Width*g_Height*sizeof(int), cudaMemcpyDeviceToHost) );
}
extern "C" void debugGetDMI_LR(float *h_data) {
	CUDA_SAFE_CALL( cudaMemcpy(h_data, d_DMI_LR, g_Width*g_Height*sizeof(float), cudaMemcpyDeviceToHost) );
}
extern "C" void debugGetDMI_RL(float *h_data) {
	CUDA_SAFE_CALL( cudaMemcpy(h_data, d_DMI_RL, g_Width*g_Height*sizeof(float), cudaMemcpyDeviceToHost) );
}
