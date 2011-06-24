CXXFLAGS=-g -Wall -I./ -I${CURE_ROOT}/include/cure/ -I/usr/local/pkg/csm/git/csm/deploy/include/ -I${CURE_ROOT}/include/cure/ -I/usr/local/AndreaCensi-csm-0756879/deploy/include

LDFLAGS=-L/usr/local/pkg/csm/git/csm/deploy/lib -L/usr/local/AndreaCensi-csm-0756879/deploy/lib -L${CURE_ROOT}/lib/cure -L${CURE_ROOT}/lib/cure -lCureUtils -lCureAddressBank -lCureTransformation -lCureMath -lCureGeometry -lCureSensorData -lCureSensory -lpthread -lpeekabot -lcairo -lgsl -lgslcblas -lm #-lcsm 

OBJS = HSSScanMotionDetector.o HSSRedundantLine2DRep.o HSSScanAlgorithm.o HSSLsqLineExtractor.o HSSDoorExtractor.o HSSScan2DMatcher.o HSSCandScan2D.o HSSMotionEstimator.o HSSScan2DFilter.o HSSDisplayFunctions.o HSSLocalMap.o HSSMap.o HSSMapTransformation.o HSSAngleHistogram.o HSSVirtualScan2DFactory.o

BINS = testDoorExtractor

all: ${BINS}

testScanSLAM: testScanSLAM.cc ${OBJS}

testScanSLAM2: testScanSLAM2.cc ${OBJS}

testDoorExtractor: testDoorExtractor.cc HSSDoorExtractor.o HSSLsqLineExtractor.o HSSDisplayFunctions.o HSSRedundantLine2DRep.o 

testGridMap: testGridMap.cc ${OBJS}

clean:
	rm -f *.o *~ ${BINS} ${OBJS}
