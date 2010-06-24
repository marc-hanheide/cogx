#include <vector>
#include "DensitySampling.hpp"
#include <float.h>
#include <Navigation/LocalGridMap.hh>
#include <cogxmath.h>
#include <Pose3.h>

using namespace std;
using namespace cogx::Math;

// Represents an axis-aligned equidistant-sampled box shape, whose position 
// is relative to the position (but not the orientation) of a base object
// (object1). The points represent samples of the distribution of a specific
// spatial relation between object1 and another object, object2.
// For each spatial sample point are maintained the values of the relation,
// if object2 were in that position, and there is one value for each of a
// finite set of orientations of object1 and object2.

// Exactly which orientations can be specified externally. For example,
// if the pose of object1 is actually known, there needs be only one.

// Two clouds A and B can be composited together; this presumes that A.object2 
// and B.object1 are the same object, and that the set of
// orientations of A.object2 and B.object1 are also the same.
// Chaining the cloud gives rise to a new cloud C, with
// C.object1 = A.object1
// C.orientations1 = A.orientations1
// C.object2 = B.object2
// C.orientations2 = B.orientations2
// C.samplingInterval = smallest common prime factor of A.samplingInterval and
//	B.samplingInterval
// C.sampleOffset = A.sampleOffset + B.sampleOffset
// 
// The values of each sample point of C are acquired by "convolving" 
// A and B; i.e., B is placed in every point in A and each of B's points weighted
// by the A point is put into C.

// Finally, a cloud can be KDE-sampled into a grid. This grid should have a cell size
// such that the sampling interval of the cloud is an integer multiple of it, or 
// aliasing effects will distort the result (more than necessary).

namespace spatial {

spatial::Object *
copyObject(const spatial::Object *o)
{
  spatial::Object *ret;
  switch (o->type) {
    case OBJECT_HOLLOW_BOX:
      {
	HollowBoxObject *o1hb = (HollowBoxObject *)o;
	HollowBoxObject *hb = new HollowBoxObject;
	ret = hb;
	hb->thickness = o1hb->thickness;
	hb->radius1 = o1hb->radius1;
	hb->radius2 = o1hb->radius2;
	hb->radius3 = o1hb->radius3;
      }
      break;
    case OBJECT_BOX:
      {
	BoxObject *o1b = (BoxObject *)o;
	BoxObject *b = new BoxObject;
	ret = b;
	b->radius1 = o1b->radius1;
	b->radius2 = o1b->radius2;
	b->radius3 = o1b->radius3;
      }
      break;
    case OBJECT_PLANE:
      {
	PlaneObject *o1p = (PlaneObject *)o;
	PlaneObject *p = new PlaneObject;
	ret = p;
	p->shape = o1p->shape;
	p->radius1 = o1p->radius1;
	p->radius2 = o1p->radius2;
      }
      break;
    default:
      cerr << "Error: unsupported object type!\n";
      exit(1);
  }
  ret->type = o->type;
  ret->pose = o->pose;
  return ret;
}

SampleCloud::SampleCloud(const spatial::Object *o1, const spatial::Object *o2, 
    Vector3 offset,
    double intervalQuantum,
    int intervalMultiplier,
    int xExt,
    int yExt,
    int zExt,
    const vector<Matrix33> &orientations1,
    const vector<Matrix33> &orientations2) :
  sampleOffset(offset),
  sampleIntervalQuantum(intervalQuantum),
  sampleIntervalMultiplier(intervalMultiplier),
  xExtent(xExt),
  yExtent(yExt),
  zExtent(zExt),
  object1Orientations(orientations1),
  object2Orientations(orientations2)
{
  object1 = copyObject(o1);
  object2 = copyObject(o2);
}

SampleCloud::SampleCloud(const SampleCloud &a) :
  sampleOffset(a.sampleOffset),
  sampleIntervalQuantum(a.sampleIntervalQuantum),
  sampleIntervalMultiplier(a.sampleIntervalMultiplier),
  xExtent(a.xExtent),
  yExtent(a.yExtent),
  zExtent(a.zExtent),
  object1Orientations(a.object1Orientations),
  object2Orientations(a.object2Orientations)
{
  object1 = copyObject(a.object1);
  object2 = copyObject(a.object2);
}

SampleCloud & 
SampleCloud::operator=(const SampleCloud &a) 
{
  sampleOffset = a.sampleOffset;
  sampleIntervalQuantum = a.sampleIntervalQuantum;
  sampleIntervalMultiplier = a.sampleIntervalMultiplier;
  xExtent = a.xExtent;
  yExtent = a.yExtent;
  zExtent = a.zExtent;
  object1Orientations = a.object1Orientations;
  object2Orientations = a.object2Orientations;
  values = a.values;

  if (object1 != 0) delete object1;
  if (object2 != 0) delete object2;

  object1 = copyObject(a.object1);
  object2 = copyObject(a.object2);
  return *this;
}

inline bool
isInTriangle(double x, double y, const vector<Vector3> &triangle) {
  for (int i = 0; i < 3; i++) {
    int iplus = i == 2 ? 0 : i+1;
    Vector3 side = triangle[iplus] - triangle[i];
    Vector3 diff = vector3(x,y,0) - triangle[i];
    if (cross(side,diff).z < 0)
      return false;
  }
  return true;
}

struct OffsetKernel {
  int minxi;
  int maxxi;
  int minyi;
  int maxyi;
  vector<double> weights;
};


void
sampleBinaryRelationRecursively(const vector <SpatialRelationType> &relations,
    const vector<spatial::Object *> &objects,
    int currentLevel, Cure::LocalGridMap<double> &outMap,
    int sampleNumberTarget, int orientationQuantization, double kernelWidthFactor, 
    double &total, const vector<Vector3> &triangle, double baseOnness)
{
  spatial::Object *supportObject = objects[currentLevel+1];
  if (supportObject->pose.pos.x == -FLT_MAX) {
    cerr << "Error! Support object pose uninitialized!\n";
    return;
  }

  SpatialRelationType relationType = relations[currentLevel];
  spatial::Object *onObject = objects[currentLevel];

  Pose3 oldPose = onObject->pose;

  double frameRadius;
  double maxVertical;
  double minVertical;
  double maxLateral;
  switch (relationType) {

    case RELATION_ON:
      if (supportObject->type == spatial::OBJECT_PLANE) {
	spatial::PlaneObject &table1 = (spatial::PlaneObject &)(*supportObject);
	if (table1.shape == spatial::PLANE_OBJECT_RECTANGLE) {
	  frameRadius = table1.radius1 > table1.radius2 ?
	    table1.radius1 : table1.radius2;
	}
	else {
	  cerr << "Unsupported object type!\n";
	  return;
	}
      }
      else if (supportObject->type == spatial::OBJECT_BOX ||
	  supportObject->type == spatial::OBJECT_HOLLOW_BOX) {
	spatial::BoxObject &box1 = (spatial::BoxObject &)(*supportObject);
	frameRadius = box1.radius1 > box1.radius2 ?
	  box1.radius1 : box1.radius2;
	frameRadius = frameRadius > box1.radius3 ? 
	  frameRadius : box1.radius3;
      }
      else {
	cerr << "Unsupported object type!\n";
	return;
      }
      maxLateral = frameRadius*1.5;
      minVertical = -frameRadius*1.5;
      maxVertical = frameRadius*3;
      break;

    case RELATION_IN:
      if (supportObject->type == spatial::OBJECT_HOLLOW_BOX) {
	spatial::BoxObject &box1 = (spatial::BoxObject &)(*supportObject);
	frameRadius = box1.radius1 > box1.radius2 ?
	  box1.radius1 : box1.radius2;
	frameRadius = frameRadius > box1.radius3 ? 
	  frameRadius : box1.radius3;
      }
      else {
	cerr << "Unsupported object type!\n";
	return;
      }
      maxLateral = frameRadius*1.5;
      minVertical = -frameRadius*1.5;

      maxVertical = frameRadius*1.5;
      break;
  }

  int mapSize = outMap.getSize();
  double cellSize = outMap.getCellSize();

  double x1 = -maxLateral + supportObject->pose.pos.x;
  double x2 = maxLateral + supportObject->pose.pos.x;
  double y1 = -maxLateral + supportObject->pose.pos.y;
  double y2 = maxLateral + supportObject->pose.pos.y;
  double zmin = supportObject->pose.pos.z + minVertical;
  double zmax = supportObject->pose.pos.z + maxVertical;

  int sampleNumberTargetUsed;
  switch (objects.size()-2 - currentLevel) {
    case 0: //Base level
      sampleNumberTargetUsed = sampleNumberTarget;
      break;
    case 1: //One level of indirection
    default:
      sampleNumberTargetUsed = sampleNumberTarget / 10;
      break;
  }


  // Distribute N samples over the whole volume
  double sampleVolume = (x2-x1)*(y2-y1)*(zmax-zmin);

  // samplingInterval: distance between successive samples.
  // Can be set to a non-multiple of cellSize, but that may lead to
  // aliasing problems.

  double samplingInterval = pow(sampleVolume/sampleNumberTargetUsed, 1/3.0); 
  // Round samplingInterval upward to the nearest multiple of cellSize
  // (keeps the number of sampled positions less than sampleNumberTarget)
  samplingInterval = cellSize*(ceil(samplingInterval/cellSize));

  double kernelRadius = samplingInterval * kernelWidthFactor;
  if (kernelRadius < cellSize)
    kernelRadius = cellSize;
  int kernelWidth = (int)(ceil(kernelRadius/cellSize)+0.1);
  //How many kernelRadiuses per cell
  double kernelStep = cellSize/kernelRadius; 

  vector<Matrix33> orientations;
  getRandomSampleSphere(orientations, orientationQuantization);

  double z0 = ((double)rand())/RAND_MAX*samplingInterval;
  //Offset, 0-centered, along length samplingInterval
  double xOffset = (((double)rand())/RAND_MAX) * samplingInterval; 
  double yOffset = (((double)rand())/RAND_MAX) * samplingInterval; 
  for (double sx = xOffset + x1;
      sx < x2; sx += samplingInterval) {
    for (double sy = yOffset + y1;
	sy < y2; sy += samplingInterval) {
      if (triangle.size() > 0 && currentLevel == 0 &&
	  !isInTriangle(sx, sy, triangle))
	continue;

      //Put together a kernel offset by this much
      OffsetKernel kernel;

      int sampleMapX, sampleMapY;
      outMap.worldCoords2Index(sx, sy, sampleMapX, sampleMapY);

      // Offset of sample position from center of its containing cell
      double offsetInCellX = sx - cellSize * (floor(sx / cellSize)+0.5);
      double offsetInCellY = sy - cellSize * (floor(sy / cellSize)+0.5);

      kernel.minxi = -kernelWidth; //Relative indices of kernel patch extents
      kernel.minyi = -kernelWidth;
      kernel.maxxi = +kernelWidth;
      kernel.maxyi = +kernelWidth;

      double minx, miny; 
      outMap.index2WorldCoords(kernel.minxi,kernel.minyi,minx,miny);
      minx -= offsetInCellX; //Relative coords of minxi, minyi in meters
      miny -= offsetInCellY;

      minx /= kernelRadius; //Relative coords of minxi, minyi in kernel radii
      miny /= kernelRadius;

      double sumWeights = 0.0;
      double x = minx;
      for (int xi = kernel.minxi; xi <= kernel.maxxi; xi++, x+=kernelStep) {
	double y = miny;
	for (int yi = kernel.minyi; yi <= kernel.maxyi; yi++, y+=kernelStep) {
	  double sq = x*x+y*y;
	  //double sqsum = 1 - sq;
	  double sqsum = (1-sq)*(1-sq)*(1-sq);
	  if (sqsum > 0) {
	    kernel.weights.push_back(sqsum);
	    sumWeights+=sqsum;
	  }
	  else {
	    kernel.weights.push_back(0);
	  }
	}
      }
      for (unsigned int i = 0; i < kernel.weights.size(); i++) {
	kernel.weights[i] = kernel.weights[i]/sumWeights;
      }

      double totalValueThisXY = 0.0; //Sum sample values of all samples at this
      //(x,y)
      for (double sz = z0+zmin; sz < zmax; sz+=samplingInterval) {
	double totalValueThisXYZ = 0;
	for (unsigned int orientationNo = 0; orientationNo < orientations.size(); orientationNo++) {
	  onObject->pose.rot = orientations[orientationNo];
	  onObject->pose.pos.x = sx;
	  onObject->pose.pos.y = sy;
	  onObject->pose.pos.z = sz;
	  double value;
	  if (relationType == RELATION_ON) {
	    value = evaluateOnness(supportObject, onObject);
	  }
	  else if (relationType == RELATION_IN) {
	    value = evaluateInness(supportObject, onObject);
	  }
	  else {
	    cerr << "Error! Unknown binary relation type!\n";
	    return;
	  }

	  if (currentLevel == 0) {
	    totalValueThisXY += value;
	    totalValueThisXYZ += value;
	  }
	  else {
	    // Sample and recurse, if the value is above a threshold
	    if (value > 0.5) {
	      sampleBinaryRelationRecursively(relations, objects, currentLevel-1,
		  outMap, sampleNumberTarget, orientationQuantization, 
		  kernelWidthFactor, total, triangle, value * baseOnness);
	    }
	  }
	}
      }

      if (currentLevel == 0) {
	// This is the trajector itself
	// Accumulate the kernel by this much
	int kernelCellNo = 0;
	for (int xi = sampleMapX+kernel.minxi; xi <= sampleMapX+kernel.maxxi; xi++) {
	  for (int yi = sampleMapY+kernel.minyi; yi <= sampleMapY+kernel.maxyi; yi++, kernelCellNo++) {
	    if (xi <= - mapSize || xi >= mapSize ||
		yi <= - mapSize || yi >= mapSize)
	      continue;

	    outMap(xi, yi)+=kernel.weights[kernelCellNo] * totalValueThisXY * baseOnness;
	    total+=kernel.weights[kernelCellNo] * totalValueThisXY * baseOnness;
	  }
	}
      }
      else {
      }
    }
  }
  onObject->pose = oldPose;
}

void
sampleBinaryRelationSystematically(const vector <SpatialRelationType> &relations,
    const vector<spatial::Object *> &objects,
    Cure::LocalGridMap<double> &outMap,
    int sampleNumberTarget, int orientationQuantization, double kernelWidthFactor,
    double &total, double baseValue)
{
  spatial::Object *supportObject = objects.back();
  if (supportObject->pose.pos.x == -FLT_MAX) {
    cerr << "Error! Support object pose uninitialized!\n";
    return;
  }

  // Set up a systematically sampled sphere for each variable object
  vector<vector<Matrix33> >objectOrientations;
  for (unsigned int i = 1; i < objects.size(); i++) {
    objectOrientations.push_back(vector<Matrix33>());
    getRandomSampleSphere(objectOrientations.back(), orientationQuantization);
  }
  objectOrientations.push_back(vector<Matrix33>());
  objectOrientations.back().push_back(supportObject->pose.rot);


  // Set up the parameters of the position sampling
  // If the relation is independent of absolute orientation, this
  // sample system will rotate with the object;
  // otherwise it will be in global coordinates.

  SampleCloud totalCloud;

  unsigned int currentLevel = objects.size() - 1;

  do {
    currentLevel--;
    SpatialRelationType relationType = relations[currentLevel];
    spatial::Object *onObject = objects[currentLevel];
    supportObject = objects[currentLevel+1];

    Pose3 oldPose = onObject->pose;


    // Find sample box extents depending on support object dimensions
    // and relation type
    double frameRadius;
    double maxVertical;
    double minVertical;
    double maxLateral;
    switch (relationType) {

      case RELATION_ON:
	if (supportObject->type == spatial::OBJECT_PLANE) {
	  spatial::PlaneObject &table1 = (spatial::PlaneObject &)(*supportObject);
	  if (table1.shape == spatial::PLANE_OBJECT_RECTANGLE) {
	    frameRadius = table1.radius1 > table1.radius2 ?
	      table1.radius1 : table1.radius2;
	  }
	  else {
	    cerr << "Unsupported object type!\n";
	    return;
	  }
	}
	else if (supportObject->type == spatial::OBJECT_BOX ||
	    supportObject->type == spatial::OBJECT_HOLLOW_BOX) {
	  spatial::BoxObject &box1 = (spatial::BoxObject &)(*supportObject);
	  frameRadius = box1.radius1 > box1.radius2 ?
	    box1.radius1 : box1.radius2;
	  frameRadius = frameRadius > box1.radius3 ? 
	    frameRadius : box1.radius3;
	}
	else {
	  cerr << "Unsupported object type!\n";
	  return;
	}
	maxLateral = frameRadius*1.5;
	minVertical = -frameRadius*1.5;
	maxVertical = frameRadius*3;
	break;

      case RELATION_IN:
	if (supportObject->type == spatial::OBJECT_HOLLOW_BOX) {
	  spatial::BoxObject &box1 = (spatial::BoxObject &)(*supportObject);
	  frameRadius = box1.radius1 > box1.radius2 ?
	    box1.radius1 : box1.radius2;
	  frameRadius = frameRadius > box1.radius3 ? 
	    frameRadius : box1.radius3;
	}
	else {
	  cerr << "Unsupported object type!\n";
	  return;
	}
	maxLateral = frameRadius*1.5;
	minVertical = -frameRadius*1.5;

	maxVertical = frameRadius*1.5;
	break;
    }

    int mapSize = outMap.getSize();
    double cellSize = outMap.getCellSize();

    double x1 = -maxLateral + supportObject->pose.pos.x;
    double x2 = maxLateral + supportObject->pose.pos.x;
    double y1 = -maxLateral + supportObject->pose.pos.y;
    double y2 = maxLateral + supportObject->pose.pos.y;
    double zmin = supportObject->pose.pos.z + minVertical;
    double zmax = supportObject->pose.pos.z + maxVertical;

    // Distribute N samples over the whole volume
    double sampleVolume = (x2-x1)*(y2-y1)*(zmax-zmin);

    // samplingInterval: distance between successive samples.
    // Can be set to a non-multiple of cellSize, but that may lead to
    // aliasing problems.

    double samplingInterval = pow(sampleVolume/sampleNumberTarget, 1/3.0); 
    // Round samplingInterval upward to the nearest multiple of cellSize
    // (keeps the number of sampled positions less than sampleNumberTarget)
    samplingInterval = cellSize*(ceil(samplingInterval/cellSize));
    int samplingMultiplier = (int)(samplingInterval/cellSize+0.1);

    double z0 = ((double)rand())/RAND_MAX*samplingInterval;
    //Offset, 0-centered, along length samplingInterval
    double xOffset = (((double)rand())/RAND_MAX) * samplingInterval; 
    double yOffset = (((double)rand())/RAND_MAX) * samplingInterval; 

    int xExtent = (int)(maxLateral / samplingInterval);
    int yExtent = (int)(maxLateral / samplingInterval);
    int zExtent = (int)(maxVertical / samplingInterval);

    SampleCloud cloud(supportObject, onObject, 
	vector3(xOffset, yOffset, z0),
	cellSize, samplingMultiplier,
	xExtent,
	yExtent,
	zExtent,
	objectOrientations[currentLevel+1],
	objectOrientations[currentLevel]);

//    log("Computing relation: %i", currentLevel);
    cloud.compute(relationType);
    if (currentLevel == objects.size()-2) {
      totalCloud = cloud;
    }
    else {
//      log("Compositing relations");
      totalCloud = totalCloud.composit(cloud);
    }

  } while (currentLevel > 0);

//  log("Writing into 2D grid");
  totalCloud.KernelDensityEstimation2D(outMap, 
      supportObject->pose.pos, kernelWidthFactor, 
      total, baseValue);

}

SampleCloud
SampleCloud::composit(const SampleCloud &B) const {
  const SampleCloud &A = *this;

  if (A.sampleIntervalQuantum != B.sampleIntervalQuantum) {
    cerr << "Interval length mismatch!";
    exit(1);
  }

  //Find largest common factor between interval multipliers
  int maxMult, minMult;
  if (A.sampleIntervalMultiplier > B.sampleIntervalMultiplier) {
    maxMult = A.sampleIntervalMultiplier;
    minMult = B.sampleIntervalMultiplier;
  }
  else {
    maxMult = B.sampleIntervalMultiplier;
    minMult = A.sampleIntervalMultiplier;
  }
  int i;
  for (i = minMult; i > 1; i--) {
    if (maxMult % i == 0 && minMult % i == 0) 
      break;
  }

  int newXExtent = A.xExtent * A.sampleIntervalMultiplier + 
    B.xExtent * B.sampleIntervalMultiplier;
  int newYExtent = A.yExtent * A.sampleIntervalMultiplier + 
    B.yExtent * B.sampleIntervalMultiplier;
  int newZExtent = A.zExtent * A.sampleIntervalMultiplier + 
    B.zExtent * B.sampleIntervalMultiplier;

  SampleCloud C (A.object1, B.object2, 
      A.sampleOffset + B.sampleOffset,
      A.sampleIntervalQuantum,
      i, 
      newXExtent,
      newYExtent,
      newZExtent,
      A.object1Orientations,
      B.object2Orientations);

  long numberOfValuesInC = (2*newXExtent+1) * (2*newYExtent+1) * (2*newZExtent+1) 
    * A.object1Orientations.size() * B.object2Orientations.size();

  C.values.resize(numberOfValuesInC); //Ooof!

  int AStepInC = A.sampleIntervalMultiplier / C.sampleIntervalMultiplier;
  int BStepInC = B.sampleIntervalMultiplier / C.sampleIntervalMultiplier;

  //  int AXL = 2 * A.xExtent + 1;
  int AYL = 2 * A.yExtent + 1;
  int AZL = 2 * A.zExtent + 1;
  //  int BXL = 2 * B.xExtent + 1;
  int BYL = 2 * B.yExtent + 1;
  int BZL = 2 * B.zExtent + 1;
  //  int CXL = 2 * C.xExtent + 1;
  int CYL = 2 * C.yExtent + 1;
  int CZL = 2 * C.zExtent + 1;

  int nOrientations1 = A.object1Orientations.size();
  int nOrientations2 = B.object2Orientations.size();
  int nSummedOrientations = A.object2Orientations.size();

  int valuesPer3DPoint = nOrientations1 * nOrientations2;

  // Now, loop over all the sample points in A

  long samplesInA = AYL * AZL * (2*A.xExtent+1);
  long samplesInB = BYL * BZL * (2*B.xExtent+1);

  for (int a = 0, ca = 0; a < samplesInA; a++, ca+=AStepInC) {
    long pointOffsetInA = valuesPer3DPoint * a;
    for (int b = 0, cb = 0; b < samplesInB; b++, cb+=BStepInC) {
      long pointOffsetInB = valuesPer3DPoint * b;
      long pointOffsetInC = valuesPer3DPoint * (ca+cb);

      for (int i = 0; i < nOrientations1; i++) {
	for (int j = 0; j < nOrientations2; j++) {
	  double sum = 0.0;
	  for (int k = 0; k < nSummedOrientations; k++) {
	    sum += A.values[pointOffsetInA + nSummedOrientations * i + k]
	      * B.values[pointOffsetInB + nOrientations2 * k + j];
	  }
	  C.values[pointOffsetInC + nOrientations2 * i + j] += sum;
	}
      }
    }
  }

  return C;
}

void
SampleCloud::compute(SpatialRelationType rel) {
  // Fill the values vector with values for the relation in question, 
  // for each relative position, and each combination of orientations
  // for the two objects.

  // First make sure the values vector has the right size
  long numberOfValues = (2*xExtent+1) * (2*yExtent+1) * (2*zExtent+1) 
    * object1Orientations.size() * object2Orientations.size();

  values.resize(numberOfValues); //Ooof!

  // Loop over relative positions
  double sampleInterval = sampleIntervalQuantum * sampleIntervalMultiplier;
  vector<double>::iterator it = values.begin();
  double minX = sampleOffset.x - xExtent * sampleInterval;
  double minY = sampleOffset.y - yExtent * sampleInterval;
  double minZ = sampleOffset.z - zExtent * sampleInterval;

  double x = minX, y, z;
  object1->pose.pos = vector3(0,0,0);
  std::set<int> orientationsOverThreshold;
  long samplesOverThreshold = 0;
  switch (rel) {
    case RELATION_ON:
      for (int xi = -xExtent; xi <= xExtent; xi++, x += sampleInterval) {
	y = minY;
	for (int yi = -yExtent; yi <= yExtent; yi++, y += sampleInterval) {
	  z = minZ;
	  for (int zi = -zExtent; zi <= zExtent; zi++, z += sampleInterval) {
	    object2->pose.pos = vector3(x,y,z);

	    // Loop over combinations of orientations
	    for (vector<Matrix33>::iterator o1it = object1Orientations.begin();
		o1it != object1Orientations.end(); o1it++) {
	      object1->pose.rot = *o1it;
	    int i = 0;
	      for (vector<Matrix33>::iterator o2it = object2Orientations.begin();
		  o2it != object2Orientations.end(); o2it++) {
		object2->pose.rot = *o2it;
		(*it) = evaluateOnness(object1, object2);
//		if (*it > 0.5) {
//		  orientationsOverThreshold.insert(i);
//		  samplesOverThreshold++;
//		}
		it++;
	      i++;
	      }
	    }
	  }
	}
      }
      break;
    case RELATION_IN:
      for (int xi = -xExtent; xi <= xExtent; xi++, x += sampleInterval) {
	y = minY;
	for (int yi = -yExtent; yi <= yExtent; yi++, y += sampleInterval) {
	  z = minZ;
	  for (int zi = -zExtent; zi <= zExtent; zi++, z += sampleInterval) {
	    object2->pose.pos = vector3(x,y,z);

	    // Loop over combinations of orientations
	    for (vector<Matrix33>::iterator o1it = object1Orientations.begin();
		o1it != object1Orientations.end(); o1it++) {
	      object1->pose.rot = *o1it;
	      for (vector<Matrix33>::iterator o2it = object2Orientations.begin();
		  o2it != object2Orientations.end(); o2it++) {
		object2->pose.rot = *o2it;
		(*it) = evaluateInness(object1, object2);
		it++;
	      }
	    }
	  }
	}
      }
      break;
    default:
      cerr << "Error! Unsupported relation\n";
      exit(1);
  }
  cout << orientationsOverThreshold.size() << " orientations\n";
}

void 
SampleCloud::KernelDensityEstimation2D(Cure::LocalGridMap<double> &outMap,
    Vector3 cloudCenter, double kernelWidthFactor, double &total, double baseValue)
{
  double sampleInterval = sampleIntervalQuantum * sampleIntervalMultiplier;
  double cellSize = outMap.getCellSize();
  int mapSize = outMap.getSize();

  double kernelRadius = sampleInterval * kernelWidthFactor;
  if (kernelRadius < cellSize)
    kernelRadius = cellSize;
  int kernelWidth = (int)(ceil(kernelRadius/cellSize)+0.1);
  //How many kernelRadius:es per cell
  double kernelStep = cellSize/kernelRadius; 

  //Put together a kernel with this width, offset by sampleOffset
  OffsetKernel kernel;

  Vector3 sampleCenter = cloudCenter + sampleOffset;

  double sx = sampleCenter.x;
  double sy = sampleCenter.y;
//  double sz = sampleCenter.z;

  int sampleMapX, sampleMapY;
  outMap.worldCoords2Index(sx, sy, sampleMapX, sampleMapY);


  kernel.minxi = -kernelWidth; //Relative indices of kernel patch extents
  kernel.minyi = -kernelWidth;
  kernel.maxxi = +kernelWidth;
  kernel.maxyi = +kernelWidth;

  double minx = (sx - kernelWidth * cellSize)/kernelRadius; //Note: Normalized coordinates
  double miny = (sy - kernelWidth * cellSize)/kernelRadius; //(relative to kernel radius)

  double sumWeights = 0.0;
  double x = minx;
  for (int xi = kernel.minxi; xi <= kernel.maxxi; xi++, x+=kernelStep) {
    double y = miny;
    for (int yi = kernel.minyi; yi <= kernel.maxyi; yi++, y+=kernelStep) {
      double sq = x*x+y*y;
      //double sqsum = 1 - sq;
      double sqsum = (1-sq)*(1-sq)*(1-sq);
      if (sqsum > 0) {
	kernel.weights.push_back(sqsum);
	sumWeights+=sqsum;
      }
      else {
	kernel.weights.push_back(0);
      }
    }
  }
  for (unsigned int i = 0; i < kernel.weights.size(); i++) {
    kernel.weights[i] = kernel.weights[i]/sumWeights;
  }


  //Kernel done, now KDE the heck out of it!

  vector<double>::iterator it = values.begin();
  double minX = sampleOffset.x - xExtent * sampleInterval;
  double minY = sampleOffset.y - yExtent * sampleInterval;
  double minZ = sampleOffset.z - zExtent * sampleInterval;

  const unsigned int totalOrientationCombinations = object2Orientations.size() * object1Orientations.size();

  x = minX;
  // Loop over samples in cloud
  for (int xi = -xExtent; xi <= xExtent; xi++, x += sampleInterval) {
    double y = minY;
    for (int yi = -yExtent; yi <= yExtent; yi++, y += sampleInterval) {
      double z = minZ;
      //Sum up all weight inside a column of samples before accumulating the kernel
      double totalValueThisXY = 0.0;
      for (int zi = -zExtent; zi <= zExtent; zi++, z += sampleInterval) {
	// Loop over combinations of orientations
	for (unsigned int i = 0; i < totalOrientationCombinations; i++) {
	  totalValueThisXY += *it;
	  it++;
	}
      }

      // Loop over kernel
      int kernelCellNo = 0;
      int kX = sampleMapX + xi * sampleIntervalMultiplier;
      int kY = sampleMapY + yi * sampleIntervalMultiplier;
      for (int kxi = kX+kernel.minxi; kxi <= kX+kernel.maxxi; kxi++) {
	for (int kyi = kY+kernel.minyi; kyi <= kY+kernel.maxyi; kyi++, kernelCellNo++) {
	  if (kxi <= - mapSize || kxi >= mapSize ||
	      kyi <= - mapSize || kyi >= mapSize)
	    continue;

	  outMap(kxi, kyi)+=kernel.weights[kernelCellNo] * totalValueThisXY * baseValue;
	  total+=kernel.weights[kernelCellNo] * totalValueThisXY * baseValue;
	}
      }
    }
  }
}

}
